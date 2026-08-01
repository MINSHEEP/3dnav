# map_switcher_service.py
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from lzp_interface.srv import SwitchMap
from nav2_msgs.srv import LoadMap
import os
import yaml
import re
import threading
import time
import hashlib
import math
import numpy as np
from PIL import Image
from lzp_interface.srv import EditMap  # 导入新定义的服务
from ament_index_python.packages import get_package_share_directory
from lzp_interface.srv import ControlLaunch
from lzp_interface.srv import GetMapList  # 需要添加这个导入
from lzp_interface.srv import GetCurrentMap  # 新增：查询当前地图服务
from lzp_interface.srv import DeleteMap      # 新增：删除地图服务
from lzp_interface.msg import CurrentMap
from lzp_interface.srv import GetEditedMap   # 新增：获取编辑后的地图
from lzp_interface.srv import SetMapPixelIndices
from std_msgs.msg import Bool

from .route_database import RouteDatabase


LOCALIZATION_MAP_SCHEMA = 'ros2_mid360_localization_map'
LOCALIZATION_MAP_VERSION = 1
LOCALIZATION_METADATA_SUFFIX = '.localization.yaml'
FILE_HASH_CHUNK_SIZE = 4 * 1024 * 1024
GEOMETRY_ABSOLUTE_TOLERANCE = 1.0e-6
MAX_LOCALIZATION_POINT_COUNT = 1_000_000
MIN_LOCALIZATION_POINT_COUNT = 2_000
MAX_LOCALIZATION_PCD_BYTES = 64 * 1024 * 1024


class MapSwitcherService(Node):
    def __init__(self):
        super().__init__('map_switcher_service')
        
        # 创建可重入回调组以支持并发处理
        self.callback_group = ReentrantCallbackGroup()
        
        # 地图文件目录
        self.map_directory = os.path.join('/home/a/nav_ws/src/ros2_humble_main/src/navigation/robot_navigation2', 'maps')
        self.pcdmap_directory = os.path.join('/home/a/nav_ws/src/ros2_humble_main/src/navigation/robot_navigation2', 'pcdmaps')
        
        # 缓存已加载的地图图像，用于批量编辑
        self.loaded_maps_cache = {}  # {map_name: {'image': img_array, 'original_path': path}}
        # The services share a reentrant callback group, so cache updates must be
        # serialized.  Each cached map also owns an undo stack.
        self.map_cache_lock = threading.RLock()
        self._undo_sequence = 0
        self.map_load_lock = threading.RLock()
        self.pending_map_name = None
        # Authoritative live map name. The YAML value is only the startup
        # default and must not overwrite a successful LoadMap result.
        self.active_map_name = None
        self.navigation_active = False
        configured_map_server_timeout = self.declare_parameter(
            'map_server_active_timeout_sec', 60.0).value
        try:
            self.map_server_active_timeout_sec = max(
                20.0, float(configured_map_server_timeout))
        except (TypeError, ValueError):
            self.map_server_active_timeout_sec = 60.0
            self.get_logger().warn(
                'Invalid map_server_active_timeout_sec; using 60 seconds')
        route_db_path = self.declare_parameter('route_db_path', '').value
        self.route_database = RouteDatabase(route_db_path or None)

        self.set_pixel_indices_service = self.create_service(
            SetMapPixelIndices, 'set_map_pixel_indices', self.handle_set_map_pixel_indices,
            callback_group=self.callback_group)


        # 新增：获取编辑后地图的服务
        self.get_edited_map_service = self.create_service(
            GetEditedMap, 'get_edited_map', self.handle_get_edited_map,
            callback_group=self.callback_group)
        # 新增：保存地图服务（用于统一保存所有更改）
        self.save_maps_service = self.create_service(
            SwitchMap, 'save_edited_maps', self.handle_save_edited_maps,  # 使用SwitchMap作为临时类型
            callback_group=self.callback_group)
        
        # 新增：撤销编辑服务
        self.undo_edit_service = self.create_service(
            SwitchMap, 'undo_map_edits', self.handle_undo_edits,  # 使用SwitchMap作为临时类型
            callback_group=self.callback_group)



        # 创建服务，使用可重入回调组
        self.switch_map_service = self.create_service(
            SwitchMap, 'switch_map', self.handle_switch_map,
            callback_group=self.callback_group)
        
                # 创建获取地图列表的服务
        self.get_map_list_service = self.create_service(
            GetMapList, 'get_map_list', self.handle_get_map_list,
            callback_group=self.callback_group)
        

        # 新增：查询当前地图服务
        # 新增：当前地图信息话题发布者（替代服务）
        map_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.current_map_publisher = self.create_publisher(
            CurrentMap, 'current_map_info', map_state_qos)
        self.create_subscription(
            Bool,
            '/navigation_active',
            self.navigation_active_callback,
            map_state_qos,
            callback_group=self.callback_group)
        
        # 新增：删除地图服务
        self.delete_map_service = self.create_service(
            DeleteMap, 'delete_map', self.handle_delete_map,
            callback_group=self.callback_group)

        # 新增：地图编辑服务
        self.edit_map_service = self.create_service(
            EditMap, 'edit_map', self.handle_edit_map,
            callback_group=self.callback_group)
            
        # 连接到地图服务器的load_map服务
        self.load_map_cli = self.create_client(
            LoadMap, '/map_server/load_map',
            callback_group=self.callback_group)
        self.map_server_state_cli = self.create_client(
            GetState, '/map_server/get_state',
            callback_group=self.callback_group)
        

                # 连接到launch管理器的重启服务
        self.restart_launch_cli = self.create_client(
            ControlLaunch, 'control_launch',  # 假设服务名为restart_navigation
            callback_group=self.callback_group)
        
        self.get_logger().info('Map Switcher Service started')
        self.pending_requests = {}  # 存储待处理的请求
         # 新增：启动定时器定期发布当前地图信息
        self.current_map_timer = self.create_timer(1.0, self.publish_current_map_info)

    def navigation_active_callback(self, message):
        self.navigation_active = bool(message.data)


    def pgm_to_occgrid(self,pgm_value):
        if pgm_value == 0:      # 黑色 = 障碍物
            return 100
        elif pgm_value == 254:  # 白色 = 自由空间
            return 0
        elif pgm_value in (128, 205):  # ROS/Nav2 maps commonly use gray 205 for unknown
            return -1
        else:  # 其他值按比例转换
            return int((254 - pgm_value) * 100 / 254)

    @staticmethod
    def _safe_map_name(raw_name, allow_empty=False):
        """Return a basename-only map name safe for use below maps/."""
        name = str(raw_name or '').strip()
        if allow_empty and not name:
            return ''
        if (
            not name
            or len(name) > 128
            or name in ('.', '..')
            or os.path.basename(name) != name
            or '/' in name
            or '\\' in name
            or any(ord(character) < 32 for character in name)
        ):
            raise ValueError('Invalid map name; use a plain filename without path separators')
        return name

    @staticmethod
    def _load_yaml_mapping(path, description):
        """Load a YAML document and require a mapping at its root."""
        try:
            with open(path, 'r', encoding='utf-8') as stream:
                document = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as error:
            raise ValueError(
                f'无法读取{description}“{os.path.basename(path)}”：{error}') from error
        if not isinstance(document, dict):
            raise ValueError(f'{description}“{os.path.basename(path)}”的根节点必须是对象')
        return document

    @staticmethod
    def _finite_float(value, field_name):
        try:
            converted = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(f'定位元数据字段 {field_name} 不是有效数字') from error
        if not math.isfinite(converted):
            raise ValueError(f'定位元数据字段 {field_name} 不是有限数字')
        return converted

    @classmethod
    def _finite_float_list(cls, value, expected_length, field_name):
        if not isinstance(value, (list, tuple)) or len(value) != expected_length:
            raise ValueError(
                f'定位元数据字段 {field_name} 必须包含 {expected_length} 个数字')
        return [
            cls._finite_float(item, f'{field_name}[{index}]')
            for index, item in enumerate(value)
        ]

    @staticmethod
    def _same_number(left, right):
        return math.isclose(
            float(left),
            float(right),
            rel_tol=0.0,
            abs_tol=GEOMETRY_ABSOLUTE_TOLERANCE)

    @staticmethod
    def _sha256_file(path):
        digest = hashlib.sha256()
        with open(path, 'rb') as stream:
            while True:
                chunk = stream.read(FILE_HASH_CHUNK_SIZE)
                if not chunk:
                    break
                digest.update(chunk)
        return digest.hexdigest()

    def validate_localization_map(self, map_name, map_yaml_path):
        """Validate the 2D/3D map pair before Nav2 is allowed to load it.

        Maps saved before the versioned localization metadata was introduced
        are intentionally rejected.  This prevents an unrelated or stale PCD
        from silently refining an otherwise valid 2D initial pose.
        """
        expected_image_name = f'{map_name}.pgm'
        expected_pcd_name = f'{map_name}.pcd'
        image_path = os.path.join(self.map_directory, expected_image_name)
        pcd_path = os.path.join(self.pcdmap_directory, expected_pcd_name)
        metadata_path = os.path.join(
            self.pcdmap_directory,
            f'{map_name}{LOCALIZATION_METADATA_SUFFIX}')

        if not os.path.isfile(image_path):
            raise ValueError(
                f'缺少同名二维地图 {expected_image_name}，禁止导航')
        if not os.path.isfile(pcd_path):
            raise ValueError(
                f'缺少同名定位点云 {expected_pcd_name}，禁止导航；请重新建图并保存')
        if not os.path.isfile(metadata_path):
            raise ValueError(
                f'地图“{map_name}”缺少新版定位校验文件 '
                f'{os.path.basename(metadata_path)}，属于旧地图，禁止导航；'
                '请重新建图并保存')

        map_yaml = self._load_yaml_mapping(map_yaml_path, '二维地图配置')
        metadata = self._load_yaml_mapping(metadata_path, '定位元数据')

        if metadata.get('schema') != LOCALIZATION_MAP_SCHEMA:
            raise ValueError(
                f'地图“{map_name}”的定位元数据 schema 无效，禁止导航；'
                '请使用新版建图保存流程重新保存')
        if metadata.get('version') != LOCALIZATION_MAP_VERSION:
            raise ValueError(
                f'地图“{map_name}”的定位元数据版本不受支持'
                f'（需要 {LOCALIZATION_MAP_VERSION}），禁止导航')
        if metadata.get('map_name') != map_name:
            raise ValueError(
                f'定位元数据 map_name 与所选地图“{map_name}”不一致，禁止导航')
        if metadata.get('frame_id') != 'map':
            raise ValueError('定位 PCD 的 frame_id 必须为 map，禁止导航')
        if metadata.get('pcd_file') != expected_pcd_name:
            raise ValueError(
                f'定位元数据 pcd_file 必须为 {expected_pcd_name}，禁止导航')

        try:
            point_count = int(metadata.get('point_count'))
            recorded_pcd_size = int(metadata.get('pcd_size_bytes'))
        except (TypeError, ValueError) as error:
            raise ValueError('定位元数据中的点数或 PCD 大小无效，禁止导航') from error
        if point_count < MIN_LOCALIZATION_POINT_COUNT:
            raise ValueError(
                f'定位 PCD 只有 {point_count} 个持久体素点，少于可靠三维精修所需的 '
                f'{MIN_LOCALIZATION_POINT_COUNT} 点，禁止导航；请延长建图时间并重新保存')
        if point_count > MAX_LOCALIZATION_POINT_COUNT:
            raise ValueError(
                f'定位 PCD 声明 {point_count} 个点，超过鲁班猫低内存上限 '
                f'{MAX_LOCALIZATION_POINT_COUNT}，禁止导航；请重新降采样保存')
        actual_pcd_size = os.path.getsize(pcd_path)
        if recorded_pcd_size <= 0 or actual_pcd_size != recorded_pcd_size:
            raise ValueError(
                f'定位 PCD 大小与元数据不一致'
                f'（文件 {actual_pcd_size} 字节，记录 {recorded_pcd_size} 字节），'
                '文件可能不完整，禁止导航')
        if actual_pcd_size > MAX_LOCALIZATION_PCD_BYTES:
            raise ValueError(
                f'定位 PCD 大小为 {actual_pcd_size} 字节，超过 64 MiB '
                '低内存上限，禁止导航；请重新降采样保存')

        recorded_hash = str(metadata.get('pcd_sha256') or '').strip().lower()
        if not re.fullmatch(r'[0-9a-f]{64}', recorded_hash):
            raise ValueError('定位元数据中的 pcd_sha256 无效，禁止导航')
        actual_hash = self._sha256_file(pcd_path)
        if actual_hash != recorded_hash:
            raise ValueError('定位 PCD 校验和不匹配，文件可能损坏，禁止导航')

        voxel_size = self._finite_float(metadata.get('voxel_size'), 'voxel_size')
        if voxel_size <= 0.0:
            raise ValueError('定位元数据中的 voxel_size 必须大于 0，禁止导航')

        geometry = metadata.get('map_geometry')
        if not isinstance(geometry, dict):
            raise ValueError('定位元数据缺少 map_geometry，禁止导航')
        if geometry.get('image') != expected_image_name:
            raise ValueError(
                f'定位元数据 map_geometry.image 必须为 {expected_image_name}，禁止导航')
        map_yaml_image = os.path.basename(str(map_yaml.get('image') or '').strip())
        if map_yaml_image != expected_image_name:
            raise ValueError(
                f'二维地图 YAML 引用的图像不是同名文件 {expected_image_name}，禁止导航')

        try:
            recorded_width = int(geometry.get('width'))
            recorded_height = int(geometry.get('height'))
        except (TypeError, ValueError) as error:
            raise ValueError('定位元数据中的地图宽高无效，禁止导航') from error
        with Image.open(image_path) as image:
            actual_width, actual_height = image.size
        if (
            recorded_width <= 0
            or recorded_height <= 0
            or recorded_width != actual_width
            or recorded_height != actual_height
        ):
            raise ValueError(
                '二维地图尺寸与定位元数据不一致'
                f'（图像 {actual_width}x{actual_height}，'
                f'记录 {recorded_width}x{recorded_height}），禁止导航')

        yaml_resolution = self._finite_float(map_yaml.get('resolution'), 'map.resolution')
        metadata_resolution = self._finite_float(
            geometry.get('resolution'), 'map_geometry.resolution')
        if not self._same_number(yaml_resolution, metadata_resolution):
            raise ValueError('二维地图分辨率与定位元数据不一致，禁止导航')

        yaml_origin = self._finite_float_list(map_yaml.get('origin'), 3, 'map.origin')
        metadata_origin = self._finite_float_list(
            geometry.get('origin'), 3, 'map_geometry.origin')
        if any(
            not self._same_number(yaml_value, metadata_value)
            for yaml_value, metadata_value in zip(yaml_origin, metadata_origin)
        ):
            raise ValueError('二维地图原点与定位元数据不一致，禁止导航')

        for field_name in ('negate', 'occupied_thresh', 'free_thresh'):
            yaml_value = self._finite_float(map_yaml.get(field_name), f'map.{field_name}')
            metadata_value = self._finite_float(
                geometry.get(field_name), f'map_geometry.{field_name}')
            if not self._same_number(yaml_value, metadata_value):
                raise ValueError(
                    f'二维地图字段 {field_name} 与定位元数据不一致，禁止导航')

        self.get_logger().info(
            f'Localization assets verified for map {map_name}: '
            f'{point_count} points, {actual_pcd_size} bytes, voxel {voxel_size:.3f} m')
        return metadata


    @staticmethod
    def _map_file_signature(image_path):
        stat_result = os.stat(image_path)
        return (int(stat_result.st_mtime_ns), int(stat_result.st_size))

    def _load_map_into_cache(self, map_name, image_path):
        """Load a PGM and invalidate stale cache after same-name overwrite."""
        disk_signature = self._map_file_signature(image_path)
        cached_data = self.loaded_maps_cache.get(map_name)
        if (
            cached_data is not None
            and cached_data.get('disk_signature') == disk_signature
        ):
            # Keep old in-memory entries compatible if this service is updated
            # while it is running.
            cached_data.setdefault('saved_image', cached_data['image'].copy())
            cached_data.setdefault('undo_history', [])
            cached_data.setdefault('has_changes', False)
            return cached_data
        if cached_data is not None:
            self.get_logger().warning(
                f'Map file {map_name} changed on disk; discarding the stale '
                'editor cache and reloading the transactionally overwritten map.'
            )

        with Image.open(image_path) as img:
            # PGM maps are grayscale. Converting explicitly also keeps the
            # cached image two-dimensional when a compatible image is supplied.
            image = np.array(img.convert('L'))

        cached_data = {
            'image': image.copy(),
            'saved_image': image.copy(),
            'original_path': image_path,
            'disk_signature': disk_signature,
            'has_changes': False,
            'undo_history': [],
        }
        self.loaded_maps_cache[map_name] = cached_data
        self.get_logger().info(f'Loaded map {map_name} into editor cache')
        return cached_data

    @staticmethod
    def _unique_valid_indices(indices, total_elements):
        """Return valid, de-duplicated flat indices and the invalid count."""
        valid_indices = []
        seen_indices = set()
        invalid_count = 0

        for raw_index in indices:
            index = int(raw_index)
            if index < 0 or index >= total_elements:
                invalid_count += 1
                continue
            if index not in seen_indices:
                seen_indices.add(index)
                valid_indices.append(index)

        return valid_indices, invalid_count

    def _refresh_change_flag(self, cached_data):
        """Track whether the cached image differs from the last saved PGM."""
        saved_image = cached_data.get('saved_image')
        if saved_image is None:
            saved_image = cached_data['image'].copy()
            cached_data['saved_image'] = saved_image
        cached_data['has_changes'] = not np.array_equal(
            cached_data['image'], saved_image)

    def _record_undo_operation(self, cached_data, indices, previous_values):
        """Append one complete brush/rectangle edit to a map-local history."""
        if len(indices) == 0:
            return

        self._undo_sequence += 1
        cached_data.setdefault('undo_history', []).append({
            'sequence': self._undo_sequence,
            'indices': np.asarray(indices, dtype=np.intp).copy(),
            'previous_values': np.asarray(previous_values).copy(),
        })

    def _apply_batched_pixel_edit(self, cached_data, indices, pixel_value):
        """Apply one batched edit and save only the pixels needed for undo."""
        image = cached_data['image']
        valid_indices, invalid_count = self._unique_valid_indices(indices, image.size)
        if not valid_indices:
            return 0, 0, invalid_count

        flat_image = image.reshape(-1)
        valid_indices = np.asarray(valid_indices, dtype=np.intp)
        previous_values = flat_image[valid_indices].copy()
        changed_mask = previous_values != pixel_value
        changed_indices = valid_indices[changed_mask]

        if len(changed_indices) == 0:
            self._refresh_change_flag(cached_data)
            return len(valid_indices), 0, invalid_count

        changed_previous_values = previous_values[changed_mask]
        self._record_undo_operation(
            cached_data, changed_indices, changed_previous_values)
        flat_image[changed_indices] = pixel_value
        cached_data['image'] = flat_image.reshape(image.shape)
        self._refresh_change_flag(cached_data)
        return len(valid_indices), len(changed_indices), invalid_count

    def _undo_last_operation(self, cached_data):
        """Restore the pixels changed by the most recent operation."""
        history = cached_data.setdefault('undo_history', [])
        if not history:
            return False

        operation = history.pop()
        image = cached_data['image']
        flat_image = image.reshape(-1)
        indices = np.asarray(operation['indices'], dtype=np.intp)
        previous_values = np.asarray(operation['previous_values'])

        if len(indices) != len(previous_values):
            raise ValueError('Undo history entry has mismatched indices and values')
        if np.any(indices < 0) or np.any(indices >= flat_image.size):
            raise ValueError('Undo history entry contains out-of-range indices')

        flat_image[indices] = previous_values
        cached_data['image'] = flat_image.reshape(image.shape)
        self._refresh_change_flag(cached_data)
        return True

    def _find_undo_target(self, requested_map_name):
        """Find the requested map or the globally most recent edit when blank."""
        if requested_map_name:
            cached_data = self.loaded_maps_cache.get(requested_map_name)
            if cached_data and cached_data.get('undo_history'):
                return requested_map_name, cached_data
            return None, None

        latest = None
        for map_name, cached_data in self.loaded_maps_cache.items():
            history = cached_data.get('undo_history', [])
            if history:
                candidate = (history[-1]['sequence'], map_name, cached_data)
                if latest is None or candidate[0] > latest[0]:
                    latest = candidate

        if latest is None:
            return None, None
        return latest[1], latest[2]

    def handle_set_map_pixel_indices(self, request, response):
        """Apply one batched brush or rectangle edit to an in-memory PGM."""
        self.get_logger().info(
            f'Received request to set pixel values by indices for map: {request.map_name} '
            f'value: {request.pixel_value}, indices count: {len(request.indices)}'
        )

        try:
            map_name = self._safe_map_name(request.map_name)
            image_path = os.path.join(self.map_directory, f'{map_name}.pgm')
            if not os.path.exists(image_path):
                response.success = False
                response.message = f'Map image file not found: {image_path}'
                self.get_logger().warn(response.message)
                return response
            clamped_value = max(0, min(254, int(request.pixel_value)))

            # One SetMapPixelIndices request represents a complete frontend
            # drawing action. Store its original values as a single undo entry.
            with self.map_cache_lock:
                cached_data = self._load_map_into_cache(map_name, image_path)
                valid_count, changed_count, invalid_count = self._apply_batched_pixel_edit(
                    cached_data, request.indices, clamped_value)

            if invalid_count:
                self.get_logger().warn(
                    f'{invalid_count} indices are out of range and were ignored')
            if valid_count == 0:
                response.success = False
                response.message = 'No valid indices provided'
                return response

            response.success = True
            response.message = (
                f'Applied one edit to map {map_name}: '
                f'{changed_count} changed of {valid_count} valid pixels, '
                f'value {clamped_value}')
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Error handling set map pixel indices request: {str(e)}'
            self.get_logger().error(response.message)

        return response
    def handle_get_edited_map(self, request, response):
        """处理获取编辑后地图的请求"""
        self.get_logger().info(f'Received request to get edited map: {request.map_name}')
        try:
            map_name = self._safe_map_name(request.map_name)
            yaml_path = os.path.join(self.map_directory, f'{map_name}.yaml')
            image_path = os.path.join(self.map_directory, f'{map_name}.pgm')
            if not os.path.exists(image_path):
                response.success = False
                response.message = f'Map image file not found: {image_path}'
                self.get_logger().warn(response.message)
                return response

            # Return a consistent snapshot while another client may be drawing.
            with self.map_cache_lock:
                cached_data = self._load_map_into_cache(map_name, image_path)
                img_array = cached_data['image'].copy()
                has_changes = cached_data.get('has_changes', False)
            
            # 将numpy数组转换为字节流以便传输
            img_array_corrected = np.flipud(img_array) 
            flat_array = img_array_corrected.flatten()
            data = flat_array.tolist()
                        #转换成OccupancyGrid格式
            for i in range(len(data)):
                data[i] = self.pgm_to_occgrid(data[i])
            #获取resolution和origin
            # yaml_path = os.path.join(self.map_directory, f'{request.map_name}.yaml')
            with open(yaml_path, 'r', encoding='utf-8') as f:
                map_yaml = yaml.safe_load(f)
            resolution = map_yaml.get('resolution', 0.05)
            # 获取origin并确保每个元素都是浮点数
            origin_raw = map_yaml.get('origin', [0.0, 0.0, 0.0])
            origin = [round(float(val), 1) for val in origin_raw]  # 保留6位小数
            formatted_origin = [float(f"{val:.1f}") for val in origin]
            response.origin = formatted_origin
            response.resolution = resolution
            # response.origin = origin
            response.image_data = data  # 转换为Python列表            

            response.width = img_array.shape[1]  # 宽度
            response.height = img_array.shape[0]  # 高度
            response.has_changes = has_changes
            
            response.success = True
            response.message = f'Successfully retrieved map {map_name} from memory, size: {response.width}x{response.height}'
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f'Error retrieving edited map: {str(e)}'
            self.get_logger().error(response.message)
        
        return response

    def handle_edit_map(self, request, response):
        """Compatibility rectangle editor backed by the same undo history."""
        self.get_logger().info(
            f'Received edit map request - Action: {request.action}, '
            f'Map: {request.map_name}, Coordinates: ({request.x1},{request.y1}) to ({request.x2},{request.y2})'
        )

        try:
            map_name = self._safe_map_name(request.map_name)
            image_path = os.path.join(self.map_directory, f'{map_name}.pgm')
            if not os.path.exists(image_path):
                response.success = False
                response.message = f'Map image file not found: {image_path}'
                self.get_logger().warn(response.message)
                return response

            x_min = min(request.x1, request.x2)
            y_min = min(request.y1, request.y2)
            x_max = max(request.x1, request.x2)
            y_max = max(request.y1, request.y2)
            width = x_max - x_min
            height = y_max - y_min

            if width <= 0 or height <= 0:
                response.success = False
                response.message = f'Invalid coordinates: area must have positive dimensions, got width={width}, height={height}'
                self.get_logger().warn(response.message)
                return response

            if request.action == "add_obstacle":
                pixel_value = 0
            elif request.action in ("remove_obstacle", "clear_area"):
                pixel_value = 254
            elif request.action == "set_value":
                pixel_value = int((100 - request.obstacle_value) * 254 / 100)
                pixel_value = max(0, min(254, pixel_value))
            else:
                response.success = False
                response.message = f'Unknown action: {request.action}'
                self.get_logger().error(response.message)
                return response

            with self.map_cache_lock:
                cached_data = self._load_map_into_cache(map_name, image_path)
                img_height, img_width = cached_data['image'].shape
                if (x_min < 0 or y_min < 0 or
                        x_max > img_width or y_max > img_height):
                    response.success = False
                    response.message = (
                        f'Coordinates out of bounds: image size {img_width}x{img_height}, '
                        f'requested ({x_min}, {y_min}, {x_max}, {y_max})')
                    self.get_logger().warn(response.message)
                    return response

                rows = np.arange(y_min, y_max, dtype=np.intp)[:, None]
                columns = np.arange(x_min, x_max, dtype=np.intp)
                indices = (rows * img_width + columns).reshape(-1)
                _, changed_count, _ = self._apply_batched_pixel_edit(
                    cached_data, indices, pixel_value)

            response.success = True
            response.message = (
                f'Applied one rectangle edit to map {map_name}; '
                f'{changed_count} pixels changed. Use save_edited_maps to persist changes.')
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Error editing map: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def handle_save_edited_maps(self, request, response):
        """Overwrite edited PGM files and establish a new undo baseline."""
        try:
            requested_map_name = self._safe_map_name(
                request.map_name, allow_empty=True)
        except ValueError as error:
            response.success = False
            response.message = str(error)
            return response
        self.get_logger().info(
            f'Received request to save edited maps'
            f'{f" for {requested_map_name}" if requested_map_name else ""}')

        saved_count = 0
        error_count = 0

        with self.map_cache_lock:
            if requested_map_name:
                cached_data = self.loaded_maps_cache.get(requested_map_name)
                map_entries = ([] if cached_data is None else
                               [(requested_map_name, cached_data)])
            else:
                # Preserve the existing no-map-name behaviour: save every
                # dirty cached map.
                map_entries = list(self.loaded_maps_cache.items())

            for map_name, cached_data in map_entries:
                if not cached_data.get('has_changes', False):
                    continue
                try:
                    # This deliberately writes to the original PGM path rather
                    # than creating a second map. YAML metadata is untouched.
                    edited_img = Image.fromarray(
                        cached_data['image'], mode='L')
                    edited_img.save(cached_data['original_path'])

                    cached_data['saved_image'] = cached_data['image'].copy()
                    cached_data['disk_signature'] = self._map_file_signature(
                        cached_data['original_path'])
                    cached_data['has_changes'] = False
                    cached_data['undo_history'] = []
                    saved_count += 1
                    self.get_logger().info(f'Successfully saved edited map: {map_name}')
                except Exception as e:
                    error_count += 1
                    self.get_logger().error(f'Error saving map {map_name}: {str(e)}')

        if requested_map_name and not map_entries:
            response.success = False
            response.message = f'Map is not loaded for editing: {requested_map_name}'
            self.get_logger().warn(response.message)
            return response

        response.success = error_count == 0
        response.message = f'Saved {saved_count} maps with {error_count} errors'
        self.get_logger().info(response.message)

        return response

    def handle_undo_edits(self, request, response):
        """Undo exactly one complete batched edit without touching the PGM."""
        try:
            requested_map_name = self._safe_map_name(
                request.map_name, allow_empty=True)
        except ValueError as error:
            response.success = False
            response.message = str(error)
            return response
        self.get_logger().info(
            f'Received request to undo one edit'
            f'{f" for {requested_map_name}" if requested_map_name else ""}')

        try:
            with self.map_cache_lock:
                map_name, cached_data = self._find_undo_target(requested_map_name)
                if cached_data is None:
                    response.success = False
                    response.message = (
                        f'No undo history for map: {requested_map_name}'
                        if requested_map_name else
                        'No map edit is available to undo')
                    self.get_logger().warn(response.message)
                    return response

                self._undo_last_operation(cached_data)

            response.success = True
            response.message = f'Undid one edit on map: {map_name}'
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Error undoing map edit: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def edit_map_image(self, image_path, x_start, y_start, width, height, action, obstacle_value=100):
        """编辑地图图像文件 - 保留原始方法供其他用途"""
        try:
            # 打开图像
            img = Image.open(image_path)
            img_array = np.array(img)
            
            # 验证坐标范围
            img_height, img_width = img_array.shape
            if (x_start < 0 or y_start < 0 or 
                x_start + width > img_width or y_start + height > img_height):
                self.get_logger().error(
                    f'Coordinates out of bounds: image size {img_width}x{img_height}, '
                    f'requested ({x_start}, {y_start}, {width}x{height})'
                )
                return False
            
            # 根据操作类型编辑地图
            if action == "add_obstacle":
                img_array[y_start:y_start+height, x_start:x_start+width] = 0
            elif action == "remove_obstacle":
                img_array[y_start:y_start+height, x_start:x_start+width] = 254
            elif action == "clear_area":
                img_array[y_start:y_start+height, x_start:x_start+width] = 254
            elif action == "set_value":
                pixel_value = int((100 - obstacle_value) * 254 / 100)
                pixel_value = max(0, min(254, pixel_value))
                img_array[y_start:y_start+height, x_start:x_start+width] = pixel_value
            else:
                self.get_logger().error(f'Unknown action: {action}')
                return False
            
            # 保存修改后的图像
            edited_img = Image.fromarray(img_array, mode='L')  # 灰度模式
            edited_img.save(image_path)
            
            self.get_logger().info(f'Successfully edited map image: {image_path}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error editing map image {image_path}: {str(e)}')
            return False




    def handle_switch_map(self, request, response):
        """处理地图切换请求"""
        self.get_logger().info(f'Received switch map request for: {request.map_name}')

        if self.navigation_active:
            response.success = False
            response.message = (
                'Navigation is active. Stop the current navigation task '
                'before switching maps.')
            self.get_logger().warn(response.message)
            return response

        try:
            map_name = self._safe_map_name(request.map_name)
        except ValueError as error:
            response.success = False
            response.message = str(error)
            self.get_logger().warn(response.message)
            return response
        map_path = os.path.join(self.map_directory, f'{map_name}.yaml')

        # 检查地图文件是否存在
        if not os.path.exists(map_path):
            response.success = False
            response.message = f'Map file not found: {map_path}'
            self.get_logger().warn(response.message)
            return response

        try:
            self.validate_localization_map(map_name, map_path)
        except (OSError, ValueError) as error:
            response.success = False
            response.message = f'地图“{map_name}”未通过导航资格校验：{error}'
            self.get_logger().error(response.message)
            return response

        with self.map_load_lock:
            if self.pending_map_name is not None:
                response.success = False
                response.message = (
                    f'Another map is still loading: {self.pending_map_name}')
                self.get_logger().warn(response.message)
                return response
            
        # Do not acknowledge the service until Nav2 has returned the final
        # LoadMap result.  Otherwise a periodic current-map announcement can
        # be mistaken by the browser for completion of this particular load,
        # especially when reloading the same map name.
        load_success, load_message = self.load_map_blocking(
            map_path, map_name)
        response.success = load_success
        response.message = load_message
        return response


    def publish_current_map_info(self):
            """定期发布当前地图信息"""
            try:
                # While LoadMap is running, only its completion callback may
                # confirm success.  Publishing the old/default name here could
                # make the browser mistake a pending request for a successful one.
                with self.map_load_lock:
                    if self.pending_map_name is not None:
                        return
                    if not self.load_map_cli.service_is_ready():
                        # The persistent web backend outlives navigation mode.
                        # Once map_server disappears, the previous map is no
                        # longer an active live map and must not be advertised.
                        self.active_map_name = None
                current_map = self.get_current_map_name()
                msg = CurrentMap()
                if current_map:
                    msg.map_name = current_map
                    msg.success = True
                    msg.message = f"Current map: {current_map}"
                else:
                    msg.map_name = ""
                    msg.success = False
                    msg.message = "Could not determine current map"
                
                self.current_map_publisher.publish(msg)
                # self.get_logger().info(f'Published current map info: {current_map}')
            except Exception as e:
                self.get_logger().error(f'Error publishing current map info: {str(e)}')

    def publish_map_load_result(self, map_name, success, message):
        """Publish only confirmed active-map state.

        Load failures are returned by the blocking /switch_map service.  They
        must not be latched on /current_map_info, whose sole contract is the
        last successfully active map.
        """
        if not success:
            return
        msg = CurrentMap()
        msg.map_name = map_name
        msg.success = True
        msg.message = message
        self.current_map_publisher.publish(msg)

    def handle_delete_map(self, request, response):
        """处理删除地图请求"""
        try:
            map_name = self._safe_map_name(request.map_name)
        except ValueError as error:
            response.success = False
            response.message = str(error)
            self.get_logger().warn(response.message)
            return response
        self.get_logger().info(f'Received request to delete map: {map_name}')
        
        try:
            # 获取当前地图名称
            current_map = self.get_current_map_name()
            
            # 检查是否尝试删除当前使用的地图
            if current_map and current_map == map_name:
                response.success = False
                response.message = f'Cannot delete current map: {map_name}'
                self.get_logger().warn(response.message)
                return response
            
            # 删除2D地图文件
            map_files_deleted = self.delete_map_files(map_name)
            
            # 删除3D地图文件（如果存在）
            pcd_files_deleted = self.delete_pcd_files(map_name)
            
            if map_files_deleted or pcd_files_deleted:
                deleted_routes = self.route_database.delete_routes_by_map(
                    map_name)
                response.success = True
                response.message = (
                    f'Deleted map files for: {map_name}; '
                    f'deleted {deleted_routes} associated routes')
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = f'No map files found for: {map_name}'
                self.get_logger().warn(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Error deleting map: {str(e)}"
            self.get_logger().error(response.message)
        
        return response

    def get_current_map_name(self):
        """Return only a map confirmed by a successful live LoadMap call."""
        with self.map_load_lock:
            return self.active_map_name


    def delete_map_files(self, map_name):
        """删除指定地图的2D文件（.yaml, .pgm/.png等）"""
        deleted_count = 0
        
        # 删除yaml文件
        yaml_path = os.path.join(self.map_directory, f'{map_name}.yaml')
        if os.path.exists(yaml_path):
            os.remove(yaml_path)
            self.get_logger().info(f'Deleted map yaml file: {yaml_path}')
            deleted_count += 1
        
        # 删除图像文件（通常与yaml同名）
        image_extensions = ['.pgm', '.png', '.jpg', '.jpeg']
        for ext in image_extensions:
            img_path = os.path.join(self.map_directory, f'{map_name}{ext}')
            if os.path.exists(img_path):
                os.remove(img_path)
                self.get_logger().info(f'Deleted map image file: {img_path}')
                deleted_count += 1
        
        return deleted_count > 0

    def delete_pcd_files(self, map_name):
        """删除指定地图的3D文件及其定位校验元数据。"""
        deleted = False
        pcd_path = os.path.join(self.pcdmap_directory, f'{map_name}.pcd')
        if os.path.exists(pcd_path):
            os.remove(pcd_path)
            self.get_logger().info(f'Deleted PCD file: {pcd_path}')
            deleted = True
        metadata_path = os.path.join(
            self.pcdmap_directory,
            f'{map_name}{LOCALIZATION_METADATA_SUFFIX}')
        if os.path.exists(metadata_path):
            os.remove(metadata_path)
            self.get_logger().info(
                f'Deleted localization metadata file: {metadata_path}')
            deleted = True
        return deleted



    def load_map_blocking(self, map_path, map_name):
        """Load one map and return only after map_server reports completion."""
        self.get_logger().info(f'Starting confirmed map loading for: {map_name}')

        with self.map_load_lock:
            if self.pending_map_name is not None:
                message = f'Another map is still loading: {self.pending_map_name}'
                self.get_logger().warn(message)
                return False, message
            self.pending_map_name = map_name

        try:
            active, lifecycle_state = self.wait_for_map_server_active(
                timeout_sec=self.map_server_active_timeout_sec)
            if not active:
                message = (
                    'Nav2 map server did not become active within '
                    f'{self.map_server_active_timeout_sec:.0f} seconds '
                    f'(last lifecycle state: {lifecycle_state}). '
                    'Wait for navigation mode to finish starting, then retry.')
                self.get_logger().error(message)
                return False, message
            if not self.load_map_cli.wait_for_service(timeout_sec=2.0):
                message = (
                    'Nav2 map server became active, but its LoadMap service '
                    'is not available.')
                self.get_logger().error(message)
                return False, message

            request = LoadMap.Request()
            request.map_url = map_path
            future = self.load_map_cli.call_async(request)
            completed = threading.Event()
            future.add_done_callback(lambda _future: completed.set())
            if not completed.wait(timeout=30.0):
                try:
                    future.cancel()
                except Exception:
                    pass
                message = (
                    f'Loading map {map_name} timed out after 30 seconds; '
                    'map_server did not return a final result.')
                self.get_logger().error(message)
                self.publish_map_load_result(map_name, False, message)
                return False, message

            result = future.result()
            success = result.result == LoadMap.Response.RESULT_SUCCESS
            if not success:
                message = (
                    f'Failed to load map: {map_name} '
                    f'(map_server result {result.result})')
                self.get_logger().error(message)
                self.publish_map_load_result(map_name, False, message)
                return False, message

            with self.map_load_lock:
                self.active_map_name = map_name
            config_updated = self.update_map_path_in_yaml(map_name)
            message = f'Successfully loaded and confirmed map: {map_name}'
            self.get_logger().info(message)
            if not config_updated:
                self.get_logger().warn(
                    'The map is active, but the startup default path could '
                    'not be persisted. The live map state remains correct.')
            self.publish_map_load_result(map_name, True, message)
            return True, message
        except Exception as error:
            message = f'Failed to call load_map service for {map_name}: {error}'
            self.get_logger().error(message)
            self.publish_map_load_result(map_name, False, message)
            return False, message
        finally:
            with self.map_load_lock:
                if self.pending_map_name == map_name:
                    self.pending_map_name = None

    def wait_for_map_server_active(self, timeout_sec):
        """Wait until map_server can accept LoadMap, not merely advertise it."""
        deadline = time.monotonic() + timeout_sec
        last_state = 'state service unavailable'

        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break

            if not self.map_server_state_cli.wait_for_service(
                    timeout_sec=min(0.5, remaining)):
                continue

            future = self.map_server_state_cli.call_async(GetState.Request())
            completed = threading.Event()
            future.add_done_callback(lambda _future: completed.set())
            if not completed.wait(timeout=min(1.0, remaining)):
                try:
                    future.cancel()
                except Exception:
                    pass
                last_state = 'state query timed out'
                continue

            try:
                response = future.result()
            except Exception as error:
                last_state = f'state query failed: {error}'
                continue

            state = response.current_state
            last_state = state.label or str(state.id)
            if state.id == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info(
                    'map_server is ACTIVE and ready to accept LoadMap')
                return True, last_state

            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(0.1, remaining))

        return False, last_state

    def load_map_async(self, map_path, map_name):
        """异步加载指定地图"""
        self.get_logger().info(f'Starting async map loading for: {map_name}')

        with self.map_load_lock:
            if self.pending_map_name is not None:
                self.get_logger().warn(
                    f'Map load already in progress: {self.pending_map_name}')
                return False
            self.pending_map_name = map_name
        
        # car_vel.launch.py deliberately delays Nav2 while LiDAR/LIO starts.
        # A lifecycle service is advertised before map_server becomes ACTIVE;
        # wait for the state that actually permits LoadMap requests.
        active, lifecycle_state = self.wait_for_map_server_active(
            timeout_sec=self.map_server_active_timeout_sec)
        if not active:
            self.get_logger().error(
                'map_server did not become active after '
                f'{self.map_server_active_timeout_sec:.0f} seconds '
                f'(last lifecycle state: {lifecycle_state})')
            with self.map_load_lock:
                if self.pending_map_name == map_name:
                    self.pending_map_name = None
            return False
        if not self.load_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                'map_server is active but LoadMap service is unavailable')
            with self.map_load_lock:
                if self.pending_map_name == map_name:
                    self.pending_map_name = None
            return False
            
        req = LoadMap.Request()
        req.map_url = map_path
        
        # 异步调用
        future = self.load_map_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.load_map_callback(f, map_name))
        return True

    def load_map_callback(self, future, map_name):
        """地图加载完成回调"""
        try:
            result = future.result()
            success = result.result == LoadMap.Response.RESULT_SUCCESS
            if success:
                with self.map_load_lock:
                    self.active_map_name = map_name
                # Keep the selected map as the default for the next startup.
                config_updated = self.update_map_path_in_yaml(map_name)
                self.get_logger().info(f'Successfully loaded map: {map_name}')
                if not config_updated:
                    self.get_logger().warn(
                        'The map is active, but the default map path could not be persisted.')
                # LoadMap changes the live map_server in place.  Restarting the
                # complete navigation launch here would discard AMCL state and
                # any goal sent by the web page.
                self.publish_map_load_result(
                    map_name, True, f'Successfully loaded map: {map_name}')
            else:
                message = f'Failed to load map: {map_name} (result {result.result})'
                self.get_logger().error(message)
                self.publish_map_load_result(map_name, False, message)
        except Exception as e:
            message = f'Failed to call load_map service: {e}'
            self.get_logger().error(message)
            self.publish_map_load_result(map_name, False, message)
        finally:
            with self.map_load_lock:
                if self.pending_map_name == map_name:
                    self.pending_map_name = None

    def restart_navigation_launch(self):
        """调用launch管理器重启导航"""
        if not self.restart_launch_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('RestartLaunch service not available')
            return False
            
        req = ControlLaunch.Request()
        # 可以根据需要设置请求参数
        req.launch_type = "car_vel"  # 或其他标识符
        req.package_name = "car_vel"
        req.action = "restart"
        
        # 异步调用重启服务
        future = self.restart_launch_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.restart_launch_callback(f))
        return True
    


    def handle_get_map_list(self, request, response):
        """处理获取地图列表请求"""
        self.get_logger().info('Received request to get map list')
        
        try:
            map_names = self.get_available_maps()
            response.map_names = map_names
            response.success = True
            response.message = f"Found {len(map_names)} maps"
            self.get_logger().info(f'Returned map list: {map_names}')
        except Exception as e:
            response.success = False
            response.message = f"Error getting map list: {str(e)}"
            self.get_logger().error(response.message)
        
        return response

    def get_available_maps(self):
        """获取可用地图列表"""
        if not os.path.exists(self.map_directory):
            self.get_logger().error(f'Map directory does not exist: {self.map_directory}')
            return []
        
        map_names = []
        for filename in os.listdir(self.map_directory):
            if filename.endswith('.yaml') or filename.endswith('.YAML'):
                # 去掉扩展名，只保留文件名
                map_name = os.path.splitext(filename)[0]
                map_names.append(map_name)
        
        # 排序以保持一致的顺序
        map_names.sort()
        return map_names

    def restart_launch_callback(self, future):
        """重启服务完成回调"""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Navigation launch restarted successfully')
            else:
                self.get_logger().error(f'Failed to restart navigation: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Failed to call restart launch service: {e}')
    def update_map_path_in_yaml(self, map_name):
        """更新nav2_params.yaml中的地图路径"""
        fishbot_navigation2_dir = get_package_share_directory('robot_navigation2')
        params_file = os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml')
        
        
        try:
            # 构建新的地图路径
            new_map_path = os.path.join(self.map_directory, f'{map_name}.yaml')
            
            # 读取原始文件内容
            with open(params_file, 'r', encoding='utf-8') as file:
                original_content = file.read()
            
            # 保存旧路径用于日志
            import re
            old_path_match = re.search(r'yaml_filename:\s*["\']?([^\s"\'\n]+)["\']?', original_content)
            old_path = old_path_match.group(1) if old_path_match else "Unknown"
            
            # 替换yaml_filename的值
            updated_content = re.sub(
                r'(yaml_filename:\s*["\']?)[^\s"\'\n]+(["\']?)',
                f'\\g<1>{new_map_path}\\g<2>',
                original_content
            )
            
            # 写回文件
            with open(params_file, 'w', encoding='utf-8') as file:
                file.write(updated_content)
            
            self.get_logger().info(f'成功更新配置文件中的地图路径为: {new_map_path} (原路径: {old_path})')
            return True

            
        except Exception as e:
            self.get_logger().error(f'更新YAML配置文件失败: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = MapSwitcherService()
    
    # 使用多线程执行器支持并发处理
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

