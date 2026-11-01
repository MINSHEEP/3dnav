#!/usr/bin/env python3

"""Atomically save matched 2D, localization-PCD and terrain artifacts."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
import threading
import time
from datetime import datetime, timezone
from typing import Any

import numpy as np
from PIL import Image
import yaml

import rclpy
from lzp_interface.srv import MapSave, RebuildTerrain
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Trigger

from .occupancy_grid_io import (
    DEFAULT_MAXIMUM_CELLS,
    OccupancyGridSnapshotError,
    write_occupancy_grid_snapshot,
)
from .pcd_io import load_pcd_xyz
from .saved_map_lock import saved_map_lock
from .terrain_grid import (
    GroundSupportGrid,
    build_terrain_grid,
    summarize_terrain_quality,
)
from .terrain_grid_io import (
    create_terrain_grid_metadata,
    load_bound_terrain_grid,
    write_terrain_grid,
)


METADATA_SCHEMA = 'ros2_mid360_localization_map'
METADATA_VERSION = 1
MAP_NAME_PATTERN = re.compile(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}')
MAP_FREE_THRESHOLD = 0.196
MAP_OCCUPIED_THRESHOLD = 0.65
MAP_THRESHOLD_TOLERANCE = 1.0e-9


class MapSaveError(RuntimeError):
    """Expected, user-facing map-save failure."""


class MapSaveService(Node):
    """Save 2-D, localization and terrain artifacts as one transaction."""

    def __init__(self) -> None:
        super().__init__('map_save_service')
        self.callback_group = ReentrantCallbackGroup()
        self._save_lock = threading.Lock()
        self._projected_map_lock = threading.Lock()
        self._latest_projected_map: dict[str, Any] | None = None

        default_workspace = Path(
            '/home/a/3dnav/src/ros2_humble_main/src/navigation/'
            'robot_navigation2'
        )
        self.declare_parameter(
            'map_directory', str(default_workspace / 'maps')
        )
        self.declare_parameter(
            'pcd_map_directory', str(default_workspace / 'pcdmaps')
        )
        self.declare_parameter('mapping_frame', 'odom')
        self.declare_parameter('pcd_service_wait_seconds', 5.0)
        self.declare_parameter('pcd_save_timeout_seconds', 45.0)
        self.declare_parameter(
            'projected_map_maximum_cells', DEFAULT_MAXIMUM_CELLS)
        self.declare_parameter('terrain_grid_resolution', 0.25)
        self.declare_parameter('terrain_grid_maximum_cells', 400_000)
        self.declare_parameter('terrain_caution_slope_deg', 12.0)
        self.declare_parameter('terrain_special_slope_deg', 18.0)
        self.declare_parameter('terrain_blocked_slope_deg', 28.0)
        self.declare_parameter('terrain_caution_step_m', 0.08)
        self.declare_parameter('terrain_special_step_m', 0.12)
        self.declare_parameter('terrain_blocked_step_m', 0.18)
        self.declare_parameter('terrain_roughness_caution_m', 0.05)
        self.declare_parameter('terrain_minimum_confidence', 0.35)
        self.declare_parameter('terrain_overhead_search_radius_m', 1.0)
        self.declare_parameter('terrain_overhead_min_clearance_m', 0.75)

        self.map_directory = Path(
            self.get_parameter('map_directory').value
        ).expanduser().resolve()
        self.pcd_map_directory = Path(
            self.get_parameter('pcd_map_directory').value
        ).expanduser().resolve()
        self.mapping_frame = str(
            self.get_parameter('mapping_frame').value
        ).lstrip('/')
        self.pcd_service_wait_seconds = float(
            self.get_parameter('pcd_service_wait_seconds').value
        )
        self.pcd_save_timeout_seconds = float(
            self.get_parameter('pcd_save_timeout_seconds').value
        )
        self.projected_map_maximum_cells = int(
            self.get_parameter('projected_map_maximum_cells').value
        )
        self.terrain_grid_parameters = {
            'resolution': float(self.get_parameter(
                'terrain_grid_resolution').value),
            'maximum_cells': int(self.get_parameter(
                'terrain_grid_maximum_cells').value),
            'caution_slope_deg': float(self.get_parameter(
                'terrain_caution_slope_deg').value),
            'special_slope_deg': float(self.get_parameter(
                'terrain_special_slope_deg').value),
            'blocked_slope_deg': float(self.get_parameter(
                'terrain_blocked_slope_deg').value),
            'caution_step_m': float(self.get_parameter(
                'terrain_caution_step_m').value),
            'special_step_m': float(self.get_parameter(
                'terrain_special_step_m').value),
            'blocked_step_m': float(self.get_parameter(
                'terrain_blocked_step_m').value),
            'roughness_caution_m': float(self.get_parameter(
                'terrain_roughness_caution_m').value),
            'minimum_confidence': float(self.get_parameter(
                'terrain_minimum_confidence').value),
            'overhead_search_radius_m': float(self.get_parameter(
                'terrain_overhead_search_radius_m').value),
            'overhead_min_clearance_m': float(self.get_parameter(
                'terrain_overhead_min_clearance_m').value),
        }

        self.map_directory.mkdir(parents=True, exist_ok=True)
        self.pcd_map_directory.mkdir(parents=True, exist_ok=True)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.projected_map_subscription = self.create_subscription(
            OccupancyGrid,
            '/projected_map',
            self._projected_map_callback,
            map_qos,
            callback_group=self.callback_group,
        )
        self.localization_pcd_client = self.create_client(
            Trigger,
            '/lio/save_localization_pcd',
            callback_group=self.callback_group,
        )
        self.srv = self.create_service(
            MapSave,
            '/map_save',
            self.map_save_callback,
            callback_group=self.callback_group,
        )
        self.rebuild_terrain_service = self.create_service(
            RebuildTerrain,
            '/rebuild_terrain',
            self.rebuild_terrain_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            'Atomic map save service is ready on /map_save'
        )
        self.get_logger().info(
            'Terrain rebuild service is ready on /rebuild_terrain'
        )
        self.get_logger().info(
            f'2D map directory: {self.map_directory}'
        )
        self.get_logger().info(
            f'Localization PCD directory: {self.pcd_map_directory}'
        )

    def _projected_map_callback(self, message: OccupancyGrid) -> None:
        info = message.info
        snapshot = {
            'frame_id': message.header.frame_id.lstrip('/'),
            'stamp_ns': (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            ),
            'width': int(info.width),
            'height': int(info.height),
            'resolution': float(info.resolution),
            'origin': [
                float(info.origin.position.x),
                float(info.origin.position.y),
                self._quaternion_yaw(
                    float(info.origin.orientation.z),
                    float(info.origin.orientation.w),
                ),
            ],
            # Deep-copy the full grid here.  A save must serialize exactly this
            # message, rather than launch a second subscriber that may receive
            # a newer grid or time out during DDS discovery.
            'data': np.asarray(message.data, dtype=np.int8).copy(),
            'received_monotonic': time.monotonic(),
        }
        with self._projected_map_lock:
            self._latest_projected_map = snapshot

    @staticmethod
    def _quaternion_yaw(z: float, w: float) -> float:
        # OccupancyGrid origins are planar in this project (x=y=0 quaternion).
        return 2.0 * __import__('math').atan2(z, w)

    def map_save_callback(self, request: Any, response: Any) -> Any:
        """Handle one all-or-nothing 2D + localization-PCD save."""
        if not self._save_lock.acquire(blocking=False):
            response.success = False
            response.message = (
                'Another map save is already in progress. Please wait.'
            )
            self.get_logger().warning(response.message)
            return response

        try:
            with saved_map_lock(self.pcd_map_directory):
                return self._save_map(request, response)
        except (OSError, ValueError) as error:
            response.success, response.message = False, str(error)
            return response
        finally:
            self._save_lock.release()

    def rebuild_terrain_callback(self, request: Any, response: Any) -> Any:
        """Rebuild only the derived terrain artifact for a saved map."""

        if not self._save_lock.acquire(blocking=False):
            response.success = False
            response.message = 'A map save or terrain rebuild is in progress.'
            response.quality_json = '{}'
            return response
        try:
            with saved_map_lock(self.pcd_map_directory):
                return self._rebuild_saved_terrain(request, response)
        except (OSError, ValueError) as error:
            response.success, response.message, response.quality_json = False, str(error), '{}'
            return response
        finally:
            self._save_lock.release()

    def _rebuild_saved_terrain(self, request: Any, response: Any) -> Any:
        map_name = str(request.map_name).strip()
        response.quality_json = '{}'
        if not MAP_NAME_PATTERN.fullmatch(map_name):
            response.success = False
            response.message = '请选择合法的已保存地图名称。'
            return response
        paths = self._target_paths(map_name)
        if (self.pcd_map_directory / f'.{map_name}.cloud-edit-pending').exists():
            response.success, response.message = False, '三维编辑事务未完成，请先恢复备份'
            return response
        required = ('pgm', 'yaml', 'pcd', 'metadata')
        missing = [paths[key].name for key in required
                   if not paths[key].is_file()]
        if missing:
            response.success = False
            response.message = '地形重建缺少文件：' + ', '.join(missing)
            return response

        transaction_directory = None
        try:
            with paths['metadata'].open('r', encoding='utf-8') as stream:
                metadata = yaml.safe_load(stream)
            if not isinstance(metadata, dict):
                raise MapSaveError('定位元数据不是有效对象。')
            if (
                metadata.get('schema') != METADATA_SCHEMA
                or metadata.get('version') != METADATA_VERSION
                or metadata.get('map_name') != map_name
            ):
                raise MapSaveError('定位元数据版本或地图名称不匹配。')
            pcd_sha256 = self._sha256(paths['pcd'])
            if metadata.get('pcd_sha256') != pcd_sha256:
                raise MapSaveError('PCD校验和与定位元数据不一致。')
            pcd_header = self._validate_pcd(paths['pcd'])
            pcd_size = paths['pcd'].stat().st_size
            map_geometry = self._validate_and_normalize_2d_map(
                self.map_directory, map_name, normalize=False)
            ground_support = self._load_ground_support(
                self.map_directory, map_name, map_geometry)
            terrain_grid = build_terrain_grid(
                load_pcd_xyz(paths['pcd']),
                ground_support=ground_support,
                **self.terrain_grid_parameters,
            )
            quality = summarize_terrain_quality(
                terrain_grid, ground_support)
            generation = {
                'algorithm': 'clustered_measured_ground_v3',
                'source': 'saved_localization_pcd_rebuild',
                'source_pcd_sha256': pcd_sha256,
                'parameters': dict(self.terrain_grid_parameters),
                'quality': quality,
            }
            transaction_directory = Path(tempfile.mkdtemp(
                prefix=f'.{map_name}.terrain-rebuild-',
                dir=str(self.pcd_map_directory),
            ))
            transaction_terrain = (
                transaction_directory / f'{map_name}.terrain.npz')
            summary = write_terrain_grid(
                transaction_terrain,
                map_name=map_name,
                frame_id='map',
                grid=terrain_grid,
                generation=generation,
            )
            terrain_metadata = create_terrain_grid_metadata(
                transaction_terrain,
                summary=summary,
                source_pcd_sha256=pcd_sha256,
            )
            metadata['terrain_grid'] = terrain_metadata
            files = metadata.setdefault('files', {})
            files['terrain_grid'] = {
                'filename': terrain_metadata['filename'],
                'size_bytes': terrain_metadata['size_bytes'],
                'sha256': terrain_metadata['sha256'],
                'source_pcd_sha256': pcd_sha256,
            }
            metadata['terrain_rebuilt_at_utc'] = (
                datetime.now(timezone.utc).isoformat())
            transaction_metadata = (
                transaction_directory / f'{map_name}.localization.yaml')
            self._write_yaml(transaction_metadata, metadata)
            self._validate_metadata(
                transaction_metadata,
                map_name,
                map_geometry,
                pcd_header,
                pcd_size,
                pcd_sha256,
                terrain_metadata,
            )
            self._commit_artifacts_transaction(
                {
                    'terrain': transaction_terrain,
                    'metadata': transaction_metadata,
                },
                {
                    'terrain': paths['terrain'],
                    'metadata': paths['metadata'],
                },
                install_order=('terrain', 'metadata'),
            )
            response.success = True
            response.quality_json = json.dumps(
                quality, ensure_ascii=False, separators=(',', ':'))
            response.message = (
                f'地图“{map_name}”地形已按地表算法重建。'
                + self._quality_message(quality)
            )
            self.get_logger().info(response.message)
        except Exception as error:
            response.success = False
            response.message = f'地形重建失败，原文件已保留：{error}'
            self.get_logger().error(response.message)
        finally:
            if transaction_directory is not None:
                shutil.rmtree(transaction_directory, ignore_errors=True)
        return response

    def _save_map(self, request: Any, response: Any) -> Any:
        map_name = request.map_name.strip()
        if not map_name:
            response.success = False
            response.message = 'Map name is required.'
            self.get_logger().error(response.message)
            return response
        if not MAP_NAME_PATTERN.fullmatch(map_name):
            response.success = False
            response.message = (
                'Map name may contain only letters, digits, underscores, and '
                'hyphens (maximum 64 characters).'
            )
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(
            f'Starting atomic map save for "{map_name}".'
        )
        target_paths = self._target_paths(map_name)
        if (self.pcd_map_directory / f'.{map_name}.cloud-edit-pending').exists():
            response.success, response.message = False, '三维编辑事务未完成，请先恢复备份'
            return response

        transaction_directory: Path | None = None
        lio_staging_pcd: Path | None = None
        try:
            projected_map = self._require_projected_map()
            transaction_directory = Path(
                tempfile.mkdtemp(
                    prefix=f'.{map_name}.save-',
                    dir=str(self.map_directory),
                )
            )
            write_occupancy_grid_snapshot(
                projected_map,
                transaction_directory,
                map_name,
                free_threshold=MAP_FREE_THRESHOLD,
                occupied_threshold=MAP_OCCUPIED_THRESHOLD,
                maximum_cells=self.projected_map_maximum_cells,
            )
            map_geometry = self._validate_and_normalize_2d_map(
                transaction_directory, map_name
            )
            self._validate_projected_map_geometry(
                projected_map, map_geometry
            )

            lio_snapshot = self._request_localization_pcd()
            lio_staging_pcd = Path(lio_snapshot['path']).resolve()
            capture_frame = str(
                lio_snapshot.get('capture_frame', '')
            ).lstrip('/')
            if capture_frame != projected_map['frame_id']:
                raise MapSaveError(
                    '2D/3D frame mismatch: /projected_map is in '
                    f'"{projected_map["frame_id"]}", but the localization PCD '
                    f'is in "{capture_frame}". Nothing was saved.'
                )

            transaction_pcd = transaction_directory / f'{map_name}.pcd'
            shutil.copy2(lio_staging_pcd, transaction_pcd)
            pcd_header = self._validate_pcd(transaction_pcd)
            response_point_count = int(lio_snapshot['point_count'])
            if pcd_header['point_count'] != response_point_count:
                raise MapSaveError(
                    'Localization PCD validation failed: service point count '
                    f'was {response_point_count}, file header reports '
                    f'{pcd_header["point_count"]}.'
                )

            pcd_size = transaction_pcd.stat().st_size
            pcd_sha256 = self._sha256(transaction_pcd)
            ground_support = self._load_ground_support(
                transaction_directory, map_name, map_geometry)
            terrain_generation = {
                'algorithm': 'clustered_measured_ground_v3',
                'source': 'complete_localization_pcd',
                'source_pcd_sha256': pcd_sha256,
                'parameters': dict(self.terrain_grid_parameters),
            }
            terrain_grid = build_terrain_grid(
                load_pcd_xyz(transaction_pcd),
                ground_support=ground_support,
                **self.terrain_grid_parameters,
            )
            terrain_generation['quality'] = summarize_terrain_quality(
                terrain_grid, ground_support)
            transaction_terrain = (
                transaction_directory / f'{map_name}.terrain.npz'
            )
            terrain_summary = write_terrain_grid(
                transaction_terrain,
                map_name=map_name,
                frame_id='map',
                grid=terrain_grid,
                generation=terrain_generation,
            )
            terrain_metadata = create_terrain_grid_metadata(
                transaction_terrain,
                summary=terrain_summary,
                source_pcd_sha256=pcd_sha256,
            )
            metadata = self._build_metadata(
                map_name=map_name,
                map_geometry=map_geometry,
                projected_map=projected_map,
                lio_snapshot=lio_snapshot,
                pcd_header=pcd_header,
                pcd_size=pcd_size,
                pcd_sha256=pcd_sha256,
                terrain_metadata=terrain_metadata,
                transaction_directory=transaction_directory,
            )
            transaction_metadata = (
                transaction_directory / f'{map_name}.localization.yaml'
            )
            self._write_yaml(transaction_metadata, metadata)
            self._validate_metadata(
                transaction_metadata,
                map_name,
                map_geometry,
                pcd_header,
                pcd_size,
                pcd_sha256,
                terrain_metadata,
            )

            source_paths = {
                'pgm': transaction_directory / f'{map_name}.pgm',
                'yaml': transaction_directory / f'{map_name}.yaml',
                'pcd': transaction_pcd,
                'terrain': transaction_terrain,
                'metadata': transaction_metadata,
            }
            replaced_existing = any(
                path.exists() for path in target_paths.values()
            )
            self._commit_artifacts_transaction(
                source_paths, target_paths
            )

            response.success = True
            response.message = (
                f'Map "{map_name}" '
                f'{"overwritten" if replaced_existing else "saved"} '
                'successfully: validated PGM/YAML + '
                f'{pcd_header["point_count"]} filtered localization points + '
                f'{terrain_metadata["known_cells"]} terrain cells + '
                'versioned metadata. '
                + self._quality_message(
                    terrain_generation['quality'])
            )
            self.get_logger().info(response.message)
            return response
        except (
            MapSaveError,
            OccupancyGridSnapshotError,
            OSError,
            ValueError,
            KeyError,
            json.JSONDecodeError,
        ) as error:
            response.success = False
            response.message = f'Map save failed: {error}'
            self.get_logger().error(response.message)
        except Exception as error:  # Guard the service process from bad I/O.
            response.success = False
            response.message = (
                'Unexpected map-save failure; no partial new map was retained.'
            )
            self.get_logger().error(
                f'{response.message} Details: {error}'
            )
        finally:
            self._remove_lio_staging_file(lio_staging_pcd)
            if transaction_directory is not None:
                shutil.rmtree(transaction_directory, ignore_errors=True)

        return response

    def _require_projected_map(self) -> dict[str, Any]:
        with self._projected_map_lock:
            snapshot = (
                dict(self._latest_projected_map)
                if self._latest_projected_map is not None
                else None
            )
        if snapshot is None:
            raise MapSaveError(
                'No /projected_map has been received. Start mapping and wait '
                'for a visible 2D grid before saving.'
            )
        if snapshot['frame_id'] != self.mapping_frame:
            raise MapSaveError(
                f'/projected_map frame is "{snapshot["frame_id"]}", expected '
                f'"{self.mapping_frame}". Saving would misalign the PGM and PCD.'
            )
        if (
            snapshot['width'] <= 0
            or snapshot['height'] <= 0
            or snapshot['resolution'] <= 0.0
        ):
            raise MapSaveError('/projected_map has invalid geometry.')
        return snapshot

    def _validate_and_normalize_2d_map(
        self, transaction_directory: Path, map_name: str, *, normalize=True
    ) -> dict[str, Any]:
        yaml_path = transaction_directory / f'{map_name}.yaml'
        pgm_path = transaction_directory / f'{map_name}.pgm'
        if not yaml_path.is_file() or not pgm_path.is_file():
            raise MapSaveError(
                '2D map save did not produce both .yaml and .pgm files.'
            )

        with yaml_path.open('r', encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict):
            raise MapSaveError('2D map YAML is not a mapping.')

        try:
            resolution = float(document['resolution'])
            origin = [float(value) for value in document['origin']]
            negate = int(document['negate'])
            occupied_thresh = float(document['occupied_thresh'])
            free_thresh = float(document['free_thresh'])
        except (KeyError, TypeError, ValueError) as error:
            raise MapSaveError(
                f'2D map YAML geometry is invalid: {error}'
            ) from error
        if resolution <= 0.0 or len(origin) != 3:
            raise MapSaveError(
                '2D map YAML must have positive resolution and a 3-value origin.'
            )
        if not (0.0 <= free_thresh < occupied_thresh <= 1.0):
            raise MapSaveError('2D map YAML occupancy thresholds are invalid.')
        if (
            not math.isclose(
                free_thresh,
                MAP_FREE_THRESHOLD,
                rel_tol=0.0,
                abs_tol=MAP_THRESHOLD_TOLERANCE,
            )
            or not math.isclose(
                occupied_thresh,
                MAP_OCCUPIED_THRESHOLD,
                rel_tol=0.0,
                abs_tol=MAP_THRESHOLD_TOLERANCE,
            )
        ):
            raise MapSaveError(
                '2D map saver returned unexpected occupancy thresholds '
                f'(free={free_thresh}, occupied={occupied_thresh}); expected '
                f'free={MAP_FREE_THRESHOLD} and '
                f'occupied={MAP_OCCUPIED_THRESHOLD}.'
            )
        if negate not in (0, 1):
            raise MapSaveError('2D map YAML negate must be 0 or 1.')

        try:
            with Image.open(pgm_path) as image:
                image.load()
                width, height = image.size
                image_format = image.format
        except Exception as error:
            raise MapSaveError(
                f'PGM image cannot be decoded: {error}'
            ) from error
        if width <= 0 or height <= 0 or image_format != 'PPM':
            # Pillow identifies both PBM/PGM/PPM family members as "PPM".
            raise MapSaveError('Saved PGM image has an invalid format or size.')

        # Keep the YAML relocatable and make its image name match the final file.
        if normalize:
            document['image'] = f'{map_name}.pgm'
            self._write_yaml(yaml_path, document)
        return {
            'image': f'{map_name}.pgm',
            'width': int(width),
            'height': int(height),
            'resolution': resolution,
            'origin': origin,
            'negate': negate,
            'occupied_thresh': occupied_thresh,
            'free_thresh': free_thresh,
        }

    @staticmethod
    def _load_ground_support(
        directory: Path,
        map_name: str,
        map_geometry: dict[str, Any],
    ) -> GroundSupportGrid:
        """Decode PGM pixels into an explicit ROS-row-order free mask."""

        pgm_path = directory / f'{map_name}.pgm'
        with Image.open(pgm_path) as image:
            pixels = np.array(
                image.convert('L'), dtype=np.uint8, copy=True)
        if pixels.shape != (
            int(map_geometry['height']), int(map_geometry['width'])
        ):
            raise MapSaveError('2D free-space mask geometry is inconsistent.')
        probability = pixels.astype(np.float32) / 255.0
        if int(map_geometry['negate']) == 0:
            probability = 1.0 - probability
        # Image rows start at the top, while OccupancyGrid rows start at the
        # saved origin.  Only cells below free_thresh authorize reconstruction.
        free_mask = np.flipud(
            probability < float(map_geometry['free_thresh']))
        origin = map_geometry['origin']
        return GroundSupportGrid(
            resolution=float(map_geometry['resolution']),
            width=int(map_geometry['width']),
            height=int(map_geometry['height']),
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=float(origin[2]),
            free_mask=free_mask,
        )

    @staticmethod
    def _quality_message(quality: dict[str, Any]) -> str:
        coverage = 100.0 * float(quality.get('coverage_ratio', 0.0))
        largest = 100.0 * float(
            quality.get('largest_component_ratio', 0.0))
        warnings = quality.get('warnings') or []
        prefix = 'Terrain quality warning' if warnings else 'Terrain quality'
        return (
            f'{prefix}: coverage={coverage:.1f}%, '
            f'largest traversable component={largest:.1f}%.'
        )

    @staticmethod
    def _validate_projected_map_geometry(
        projected_map: dict[str, Any],
        map_geometry: dict[str, Any],
    ) -> None:
        """Ensure map_saver wrote the same grid observed by this service."""
        if (
            int(projected_map['width']) != int(map_geometry['width'])
            or int(projected_map['height']) != int(map_geometry['height'])
        ):
            raise MapSaveError(
                'The saved PGM dimensions do not match the latest '
                '/projected_map. The grid changed during saving; retry once '
                'the current map update has completed.'
            )
        if not math.isclose(
            float(projected_map['resolution']),
            float(map_geometry['resolution']),
            rel_tol=0.0,
            abs_tol=1.0e-6,
        ):
            raise MapSaveError(
                'The saved map resolution does not match /projected_map.'
            )

        projected_origin = projected_map['origin']
        saved_origin = map_geometry['origin']
        if len(projected_origin) != 3 or len(saved_origin) != 3:
            raise MapSaveError('The 2D map origin is invalid.')
        linear_origin_matches = all(
            math.isclose(
                float(projected_origin[index]),
                float(saved_origin[index]),
                rel_tol=0.0,
                abs_tol=1.0e-5,
            )
            for index in (0, 1)
        )
        yaw_error = math.atan2(
            math.sin(float(projected_origin[2]) - float(saved_origin[2])),
            math.cos(float(projected_origin[2]) - float(saved_origin[2])),
        )
        if not linear_origin_matches or abs(yaw_error) > 1.0e-5:
            raise MapSaveError(
                'The saved map origin does not match /projected_map. '
                'Nothing was overwritten.'
            )

    def _request_localization_pcd(self) -> dict[str, Any]:
        if not self.localization_pcd_client.wait_for_service(
            timeout_sec=self.pcd_service_wait_seconds
        ):
            raise MapSaveError(
                '/lio/save_localization_pcd is unavailable. Confirm that '
                'Small Point-LIO is running in mapping mode with '
                'enable_localization_map:=true.'
            )

        future = self.localization_pcd_client.call_async(Trigger.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=self.pcd_save_timeout_seconds):
            future.cancel()
            raise MapSaveError(
                'Filtered localization PCD generation timed out.'
            )
        service_response = future.result()
        if service_response is None:
            raise MapSaveError(
                'Filtered localization PCD service returned no response.'
            )
        if not service_response.success:
            raise MapSaveError(
                'Filtered localization PCD generation failed: '
                f'{service_response.message}'
            )

        snapshot = json.loads(service_response.message)
        required = {
            'schema',
            'version',
            'path',
            'capture_frame',
            'point_count',
            'voxel_size',
            'min_observations',
            'observation_stride',
            'received_frames',
            'sampled_frames',
        }
        missing = sorted(required.difference(snapshot))
        if missing:
            raise MapSaveError(
                f'Localization PCD response is missing fields: {missing}'
            )
        if (
            snapshot['schema'] != 'small_point_lio_localization_snapshot'
            or int(snapshot['version']) != 1
        ):
            raise MapSaveError(
                'Localization PCD response schema/version is unsupported.'
            )
        path = Path(str(snapshot['path'])).expanduser().resolve()
        if path.suffix.lower() != '.pcd' or not path.is_file():
            raise MapSaveError(
                f'Localization PCD service returned an invalid path: {path}'
            )
        if int(snapshot['point_count']) <= 0:
            raise MapSaveError(
                'Localization PCD service returned zero points.'
            )
        snapshot['path'] = str(path)
        return snapshot

    @staticmethod
    def _validate_pcd(path: Path) -> dict[str, Any]:
        fields: list[str] | None = None
        point_count: int | None = None
        data_format: str | None = None
        with path.open('rb') as stream:
            for _ in range(64):
                raw_line = stream.readline(8192)
                if not raw_line:
                    break
                try:
                    line = raw_line.decode('ascii').strip()
                except UnicodeDecodeError as error:
                    raise MapSaveError(
                        'PCD header is not valid ASCII.'
                    ) from error
                if not line or line.startswith('#'):
                    continue
                tokens = line.split()
                key = tokens[0].upper()
                if key in ('FIELDS', 'COLUMNS'):
                    fields = [field.lower() for field in tokens[1:]]
                elif key == 'POINTS' and len(tokens) == 2:
                    point_count = int(tokens[1])
                elif key == 'DATA' and len(tokens) == 2:
                    data_format = tokens[1].lower()
                    break
        if fields is None or not {'x', 'y', 'z'}.issubset(fields):
            raise MapSaveError('PCD header does not contain x/y/z fields.')
        if point_count is None or point_count <= 0:
            raise MapSaveError('PCD header has no valid POINTS count.')
        if data_format not in ('binary', 'binary_compressed'):
            raise MapSaveError(
                f'PCD data format "{data_format}" is unsupported.'
            )
        if path.stat().st_size <= 200:
            raise MapSaveError('PCD file is unexpectedly small.')
        return {
            'fields': fields,
            'point_count': point_count,
            'data_format': data_format,
        }

    def _build_metadata(
        self,
        *,
        map_name: str,
        map_geometry: dict[str, Any],
        projected_map: dict[str, Any],
        lio_snapshot: dict[str, Any],
        pcd_header: dict[str, Any],
        pcd_size: int,
        pcd_sha256: str,
        terrain_metadata: dict[str, Any],
        transaction_directory: Path,
    ) -> dict[str, Any]:
        yaml_size = (
            transaction_directory / f'{map_name}.yaml'
        ).stat().st_size
        pgm_size = (
            transaction_directory / f'{map_name}.pgm'
        ).stat().st_size
        point_count = int(pcd_header['point_count'])
        voxel_size = float(lio_snapshot['voxel_size'])
        pcd_file = f'{map_name}.pcd'
        return {
            # Canonical fields consumed by map switching and relocalization.
            'schema': METADATA_SCHEMA,
            'version': METADATA_VERSION,
            'map_name': map_name,
            'frame_id': 'map',
            'pcd_file': pcd_file,
            'point_count': point_count,
            'pcd_size_bytes': int(pcd_size),
            'pcd_sha256': pcd_sha256,
            'voxel_size': voxel_size,
            'created_at_utc': datetime.now(timezone.utc).isoformat(),
            'map_geometry': map_geometry,
            'terrain_grid': terrain_metadata,
            # Rich provenance retained for diagnostics and future migrations.
            'coordinate_system': {
                'map_frame': 'map',
                'pcd_frame': 'map',
                'capture_frame': str(lio_snapshot['capture_frame']),
                'projected_map_frame': projected_map['frame_id'],
                'pcd_to_map': 'identity',
                'compatibility': (
                    'The saved 2D map and PCD share mapping-session odom '
                    'coordinates, which become the saved map frame on reload.'
                ),
            },
            'pcd': {
                'file': pcd_file,
                'point_count': point_count,
                'size_bytes': int(pcd_size),
                'sha256': pcd_sha256,
            },
            'localization_pcd': {
                'filename': pcd_file,
                'point_count': point_count,
                'size_bytes': int(pcd_size),
                'sha256': pcd_sha256,
                'format': pcd_header['data_format'],
                'fields': pcd_header['fields'],
                'voxel_size': voxel_size,
                'filter': {
                    'type': 'distinct_frame_voxel_persistence',
                    'mode': str(lio_snapshot.get(
                        'filter_mode', 'protected'
                    )),
                    'min_observations': int(
                        lio_snapshot['min_observations']
                    ),
                    'observation_stride': int(
                        lio_snapshot['observation_stride']
                    ),
                    'received_frames': int(
                        lio_snapshot['received_frames']
                    ),
                    'sampled_frames': int(
                        lio_snapshot['sampled_frames']
                    ),
                    'tracked_voxels': int(
                        lio_snapshot.get('tracked_voxels', point_count)
                    ),
                    'ground_preservation': {
                        'enabled': bool(lio_snapshot.get(
                            'ground_preservation_enabled', False
                        )),
                        'cell_size': float(lio_snapshot.get(
                            'ground_cell_size', 0.0
                        )),
                        'max_height_above_envelope': float(
                            lio_snapshot.get(
                                'ground_max_height_above_envelope', 0.0
                            )
                        ),
                        'max_slope_deg': float(lio_snapshot.get(
                            'ground_max_slope_deg', 0.0
                        )),
                        'min_neighbor_cells': int(lio_snapshot.get(
                            'ground_min_neighbor_cells', 0
                        )),
                        'min_observations': int(lio_snapshot.get(
                            'ground_min_observations', 0
                        )),
                        'min_stable_span_frames': int(lio_snapshot.get(
                            'ground_min_stable_span_frames', 0
                        )),
                        'protected_voxels_at_save': int(lio_snapshot.get(
                            'ground_protected_voxels', 0
                        )),
                    },
                },
            },
            'capture': {
                'source_topic': '/cloud_registered',
                'projected_map_topic': '/projected_map',
                'projected_map_stamp_ns': int(
                    projected_map['stamp_ns']
                ),
                'pcd_capture_unix_ns': int(
                    lio_snapshot.get('capture_unix_ns', 0)
                ),
            },
            'files': {
                'map_yaml': {
                    'filename': f'{map_name}.yaml',
                    'size_bytes': int(yaml_size),
                },
                # No PGM content hash by design: the map editor may modify
                # occupancy pixels while preserving geometry and PCD validity.
                'map_image': {
                    'filename': f'{map_name}.pgm',
                    'size_bytes_at_creation': int(pgm_size),
                },
                'localization_pcd': {
                    'filename': pcd_file,
                    'size_bytes': int(pcd_size),
                    'sha256': pcd_sha256,
                },
                'terrain_grid': {
                    'filename': terrain_metadata['filename'],
                    'size_bytes': terrain_metadata['size_bytes'],
                    'sha256': terrain_metadata['sha256'],
                    'source_pcd_sha256': pcd_sha256,
                },
            },
        }

    @staticmethod
    def _validate_metadata(
        path: Path,
        map_name: str,
        map_geometry: dict[str, Any],
        pcd_header: dict[str, Any],
        pcd_size: int,
        pcd_sha256: str,
        terrain_metadata: dict[str, Any],
    ) -> None:
        with path.open('r', encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        expected = {
            'schema': METADATA_SCHEMA,
            'version': METADATA_VERSION,
            'map_name': map_name,
            'frame_id': 'map',
            'pcd_file': f'{map_name}.pcd',
            'point_count': int(pcd_header['point_count']),
            'pcd_size_bytes': int(pcd_size),
            'pcd_sha256': pcd_sha256,
        }
        for key, value in expected.items():
            if document.get(key) != value:
                raise MapSaveError(
                    f'Generated localization metadata failed field "{key}" '
                    'validation.'
                )
        geometry = document.get('map_geometry')
        if not isinstance(geometry, dict):
            raise MapSaveError(
                'Generated localization metadata has no map_geometry.'
            )
        for key in ('resolution', 'width', 'height', 'origin'):
            if geometry.get(key) != map_geometry[key]:
                raise MapSaveError(
                    f'Generated metadata map_geometry.{key} is inconsistent.'
                )
        if document.get('terrain_grid') != terrain_metadata:
            raise MapSaveError(
                'Generated terrain metadata is inconsistent.'
            )
        try:
            loaded_terrain = load_bound_terrain_grid(
                path.parent,
                map_name=map_name,
                localization_metadata=document,
                actual_pcd_sha256=pcd_sha256,
            )
        except ValueError as error:
            raise MapSaveError(
                f'Generated terrain artifact failed validation: {error}'
            ) from error
        if loaded_terrain is None:
            raise MapSaveError(
                'Generated metadata omitted the required terrain artifact.'
            )

    def _target_paths(self, map_name: str) -> dict[str, Path]:
        return {
            'pgm': self.map_directory / f'{map_name}.pgm',
            'yaml': self.map_directory / f'{map_name}.yaml',
            'pcd': self.pcd_map_directory / f'{map_name}.pcd',
            'terrain': (
                self.pcd_map_directory / f'{map_name}.terrain.npz'
            ),
            'metadata': (
                self.pcd_map_directory
                / f'{map_name}.localization.yaml'
            ),
        }

    def _commit_artifacts_transaction(
        self,
        source_paths: dict[str, Path],
        target_paths: dict[str, Path],
        *,
        install_order=None,
    ) -> None:
        """Replace all five artifacts, restoring every old file on failure.

        New files are copied and fsynced beside their final destinations before
        any replacement starts.  Existing files are backed up first (normally
        with cheap same-filesystem hard links).  Metadata is replaced last, so
        strict readers never accept a partially installed map.
        """
        if install_order is None:
            install_order = ('pgm', 'yaml', 'pcd', 'terrain', 'metadata')
        install_order = tuple(install_order)
        if set(install_order) != set(source_paths) or set(source_paths) != set(
                target_paths):
            raise ValueError('artifact transaction key set is inconsistent')
        backup_directories: dict[Path, Path] = {}
        backups: dict[str, Path] = {}
        staged_targets: dict[str, Path] = {}
        replaced: list[str] = []
        preserve_backups = False

        try:
            # Preserve every pre-existing artifact before touching a target.
            for key in install_order:
                destination = target_paths[key]
                if not destination.exists():
                    continue
                backup_directory = backup_directories.get(destination.parent)
                if backup_directory is None:
                    backup_directory = Path(
                        tempfile.mkdtemp(
                            prefix='.map-save-backup-',
                            dir=str(destination.parent),
                        )
                    )
                    backup_directories[destination.parent] = backup_directory
                backup = backup_directory / destination.name
                self._backup_file(destination, backup)
                backups[key] = backup

            # Stage each validated source on the destination filesystem.  This
            # makes every os.replace below atomic even if maps/ and pcdmaps/
            # reside on different filesystems.
            for key in install_order:
                staged_targets[key] = self._stage_for_replace(
                    source_paths[key], target_paths[key]
                )

            for key in install_order:
                os.replace(staged_targets[key], target_paths[key])
                replaced.append(key)
            self._fsync_directory(self.map_directory)
            self._fsync_directory(self.pcd_map_directory)
        except Exception as error:
            restore_errors: list[str] = []
            for key in reversed(replaced):
                destination = target_paths[key]
                try:
                    if key in backups:
                        os.replace(backups[key], destination)
                    else:
                        destination.unlink(missing_ok=True)
                except OSError as restore_error:
                    restore_errors.append(
                        f'{destination}: {restore_error}'
                    )
            try:
                self._fsync_directory(self.map_directory)
                self._fsync_directory(self.pcd_map_directory)
            except OSError as sync_error:
                restore_errors.append(f'directory fsync: {sync_error}')
            if restore_errors:
                preserve_backups = True
                recovery_locations = ', '.join(
                    str(path) for path in backup_directories.values()
                )
                raise MapSaveError(
                    f'Artifact commit failed ({error}); rollback also '
                    f'encountered: {"; ".join(restore_errors)}. Recovery '
                    f'backups were retained in: {recovery_locations}'
                ) from error
            raise MapSaveError(
                f'Artifact commit failed and the previous map was restored: '
                f'{error}'
            ) from error
        finally:
            for staged in staged_targets.values():
                try:
                    staged.unlink(missing_ok=True)
                except OSError:
                    pass
            if not preserve_backups:
                for directory in backup_directories.values():
                    shutil.rmtree(directory, ignore_errors=True)

    @staticmethod
    def _backup_file(source: Path, backup: Path) -> None:
        try:
            os.link(source, backup)
            return
        except OSError:
            pass
        with source.open('rb') as input_stream, backup.open('xb') as output:
            shutil.copyfileobj(input_stream, output, length=1024 * 1024)
            output.flush()
            os.fsync(output.fileno())
        shutil.copystat(source, backup, follow_symlinks=False)

    @staticmethod
    def _stage_for_replace(source: Path, destination: Path) -> Path:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f'.{destination.name}.new-',
            dir=str(destination.parent),
        )
        staged = Path(temporary_name)
        try:
            with os.fdopen(descriptor, 'wb') as output:
                descriptor = -1
                with source.open('rb') as input_stream:
                    shutil.copyfileobj(
                        input_stream, output, length=1024 * 1024
                    )
                output.flush()
                os.fsync(output.fileno())
            shutil.copystat(source, staged, follow_symlinks=False)
            return staged
        except Exception:
            if descriptor >= 0:
                os.close(descriptor)
            try:
                staged.unlink(missing_ok=True)
            except OSError:
                pass
            raise

    @staticmethod
    def _remove_lio_staging_file(path: Path | None) -> None:
        if path is None:
            return
        # Only remove the unique files produced by Small Point-LIO's staging
        # service. Never unlink a user-selected or final map path.
        if (
            path.suffix.lower() == '.pcd'
            and path.name.startswith('localization-map-')
        ):
            try:
                path.unlink(missing_ok=True)
            except OSError:
                pass

    @staticmethod
    def _write_yaml(path: Path, document: dict[str, Any]) -> None:
        with path.open('w', encoding='utf-8', newline='\n') as stream:
            yaml.safe_dump(
                document,
                stream,
                allow_unicode=True,
                sort_keys=False,
            )
            stream.flush()
            os.fsync(stream.fileno())

    @staticmethod
    def _sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        return digest.hexdigest()

    @staticmethod
    def _fsync_directory(path: Path) -> None:
        if os.name == 'nt':
            return
        descriptor = os.open(path, os.O_RDONLY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MapSaveService()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Map save service interrupted')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
