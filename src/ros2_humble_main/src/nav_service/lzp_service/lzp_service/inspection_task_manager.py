#!/usr/bin/env python3
"""Persistent inspection task orchestration for routes, camera and YOLO."""

from __future__ import annotations

import copy
import json
import math
import os
import shutil
import threading
import time
import uuid
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional
from urllib.error import HTTPError, URLError
from urllib.request import ProxyHandler, Request, build_opener

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from lzp_interface.msg import RouteExecutionStatus
from lzp_interface.srv import (
    AcknowledgeInspectionAlert,
    ControlInspectionTask,
    ControlRoute,
    DeleteInspectionHistory,
    GetInspectionReport,
    GetInspectionHistory,
    GetInspectionState,
    GetInspectionTemplate,
    ListInspectionHistory,
    QueryInspectionReports,
    Reserve3DInspectionRoute,
    Revoke3DInspectionRoute,
    ReviewInspectionReport,
    SaveInspectionTemplate,
    SetInspectionEnvironmentConfig,
    StartInspectionTask,
    StartRoute,
)

from .inspection_database import InspectionDatabase, utc_now
from .environment_alerts import (
    EnvironmentAlertConfigStore,
    EnvironmentAlertPolicy,
    MAX_CO2_THRESHOLD_PPM,
    MIN_CO2_THRESHOLD_PPM,
)
from .detection_settings import DetectionSettings
from .navigation_mode import (
    TERRAIN_3D,
    navigation_mode_label,
    normalize_navigation_mode,
)
from .route_database import RouteDatabase


ACTIVE_STATES = {
    'preparing', 'starting_route', 'navigating', 'point_hold',
    'inspecting', 'releasing_point', 'returning_to_start',
    'interval_wait', 'pausing', 'paused', 'resuming', 'cancelling',
}
TERMINAL_STATES = {'idle', 'completed', 'cancelled', 'failed'}
VALID_DIRECTIONS = {'current', 'front', 'left', 'right', 'back', 'full'}
FULL_SCAN_DIRECTIONS = ('front', 'left', 'back', 'right')
MIN_CONFIRMED_DETECTION_CONFIDENCE = 0.50


def _json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(',', ':'))


def _safe_float(value: Any) -> Optional[float]:
    if value in ('', None):
        return None
    number = float(value)
    if not number > 0.0:
        raise ValueError('变倍倍率必须大于 0')
    return number


def _confirmed_detections(
        detections: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Validate typed detector observations before they become safety alerts."""
    confirmed = []
    for detection in detections:
        if not isinstance(detection, dict):
            continue
        if not bool(detection.get('alert', True)):
            continue
        category = str(detection.get('category') or '')
        if category in {'cigarettes', 'cigarette', 'cigarette_butt'}:
            continue  # Retired object category; smoking requires person evidence.
        if category in {'fall', 'fight', 'smoking'}:
            # Behavior is temporal evidence, never an ordinary single-frame box.
            if (detection.get('state') != 'valid'
                    or detection.get('updated') is not True
                    or detection.get('fresh') is not True
                    or detection.get('alert') is not True
                    or detection.get('label') != category
                    or not detection.get('source_epoch')):
                continue
            try:
                stamp = int(detection.get('stamp_ns', 0))
            except (TypeError, ValueError, OverflowError):
                continue
            if stamp <= 0:
                continue
            if category == 'fall':
                if (detection.get('score_kind') == 'rule_evidence'
                        and detection.get('confidence') is None):
                    confirmed.append(detection)
                continue
            if category == 'smoking':
                details = detection.get('evidence_details') or {}
                try:
                    score = float(detection.get('confidence'))
                except (ValueError, TypeError):
                    continue
                kind = detection.get('score_kind')
                if (detection.get('scope') != 'person' or detection.get('track_id') is None
                        or detection.get('measurement_type') != 'associated_detection_score'
                        or not isinstance(details, dict) or details.get('head_observable') is not True
                        or not math.isfinite(score) or not 0 <= score <= 1):
                    continue
                direct = kind == 'detection_score' and score >= 0.5 and bool(details.get('selected_detection'))
                hand = details.get('hand_to_mouth') or {}
                associated = (kind == 'temporally_associated_detection_score'
                              and isinstance(hand, dict) and hand.get('confirmed') is True
                              and hand.get('short_occlusion') is not True and bool(hand.get('detection')))
                if direct or associated:
                    confirmed.append(detection)
                continue
            if (detection.get('scope') != 'scene'
                    or detection.get('score_kind') != 'model_probability'):
                continue
        if str(detection.get('measurement_type')) == 'temperature':
            try:
                maximum = float(detection.get('max_temperature_c'))
                threshold = float(detection.get('threshold_c'))
            except (TypeError, ValueError):
                continue
            if math.isfinite(maximum) and math.isfinite(threshold) \
                    and maximum >= threshold:
                confirmed.append(detection)
            continue
        try:
            confidence = float(detection.get('confidence', 0.0))
        except (TypeError, ValueError):
            continue
        if math.isfinite(confidence) and MIN_CONFIRMED_DETECTION_CONFIDENCE <= confidence <= 1.0:
            confirmed.append(detection)
    return confirmed


def _alert_storage_fields(
        detection: Dict[str, Any], evidence: Dict[str, Any]) -> Dict[str, Any]:
    """Translate one typed observation into the stable database vocabulary."""
    measurement_type = str(
        detection.get('measurement_type') or 'confidence')
    if detection.get('category') == 'fall':
        measurement_type = 'rule_evidence'
    # A temporal result may be from an earlier frame than the YOLO capture.
    # Use only its own evidence; never label another frame as behavior evidence.
    if detection.get('category') in {'fall', 'fight', 'smoking'}:
        evidence = dict(detection.get('evidence') or {})
    fields = {
        'measurement_type': measurement_type,
        'severity': str(detection.get('severity') or 'warning'),
        'confidence': (None if measurement_type == 'rule_evidence'
                       else float(detection.get('confidence') or 0.0)),
        'raw_image': str(evidence.get('raw_image') or ''),
        'annotated_image': str(evidence.get('annotated_image') or ''),
        'thermal_raw_image': str(evidence.get('thermal_raw_image') or ''),
        'thermal_annotated_image': str(
            evidence.get('thermal_annotated_image') or ''),
    }
    if measurement_type == 'temperature':
        fields['measurement'] = {
            key: detection[key]
            for key in (
                'max_temperature_c', 'threshold_c', 'unit',
                'x_normalized', 'y_normalized', 'temperature_source')
            if key in detection
        }
    elif detection.get('category') in {'fall', 'fight', 'smoking'}:
        fields['measurement'] = {
            key: detection[key] for key in (
                'label', 'score_kind', 'scope', 'state', 'updated', 'fresh',
                'source_epoch', 'stamp_ns', 'prediction_stamp_ns', 'person_id', 'track_id',
                'evidence_details', 'rule_evidence', 'reason', 'window_seconds', 'confidence')
            if key in detection
        }
    return fields


def _detection_summary(detection: Dict[str, Any]) -> str:
    if detection.get('category') == 'fall':
        return '疑似跌倒（连续动作规则证据，无概率分数）'
    if detection.get('category') == 'fight':
        return f'疑似打架（场景级，模型分数 {float(detection.get("confidence", 0.0)):.0%}）'
    if detection.get('category') == 'smoking':
        return f'疑似人员吸烟（人物关联检测分数 {float(detection.get("confidence", 0.0)):.0%}，非行为概率）'
    if str(detection.get('measurement_type')) == 'temperature':
        maximum = float(detection.get('max_temperature_c', 0.0))
        threshold = float(detection.get('threshold_c', 0.0))
        return f'{maximum:.1f} ℃（阈值 {threshold:.1f} ℃）'
    return f'{float(detection.get("confidence", 0.0)):.0%}'


def _behavior_capture_detail(capture: Dict[str, Any]) -> str:
    """Do not describe missing temporal evidence as a normal inspection."""
    behavior = capture.get('behavior')
    if not isinstance(behavior, dict):
        return ''
    labels = {'fall': '跌倒', 'fight': '打架', 'smoking': '吸烟'}
    pending = []
    for key, branch in (behavior.get('branches') or {}).items():
        if key not in labels or not isinstance(branch, dict):
            continue
        if branch.get('enabled') and branch.get('state') != 'valid':
            pending.append(f'{labels[key]}：{branch.get("state") or "不可判断"}')
    return ('行为识别未形成有效结论（' + '；'.join(pending) + '）') if pending else ''


def _interruptible_wait(
        seconds: float, pause_event: threading.Event,
        cancel_event: threading.Event,
        skip_event: threading.Event) -> bool:
    deadline = time.monotonic() + max(0.0, seconds)
    while time.monotonic() < deadline:
        if cancel_event.is_set() or skip_event.is_set():
            return False
        if pause_event.is_set():
            time.sleep(0.1)
            deadline += 0.1
            continue
        time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
    return True


class CameraInspectionClient:
    """Small HTTP client for the local Hikvision camera process."""

    def __init__(self, base_url: str, timeout: float = 8.0):
        self.base_url = base_url.rstrip('/')
        self.timeout = max(1.0, float(timeout))
        self.task_id = ''
        self._opener = build_opener(ProxyHandler({}))

    def set_context(self, task_id: str) -> None:
        self.task_id = str(task_id)

    def request(
            self, path: str, payload: Optional[Dict[str, Any]] = None,
            timeout: Optional[float] = None) -> Dict[str, Any]:
        data = None
        headers = {'Accept': 'application/json'}
        method = 'GET'
        if payload is not None:
            data = _json(payload).encode('utf-8')
            headers['Content-Type'] = 'application/json; charset=utf-8'
            method = 'POST'
        request = Request(
            f'{self.base_url}{path}', data=data, headers=headers,
            method=method)
        try:
            with self._opener.open(
                    request, timeout=timeout or self.timeout) as response:
                body = response.read().decode('utf-8')
        except HTTPError as error:
            body = error.read().decode('utf-8', errors='replace')
            raise RuntimeError(
                f'相机接口 {path} 返回 HTTP {error.code}: {body}') from error
        except (URLError, TimeoutError, OSError) as error:
            raise RuntimeError(f'相机接口 {path} 不可用: {error}') from error
        result = json.loads(body or '{}')
        if isinstance(result, dict) and result.get('success') is False:
            raise RuntimeError(str(result.get('error') or result.get('message')))
        return result if isinstance(result, dict) else {'result': result}

    def set_mode(self, mode: str) -> Dict[str, Any]:
        return self.request(
            '/api/inspection/mode',
            {'mode': mode, 'task_id': self.task_id}, timeout=30.0)

    def state(self) -> Dict[str, Any]:
        return self.request('/api/inspection/state')

    def move(self, direction: str) -> Dict[str, Any]:
        return self.request(
            '/api/inspection/ptz',
            {'direction': direction, 'speed': 'slow'}, timeout=20.0)

    def set_zoom(self, zoom_ratio: float) -> Dict[str, Any]:
        return self.request(
            '/api/inspection/ptz',
            {'action': 'set_zoom', 'zoom_ratio': zoom_ratio}, timeout=20.0)

    def capture(
            self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.request('/api/inspection/capture', payload, timeout=20.0)

    def events(self, since: int) -> Dict[str, Any]:
        return self.request(f'/api/inspection/events?since={max(0, int(since))}')

    def speak_environment_alert(self, key: str) -> Dict[str, Any]:
        return self.request(
            '/api/inspection/speech', {'key': str(key)}, timeout=5.0)


class InspectionTaskManager(Node):
    """Run recoverable inspection jobs independently of the browser."""

    VALID_CONTROLS = {'pause', 'resume', 'skip', 'cancel'}

    def __init__(self):
        super().__init__('inspection_task_manager')
        self._group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        route_db_path = str(self.declare_parameter('route_db_path', '').value)
        inspection_db_path = str(
            self.declare_parameter('inspection_db_path', '').value)
        evidence_root = str(
            self.declare_parameter('inspection_evidence_root', '').value)
        camera_url = str(self.declare_parameter(
            'camera_api_url', 'http://127.0.0.1:8766').value)
        camera_timeout = float(
            self.declare_parameter('camera_api_timeout_sec', 8.0).value)
        environment_config_path = str(self.declare_parameter(
            'environment_alert_config_path', '').value)
        environment_confirmations = max(1, int(self.declare_parameter(
            'environment_alert_confirmations', 3).value))
        environment_hysteresis = max(0.0, float(self.declare_parameter(
            'co2_alert_hysteresis_ppm', 100.0).value))
        environment_cooldown = max(1.0, float(self.declare_parameter(
            'environment_alert_cooldown_sec', 60.0).value))
        self._environment_sample_max_age = max(1.0, float(
            self.declare_parameter(
                'environment_sample_max_age_sec', 5.0).value))
        self._interval_scale = max(0.001, float(self.declare_parameter(
            'loop_interval_scale_sec', 60.0).value))
        self._route_service_timeout_sec = max(1.0, float(
            self.declare_parameter('route_service_timeout_sec', 10.0).value))
        self._storage_warning_free_bytes = max(0, int(float(
            self.declare_parameter(
                'storage_warning_free_gb', 5.0).value) * 1024 ** 3))
        report_retention_days = max(1, int(self.declare_parameter(
            'inspection_report_retention_days', 365).value))

        default_root = Path.home() / '.ros' / 'robot_navigation' / 'inspection_data'
        self._evidence_root = Path(evidence_root or default_root).expanduser().resolve()
        self._evidence_root.mkdir(parents=True, exist_ok=True)
        self._routes = RouteDatabase(route_db_path or None)
        self._database = InspectionDatabase(inspection_db_path or None)
        report_cutoff = (
            datetime.now(timezone.utc) - timedelta(days=report_retention_days)
        ).isoformat(timespec='seconds')
        expired_tasks = self._database.delete_completed_before(report_cutoff)
        for expired_task_id in expired_tasks:
            task_directory = (self._evidence_root / expired_task_id).resolve()
            if task_directory.parent == self._evidence_root:
                shutil.rmtree(task_directory, ignore_errors=True)
        if expired_tasks:
            self.get_logger().info(
                f'Expired {len(expired_tasks)} inspection reports older than '
                f'{report_retention_days} days.')
        self._camera = CameraInspectionClient(camera_url, camera_timeout)
        self._detection_settings = DetectionSettings()
        default_environment_path = (
            Path.home() / '.ros' / 'robot_navigation' /
            'environment_alerts.json')
        self._environment_config = EnvironmentAlertConfigStore(
            environment_config_path or default_environment_path)
        environment_state = self._environment_config.snapshot()
        self._environment_policy = EnvironmentAlertPolicy(
            environment_state['co2_threshold_ppm'],
            confirmations=environment_confirmations,
            hysteresis_ppm=environment_hysteresis,
            cooldown_seconds=environment_cooldown,
        )
        self._workers = ThreadPoolExecutor(
            max_workers=3, thread_name_prefix='inspection-worker')
        self._selected_navigation_mode = TERRAIN_3D
        self._navigation_mode_seen = False
        self._selected_robot_body = ''

        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._status_publisher = self.create_publisher(
            String, '/inspection/status', transient)
        self._alerts_publisher = self.create_publisher(
            String, '/inspection/alerts', transient)
        self._log_publisher = self.create_publisher(
            String, '/inspection/log', volatile)
        self._environment_config_publisher = self.create_publisher(
            String, '/inspection/environment_config', transient)
        self.create_subscription(
            RouteExecutionStatus,
            '/route_execution_status',
            self._route_status_callback,
            transient,
            callback_group=self._group,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/navigation_pose',
            self._pose_callback,
            10,
            callback_group=self._group,
        )
        self.create_subscription(
            Bool, '/nav2_ready', self._nav2_ready_callback, transient,
            callback_group=self._group)
        self.create_subscription(
            String, '/relocalization/status',
            self._relocalization_status_callback, transient,
            callback_group=self._group)
        self.create_subscription(
            String, '/navigation/mode',
            self._navigation_mode_callback, transient,
            callback_group=self._group)
        self.create_subscription(
            String, '/robot_body/status',
            self._robot_body_status_callback, transient,
            callback_group=self._group)
        self.create_subscription(
            String, '/air_quality/m701', self._m701_callback, 10,
            callback_group=self._group)

        self._route_start_client = self.create_client(
            StartRoute, '/start_route', callback_group=self._group)
        self._reserve_3d_route_client = self.create_client(
            Reserve3DInspectionRoute,
            '/navigation/reserve_3d_inspection_route',
            callback_group=self._group)
        self._revoke_3d_route_client = self.create_client(
            Revoke3DInspectionRoute,
            '/navigation/revoke_3d_inspection_route',
            callback_group=self._group)
        self._route_control_client = self.create_client(
            ControlRoute, '/control_route', callback_group=self._group)
        self._nav_cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal',
            callback_group=self._group)

        self.create_service(
            SaveInspectionTemplate, '/inspection/save_template',
            self._save_template_callback, callback_group=self._group)
        self.create_service(
            GetInspectionTemplate, '/inspection/get_template',
            self._get_template_callback, callback_group=self._group)
        self.create_service(
            StartInspectionTask, '/inspection/start',
            self._start_task_callback, callback_group=self._group)
        self.create_service(
            ControlInspectionTask, '/inspection/control',
            self._control_task_callback, callback_group=self._group)
        self.create_service(
            GetInspectionState, '/inspection/get_state',
            self._get_state_callback, callback_group=self._group)
        self.create_service(
            ListInspectionHistory, '/inspection/list_history',
            self._list_history_callback, callback_group=self._group)
        self.create_service(
            GetInspectionHistory, '/inspection/get_history',
            self._get_history_callback, callback_group=self._group)
        self.create_service(
            DeleteInspectionHistory, '/inspection/delete_history',
            self._delete_history_callback, callback_group=self._group)
        self.create_service(
            AcknowledgeInspectionAlert, '/inspection/acknowledge_alert',
            self._acknowledge_alert_callback, callback_group=self._group)
        self.create_service(
            QueryInspectionReports, '/inspection/query_reports',
            self._query_reports_callback, callback_group=self._group)
        self.create_service(
            GetInspectionReport, '/inspection/get_report',
            self._get_report_callback, callback_group=self._group)
        self.create_service(
            ReviewInspectionReport, '/inspection/review_report',
            self._review_report_callback, callback_group=self._group)
        self.create_service(
            SetInspectionEnvironmentConfig,
            '/inspection/set_environment_config',
            self._set_environment_config_callback,
            callback_group=self._group)

        self._task: Optional[Dict[str, Any]] = None
        self._route: Optional[Dict[str, Any]] = None
        self._state = self._idle_state()
        self._checkpoints: Dict[int, Dict[str, Any]] = {}
        self._route_status: Optional[RouteExecutionStatus] = None
        self._last_route_event_sequence = 0
        self._route_request_future = None
        self._pending_task_start = None
        self._route_request_purpose = ''
        self._route_request_started_monotonic = 0.0
        self._pending_route_start: Optional[Dict[str, Any]] = None
        self._pending_route_control: Optional[Dict[str, Any]] = None
        self._pending_3d_revoke: Optional[Dict[str, Any]] = None
        self._last_3d_revoke_attempt = 0.0
        self._3d_authorization_generation = 0
        self._route_session_active = False
        self._route_session_purpose = ''
        self._route_completion_event_floor = 0
        self._camera_future: Optional[Future] = None
        self._camera_future_purpose = ''
        self._pending_camera_mode: Optional[tuple[str, str]] = None
        self._inspection_future: Optional[Future] = None
        self._event_future: Optional[Future] = None
        self._last_camera_event_sequence = 0
        self._event_poll_due = 0.0
        self._processed_hold_key = ''
        self._pause_event = threading.Event()
        self._cancel_event = threading.Event()
        self._skip_event = threading.Event()
        self._latest_pose: Optional[Dict[str, float]] = None
        self._nav2_ready = False
        self._localization_ready = False
        self._localized_map_name = ''
        self._last_interval_persist_second: Optional[int] = None
        self._storage_check_due = 0.0
        self._storage_warning_active = False
        self._last_publish_payload = ''
        self._last_alert_payload = ''
        self._last_environment_payload = ''
        self._latest_m701: Optional[Dict[str, Any]] = None
        self._latest_m701_received_monotonic = 0.0
        self._m701_sample_sequence = 0
        self._last_environment_sample_sequence = 0
        self._environment_speech_queue = deque()
        self._environment_speech_pending = set()
        self._environment_speech_future: Optional[Future] = None
        self._environment_speech_active_key = ''

        self._recover_active_task()
        self.create_timer(0.2, self._timer_callback, callback_group=self._group)
        self._publish(force=True)
        self._publish_environment_config(force=True)

    @staticmethod
    def _idle_state() -> Dict[str, Any]:
        return {
            'task_id': '', 'name': '', 'state': 'idle', 'outcome': '',
            'active': False, 'paused': False, 'route_id': 0,
            'route_name': '', 'map_name': '', 'current_loop': 0,
            'loop_count': 0, 'bidirectional': False,
            'navigation_mode': '',
            'loop_interval_minutes': 30.0, 'reverse': False,
            'current_point_id': 0, 'current_point_index': -1,
            'current_point_name': '', 'current_direction': '',
            'completed_points': 0, 'total_points': 0,
            'completed_detection_points': 0,
            'total_detection_points': 0,
            'failed_points': 0, 'skipped_points': 0,
            'route_state': 'stopped', 'message': '暂无巡检任务',
            'estimated_remaining_seconds': 0,
            'progress_percent': 0.0, 'interval_remaining_seconds': 0,
            'unacknowledged_alerts': 0, 'updated_at': utc_now(),
            'point_statuses': {}, 'previous_point_id': 0,
        }

    def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        self._latest_pose = {
            'x': float(message.pose.pose.position.x),
            'y': float(message.pose.pose.position.y),
        }

    def _nav2_ready_callback(self, message: Bool) -> None:
        self._nav2_ready = bool(message.data)

    def _relocalization_status_callback(self, message: String) -> None:
        try:
            status = json.loads(str(message.data or '{}'))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        state = str(status.get('state') or '')
        success = bool(status.get('success')) or state in (
            'succeeded', 'fallback_succeeded')
        terminal = bool(status.get('terminal')) or state in (
            'succeeded', 'fallback_succeeded', 'failed', 'canceled')
        if success and terminal:
            self._localization_ready = True
            self._localized_map_name = str(status.get('map_name') or '')
        elif state:
            self._localization_ready = False
            if terminal:
                self._localized_map_name = ''

    def _route_to_dict(self, route: Dict[str, Any]) -> Dict[str, Any]:
        return {
            'id': int(route['id']),
            'name': str(route['name']),
            'map_name': str(route['map_name']),
            'points': [dict(point) for point in route['points']],
        }

    def _validate_template(
            self, route: Dict[str, Any], config: Dict[str, Any]) -> Dict[str, Any]:
        if not isinstance(config, dict):
            raise ValueError('巡检模板必须是 JSON 对象')
        raw_checkpoints = config.get('checkpoints', [])
        if not isinstance(raw_checkpoints, list):
            raise ValueError('checkpoints 必须是数组')
        route_point_ids = {int(point['id']) for point in route['points']}
        seen = set()
        checkpoints = []
        orphaned = []
        for raw in raw_checkpoints:
            if not isinstance(raw, dict):
                raise ValueError('每个检测点配置必须是对象')
            point_id = int(raw.get('route_point_id', 0))
            if point_id <= 0 or point_id in seen:
                raise ValueError('检测点 ID 无效或重复')
            seen.add(point_id)
            if point_id not in route_point_ids:
                orphaned.append(point_id)
                continue
            checkpoints.append({
                'route_point_id': point_id,
                # Fixed USB camera: keep legacy fields in the serialized
                # template so existing history/report readers remain valid.
                'directions': ['current'],
                'zoom_ratio': None,
                'save_images': bool(raw.get('save_images', False)),
            })
        return {
            'route_id': int(route['id']),
            'route_name': str(route['name']),
            'map_name': str(route['map_name']),
            'checkpoints': checkpoints,
            'orphaned_point_ids': orphaned,
        }

    def _save_template_callback(self, request, response):
        try:
            route = self._routes.get_route(request.route_id)
            if route is None:
                raise ValueError('路线不存在')
            config = json.loads(str(request.config_json) or '{}')
            template = self._validate_template(route, config)
            template = self._database.save_template(request.route_id, template)
            response.success = True
            response.message = (
                '模板已保存' if not template.get('orphaned_point_ids')
                else '模板已保存，已忽略不再属于路线的检测点')
            response.template_json = _json(template)
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            response.success = False
            response.message = str(error)
            response.template_json = '{}'
        return response

    def _get_template_callback(self, request, response):
        route = self._routes.get_route(request.route_id)
        if route is None:
            response.success = False
            response.message = '路线不存在'
            response.template_json = '{}'
            return response
        stored = self._database.get_template(request.route_id) or {
            'route_id': int(request.route_id), 'checkpoints': []}
        try:
            template = self._validate_template(route, stored)
            response.success = True
            response.message = '模板已加载'
            response.template_json = _json(template)
        except ValueError as error:
            response.success = False
            response.message = str(error)
            response.template_json = '{}'
        return response

    def _is_active(self) -> bool:
        return bool(self._state.get('active')) and self._state.get('state') in ACTIVE_STATES

    def _uses_previewed_3d_route(self) -> bool:
        return bool((self._task or {}).get('request', {}).get('previewed_3d'))

    def _drive_pending_3d_revoke(self) -> None:
        pending = self._pending_3d_revoke
        if not pending:
            return
        now = time.monotonic()
        if now - self._last_3d_revoke_attempt < 1.0:
            return
        self._last_3d_revoke_attempt = now
        # Independent urgent stop/pause: never queue this behind a slow
        # StartRoute future. It is best effort; only the revoke service can
        # fence in-flight planning, so we do not claim completion from it.
        if self._route_control_client.service_is_ready():
            urgent = ControlRoute.Request()
            urgent.command = (
                'pause' if pending['action'] == 'pause' else 'stop')
            urgent.owner_token = str(self._state.get('task_id') or '')
            urgent.authorization_generation = (
                self._3d_authorization_generation)
            try:
                self._route_control_client.call_async(urgent)
            except Exception as error:
                self._log(f'3D 紧急路线控制请求失败：{error}', 'warning')
        if not self._revoke_3d_route_client.service_is_ready():
            self._state['message'] = '等待 3D 路线安全撤销服务；任务尚未确认停止'
            return
        request = Revoke3DInspectionRoute.Request()
        request.owner_token = str(self._state.get('task_id') or '')
        request.action = str(pending['action'])
        try:
            future = self._revoke_3d_route_client.call_async(request)
            completed = threading.Event()
            future.add_done_callback(lambda _result: completed.set())
            if not completed.wait(2.0):
                self._state['message'] = '3D 路线授权撤销等待确认，将继续重试'
                return
            result = future.result()
            if not result.success:
                self._state['message'] = (
                    f'3D 路线安全撤销未确认：{result.message}；将继续重试')
                return
            if not result.motion_stopped:
                self._state['message'] = (
                    '3D 路线授权已撤销，正在等待导航确认停止；将继续检查')
                return
        except Exception as error:
            self._state['message'] = (
                f'3D 路线安全撤销失败：{error}；将继续重试')
            return
        # The revoke generation has fenced every old StartRoute response.
        self._3d_authorization_generation = int(
            result.authorization_generation)
        # Detach its client future before a resumed or new task can queue a
        # fresh request; late completion must not rewrite task state.
        old_purpose = self._route_request_purpose
        if self._route_request_future is not None:
            try:
                self._route_request_future.cancel()
            except Exception:
                pass
        self._route_request_future = None
        self._route_request_purpose = ''
        self._route_request_started_monotonic = 0.0
        self._pending_route_start = None
        self._pending_route_control = None
        self._route_session_active = bool(result.route_session_active)
        if not self._route_session_active:
            self._route_session_purpose = ''
        elif old_purpose.startswith('start:'):
            self._route_session_purpose = old_purpose.split(':', 1)[1]
        self._pending_3d_revoke = None
        if pending['action'] == 'pause':
            self._state.update({
                'state': 'paused', 'paused': True,
                'message': '巡检任务已暂停；3D 路线授权屏障已确认',
            })
            self._database.add_event(
                self._state['task_id'], 'task_paused', '操作员暂停任务')
            self._log('巡检任务已暂停，3D 路线授权屏障已确认')
        elif pending['terminal'] == 'failed':
            self._fail_task(pending['reason'], authorization_revoked=True)
        else:
            self._complete_cancel_task()

    def _schedule_3d_revoke(
            self, action: str, *, terminal: str = '',
            reason: str = '') -> None:
        self._pending_3d_revoke = {
            'action': action, 'terminal': terminal, 'reason': reason}
        self._last_3d_revoke_attempt = 0.0
        self._drive_pending_3d_revoke()

    def _resume_3d_route_authorization(self) -> tuple[bool, str]:
        if not self._revoke_3d_route_client.service_is_ready():
            return False, '3D 路线授权服务尚未就绪，请稍后重试'
        request = Revoke3DInspectionRoute.Request()
        request.owner_token = str(self._state.get('task_id') or '')
        request.action = 'resume'
        try:
            future = self._revoke_3d_route_client.call_async(request)
            completed = threading.Event()
            future.add_done_callback(lambda _result: completed.set())
            if not completed.wait(3.0):
                return False, '恢复 3D 路线授权超时，请稍后重试'
            result = future.result()
            if result.success:
                self._3d_authorization_generation = int(
                    result.authorization_generation)
            return bool(result.success), str(result.message)
        except Exception as error:
            return False, f'恢复 3D 路线授权失败：{error}'

    def _navigation_mode_callback(self, message: String) -> None:
        try:
            payload = json.loads(str(message.data))
            value = payload.get('mode') if isinstance(payload, dict) else payload
            mode = normalize_navigation_mode(value)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(
                f'巡检忽略了无效导航模式状态：{error}')
            return
        with self._lock:
            changed = (
                not self._navigation_mode_seen
                or mode != self._selected_navigation_mode)
            self._selected_navigation_mode = mode
            self._navigation_mode_seen = True
            if changed:
                self._publish(force=True)

    def _robot_body_status_callback(self, message: String) -> None:
        try:
            status = json.loads(str(message.data))
            config = status.get('config') if isinstance(status, dict) else None
            selected = str(config.get('selected') or '') if isinstance(
                config, dict) else ''
        except (TypeError, ValueError, json.JSONDecodeError):
            selected = ''
        with self._lock:
            self._selected_robot_body = selected

    def _m701_callback(self, message: String) -> None:
        try:
            payload = json.loads(str(message.data or '{}'))
            if not isinstance(payload, dict):
                raise ValueError('M701 数据不是对象')
        except (TypeError, ValueError, json.JSONDecodeError):
            payload = {'online': False, 'valid': False, 'values': None}
        with self._lock:
            self._latest_m701 = payload
            self._latest_m701_received_monotonic = time.monotonic()
            self._m701_sample_sequence += 1

    def _set_environment_config_callback(self, request, response):
        with self._lock:
            try:
                state = self._environment_config.set(
                    request.co2_threshold_ppm, updated_by='settings-page')
                self._environment_policy.configure(
                    state['co2_threshold_ppm'])
                response.success = True
                response.message = (
                    '二氧化碳告警阈值已保存，已用于当前及后续巡检')
            except (OSError, TypeError, ValueError) as error:
                state = self._environment_config.snapshot()
                response.success = False
                response.message = str(error)
            response.co2_threshold_ppm = float(
                state['co2_threshold_ppm'])
            response.version = int(state['version'])
            self._publish_environment_config(force=True)
        return response

    def _start_task_callback(self, request, response):
        with self._lock:
            if getattr(self, '_pending_task_start', None) is not None:
                response.success, response.task_id = False, ''
                response.message = '已有巡检启动请求正在确认，可等待或取消启动'
                return response
            if self._is_active():
                response.success = False
                response.message = '已有巡检任务正在运行'
                response.task_id = ''
                return response
            if self._route_request_future is not None:
                response.success = False
                response.message = '前次路线请求尚未返回，请等待停止确认后再创建任务'
                response.task_id = ''
                return response
            try:
                name = str(request.name).strip()
                if not name:
                    raise ValueError('任务名称不能为空')
                loop_count = int(request.loop_count)
                if loop_count < 1 or loop_count > 10000:
                    raise ValueError('循环次数必须在 1 到 10000 之间')
                interval = float(request.loop_interval_minutes)
                if interval < 0.0 or interval > 100000.0:
                    raise ValueError('轮次间隔必须是非负分钟数')
                route = self._routes.get_route(request.route_id)
                if route is None or not route['points']:
                    raise ValueError('请选择包含目标点的有效路线')
                if not self._nav2_ready:
                    raise ValueError('Nav2 尚未就绪，请稍后再启动巡检')
                if not self._navigation_mode_seen:
                    raise ValueError('导航模式状态尚未就绪，请稍后再启动巡检')
                if (self._selected_navigation_mode == TERRAIN_3D
                        and self._selected_robot_body == 'xingji'
                        and not str(getattr(request, 'preview_id', '') or '').strip()):
                    raise ValueError('请先在 3D 点云图预览整条巡检路线，再确认启动')
                if not self._localization_ready:
                    raise ValueError('初始定位尚未通过确认，请先设置初始位置')
                if (
                    self._localized_map_name
                    and self._localized_map_name != str(route['map_name'])
                ):
                    raise ValueError('当前定位结果不属于所选路线地图，请重新定位')
                stored = self._database.get_template(request.route_id) or {
                    'route_id': int(request.route_id), 'checkpoints': []}
                template = self._validate_template(route, stored)
            except (ValueError, TypeError) as error:
                response.success = False
                response.message = str(error)
                response.task_id = ''
                return response

            prepared = dict(task_id=uuid.uuid4().hex, name=name,
                route=copy.deepcopy(route), template=copy.deepcopy(template),
                loop_count=loop_count, interval=interval,
                bidirectional=bool(request.bidirectional),
                preview_id=str(getattr(request, 'preview_id', '') or '').strip(),
                mode=self._selected_navigation_mode, body=self._selected_robot_body,
                localized_map=self._localized_map_name)
            if self._selected_navigation_mode == TERRAIN_3D and prepared['preview_id']:
                if bool(request.bidirectional):
                    response.success = False
                    response.message = (
                        '3D 双向巡检暂未完成反向逐段预览，不能启动；'
                        '请关闭双向巡检并重新预览')
                    response.task_id = ''
                    return response
                if not self._reserve_3d_route_client.service_is_ready():
                    response.success = False
                    response.message = '3D 路线预览预约服务尚未就绪'
                    response.task_id = ''
                    return response
                pending = dict(prepared=prepared, future=None, cancelled=False,
                    cleanup_registered=False, waiter=None, started=time.monotonic())
                self._pending_task_start = pending
            else:
                return self._commit_task_start(prepared, response)

        # No shared task lock is held while a ROS executor delivers the reply.
        # Otherwise timer/status callbacks can occupy every worker waiting for
        # that lock, starving the service future until its timeout.
        try:
            reserve = Reserve3DInspectionRoute.Request()
            reserve.preview_id = prepared['preview_id']
            reserve.owner_token = prepared['task_id']
            reserve.route_id = int(prepared['route']['id'])
            reserve.loop_count = prepared['loop_count']
            reserve.bidirectional = False
            self._log('正在确认三维路线预约与启动条件，可取消本次启动。')
            self.get_logger().info('INSPECTION_START '+_json(dict(stage='reserving',
                task_id=prepared['task_id'], preview_id=prepared['preview_id'])))
            future = self._reserve_3d_route_client.call_async(reserve)
            completed = threading.Event()
            with self._lock:
                pending['future'], pending['waiter'] = future, completed
                if pending['cancelled'] or self._pending_task_start is not pending:
                    raise ValueError('启动已取消，晚到预约不会创建任务')
            future.add_done_callback(lambda _result: completed.set())
            if not completed.wait(12.0):
                raise TimeoutError('路线预约请求等待后台响应超时（12 秒）；不是预览过期')
            if not future.done():
                raise ValueError('启动已取消，晚到预约不会创建任务')
            result = future.result()
            if not result.success:
                raise ValueError(result.message)
            with self._lock:
                if pending['cancelled'] or self._pending_task_start is not pending:
                    raise ValueError('启动已取消，晚到预约不会创建任务')
                current_route = self._routes.get_route(int(prepared['route']['id']))
                if (self._is_active() or not self._nav2_ready
                        or self._selected_navigation_mode != prepared['mode']
                        or self._selected_robot_body != prepared['body']
                        or not self._localization_ready
                        or self._localized_map_name != prepared['localized_map']
                        or current_route != prepared['route']):
                    raise ValueError('预约期间地图、路线、本体或导航状态变化，未启动巡检')
                current_template = self._database.get_template(int(prepared['route']['id'])) or {
                    'route_id': int(prepared['route']['id']), 'checkpoints': []}
                if self._validate_template(current_route,current_template) != prepared['template']:
                    raise ValueError('预约期间检测点配置变化，未启动巡检')
                self._pending_task_start = None
                response = self._commit_task_start(prepared, response)
                self.get_logger().info('INSPECTION_START '+_json(dict(stage='committed',
                    task_id=prepared['task_id'],
                    elapsed_sec=round(time.monotonic()-pending['started'],3))))
                return response
        except Exception as error:
            with self._lock:
                self._abandon_task_start(pending)
            response.success, response.task_id = False, ''
            response.message = f'3D 巡检未启动：{error}'
            self._log(response.message)
            self.get_logger().info('INSPECTION_START '+_json(dict(stage='rejected',
                task_id=prepared['task_id'], error=str(error),
                elapsed_sec=round(time.monotonic()-pending['started'],3))))
            return response

    def _abandon_task_start(self, pending):
        """Caller holds _lock. Wake the waiter; late replies may only clean up."""
        pending['cancelled'] = True
        if getattr(self, '_pending_task_start', None) is pending:
            self._pending_task_start = None
        if pending.get('waiter') is not None:
            pending['waiter'].set()
        future = pending.get('future')
        if future is not None and not pending['cleanup_registered']:
            pending['cleanup_registered'] = True
            owner = pending['prepared']['task_id']
            future.add_done_callback(lambda f: self._release_abandoned_start(f, owner))

    def _release_abandoned_start(self, future, owner):
        try:
            if not future.result().success:
                return
            if not self._revoke_3d_route_client.service_is_ready():
                self._log('已放弃的启动收到晚到预约；路线撤销服务暂不可用，未创建巡检任务', 'warning')
                return
            revoke = Revoke3DInspectionRoute.Request()
            revoke.owner_token, revoke.action = owner, 'cancel'
            self._revoke_3d_route_client.call_async(revoke)
            self.get_logger().info('INSPECTION_START '+_json(dict(stage='late_reservation_revoke_requested',task_id=owner)))
        except Exception as error:
            self._log(f'放弃启动后的预约清理未完成：{error}', 'warning')

    def _commit_task_start(self, prepared, response):
        """Commit under _lock after reservation/context checks; no RPC wait."""
        task_id, name = prepared['task_id'], prepared['name']
        route, template = prepared['route'], prepared['template']
        loop_count, interval = prepared['loop_count'], prepared['interval']
        preview_id = prepared['preview_id']
        request_data = {
            'loop_count': loop_count,
            'loop_interval_minutes': interval,
            'bidirectional': prepared['bidirectional'],
            'navigation_mode': prepared['mode'],
            'previewed_3d': bool(
                prepared['mode'] == TERRAIN_3D
                and preview_id),
        }
        checkpoint_count = len(template['checkpoints'])
        self._state = {
            **self._idle_state(),
            'task_id': task_id, 'name': name, 'state': 'preparing',
            'active': True, 'route_id': int(route['id']),
            'route_name': str(route['name']),
            'map_name': str(route['map_name']), 'current_loop': 1,
            'loop_count': loop_count,
            'bidirectional': prepared['bidirectional'],
            'loop_interval_minutes': interval,
            'navigation_mode': prepared['mode'],
            'total_points': len(route['points']) * loop_count,
            'total_detection_points': checkpoint_count * loop_count,
            'message': '正在准备已启用的识别项目',
            'started_at': utc_now(), 'resume_state': '',
            'recovery_pending': False,
        }
        route_snapshot = self._route_to_dict(route)
        self._database.create_task(
            task_id, name, route_snapshot, request_data,
            template, self._state)
        self._task = {
            'task_id': task_id, 'name': name, 'route_id': int(route['id']),
            'route_name': str(route['name']),
            'map_name': str(route['map_name']), 'request': request_data,
            'template': template, 'state': self._state,
        }
        self._route = route_snapshot
        self._camera.set_context(task_id)
        self._checkpoints = {
            int(item['route_point_id']): item
            for item in template['checkpoints']
        }
        self._reset_runtime_events()
        mode_label = navigation_mode_label(prepared['mode'])
        self._database.add_event(
            task_id, 'task_started', f'巡检任务已创建，模式：{mode_label}')
        self._log(f'巡检任务已创建（{mode_label}），正在准备已启用的识别项目')
        self._publish(force=True)
        response.success = True
        response.message = '巡检任务已启动'
        response.task_id = task_id
        return response

    def _reset_runtime_events(self) -> None:
        self._last_route_event_sequence = 0
        self._route_session_active = False
        self._route_session_purpose = ''
        self._processed_hold_key = ''
        self._pending_route_start = None
        self._pending_route_control = None
        self._pending_3d_revoke = None
        self._last_3d_revoke_attempt = 0.0
        self._3d_authorization_generation = 0
        if self._route_request_future is not None:
            try:
                self._route_request_future.cancel()
            except Exception:
                pass
        self._route_request_future = None
        self._route_request_purpose = ''
        self._route_request_started_monotonic = 0.0
        self._route_request_started_monotonic = 0.0
        self._last_camera_event_sequence = 0
        self._pause_event.clear()
        self._cancel_event.clear()
        self._skip_event.clear()
        self._last_interval_persist_second = None

    def _control_task_callback(self, request, response):
        with self._lock:
            command = str(request.command).strip().lower()
            if command not in self.VALID_CONTROLS:
                response.success = False
                response.message = f'不支持的巡检控制命令: {command}'
                return response
            pending = getattr(self,'_pending_task_start',None)
            if pending is not None and command in ('pause','cancel'):
                self._abandon_task_start(pending)
                response.success = True
                response.message = '已取消尚未完成的巡检启动；晚到预约不会启动任务'
                self._log(response.message)
                return response
            if not self._is_active():
                response.success = False
                response.message = '当前没有活动巡检任务'
                return response
            handlers = {
                'pause': self._pause_task,
                'resume': self._resume_task,
                'skip': self._skip_current_point,
                'cancel': self._cancel_task,
            }
            success, message = handlers[command]()
            response.success = success
            response.message = message
            self._persist()
            self._publish(force=True)
            return response

    def _pause_task(self):
        if self._state['state'] == 'paused':
            return True, '任务已经暂停'
        if self._state['state'] == 'pausing':
            return True, '正在等待 3D 路线确认暂停'
        if self._uses_previewed_3d_route():
            resume_state = self._state['state']
            self._state['resume_state'] = resume_state
            if resume_state == 'interval_wait':
                deadline = float(self._state.get(
                    'interval_deadline_monotonic', 0.0))
                remaining = max(0.0, deadline - time.monotonic())
                self._state['interval_remaining_seconds'] = int(round(remaining))
                self._state['paused_interval_remaining_seconds'] = remaining
                self._state['interval_deadline_epoch'] = time.time() + remaining
            self._state.update({
                'state': 'pausing', 'paused': True,
                'message': '正在撤销 3D 路线授权并等待导航暂停确认',
            })
            self._pause_event.set()
            self._pending_route_start = None
            self._submit_camera_mode('paused', 'pause')
            self._schedule_3d_revoke('pause')
            return True, str(self._state['message'])
        resume_state = self._state['state']
        self._state['resume_state'] = resume_state
        if resume_state == 'interval_wait':
            deadline = float(
                self._state.get('interval_deadline_monotonic', 0.0))
            remaining = max(0.0, deadline - time.monotonic())
            self._state['interval_remaining_seconds'] = int(round(remaining))
            self._state['paused_interval_remaining_seconds'] = remaining
            self._state['interval_deadline_epoch'] = time.time() + remaining
        self._state['state'] = 'paused'
        self._state['paused'] = True
        self._state['message'] = '巡检任务已暂停'
        self._pause_event.set()
        if resume_state != 'interval_wait':
            self._call_route_control('pause', 'pause')
        self._submit_camera_mode('paused', 'pause')
        self._database.add_event(
            self._state['task_id'], 'task_paused', '操作员暂停任务')
        self._log('巡检任务已暂停，导航和 YOLO 均已暂停')
        return True, '巡检任务已暂停'

    def _resume_task(self):
        if self._state['state'] != 'paused':
            return False, '任务当前不是暂停状态'
        if self._uses_previewed_3d_route():
            success, detail = self._resume_3d_route_authorization()
            if not success:
                return False, detail
        resume_state = str(self._state.get('resume_state') or 'navigating')
        recovered = bool(self._state.get('recovery_pending'))
        route_status = self._route_status
        if recovered and route_status is not None and bool(route_status.active):
            if str(route_status.owner_token) != str(self._state['task_id']):
                return False, '路线正由其他任务占用，暂不能继续巡检'
            self._adopt_recovered_route(route_status)
            recovered = False
        self._state['paused'] = False
        self._state['recovery_pending'] = False
        self._pause_event.clear()
        self._state['state'] = 'resuming'
        self._state['message'] = '正在继续巡检任务'
        deferred_completion = str(
            self._state.pop('deferred_route_completion', '') or '')
        if deferred_completion:
            self._submit_camera_mode('transit', 'resume')
            if deferred_completion == 'patrol':
                self._finish_patrol_loop()
            else:
                self._enter_interval_wait()
            self._database.add_event(
                self._state['task_id'], 'task_resumed',
                '操作员继续任务并结算暂停期间已完成的路线')
            self._log('巡检任务继续运行')
            return True, '巡检任务继续运行'
        if resume_state == 'interval_wait':
            remaining = float(self._state.pop(
                'paused_interval_remaining_seconds',
                self._state.get('interval_remaining_seconds', 0.0)))
            self._state.update({
                'state': 'interval_wait',
                'interval_deadline_monotonic': time.monotonic() + remaining,
                'interval_deadline_epoch': time.time() + remaining,
                'interval_remaining_seconds': int(round(remaining)),
                'message': '轮次间隔等待中',
            })
            self._submit_camera_mode('transit', 'resume')
        if recovered or not self._route_session_active:
            if resume_state == 'interval_wait':
                pass
            elif resume_state == 'returning_to_start':
                self._begin_reposition()
            else:
                self._submit_camera_mode('transit', 'resume')
                self._begin_patrol_loop(
                    start_point_id=int(
                        self._state.get('current_point_id') or 0))
        else:
            self._call_route_control('resume', 'resume')
            camera_mode = 'inspection' if resume_state == 'inspecting' else 'transit'
            self._submit_camera_mode(camera_mode, 'resume')
            self._state['state'] = resume_state
        self._database.add_event(
            self._state['task_id'], 'task_resumed', '操作员继续任务')
        self._log('巡检任务继续运行')
        return True, '巡检任务继续运行'

    def _skip_current_point(self):
        if self._state.get('paused') or self._state['state'] == 'paused':
            return False, '请先继续任务，再跳过当前点'
        if self._state['state'] not in ('navigating', 'inspecting'):
            return False, '当前阶段没有可跳过的巡检点'
        if not self._route_session_active:
            status = getattr(self, '_route_status', None)
            if status is not None:
                self._route_status_callback(status)
            return False, '路线已结束或尚未就绪，当前没有可跳过的活动路线点'
        if self._state['state'] == 'inspecting':
            self._skip_event.set()
            self._state['message'] = '正在结束当前检测并跳过该点'
            return True, '正在跳过当前检测点'
        self._call_route_control('skip', 'skip')
        self._state['message'] = '已请求跳过当前点'
        return True, '已请求跳过当前点'

    def _cancel_task(self):
        if self._uses_previewed_3d_route():
            if self._state['state'] == 'cancelling':
                return True, '3D 路线仍在等待停止确认'
            self._pending_route_start = None
            self._cancel_event.set()
            self._skip_event.set()
            self._pause_event.set()
            self._state.update({
                'state': 'cancelling', 'paused': True,
                'message': '正在同步撤销 3D 路线授权并等待导航停止确认',
            })
            self._submit_camera_mode('paused', 'cancel')
            self._schedule_3d_revoke('cancel', terminal='cancelled')
            return True, str(self._state['message'])
        self._complete_cancel_task()
        return True, '巡检任务已终止'

    def _complete_cancel_task(self):
        task_id = self._state['task_id']
        route_started_or_in_flight = (
            self._route_session_active
            or (
                self._route_request_future is not None
                and self._route_request_purpose.startswith('start:')
            )
        )
        self._pending_route_start = None
        self._cancel_event.set()
        self._skip_event.set()
        self._pause_event.clear()
        self._state.update({
            'state': 'cancelled', 'outcome': 'cancelled', 'active': False,
            'paused': False, 'message': '任务已由操作员终止',
            'ended_at': utc_now(),
        })
        if route_started_or_in_flight and not self._uses_previewed_3d_route():
            self._call_route_control('stop', 'cancel')
        else:
            self._pending_route_control = None
        self._submit_camera_mode('paused', 'cancel')
        self._database.add_event(task_id, 'task_cancelled', '操作员终止任务')
        self._database.update_task(task_id, self._state, 'cancelled')
        self._log('巡检任务已终止')
        if self._uses_previewed_3d_route():
            # The route manager has acknowledged the owner fence; a late
            # service reply from this task must not touch the next task.
            if self._route_request_future is not None:
                try:
                    self._route_request_future.cancel()
                except Exception:
                    pass
            self._route_request_future = None
            self._route_request_purpose = ''
            self._route_request_started_monotonic = 0.0

    def _get_state_callback(self, _request, response):
        with self._lock:
            response.success = True
            response.message = '巡检状态已返回'
            response.state_json = _json(self._status_payload())
        return response

    def _list_history_callback(self, request, response):
        try:
            response.history_json = _json(self._database.list_history(
                request.limit or 50, request.offset))
            response.success = True
            response.message = '历史记录已返回'
        except (ValueError, TypeError) as error:
            response.success = False
            response.message = str(error)
            response.history_json = '[]'
        return response

    def _get_history_callback(self, request, response):
        history = self._database.get_task_details(str(request.task_id))
        response.success = history is not None
        response.message = '历史详情已返回' if history else '任务历史不存在'
        response.history_json = _json(history or {})
        return response

    def _delete_history_callback(self, request, response):
        task_id = str(request.task_id).strip()
        with self._lock:
            if self._is_active() and task_id == self._state.get('task_id'):
                response.success = False
                response.message = '不能删除正在运行的任务'
                return response
        removed = self._database.delete_task(task_id)
        task_directory = (self._evidence_root / task_id).resolve()
        if removed and task_directory.parent == self._evidence_root:
            shutil.rmtree(task_directory, ignore_errors=True)
        response.success = removed
        response.message = '任务历史和现场图片已删除' if removed else '任务历史不存在'
        return response

    def _acknowledge_alert_callback(self, request, response):
        response.success = self._database.acknowledge_alert(request.alert_id)
        response.message = '告警已确认' if response.success else '告警不存在'
        self._publish_alerts(force=True)
        return response

    def _query_reports_callback(self, request, response):
        try:
            filters = json.loads(str(request.filters_json or '{}'))
            if not isinstance(filters, dict):
                raise ValueError('报告筛选条件必须是 JSON 对象')
            result = self._database.query_reports(
                filters, request.limit or 50, request.offset)
            response.success = True
            response.message = '巡检报告已返回'
            response.reports_json = _json(result['reports'])
            response.summary_json = _json(result['summary'])
            response.total = int(result['total'])
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            response.success = False
            response.message = str(error)
            response.reports_json = '[]'
            response.summary_json = '{}'
            response.total = 0
        return response

    def _get_report_callback(self, request, response):
        report = self._database.get_report(str(request.task_id))
        response.success = report is not None
        response.message = '巡检报告详情已返回' if report else '巡检报告不存在'
        response.report_json = _json(report or {})
        return response

    def _review_report_callback(self, request, response):
        try:
            report = self._database.review_report(
                str(request.task_id), str(request.report_status),
                str(request.conclusion), str(request.reviewer))
            response.success = True
            response.message = (
                '巡检报告已关闭'
                if report['report_status'] == 'closed'
                else '巡检报告复核信息已保存'
            )
            response.report_json = _json(report)
        except (ValueError, TypeError) as error:
            response.success = False
            response.message = str(error)
            response.report_json = '{}'
        return response

    def _recover_active_task(self) -> None:
        task = self._database.get_active_task()
        if task is None:
            return
        route = self._routes.get_route(task['route_id'])
        if route is None:
            state = dict(task['state'])
            state.update({
                'state': 'failed', 'outcome': 'failed', 'active': False,
                'message': 'ROS2 重启后无法恢复：原路线已被删除',
                'ended_at': utc_now(),
            })
            self._database.update_task(task['task_id'], state, 'failed')
            self._state = state
            return
        self._task = task
        self._camera.set_context(task['task_id'])
        self._route = self._route_to_dict(route)
        self._checkpoints = {
            int(item['route_point_id']): item
            for item in task['template'].get('checkpoints', [])
        }
        self._state = dict(task['state'])
        stored_mode = (
            self._state.get('navigation_mode')
            or (task.get('request') or {}).get('navigation_mode')
            or TERRAIN_3D)
        try:
            self._state['navigation_mode'] = normalize_navigation_mode(
                stored_mode)
        except ValueError:
            self._state['navigation_mode'] = TERRAIN_3D
        previous_state = str(self._state.get('state') or 'navigating')
        resume_state = (
            str(self._state.get('resume_state') or 'navigating')
            if previous_state == 'paused'
            else previous_state
        )
        if resume_state == 'paused':
            resume_state = 'navigating'
        if resume_state == 'interval_wait':
            deadline_epoch = float(
                self._state.get('interval_deadline_epoch', 0.0) or 0.0)
            remaining = (
                max(0.0, deadline_epoch - time.time())
                if deadline_epoch > 0.0
                else max(0.0, float(
                    self._state.get('interval_remaining_seconds', 0.0)))
            )
            self._state['interval_remaining_seconds'] = int(round(remaining))
            self._state['paused_interval_remaining_seconds'] = remaining
        self._state.update({
            'state': 'paused', 'paused': True, 'active': True,
            'resume_state': resume_state,
            'recovery_pending': True,
            'message': '后台已恢复任务；请点击继续，从当前未完成点重新接续',
            'updated_at': utc_now(),
        })
        self._reset_runtime_events()
        self._pause_event.set()
        self._database.update_task(task['task_id'], self._state)
        self._database.add_event(
            task['task_id'], 'task_recovered',
            'ROS2 后台重启，任务已恢复为暂停状态', level='warning')

    def _timer_callback(self) -> None:
        with self._lock:
            self._drive_pending_3d_revoke()
            if self._state['state'] == 'preparing':
                self._drive_preparing()
            self._drive_camera_future()
            self._drive_route_request_future()
            self._drive_inspection_future()
            self._drive_interval_wait()
            self._drive_camera_events()
            self._drive_environment_alerts()
            self._drive_environment_speech()
            self._update_storage_status()
            self._update_estimate()
            self._publish()

    def _drive_preparing(self) -> None:
        if self._camera_future is None:
            self._submit_camera_mode('transit', 'prepare')

    def _submit_camera_mode(self, mode: str, purpose: str) -> None:
        if self._camera_future is not None:
            self._pending_camera_mode = (mode, purpose)
            return
        self._camera_future_purpose = purpose
        self._camera_future = self._workers.submit(self._camera.set_mode, mode)

    def _drive_camera_future(self) -> None:
        future = self._camera_future
        if future is None or not future.done():
            return
        purpose = self._camera_future_purpose
        self._camera_future = None
        self._camera_future_purpose = ''
        try:
            future.result()
            error = ''
        except Exception as exception:  # Hardware failure must not kill patrol.
            error = str(exception)
            self._log(f'相机/YOLO 模式切换失败：{error}', 'warning')
            if self._state.get('task_id'):
                self._database.add_event(
                    self._state['task_id'], 'camera_mode_failed', error,
                    level='warning')
        if (
            (self._pause_event.is_set() or self._state['state'] in TERMINAL_STATES)
            and purpose not in ('pause', 'cancel', 'complete')
        ):
            self._pending_camera_mode = None
            self._submit_camera_mode('paused', 'pause')
            return
        if purpose == 'prepare' and self._state['state'] == 'preparing':
            self._state['message'] = (
                'YOLO 已预热，正在启动路线' if not error
                else 'YOLO 预热失败，记录故障后继续启动路线')
            self._begin_patrol_loop()
        pending = self._pending_camera_mode
        self._pending_camera_mode = None
        if pending is not None:
            self._submit_camera_mode(*pending)

    def _begin_patrol_loop(self, start_point_id: int = 0) -> None:
        if not self._route or not self._is_active():
            return
        loop_number = int(self._state['current_loop'])
        reverse = bool(self._state['bidirectional'] and loop_number % 2 == 0)
        self._state.update({
            'state': 'starting_route', 'reverse': reverse,
            'message': f'正在启动第 {loop_number} 轮巡检路线',
            'interval_remaining_seconds': 0,
            'pending_start_point_id': int(start_point_id),
        })
        self._start_route(
            purpose='patrol', reverse=reverse,
            hold=True, start_point_id=start_point_id, point_limit=0)
        self._persist()

    def _begin_reposition(self) -> None:
        if not self._route:
            self._fail_task('返程复位失败：路线不可用')
            return
        first_point_id = int(self._route['points'][0]['id'])
        self._state.update({
            'state': 'returning_to_start', 'reverse': False,
            'current_direction': '',
            'message': '本轮完成，正在直接返回路线起点复位',
        })
        self._start_route(
            purpose='reposition', reverse=False, hold=False,
            start_point_id=first_point_id, point_limit=1)
        self._persist()

    def _start_route(
            self, purpose: str, reverse: bool, hold: bool,
            start_point_id: int, point_limit: int) -> None:
        # The route process may have restarted and reset its event counter.
        # Baseline the currently observed publisher, not the previous session.
        self._route_completion_event_floor = int(getattr(
            getattr(self, '_route_status', None), 'goal_event_sequence', 0))
        self._pending_route_start = {
            'purpose': purpose, 'reverse': bool(reverse), 'hold': bool(hold),
            'start_point_id': int(start_point_id),
            'point_limit': int(point_limit),
            'queued_monotonic': time.monotonic(),
        }
        self._dispatch_pending_route_request()

    def _call_route_control(self, command: str, purpose: str) -> None:
        pending = {
            'command': command, 'purpose': purpose,
            'queued_monotonic': time.monotonic(),
        }
        if command == 'stop' or self._pending_route_control is None:
            self._pending_route_control = pending
        self._dispatch_pending_route_request()

    def _dispatch_pending_route_request(self) -> None:
        if self._route_request_future is not None:
            return
        if self._pending_route_start is not None:
            if not self._route_start_client.service_is_ready():
                self._state['message'] = '等待路线导航服务就绪'
                return
            pending = self._pending_route_start
            self._pending_route_start = None
            request = StartRoute.Request()
            request.route_id = int(self._state['route_id'])
            request.loop_count = 1
            request.continuous = False
            request.reverse = bool(pending['reverse'])
            request.hold_at_each_point = bool(pending['hold'])
            request.owner_token = str(self._state['task_id'])
            request.start_point_id = int(pending['start_point_id'])
            request.point_limit = int(pending['point_limit'])
            request.navigation_mode = str(
                self._state.get('navigation_mode') or TERRAIN_3D)
            request.preview_id = ''
            request.authorization_generation = (
                self._3d_authorization_generation
                if (request.navigation_mode == TERRAIN_3D
                    and (self._task or {}).get('request', {}).get(
                        'previewed_3d')) else 0)
            request.request_deadline_unix_ns = (
                time.time_ns() + 55_000_000_000
                if (request.navigation_mode == TERRAIN_3D
                    and (self._task or {}).get('request', {}).get(
                        'previewed_3d')) else 0)
            self._route_request_purpose = f'start:{pending["purpose"]}'
            self._route_request_future = self._route_start_client.call_async(
                request)
            self._route_request_started_monotonic = time.monotonic()
            return
        if self._pending_route_control is not None:
            if not self._route_control_client.service_is_ready():
                self._state['message'] = '等待路线控制服务就绪'
                return
            pending = self._pending_route_control
            self._pending_route_control = None
            request = ControlRoute.Request()
            request.command = str(pending['command'])
            request.owner_token = str(self._state.get('task_id', ''))
            request.authorization_generation = (
                self._3d_authorization_generation
                if self._uses_previewed_3d_route() else 0)
            self._route_request_purpose = (
                f'control:{pending["purpose"]}:{pending["command"]}')
            self._route_request_future = self._route_control_client.call_async(
                request)
            self._route_request_started_monotonic = time.monotonic()

    def _drive_route_request_future(self) -> None:
        future = self._route_request_future
        active_timeout = (
            max(self._route_service_timeout_sec, 65.0)
            if (self._route_request_purpose.startswith('start:')
                and self._state.get('navigation_mode') == TERRAIN_3D
                and (self._task or {}).get('request', {}).get('previewed_3d'))
            else self._route_service_timeout_sec)
        if future is None or not future.done():
            if future is None:
                now = time.monotonic()
                if (
                    self._pending_route_start is not None
                    and now - float(self._pending_route_start.get(
                        'queued_monotonic', now))
                    >= self._route_service_timeout_sec
                ):
                    pending = self._pending_route_start
                    self._pending_route_start = None
                    self._apply_route_request_result(
                        f'start:{pending["purpose"]}', False,
                        '路线启动服务未就绪')
                if (
                    self._pending_route_control is not None
                    and now - float(self._pending_route_control.get(
                        'queued_monotonic', now))
                    >= self._route_service_timeout_sec
                ):
                    pending = self._pending_route_control
                    self._pending_route_control = None
                    self._apply_route_request_result(
                        f'control:{pending["purpose"]}:{pending["command"]}',
                        False, '路线控制服务未就绪')
                self._dispatch_pending_route_request()
            elif (
                self._route_request_started_monotonic > 0.0
                and time.monotonic() - self._route_request_started_monotonic
                >= active_timeout
            ):
                purpose = self._route_request_purpose
                try:
                    future.cancel()
                except Exception:
                    pass
                self._route_request_future = None
                self._route_request_purpose = ''
                self._route_request_started_monotonic = 0.0
                self._apply_route_request_result(
                    purpose, False,
                    f'路线服务 {active_timeout:g} 秒未响应')
                self._dispatch_pending_route_request()
                self._persist()
            return
        purpose = self._route_request_purpose
        self._route_request_future = None
        self._route_request_purpose = ''
        self._route_request_started_monotonic = 0.0
        try:
            result = future.result()
            success = bool(result.success)
            message = str(result.message)
        except Exception as exception:
            success = False
            message = str(exception)
        self._apply_route_request_result(purpose, success, message)
        self._dispatch_pending_route_request()
        self._persist()

    def _apply_route_request_result(
            self, purpose: str, success: bool, message: str) -> None:
        if purpose.startswith('start:'):
            if (self._uses_previewed_3d_route()
                    and self._state['state'] in (
                        'pausing', 'paused', 'cancelling')):
                self._route_session_active = bool(success)
                return
            route_purpose = purpose.split(':', 1)[1]
            if not success:
                self._fail_task(f'启动路线失败：{message}')
                self._call_route_control('stop', 'failed_start_cleanup')
                return
            self._route_session_active = True
            self._route_session_purpose = route_purpose
            self._processed_hold_key = ''
            self._state['pending_start_point_id'] = 0
            if self._state['state'] in TERMINAL_STATES:
                self._pending_route_control = {
                    'command': 'stop', 'purpose': 'cancel'}
            elif self._pause_event.is_set():
                self._pending_route_control = {
                    'command': 'pause', 'purpose': 'pause'}
                self._state['state'] = 'paused'
            else:
                self._state['state'] = (
                    'navigating' if route_purpose == 'patrol'
                    else 'returning_to_start')
                self._state['message'] = (
                    f'第 {self._state["current_loop"]} 轮巡检进行中'
                    if route_purpose == 'patrol' else '正在返回起点复位')
            # A very short route can finish before its Start service reply.
            # Consume that cached terminal event now that the session exists.
            status = getattr(self, '_route_status', None)
            if status is not None and str(status.state) == 'completed':
                self._route_status_callback(status)
        elif (not success and purpose == 'control:failed_start_cleanup:stop'
                and message == 'No route navigation task is active'):
            self._log('启动未成功，后台未创建活动路线；清理已结束，保留原始启动失败原因')
        elif not success and not purpose.startswith('control:cancel'):
            self._log(f'路线控制失败：{message}', 'warning')
        elif success and purpose.startswith('control:'):
            if self._state['state'] in TERMINAL_STATES:
                return
            command = purpose.rsplit(':', 1)[-1]
            if (command in ('release', 'skip') and not self._state.get('paused')
                    and self._route_session_active):
                self._state['state'] = 'navigating'
                self._state['message'] = '正在前往下一个巡检点'
            elif command == 'resume':
                self._state['message'] = '巡检任务已继续'

    def _adopt_recovered_route(self, message: RouteExecutionStatus) -> None:
        """Attach a respawned inspection manager to its surviving route."""
        if str(message.owner_token) != str(self._state.get('task_id', '')):
            return
        self._route_session_active = bool(message.active)
        self._route_completion_event_floor = int(getattr(message, 'goal_event_sequence', 0))
        resume_state = str(self._state.get('resume_state') or 'navigating')
        self._route_session_purpose = (
            'reposition' if resume_state == 'returning_to_start' else 'patrol')
        self._last_route_event_sequence = int(message.point_event_sequence)
        self._processed_hold_key = ''
        self._state['recovery_pending'] = False
        self._state['route_state'] = str(message.state)
        route_mode = str(
            getattr(message, 'navigation_mode', '') or TERRAIN_3D)
        frozen_mode = str(
            self._state.get('navigation_mode') or TERRAIN_3D)
        if route_mode != frozen_mode:
            self._pause_for_route_loss(
                f'路线导航模式 {route_mode} 与巡检冻结模式 {frozen_mode} 不一致')
            return
        self._state['message'] = '已接管重启前的路线，任务保持暂停'
        if str(message.state) not in (
                'paused_navigation', 'paused_dwell', 'paused_release',
                'paused_preflight'):
            self._call_route_control('pause', 'recover_pause')
        self._database.add_event(
            self._state['task_id'], 'route_recovered',
            '巡检后台已重新接管同一 owner 路线', level='warning')

    def _pause_for_route_loss(self, detail: str) -> None:
        if not self._is_active():
            return
        if self._uses_previewed_3d_route():
            self._state['resume_state'] = self._state['state']
            self._route_session_active = False
            self._route_session_purpose = ''
            self._state.update({
                'state': 'pausing', 'paused': True,
                'recovery_pending': True,
                'message': f'路线服务中断，正在确认 3D 导航停止：{detail}',
            })
            self._pause_event.set()
            if self._nav_cancel_client.service_is_ready():
                self._nav_cancel_client.call_async(CancelGoal.Request())
            self._submit_camera_mode('paused', 'pause')
            self._schedule_3d_revoke('pause')
            return
        if self._state['state'] != 'paused':
            self._state['resume_state'] = self._state['state']
        self._route_session_active = False
        self._route_session_purpose = ''
        self._state.update({
            'state': 'paused', 'paused': True, 'recovery_pending': True,
            'message': f'路线服务中断，任务已安全暂停：{detail}',
        })
        self._pause_event.set()
        if self._nav_cancel_client.service_is_ready():
            # A zero GoalInfo means "cancel all". This also cancels a Nav2
            # goal whose route-manager goal handle was lost during a crash.
            self._nav_cancel_client.call_async(CancelGoal.Request())
        self._submit_camera_mode('paused', 'pause')
        self._database.add_event(
            self._state['task_id'], 'route_lost', detail, level='warning')
        self._log(f'路线服务中断，巡检任务已安全暂停：{detail}', 'warning')

    def _pause_for_preflight_timeout(self, detail: str) -> None:
        """Keep the task and current point when a safety preflight times out."""
        if not self._is_active() or self._state.get('paused'):
            return
        if self._uses_previewed_3d_route():
            self._state['resume_state'] = self._state['state']
            self._state.update({
                'state': 'pausing', 'paused': True,
                'message': f'安全检查超时，正在确认 3D 导航暂停：{detail}',
            })
            self._pause_event.set()
            self._submit_camera_mode('paused', 'pause')
            self._schedule_3d_revoke('pause')
            return
        self._state['resume_state'] = self._state['state']
        self._state.update({
            'state': 'paused',
            'paused': True,
            'message': f'导航安全检查等待超时，任务已暂停：{detail}',
        })
        self._pause_event.set()
        self._submit_camera_mode('paused', 'pause')
        self._database.add_event(
            self._state['task_id'],
            'navigation_preflight_paused',
            detail,
            level='warning',
        )
        self._log(
            f'导航安全检查等待超时，当前巡检点已保留：{detail}',
            'warning',
        )

    def _route_status_callback(self, message: RouteExecutionStatus) -> None:
        with self._lock:
            self._route_status = message
            if not self._is_active():
                return
            if self._state['state'] in ('pausing', 'cancelling'):
                # The independent revoke service owns this transition; a
                # stale route-status sample must not acknowledge a stop.
                return
            if (
                self._route_session_active
                and not message.active
                and str(message.state) != 'completed'
            ):
                self._pause_for_route_loss(
                    str(message.message or message.state or '路线节点已重置'))
                self._persist()
                self._publish(force=True)
                return
            completed = (self._route_session_active and not message.active
                and str(message.state) == 'completed'
                and (str(message.owner_token) == str(self._state['task_id']) or (
                    str(getattr(message, 'last_goal_owner_token', '')) == str(self._state['task_id'])
                    and str(getattr(message, 'last_goal_result', '')) == 'route_completed'
                    and int(getattr(message, 'goal_event_sequence', 0))
                        > getattr(self, '_route_completion_event_floor', 0))))
            if str(message.owner_token) != str(self._state['task_id']) and not completed:
                if self._state.get('recovery_pending') and bool(message.active):
                    self._state['message'] = '路线正由其他任务占用，等待操作员处理'
                    self._persist()
                    self._publish(force=True)
                return
            if self._state.get('recovery_pending') and bool(message.active):
                self._adopt_recovered_route(message)
            self._state['route_state'] = str(message.state)
            if (
                bool(message.active)
                and bool(message.paused)
                and str(message.state) == 'paused_preflight'
            ):
                self._pause_for_preflight_timeout(
                    str(message.message or '定位或地形状态未能及时刷新'))
            if (
                self._state.get('paused') and bool(message.active)
                and not bool(message.paused)
                and self._route_request_future is None
                and self._pending_route_control is None
            ):
                self._state['message'] = '等待路线确认暂停'
                self._call_route_control('pause', 'enforce_pause')
            point = message.current_point
            if int(point.id) > 0:
                self._state.update({
                    'current_point_id': int(point.id),
                    'current_point_index': int(point.sequence),
                    'current_point_name': str(point.name),
                })
            event_sequence = int(message.point_event_sequence)
            if event_sequence > self._last_route_event_sequence:
                self._last_route_event_sequence = event_sequence
                self._handle_route_point_event(message)
            if message.awaiting_release and str(message.state) == 'awaiting_release':
                self._handle_point_hold(message)
            if completed:
                self._route_completion_event_floor = max(
                    getattr(self, '_route_completion_event_floor', 0),
                    int(getattr(message, 'goal_event_sequence', 0)))
                purpose = self._route_session_purpose
                self._route_session_active = False
                self._route_session_purpose = ''
                if purpose == 'patrol':
                    self._finish_patrol_loop()
                elif purpose == 'reposition':
                    self._enter_interval_wait()
            self._persist()
            self._publish()

    def _handle_route_point_event(self, message: RouteExecutionStatus) -> None:
        result = str(message.last_point_result)
        if result not in ('failed', 'skipped'):
            return
        point_id = int(message.last_point_id)
        self._database.add_point_result(self._state['task_id'], {
            'loop_number': self._state['current_loop'],
            'route_point_id': point_id,
            'point_sequence': int(message.last_point_index),
            'point_name': str(message.last_point_name),
            'result': 'arrival_failed' if result == 'failed' else 'skipped',
            'detail': str(message.last_point_detail),
        })
        self._state['completed_points'] += 1
        self._state['point_statuses'][str(point_id)] = (
            'failed' if result == 'failed' else 'skipped')
        self._state['previous_point_id'] = point_id
        if result == 'failed':
            self._state['failed_points'] += 1
            self._log(
                f'目标点“{message.last_point_name}”到达失败，继续下一个点',
                'warning')
        else:
            self._state['skipped_points'] += 1

    def _handle_point_hold(self, message: RouteExecutionStatus) -> None:
        if self._state.get('paused'):
            return
        point = message.current_point
        hold_key = (
            f'{self._state["current_loop"]}:{int(point.id)}:'
            f'{int(message.point_event_sequence)}')
        if hold_key == self._processed_hold_key:
            return
        self._processed_hold_key = hold_key
        checkpoint = self._checkpoints.get(int(point.id))
        self._state['state'] = 'point_hold'
        if checkpoint is None:
            self._record_completed_point(point, 'completed', '普通巡检点已到达')
            self._state['state'] = 'releasing_point'
            self._state['message'] = f'普通点“{point.name}”已完成，继续下一点'
            self._call_route_control('release', 'release')
            return
        if self._inspection_future is not None:
            return
        self._state.update({
            'state': 'inspecting', 'current_direction': '',
            'message': f'正在检测“{point.name}”',
        })
        point_data = {
            'id': int(point.id), 'sequence': int(point.sequence),
            'name': str(point.name), 'x': float(point.x), 'y': float(point.y),
        }
        self._skip_event.clear()
        self._inspection_future = self._workers.submit(
            self._run_checkpoint, point_data, dict(checkpoint),
            int(self._state['current_loop']))
        self._log(f'开始检测点“{point.name}”的固定视角精检')

    def _expanded_directions(self, directions: Iterable[str]) -> List[str]:
        source = list(directions)
        if not source:
            return ['current']
        expanded: List[str] = []
        for direction in source:
            values = FULL_SCAN_DIRECTIONS if direction == 'full' else (direction,)
            for value in values:
                if value not in expanded:
                    expanded.append(value)
        return expanded or ['current']

    def _run_checkpoint(
            self, point: Dict[str, Any], checkpoint: Dict[str, Any],
            loop_number: int) -> Dict[str, Any]:
        task_id = str(self._state['task_id'])
        evidence_dir = (
            self._evidence_root / task_id / f'loop_{loop_number:04d}' /
            f'point_{point["id"]}')
        evidence_dir.mkdir(parents=True, exist_ok=True)
        failures = []
        completed_directions = 0
        temperature_detections: List[Dict[str, Any]] = []
        try:
            self._camera.set_mode('inspection')
            while self._pause_event.is_set():
                if self._cancel_event.is_set() or self._skip_event.is_set():
                    break
                time.sleep(0.1)
            if not self._cancel_event.is_set() and not self._skip_event.is_set():
                with self._lock:
                    self._state['current_direction'] = 'current'
                    self._state['message'] = (
                        f'检测“{point["name"]}”：固定视角稳定中')
                    self._persist()
                if _interruptible_wait(
                        1.0, self._pause_event, self._cancel_event,
                        self._skip_event):
                    with self._lock:
                        self._state['message'] = (
                            f'检测“{point["name"]}”：固定视角精检中')
                        self._persist()
                    capture = self._camera.capture({
                        'duration_seconds': 2.0,
                        'task_id': task_id,
                        'loop_number': loop_number,
                        'route_point_id': int(point['id']),
                        'point_name': str(point['name']),
                        'direction': 'current',
                        'save_images': bool(checkpoint.get('save_images')),
                        'evidence_dir': str(evidence_dir),
                    })
                    detections = _confirmed_detections(
                        capture.get('detections') or [])
                    temperature_detections = [
                        dict(detection) for detection in detections
                        if str(detection.get('measurement_type'))
                        == 'temperature']
                    raw_image = str(capture.get('raw_image') or '')
                    annotated_image = str(capture.get('annotated_image') or '')
                    thermal_raw_image = str(
                        capture.get('thermal_raw_image') or '')
                    thermal_annotated_image = str(
                        capture.get('thermal_annotated_image') or '')
                    self._database.add_point_result(task_id, {
                        'loop_number': loop_number,
                        'route_point_id': int(point['id']),
                        'point_sequence': int(point['sequence']),
                        'point_name': str(point['name']),
                        'direction': 'current',
                        'result': 'completed',
                        'detail': _behavior_capture_detail(capture),
                        'raw_image': raw_image,
                        'annotated_image': annotated_image,
                        'thermal_raw_image': thermal_raw_image,
                        'thermal_annotated_image': thermal_annotated_image,
                        'thermal_measurement': dict(capture.get('thermal') or {}),
                    })
                    for detection in detections:
                        category = str(detection.get('category') or 'unknown')
                        alert_id = self._database.upsert_alert(task_id, {
                            'dedup_key': (
                                f'checkpoint:{loop_number}:{point["id"]}:'
                                f'current:{category}'),
                            'loop_number': loop_number,
                            'route_point_id': int(point['id']),
                            'point_name': str(point['name']),
                            'direction': 'current',
                            'category': category,
                            **_alert_storage_fields(detection, {
                                'raw_image': raw_image,
                                'annotated_image': annotated_image,
                                'thermal_raw_image': thermal_raw_image,
                                'thermal_annotated_image': thermal_annotated_image,
                            }),
                            'map_x': point.get('x'), 'map_y': point.get('y'),
                        })
                        self._log(
                            f'检测到异常：{category} '
                            f'{_detection_summary(detection)}，'
                            f'告警 #{alert_id}', 'warning')
                    completed_directions = 1
        except Exception as error:
            failures.append(str(error))
            self._database.add_point_result(task_id, {
                'loop_number': loop_number,
                'route_point_id': int(point['id']),
                'point_sequence': int(point['sequence']),
                'point_name': str(point['name']),
                'direction': 'current',
                'result': 'camera_failed', 'detail': str(error),
            })
        finally:
            try:
                self._camera.set_mode(
                    'paused' if self._pause_event.is_set() else 'transit')
            except Exception as error:
                failures.append(f'切回途中检测失败: {error}')
        return {
            'point': point, 'completed_directions': completed_directions,
            'direction_count': 1, 'failures': failures,
            'temperature_detections': temperature_detections,
            'skipped': self._skip_event.is_set(),
            'cancelled': self._cancel_event.is_set(),
        }

    def _drive_inspection_future(self) -> None:
        future = self._inspection_future
        if future is None or not future.done():
            return
        # Capture cannot be interrupted safely.  Keep its completed Future
        # pending until resume so the result cannot overwrite paused state.
        if self._state.get('paused') or self._state['state'] == 'paused':
            return
        self._inspection_future = None
        try:
            result = future.result()
        except Exception as exception:
            result = {
                'point': {
                    'id': self._state.get('current_point_id', 0),
                    'sequence': self._state.get('current_point_index', -1),
                    'name': self._state.get('current_point_name', ''),
                },
                'failures': [str(exception)], 'skipped': False,
                'cancelled': self._cancel_event.is_set(),
            }
        if result.get('cancelled') or self._state['state'] == 'cancelled':
            return
        point = result['point']
        failures = list(result.get('failures') or [])
        for detection in result.get('temperature_detections') or []:
            self._handle_thermal_environment_detection(detection)
        if result.get('skipped'):
            self._state['message'] = f'检测点“{point["name"]}”已跳过'
            # RouteExecutionStatus is the single source of truth for skipped
            # point accounting; its point event records and counts the skip.
            self._call_route_control('skip', 'skip_inspection')
        else:
            self._state['completed_points'] += 1
            self._state['completed_detection_points'] += 1
            self._state['message'] = (
                f'检测点“{point["name"]}”已完成'
                if not failures else
                f'检测点“{point["name"]}”部分失败，已记录并继续')
            self._state['point_statuses'][str(point['id'])] = (
                'failed' if failures else 'completed')
            self._state['previous_point_id'] = int(point['id'])
            if failures:
                self._database.add_event(
                    self._state['task_id'], 'checkpoint_partial_failure',
                    self._state['message'], level='warning',
                    data={'errors': failures})
            self._call_route_control('release', 'release_inspection')
        self._skip_event.clear()
        self._state['state'] = 'releasing_point'
        self._state['current_direction'] = ''
        self._persist()
        self._publish_alerts(force=True)

    def _record_completed_point(self, point, result: str, detail: str) -> None:
        self._database.add_point_result(self._state['task_id'], {
            'loop_number': self._state['current_loop'],
            'route_point_id': int(point.id),
            'point_sequence': int(point.sequence),
            'point_name': str(point.name),
            'result': result, 'detail': detail,
        })
        self._state['completed_points'] += 1
        self._state['point_statuses'][str(int(point.id))] = result
        self._state['previous_point_id'] = int(point.id)

    def _finish_patrol_loop(self) -> None:
        if self._state.get('paused') or self._state['state'] == 'paused':
            self._state['deferred_route_completion'] = 'patrol'
            self._state['message'] = '本轮路线已完成，任务保持暂停'
            self._persist()
            return
        current_loop = int(self._state['current_loop'])
        self._database.add_event(
            self._state['task_id'], 'loop_completed',
            f'第 {current_loop} 轮巡检完成',
            data={'loop_number': current_loop})
        if current_loop >= int(self._state['loop_count']):
            self._complete_task()
            return
        if self._state['bidirectional']:
            self._enter_interval_wait()
        else:
            self._begin_reposition()

    def _enter_interval_wait(self) -> None:
        if self._state.get('paused') or self._state['state'] == 'paused':
            self._state['deferred_route_completion'] = 'reposition'
            self._state['message'] = '返程复位已完成，任务保持暂停'
            self._persist()
            return
        interval_seconds = (
            float(self._state['loop_interval_minutes']) * self._interval_scale)
        self._state.update({
            'state': 'interval_wait',
            'interval_deadline_monotonic': time.monotonic() + interval_seconds,
            'interval_deadline_epoch': time.time() + interval_seconds,
            'interval_remaining_seconds': int(round(interval_seconds)),
            'message': '轮次间隔等待中', 'current_direction': '',
        })
        self._persist()

    def _drive_interval_wait(self) -> None:
        if self._state['state'] != 'interval_wait':
            return
        deadline = float(self._state.get('interval_deadline_monotonic', 0.0))
        remaining = max(0.0, deadline - time.monotonic())
        remaining_seconds = int(round(remaining))
        previous_seconds = int(
            self._state.get('interval_remaining_seconds', remaining_seconds))
        self._state['interval_remaining_seconds'] = remaining_seconds
        if remaining_seconds != previous_seconds:
            self._persist()
        if remaining > 0.0:
            return
        self._state['current_loop'] = int(self._state['current_loop']) + 1
        self._state['point_statuses'] = {}
        self._state['previous_point_id'] = 0
        self._begin_patrol_loop()

    def _complete_task(self) -> None:
        task_id = self._state['task_id']
        self._state.update({
            'state': 'completed', 'outcome': 'completed', 'active': False,
            'paused': False, 'message': '巡检任务已完成',
            'ended_at': utc_now(), 'progress_percent': 100.0,
            'estimated_remaining_seconds': 0,
        })
        self._database.add_event(task_id, 'task_completed', '巡检任务已完成')
        self._database.update_task(task_id, self._state, 'completed')
        self._submit_camera_mode('paused', 'complete')
        self._log('巡检任务全部完成')

    def _fail_task(
            self, message: str, *,
            authorization_revoked: bool = False) -> None:
        if (self._state['state'] == 'cancelling'
                and getattr(self, '_pending_3d_revoke', None)
                and self._pending_3d_revoke['terminal'] == 'cancelled'):
            return
        if (self._uses_previewed_3d_route()
                and not authorization_revoked
                and self._state['state'] not in TERMINAL_STATES):
            self._pending_route_start = None
            self._cancel_event.set()
            self._skip_event.set()
            self._pause_event.set()
            self._state.update({
                'state': 'cancelling', 'paused': True,
                'message': (
                    f'{message}；正在撤销 3D 路线授权并等待导航停止确认'),
            })
            self._submit_camera_mode('paused', 'complete')
            self._schedule_3d_revoke(
                'cancel', terminal='failed', reason=message)
            return
        task_id = str(self._state.get('task_id') or '')
        self._state.update({
            'state': 'failed', 'outcome': 'failed', 'active': False,
            'paused': False, 'message': message, 'ended_at': utc_now(),
        })
        if task_id:
            self._database.add_event(
                task_id, 'task_failed', message, level='error')
            self._database.update_task(task_id, self._state, 'failed')
        self._submit_camera_mode('paused', 'complete')
        self._log(message, 'error')
        if authorization_revoked:
            if self._route_request_future is not None:
                try:
                    self._route_request_future.cancel()
                except Exception:
                    pass
            self._route_request_future = None
            self._route_request_purpose = ''
            self._route_request_started_monotonic = 0.0
            self._pending_route_control = None

    def _drive_camera_events(self) -> None:
        if (
            not self._is_active() or self._state.get('paused')
            or self._state['state'] == 'inspecting'
        ):
            return
        now = time.monotonic()
        if self._event_future is None and now >= self._event_poll_due:
            self._event_future = self._workers.submit(
                self._camera.events, self._last_camera_event_sequence)
            self._event_poll_due = now + 0.5
            return
        if self._event_future is None or not self._event_future.done():
            return
        future = self._event_future
        self._event_future = None
        try:
            result = future.result()
        except Exception:
            self._event_poll_due = now + 2.0
            return
        for event in result.get('events', []):
            self._handle_transit_detection(event)
            self._last_camera_event_sequence = max(
                self._last_camera_event_sequence,
                int(event.get('sequence', 0)))

    def _handle_transit_detection(self, event: Dict[str, Any]) -> None:
        if str(event.get('mode')) != 'transit':
            return
        if event.get('task_id') and str(event['task_id']) != str(self._state['task_id']):
            return
        point_id = int(self._state.get('current_point_id') or 0)
        point_name = str(self._state.get('current_point_name') or '路线途中')
        previous_id = int(self._state.get('previous_point_id') or 0)
        segment = f'{previous_id}->{point_id}'
        window = int(float(event.get('timestamp_epoch', time.time())) // 30)
        detections = _confirmed_detections(event.get('detections') or [])
        for detection in detections:
            if str(detection.get('measurement_type')) == 'temperature':
                self._handle_thermal_environment_detection(detection)
            category = str(detection.get('category') or 'unknown')
            event_key = str(detection.get('event_key') or '')
            if event_key and str(detection.get('measurement_type')) == 'temperature':
                dedup_key = f'transit:{event_key}'
            else:
                dedup_key = (
                    f'transit:{self._state["current_loop"]}:{segment}:'
                    f'{category}:{window}')
            alert_id = self._database.upsert_alert(self._state['task_id'], {
                'dedup_key': dedup_key,
                'loop_number': self._state['current_loop'],
                'route_point_id': point_id,
                'point_name': '路线途中', 'segment_key': segment,
                'category': category,
                **_alert_storage_fields(detection, event),
                'map_x': (self._latest_pose or {}).get('x'),
                'map_y': (self._latest_pose or {}).get('y'),
            })
            self._log(
                f'路线途中检测到异常：{category} '
                f'{_detection_summary(detection)}，告警 #{alert_id}',
                'warning')
        if detections:
            self._publish_alerts(force=True)

    def _handle_thermal_environment_detection(
            self, detection: Dict[str, Any]) -> None:
        active = self._is_active() and not bool(self._state.get('paused'))
        settings = getattr(self, '_detection_settings', None)
        active = active and (settings is None or settings.enabled('thermal'))
        if self._environment_policy.observe_thermal(
                detection, active=active, now=time.monotonic()):
            self._queue_environment_speech('thermal_high')

    def _drive_environment_alerts(self) -> None:
        settings = getattr(self, '_detection_settings', None)
        if settings is not None and not settings.allows('co2_high'):
            self._environment_policy.observe_co2(None, valid=False, active=False, now=time.monotonic())
            self._last_environment_sample_sequence = self._m701_sample_sequence
            self._publish_environment_config()
            return
        active = self._is_active() and not bool(self._state.get('paused'))
        now = time.monotonic()
        if not active:
            self._environment_policy.reset()
            self._last_environment_sample_sequence = self._m701_sample_sequence
            self._publish_environment_config()
            return
        payload = self._latest_m701 or {}
        age = (
            now - self._latest_m701_received_monotonic
            if self._latest_m701_received_monotonic > 0.0 else math.inf)
        values = payload.get('values')
        valid = bool(
            payload.get('online') is True
            and payload.get('valid') is True
            and payload.get('warming_up') is not True
            and isinstance(values, dict)
            and age <= self._environment_sample_max_age)
        co2_ppm = values.get('co2_ppm') if isinstance(values, dict) else None
        if self._m701_sample_sequence != self._last_environment_sample_sequence:
            self._last_environment_sample_sequence = self._m701_sample_sequence
            for key in self._environment_policy.observe_co2(
                    co2_ppm, valid=valid, active=True, now=now):
                self._queue_environment_speech(key)
        elif not valid and self._environment_policy.co2_sample_valid:
            self._environment_policy.observe_co2(
                None, valid=False, active=True, now=now)
        self._publish_environment_config()

    def _queue_environment_speech(self, key: str) -> None:
        settings = getattr(self, '_detection_settings', None)
        if settings is not None and not settings.allows(key):
            return
        if key in self._environment_speech_pending:
            return
        if key == self._environment_speech_active_key:
            return
        self._environment_speech_pending.add(key)
        self._environment_speech_queue.append(key)

    def _drive_environment_speech(self) -> None:
        future = self._environment_speech_future
        if future is not None and not future.done():
            return
        if future is not None:
            key = self._environment_speech_active_key
            self._environment_speech_future = None
            self._environment_speech_active_key = ''
            try:
                future.result()
            except Exception as error:
                self._log(f'环境异常语音播报失败（{key}）：{error}', 'warning')
        if not self._environment_speech_queue:
            return
        key = self._environment_speech_queue.popleft()
        self._environment_speech_pending.discard(key)
        settings = getattr(self, '_detection_settings', None)
        if settings is not None and not settings.allows(key):
            return
        self._environment_speech_active_key = key
        self._environment_speech_future = self._workers.submit(
            self._camera.speak_environment_alert, key)

    def _environment_status_payload(self) -> Dict[str, Any]:
        config = self._environment_config.snapshot()
        policy = self._environment_policy.snapshot()
        active = self._is_active() and not bool(self._state.get('paused'))
        received = self._latest_m701_received_monotonic
        age = (
            max(0.0, time.monotonic() - received)
            if received > 0.0 else None)
        sample_fresh = age is not None and age <= self._environment_sample_max_age
        settings = getattr(self, '_detection_settings', None)
        co2_enabled = settings is None or settings.allows('co2_high')
        thermal_enabled = settings is None or settings.enabled('thermal')
        return {
            **config,
            **policy,
            'minimum_co2_threshold_ppm': MIN_CO2_THRESHOLD_PPM,
            'maximum_co2_threshold_ppm': MAX_CO2_THRESHOLD_PPM,
            'active_inspection': active,
            'co2_enabled': co2_enabled,
            'thermal_enabled': thermal_enabled,
            'sample_fresh': sample_fresh,
            'sample_age_seconds': int(age) if age is not None else None,
            'speech_pending': self._environment_speech_future is not None
            or bool(self._environment_speech_queue),
            'message': (
                '二氧化碳超限告警已在设置页关闭' if not co2_enabled else
                '巡检中，环境异常语音告警已启用'
                if active else '阈值已保存；启动巡检后启用语音告警'),
        }

    def _publish_environment_config(self, force: bool = False) -> None:
        payload = _json(self._environment_status_payload())
        if force or payload != self._last_environment_payload:
            self._last_environment_payload = payload
            self._environment_config_publisher.publish(String(data=payload))

    def _update_estimate(self) -> None:
        total = max(0, int(self._state.get('total_points', 0)))
        complete = min(total, max(0, int(self._state.get('completed_points', 0))))
        self._state['progress_percent'] = (
            round(complete * 100.0 / total, 1) if total else 0.0)
        remaining_points = max(0, total - complete)
        interval_remaining = int(self._state.get('interval_remaining_seconds', 0))
        remaining_loops = max(
            0, int(self._state.get('loop_count', 0))
            - int(self._state.get('current_loop', 0)))
        future_intervals = max(
            0, remaining_loops - (1 if self._state['state'] == 'interval_wait' else 0))
        interval_seconds = int(round(
            float(self._state.get('loop_interval_minutes', 0.0))
            * self._interval_scale))
        self._state['estimated_remaining_seconds'] = (
            remaining_points * 30 + interval_remaining
            + future_intervals * interval_seconds)
        self._state['updated_at'] = utc_now()

    def _update_storage_status(self) -> None:
        now = time.monotonic()
        if now < self._storage_check_due:
            return
        self._storage_check_due = now + 10.0
        try:
            usage = shutil.disk_usage(self._evidence_root)
        except OSError as error:
            self._state['storage_error'] = str(error)
            return
        warning = usage.free < self._storage_warning_free_bytes
        self._state.update({
            'storage_free_bytes': int(usage.free),
            'storage_total_bytes': int(usage.total),
            'storage_warning': warning,
            'storage_error': '',
        })
        if warning and not self._storage_warning_active:
            self._log(
                f'巡检图片存储空间不足：剩余 {usage.free / 1024 ** 3:.1f} GB，'
                '请删除不再需要的巡检历史', 'warning')
        self._storage_warning_active = warning

    def _status_payload(self) -> Dict[str, Any]:
        payload = dict(self._state)
        payload.pop('interval_deadline_monotonic', None)
        payload.pop('resume_state', None)
        payload['nav2_ready'] = self._nav2_ready
        payload['localization_ready'] = self._localization_ready
        payload['localized_map_name'] = self._localized_map_name
        payload['selected_navigation_mode'] = self._selected_navigation_mode
        payload['navigation_mode_seen'] = self._navigation_mode_seen
        return payload

    def _persist(self) -> None:
        if self._state.get('task_id') and self._state.get('active'):
            self._database.update_task(self._state['task_id'], self._state)

    def _publish(self, force: bool = False) -> None:
        payload = _json(self._status_payload())
        if force or payload != self._last_publish_payload:
            self._last_publish_payload = payload
            self._status_publisher.publish(String(data=payload))
        self._publish_alerts(force=force)

    def _publish_alerts(self, force: bool = False) -> None:
        task_id = str(self._state.get('task_id') or '')
        alerts = self._database.get_alerts(task_id) if task_id else []
        self._state['unacknowledged_alerts'] = sum(
            1 for item in alerts if not int(item.get('acknowledged', 0)))
        payload = _json({'task_id': task_id, 'alerts': alerts})
        if force or payload != self._last_alert_payload:
            self._last_alert_payload = payload
            self._alerts_publisher.publish(String(data=payload))

    def _log(self, message: str, level: str = 'info') -> None:
        self._log_publisher.publish(String(data=_json({
            'time': datetime.now().astimezone().isoformat(timespec='seconds'),
            'level': level, 'message': message,
        })))
        logger = self.get_logger()
        # rclpy binds a severity to each Python call site. Keep the three
        # severities on separate source lines so changing event level cannot
        # terminate the node with "Logger severity cannot be changed".
        if level == 'warning':
            logger.warning(message)
        elif level == 'error':
            logger.error(message)
        else:
            logger.info(message)

    def destroy_node(self):
        with self._lock:
            pending = getattr(self,'_pending_task_start',None)
            if pending is not None:
                self._abandon_task_start(pending)
        self._cancel_event.set()
        self._workers.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InspectionTaskManager()
    executor = MultiThreadedExecutor(num_threads=6)
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
