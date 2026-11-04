#!/usr/bin/env python3
"""Start and stop the mutually-exclusive mapping and navigation launch trees.

This node deliberately owns only the two launch trees that contain the shared
LiDAR/LIO/TF stack.  It is kept separate from ``launch_manager.launch.py`` so
the web backend stays alive while a user moves between the mapping and
navigation pages.
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
)
from rclpy.node import Node

from lzp_interface.msg import LaunchStatus
from lzp_interface.srv import ControlLaunch
from .saved_map_lock import saved_map_write_busy


class LaunchManagerService(Node):
    """Own the mapping and navigation launch processes for the web UI."""

    # Both launch files are intentionally in the same package.  Keeping this
    # whitelist here prevents the rosbridge-facing service from launching an
    # arbitrary package or executable.
    LAUNCH_FILES = {
        'mapping': 'mapping.launch.py',
        'car_vel': 'car_vel.launch.py',
    }
    MODE_ALIASES = {
        'mapping': 'mapping',
        'map': 'mapping',
        'car_vel': 'car_vel',
        'navigation': 'car_vel',
        'nav': 'car_vel',
    }
    # Only packages owned by this source workspace belong here. Dependencies
    # such as Livox may intentionally come from /opt/ros or a separately built
    # vendor overlay.  The locally customised OctoMap filter is required here.
    MODE_PACKAGES = {
        'mapping': (
            'car_vel', 'lzp_interface', 'lzp_service', 'small_point_lio',
            'octomap_server2', 'robot_navigation2',
        ),
        'car_vel': (
            'car_vel', 'lzp_interface', 'lzp_service', 'small_point_lio',
            'icp_registration', 'robot_navigation2',
        ),
    }
    MODE_RUNTIME_PACKAGES = {
        'mapping': ('livox_ros_driver2',),
        'car_vel': ('livox_ros_driver2', 'pointcloud_to_laserscan'),
    }
    GLOBAL_MAPPING_CONTRACT = 'global_mapping_cloud_filter_modes_v4'
    MAPPING_FILTER_CONTRACT = 'mapping_cloud_filter_modes_v3'

    def __init__(self):
        super().__init__('launch_manager_service')

        self.declare_parameter('launch_package', 'car_vel')
        self.declare_parameter('mode_switch_delay_sec', 1.0)
        self.declare_parameter('startup_check_sec', 0.5)
        self.declare_parameter('shutdown_timeout_sec', 8.0)
        self.declare_parameter('force_kill_timeout_sec', 2.0)
        self.declare_parameter('restart_delay_sec', 1.0)
        self.declare_parameter('fastdds_shm_cleanup_enabled', True)
        self.declare_parameter('fastdds_shm_cleanup_timeout_sec', 3.0)
        self.declare_parameter('fastdds_force_udp', True)
        self.declare_parameter('fastdds_udp_fallback_enabled', True)
        self.declare_parameter('expected_workspace', '/home/a/3dnav')
        self.declare_parameter('mapping_dynamic_filter_mode', 'protected')
        self.declare_parameter('mapping_filter_config_path', '')

        self.launch_package = self._string_parameter('launch_package', 'car_vel')
        self.mode_switch_delay_sec = self._seconds_parameter('mode_switch_delay_sec', 1.0)
        self.startup_check_sec = self._seconds_parameter('startup_check_sec', 0.5)
        self.shutdown_timeout_sec = self._seconds_parameter('shutdown_timeout_sec', 8.0)
        self.force_kill_timeout_sec = self._seconds_parameter('force_kill_timeout_sec', 2.0)
        self.restart_delay_sec = self._seconds_parameter('restart_delay_sec', 1.0)
        self.fastdds_shm_cleanup_enabled = self._bool_parameter(
            'fastdds_shm_cleanup_enabled', True)
        self.fastdds_shm_cleanup_timeout_sec = self._seconds_parameter(
            'fastdds_shm_cleanup_timeout_sec', 3.0)
        self.fastdds_force_udp = self._bool_parameter(
            'fastdds_force_udp', True)
        self.fastdds_udp_fallback_enabled = self._bool_parameter(
            'fastdds_udp_fallback_enabled', True)
        self.expected_workspace = Path(self._string_parameter(
            'expected_workspace', '/home/a/3dnav'))

        # ``transition_lock`` serializes full mode transitions.  ``process_lock``
        # protects only the dictionary, so status publication never waits for a
        # process to terminate.  RLock also makes shutdown/restart safe when
        # helpers call each other.
        self.running_processes = {}
        self.process_lock = threading.RLock()
        self.transition_lock = threading.RLock()

        self.control_service = self.create_service(
            ControlLaunch, 'control_launch', self.handle_control_launch)
        self.status_publisher = self.create_publisher(
            LaunchStatus, 'launch_status', 10)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self._init_mapping_filter_settings()

        # Own the single body worker here, so body switches and data-plane
        # launches share one transition lock rather than racing across nodes.
        from lzp_service.robot_body_manager import RobotBodyManager
        self.robot_body_manager = RobotBodyManager(self)

        self.get_logger().info(
            'Launch manager started. Mapping and navigation are page-driven '
            'and mutually exclusive.')
        self._log_workspace_provenance()

    def _workspace_provenance(self):
        """Describe which overlay supplied this installed backend module."""
        module_path_text = str(Path(__file__).resolve())
        prefix_path = str(os.environ.get('AMENT_PREFIX_PATH', ''))
        separator = ';' if ';' in prefix_path else ':'
        prefixes = [value.strip() for value in prefix_path.split(separator)
                    if value.strip()]
        expected_workspace = str(
            self.expected_workspace.resolve()).rstrip('/\\')
        expected_install = expected_workspace + '/install'

        def normalise(value):
            try:
                return str(Path(value).resolve()).replace('\\', '/').rstrip('/')
            except (OSError, ValueError):
                return str(value).replace('\\', '/').rstrip('/')
        expected_workspace_normalised = normalise(expected_workspace)
        expected_install_normalised = normalise(expected_install)
        module_normalised = normalise(module_path_text)
        module_from_expected = (
            module_normalised == expected_workspace_normalised
            or module_normalised.startswith(
                expected_workspace_normalised + '/'))
        overlay_has_expected = any(
            normalise(value) == expected_install_normalised
            or normalise(value).startswith(expected_install_normalised + '/')
            for value in prefixes
        )
        stale_prefixes = [
            value for value in prefixes
            if normalise(value).startswith('/home/a/nav_ws')
            or normalise(value).startswith('C:/home/a/nav_ws')
        ]
        return {
            'module_path': module_path_text,
            'expected_install': expected_install,
            'module_from_expected': module_from_expected,
            'overlay_has_expected': overlay_has_expected,
            'stale_prefixes': stale_prefixes,
        }

    def _log_workspace_provenance(self):
        provenance = self._workspace_provenance()
        self.get_logger().info(
            'Workspace provenance: module=%s; expected_install=%s.' % (
                provenance['module_path'], provenance['expected_install']))
        if not provenance['module_from_expected']:
            self.get_logger().error(
                'DEPLOYMENT MISMATCH: the launch manager is not running from '
                f'{self.expected_workspace}. Stop this process, source '
                f'{provenance["expected_install"]}/setup.bash in a clean '
                'terminal, and restart the persistent backend.')
        elif not provenance['overlay_has_expected']:
            self.get_logger().warn(
                'The expected workspace is not present in AMENT_PREFIX_PATH; '
                'child launch resolution may use another overlay.')
        if provenance['stale_prefixes']:
            self.get_logger().warn(
                'Stale /home/a/nav_ws prefixes remain in AMENT_PREFIX_PATH: '
                + ', '.join(provenance['stale_prefixes']))

    def _deployment_mismatches(self, mode):
        """Return project packages that are missing or resolve outside 3dnav."""
        expected_install = str(
            (self.expected_workspace / 'install').resolve()
        ).replace('\\', '/').rstrip('/')
        mismatches = []
        required_packages = self.MODE_PACKAGES.get(mode, ())
        runtime_packages = self.MODE_RUNTIME_PACKAGES.get(mode, ())
        for package_name in required_packages + runtime_packages:
            try:
                prefix = str(get_package_prefix(package_name))
            except (PackageNotFoundError, EnvironmentError) as error:
                mismatches.append((package_name, f'unavailable: {error}'))
                continue
            normalised = str(
                Path(prefix).resolve()).replace('\\', '/').rstrip('/')
            from_expected = (
                normalised == expected_install or
                normalised.startswith(expected_install + '/'))
            from_stale_workspace = (
                normalised == '/home/a/nav_ws/install' or
                normalised.startswith('/home/a/nav_ws/install/') or
                normalised == 'C:/home/a/nav_ws/install' or
                normalised.startswith('C:/home/a/nav_ws/install/'))
            if (package_name in required_packages and not from_expected) or \
                    from_stale_workspace:
                mismatches.append((package_name, prefix))
        if mode == 'mapping' and not any(
                package == 'small_point_lio' for package, _ in mismatches):
            try:
                lio_prefix = Path(get_package_prefix('small_point_lio'))
                marker, marker_value = self._read_global_mapping_contract(
                    lio_prefix)
                if marker_value != self.GLOBAL_MAPPING_CONTRACT:
                    mismatches.append((
                        'small_point_lio build contract',
                        f'{marker}: expected {self.GLOBAL_MAPPING_CONTRACT!r}, '
                        f'got {marker_value!r}',
                    ))
            except (OSError, PackageNotFoundError, EnvironmentError) as error:
                mismatches.append((
                    'small_point_lio build contract',
                    f'missing; rebuild small_point_lio ({error})',
                ))
        if mode == 'mapping' and not any(
                package == 'octomap_server2' for package, _ in mismatches):
            try:
                filter_prefix = Path(get_package_prefix('octomap_server2'))
                marker, marker_value = self._read_mapping_filter_contract(
                    filter_prefix)
                if marker_value != self.MAPPING_FILTER_CONTRACT:
                    mismatches.append((
                        'octomap_server2 mapping filter build contract',
                        f'{marker}: expected {self.MAPPING_FILTER_CONTRACT!r}, '
                        f'got {marker_value!r}',
                    ))
            except (OSError, PackageNotFoundError, EnvironmentError) as error:
                mismatches.append((
                    'octomap_server2 mapping filter build contract',
                    f'missing; rebuild octomap_server2 ({error})',
                ))
        return mismatches

    @staticmethod
    def _read_global_mapping_contract(lio_prefix):
        marker = (
            Path(lio_prefix) / 'share' / 'small_point_lio' /
            'global_mapping_contract.txt')
        return marker, marker.read_text(encoding='utf-8').strip()

    @staticmethod
    def _read_mapping_filter_contract(filter_prefix):
        marker = (
            Path(filter_prefix) / 'share' / 'octomap_server2' /
            'mapping_filter_contract.txt')
        return marker, marker.read_text(encoding='utf-8').strip()

    def _string_parameter(self, name, default):
        value = self.get_parameter(name).value
        value = str(value).strip() if value is not None else ''
        return value or default

    def _seconds_parameter(self, name, default):
        try:
            return max(0.0, float(self.get_parameter(name).value))
        except (TypeError, ValueError):
            self.get_logger().warn(
                f'Invalid {name} parameter; using {default:.1f} seconds instead.')
            return default

    def _bool_parameter(self, name, default):
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalised = value.strip().lower()
            if normalised in {'true', '1', 'yes', 'on'}:
                return True
            if normalised in {'false', '0', 'no', 'off'}:
                return False
        self.get_logger().warn(
            f'Invalid {name} parameter; using {default} instead.')
        return default

    def _mapping_dynamic_filter_mode(self):
        """Return the safe mode passed to both mapping filter chains."""
        store = getattr(self, '_mapping_filter_store', None)
        if store is not None:
            state = store.snapshot()
            if state['load_error']:
                self.get_logger().warn('建图过滤配置损坏，回退 protected：' + state['load_error'])
            return state['mode']
        try:
            value = self.get_parameter(
                'mapping_dynamic_filter_mode').value
        except AttributeError:
            # Allows the pure preflight harness to instantiate this class
            # without a live rclpy Node while exercising launch commands.
            value = 'protected'
        normalized = str(value or '').strip().lower()
        if normalized in {'protected', 'bypass', 'strict'}:
            return normalized
        self.get_logger().warn(
            'Invalid mapping_dynamic_filter_mode parameter; using protected. '
            'Valid modes: protected, bypass, strict.')
        return 'protected'

    def _init_mapping_filter_settings(self):
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, Trigger
        from lzp_service.mapping_filter import MappingFilterStore
        path = self._string_parameter('mapping_filter_config_path', '')
        if not path:
            path = Path.home() / '.ros' / 'robot_navigation' / 'mapping_filter.json'
        self._mapping_filter_store = MappingFilterStore(
            path, default_mode=self._mapping_dynamic_filter_mode())
        self._active_mapping_filter_mode = None
        self._mapping_filter_message_type = String
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._mapping_filter_status_publisher = self.create_publisher(String, '/mapping/filter/status', qos)
        self._mapping_filter_get_service = self.create_service(
            Trigger, '/mapping/filter/get', self._get_mapping_filter_callback)
        self._mapping_filter_set_service = self.create_service(
            SetBool, '/mapping/filter/set_enabled', self._set_mapping_filter_enabled_callback)

    def _mapping_filter_status(self):
        state = self._mapping_filter_store.snapshot()
        with self.process_lock:
            process = self.running_processes.get('mapping')
            running = process is not None and process.poll() is None
            active = getattr(self, '_active_mapping_filter_mode', None) if running else None
        return dict(state, mapping_running=running, active_mode=active,
                    active_enabled=None if active is None else active != 'bypass',
                    pending=running and active != state['mode'], apply_policy='next_mapping_start')

    def _publish_mapping_filter_status(self):
        publisher = getattr(self, '_mapping_filter_status_publisher', None)
        if publisher is not None:
            publisher.publish(self._mapping_filter_message_type(
                data=json.dumps(self._mapping_filter_status(), ensure_ascii=False)))

    def _get_mapping_filter_callback(self, _request, response):
        response.success = True
        response.message = json.dumps(self._mapping_filter_status(), ensure_ascii=False)
        return response

    def _set_mapping_filter_enabled_callback(self, request, response):
        with self.transition_lock:
            try:
                self._mapping_filter_store.set_enabled(request.data)
                status = self._mapping_filter_status()
                response.success = True
                response.message = json.dumps(status, ensure_ascii=False)
                self._publish_mapping_filter_status()
            except (AttributeError, OSError, ValueError, TypeError) as error:
                response.success = False
                response.message = str(error)
        return response

    @classmethod
    def _normalise_mode(cls, launch_type, allow_all=False):
        mode = str(launch_type or '').strip().lower()
        if allow_all and mode == 'all':
            return 'all'
        return cls.MODE_ALIASES.get(mode)

    def _package_is_allowed(self, package_name):
        # Empty package names are accepted for a simple web client.  Any
        # non-empty package must be the configured package that owns our two
        # whitelisted launch files.
        requested = str(package_name or '').strip().lower()
        return not requested or requested == self.launch_package.lower()

    def _prepare_fastdds_environment(self):
        """Clean stale Fast DDS SHM state before starting a launch tree.

        The official cleaner only removes zombie allocations.  If it cannot
        run, isolate the new process tree from SHM instead of deleting files
        from ``/dev/shm`` ourselves; active persistent ROS nodes remain
        untouched and can communicate with the child tree over UDPv4.
        """
        environment = os.environ.copy()
        if not self.fastdds_shm_cleanup_enabled:
            return self._select_fastdds_transport(environment, cleanup_failed=False)

        failure = ''
        timeout = max(0.1, self.fastdds_shm_cleanup_timeout_sec)
        try:
            result = subprocess.run(
                ['fastdds', 'shm', 'clean'],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
                timeout=timeout,
            )
            output = ' '.join((result.stdout or '').split())
            if result.returncode == 0:
                self.get_logger().info(
                    'Fast DDS shared-memory zombie cleanup completed.')
                return self._select_fastdds_transport(
                    environment, cleanup_failed=False)
            failure = f'exit code {result.returncode}'
            if output:
                failure += f': {output[-300:]}'
        except FileNotFoundError:
            failure = 'the fastdds maintenance CLI is not installed'
        except subprocess.TimeoutExpired:
            failure = f'cleanup exceeded {timeout:.1f}s'
        except OSError as error:
            failure = str(error)

        self.get_logger().warn(
            f'Fast DDS shared-memory cleanup was unavailable ({failure}).')
        return self._select_fastdds_transport(environment, cleanup_failed=True)

    def _select_fastdds_transport(self, environment, cleanup_failed):
        """Choose a deterministic transport for a newly managed launch tree."""
        inherited_transport = str(
            environment.get('FASTDDS_BUILTIN_TRANSPORTS', '')).strip()
        use_udp = self.fastdds_force_udp or (
            cleanup_failed and self.fastdds_udp_fallback_enabled)
        if use_udp and not inherited_transport:
            environment['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'
            level = (
                self.get_logger().warn
                if cleanup_failed else self.get_logger().info
            )
            level(
                'Starting this managed launch tree with UDPv4 only; page-driven '
                'mode restarts will not reuse Fast DDS shared-memory ports.')
        elif inherited_transport:
            self.get_logger().warn(
                'Keeping the explicitly configured '
                f'FASTDDS_BUILTIN_TRANSPORTS={inherited_transport}.')
        elif cleanup_failed:
            self.get_logger().warn(
                'Fast DDS UDP fallback is disabled; the managed launch tree '
                'will keep its default transport configuration.')
        return environment

    def handle_control_launch(self, request, response):
        """Handle a web request without exposing arbitrary process execution."""
        action = str(request.action or '').strip().lower()
        allow_all = action == 'stop'
        launch_type = self._normalise_mode(request.launch_type, allow_all=allow_all)

        if action not in ('start', 'stop', 'restart'):
            response.success = False
            response.message = f'Unsupported action: {action or "(empty)"}'
            return response

        if launch_type is None:
            response.success = False
            response.message = (
                'Unsupported launch type. Use mapping, car_vel (or navigation), '
                'or all when stopping.')
            return response

        if launch_type == 'all' and action != 'stop':
            response.success = False
            response.message = 'The all launch type is only valid with the stop action.'
            return response

        if not self._package_is_allowed(request.package_name):
            response.success = False
            response.message = (
                f'This manager only controls launch files in package {self.launch_package}.')
            return response

        try:
            if action == 'start':
                success, message = self._start_mode(launch_type)
            elif action == 'stop':
                if launch_type == 'all':
                    success, message = self._stop_all_modes()
                else:
                    success, message = self._stop_mode(launch_type)
            else:
                success, message = self._restart_mode(launch_type)
        except Exception as error:  # Keep a malformed web request from crashing the backend.
            self.get_logger().error(f'Failed to handle launch control request: {error}')
            success = False
            message = f'Launch control error: {error}'

        response.success = success
        response.message = message
        return response

    # These boolean wrappers retain the public methods used by older local
    # callers while the service callback receives the more useful explanation.
    def start_launch(self, launch_type, package_name=None):
        mode = self._normalise_mode(launch_type)
        if mode is None or not self._package_is_allowed(package_name):
            return False
        return self._start_mode(mode)[0]

    def stop_launch(self, launch_type, package_name=None):
        mode = self._normalise_mode(launch_type, allow_all=True)
        if mode is None or not self._package_is_allowed(package_name):
            return False
        if mode == 'all':
            return self._stop_all_modes()[0]
        return self._stop_mode(mode)[0]

    def restart_launch(self, launch_type, package_name=None, delay=None):
        del delay  # Delay is configured once for all clients via ROS parameters.
        mode = self._normalise_mode(launch_type)
        if mode is None or not self._package_is_allowed(package_name):
            return False
        return self._restart_mode(mode)[0]

    def _start_mode(self, mode):
        """Start *mode*, stopping the other shared-hardware mode first."""
        with self.transition_lock:
            directory = str(Path(getattr(self, 'expected_workspace', '/home/a/3dnav')) /
                'src/ros2_humble_main/src/navigation/robot_navigation2/pcdmaps')
            if saved_map_write_busy(directory):
                return False, '地图编辑/保存进行中，禁止启动建图或导航'
            body_manager = getattr(self, 'robot_body_manager', None)
            if body_manager is not None and mode == 'car_vel':
                error = body_manager.navigation_error()
                if error:
                    return False, error
            mismatches = self._deployment_mismatches(mode)
            if mismatches:
                details = ', '.join(
                    f'{package} -> {prefix}' for package, prefix in mismatches)
                message = (
                    f'Refusing to start {mode}: project packages are not '
                    f'installed from {self.expected_workspace}: {details}. '
                    f'Rebuild and source {self.expected_workspace}/install/'
                    'setup.bash in '
                    'a clean terminal, then restart launch_manager.')
                self.get_logger().error(message)
                return False, message
            self._reap_exited_processes()
            other_mode = 'car_vel' if mode == 'mapping' else 'mapping'

            # Repair a stale state from an older backend version as well: even
            # if the requested mode is already alive, the other mode must stop.
            if self._get_managed_process(other_mode) is not None:
                stopped, message = self._stop_mode_locked(other_mode)
                if not stopped:
                    return False, f'Cannot start {mode}: {message}'
                if self.mode_switch_delay_sec:
                    time.sleep(self.mode_switch_delay_sec)

            if self._get_managed_process(mode) is not None:
                self.get_logger().info(
                    f'{mode} is already running; no duplicate launch was '
                    'created.')
                return True, f'{mode} is already running.'

            command = ['ros2', 'launch', self.launch_package, self.LAUNCH_FILES[mode]]
            mapping_filter_mode = None
            if mode == 'mapping':
                mapping_filter_mode = self._mapping_dynamic_filter_mode()
                command.append(
                    'dynamic_filter_mode:=' +
                    mapping_filter_mode)
            popen_kwargs = {
                'stdin': subprocess.DEVNULL,
                'env': self._prepare_fastdds_environment(),
            }
            if os.name == 'posix':
                # Give the launch tree its own process group.  Stopping a mode
                # then reaches ros2 launch and all nodes it started.
                popen_kwargs['start_new_session'] = True

            try:
                process = subprocess.Popen(command, **popen_kwargs)
            except OSError as error:
                self.get_logger().error(f'Unable to start {mode}: {error}')
                return False, f'Unable to start {mode}: {error}'

            with self.process_lock:
                self.running_processes[mode] = process
                if mode == 'mapping':
                    self._active_mapping_filter_mode = mapping_filter_mode

            # Popen only proves that the shell command was spawned.  Catch the
            # common immediate failures (bad overlay, missing ros2, invalid
            # launch file) before telling the page that switching succeeded.
            if self.startup_check_sec:
                time.sleep(self.startup_check_sec)
            exit_code = process.poll()
            if exit_code is not None:
                self._remove_process_if_current(mode, process)
                self.get_logger().error(f'{mode} exited during startup (exit code {exit_code}).')
                return False, f'{mode} exited during startup (exit code {exit_code}).'

            self.get_logger().info(f'Started {mode} with PID {process.pid}.')
            return True, f'Started {mode}.'

    def _stop_mode(self, mode):
        with self.transition_lock:
            return self._stop_mode_locked(mode)

    def _stop_mode_locked(self, mode):
        """Stop one launch tree.  The caller holds ``transition_lock``."""
        process = self._get_managed_process(mode)
        if process is None:
            return True, f'{mode} is already stopped.'

        pid = process.pid
        try:
            self._signal_process_tree(process, signal.SIGTERM)
            try:
                process.wait(timeout=self.shutdown_timeout_sec)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(
                    f'{mode} (PID {pid}) did not stop within '
                    f'{self.shutdown_timeout_sec:.1f}s; forcing it to stop.')
                self._signal_process_tree(process, signal.SIGKILL)
                try:
                    process.wait(timeout=self.force_kill_timeout_sec)
                except subprocess.TimeoutExpired:
                    # Preserve the record so status remains truthful and a
                    # later stop request can retry rather than starting a
                    # conflicting mode.
                    return False, f'{mode} did not stop after a forced kill.'
        except ProcessLookupError:
            # It exited between poll() and signaling, which is a successful
            # idempotent stop rather than an error.
            pass
        except OSError as error:
            if process.poll() is None:
                self.get_logger().error(f'Unable to stop {mode} (PID {pid}): {error}')
                return False, f'Unable to stop {mode}: {error}'

        if process.poll() is None:
            return False, f'{mode} is still running after the stop request.'

        self._remove_process_if_current(mode, process)
        self.get_logger().info(f'Stopped {mode} with PID {pid}.')
        return True, f'Stopped {mode}.'

    def _stop_all_modes(self):
        """Stop both modes without short-circuiting if one stop fails."""
        with self.transition_lock:
            results = []
            for mode in ('mapping', 'car_vel'):
                results.append((mode, *self._stop_mode_locked(mode)))

            failures = [message for _, success, message in results if not success]
            if failures:
                return False, 'Some launch modes could not be stopped: ' + '; '.join(failures)
            return True, 'All managed launch modes are stopped.'

    def _restart_mode(self, mode):
        with self.transition_lock:
            mismatches = self._deployment_mismatches(mode)
            if mismatches:
                details = ', '.join(
                    f'{package} -> {prefix}' for package, prefix in mismatches)
                message = (
                    f'Cannot restart {mode}: project packages are not '
                    f'installed from {self.expected_workspace}: {details}.')
                self.get_logger().error(message)
                return False, message
            stopped, message = self._stop_mode_locked(mode)
            if not stopped:
                return False, f'Cannot restart {mode}: {message}'
            if self.restart_delay_sec:
                time.sleep(self.restart_delay_sec)
            started, message = self._start_mode(mode)
            if not started:
                return False, f'Cannot restart {mode}: {message}'
            return True, f'Restarted {mode}.'

    def _signal_process_tree(self, process, signum):
        """Signal the entire launch process group, not only ros2 launch."""
        if os.name == 'posix':
            os.killpg(os.getpgid(process.pid), signum)
        elif signum == signal.SIGKILL:
            process.kill()
        else:
            process.terminate()

    def _get_managed_process(self, mode):
        with self.process_lock:
            process = self.running_processes.get(mode)
            if process is not None and process.poll() is not None:
                del self.running_processes[mode]
                self.get_logger().warn(
                    f'{mode} exited unexpectedly (exit code {process.returncode}).')
                process = None
            return process

    def _remove_process_if_current(self, mode, process):
        with self.process_lock:
            if self.running_processes.get(mode) is process:
                del self.running_processes[mode]

    def _reap_exited_processes(self):
        with self.process_lock:
            exited = [
                (mode, process.returncode)
                for mode, process in self.running_processes.items()
                if process.poll() is not None
            ]
            for mode, exit_code in exited:
                del self.running_processes[mode]
                self.get_logger().warn(
                    f'{mode} exited unexpectedly (exit code {exit_code}).')

    def publish_status(self):
        self._reap_exited_processes()
        with self.process_lock:
            mapping = self.running_processes.get('mapping')
            navigation = self.running_processes.get('car_vel')

        status = LaunchStatus()
        status.mapping_running = mapping is not None
        status.navigation_running = navigation is not None
        status.mapping_pid = mapping.pid if mapping is not None else 0
        status.navigation_pid = navigation.pid if navigation is not None else 0
        status.timestamp = self.get_clock().now().to_msg()
        self.status_publisher.publish(status)
        self._publish_mapping_filter_status()

    def destroy_node(self):
        """Stop only the launch trees owned by this manager during shutdown."""
        try:
            self._stop_all_modes()
            body_manager = getattr(self, 'robot_body_manager', None)
            if body_manager is not None:
                body_manager.close()
        except Exception as error:
            self.get_logger().error(f'Error while stopping managed launch modes: {error}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaunchManagerService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
