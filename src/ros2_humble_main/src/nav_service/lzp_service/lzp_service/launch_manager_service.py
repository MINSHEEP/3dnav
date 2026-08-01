#!/usr/bin/env python3
"""Start and stop the mutually-exclusive mapping and navigation launch trees.

This node deliberately owns only the two launch trees that contain the shared
LiDAR/LIO/TF stack.  It is kept separate from ``launch_manager.launch.py`` so
the web backend stays alive while a user moves between the mapping and
navigation pages.
"""

import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from lzp_interface.msg import LaunchStatus
from lzp_interface.srv import ControlLaunch


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

    def __init__(self):
        super().__init__('launch_manager_service')

        self.declare_parameter('launch_package', 'car_vel')
        self.declare_parameter('mode_switch_delay_sec', 1.0)
        self.declare_parameter('startup_check_sec', 0.5)
        self.declare_parameter('shutdown_timeout_sec', 8.0)
        self.declare_parameter('force_kill_timeout_sec', 2.0)
        self.declare_parameter('restart_delay_sec', 1.0)

        self.launch_package = self._string_parameter('launch_package', 'car_vel')
        self.mode_switch_delay_sec = self._seconds_parameter('mode_switch_delay_sec', 1.0)
        self.startup_check_sec = self._seconds_parameter('startup_check_sec', 0.5)
        self.shutdown_timeout_sec = self._seconds_parameter('shutdown_timeout_sec', 8.0)
        self.force_kill_timeout_sec = self._seconds_parameter('force_kill_timeout_sec', 2.0)
        self.restart_delay_sec = self._seconds_parameter('restart_delay_sec', 1.0)

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

        self.get_logger().info(
            'Launch manager started. Mapping and navigation are page-driven and mutually exclusive.')

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
                self.get_logger().info(f'{mode} is already running; no duplicate launch was created.')
                return True, f'{mode} is already running.'

            command = ['ros2', 'launch', self.launch_package, self.LAUNCH_FILES[mode]]
            popen_kwargs = {'stdin': subprocess.DEVNULL}
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

    def destroy_node(self):
        """Stop only the launch trees owned by this manager during shutdown."""
        try:
            self._stop_all_modes()
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
