"""Regression tests for Fast DDS shared-memory startup maintenance."""

from pathlib import Path
import importlib.util
import subprocess
import sys
import threading
from types import ModuleType
import unittest
from unittest import mock


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "lzp_service"
    / "launch_manager_service.py"
)
LAUNCH_PATH = (
    MODULE_PATH.parents[3]
    / "car_vel"
    / "launch"
    / "launch_manager.launch.py"
)
INSTALL_SCRIPT_PATH = (
    MODULE_PATH.parents[3]
    / "car_vel"
    / "scripts"
    / "install_hikvision_camera_deps.sh"
)
CPU_CONSTRAINTS_PATH = (
    MODULE_PATH.parents[3]
    / "car_vel"
    / "requirements-pytorch-cpu.txt"
)


def _load_launch_manager_module():
    sys.path.insert(0, str(MODULE_PATH.parents[1]))
    rclpy = ModuleType("rclpy")
    rclpy_node = ModuleType("rclpy.node")
    rclpy_node.Node = object
    ament_index = ModuleType("ament_index_python")
    ament_packages = ModuleType("ament_index_python.packages")
    ament_packages.PackageNotFoundError = type(
        "PackageNotFoundError", (Exception,), {})
    ament_packages.get_package_prefix = lambda name: (
        "/opt/ros/humble" if name in {
            "livox_ros_driver2", "pointcloud_to_laserscan"
        } else f"/home/a/3dnav/install/{name}")
    interface = ModuleType("lzp_interface")
    interface_msg = ModuleType("lzp_interface.msg")
    interface_srv = ModuleType("lzp_interface.srv")
    interface_msg.LaunchStatus = type("LaunchStatus", (), {})
    interface_srv.ControlLaunch = type("ControlLaunch", (), {})

    replacements = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "ament_index_python": ament_index,
        "ament_index_python.packages": ament_packages,
        "lzp_interface": interface,
        "lzp_interface.msg": interface_msg,
        "lzp_interface.srv": interface_srv,
    }
    previous = {name: sys.modules.get(name) for name in replacements}
    sys.modules.update(replacements)
    try:
        spec = importlib.util.spec_from_file_location(
            "lzp_service.launch_manager_fastdds_test_module", MODULE_PATH)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        for name, old_module in previous.items():
            if old_module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old_module


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", str(message)))

    def warn(self, message):
        self.messages.append(("warn", str(message)))

    def error(self, message):
        self.messages.append(("error", str(message)))


class _Process:
    pid = 4321
    returncode = None

    @staticmethod
    def poll():
        return None


class TestFastDdsPreflight(unittest.TestCase):
    def test_saved_map_write_lease_blocks_launch_before_any_hardware_start(self):
        with mock.patch.object(self.module, 'saved_map_write_busy', return_value=True):
            success, message = self.manager._start_mode('car_vel')
        self.assertFalse(success)
        self.assertIn('地图编辑/保存', message)

    def setUp(self):
        self.module = _load_launch_manager_module()
        self.module.nav2_process_conflicts = lambda: []
        self.logger = _Logger()
        manager_type = self.module.LaunchManagerService
        self.manager = manager_type.__new__(manager_type)
        self.manager.launch_package = "car_vel"
        self.manager.mode_switch_delay_sec = 0.0
        self.manager.startup_check_sec = 0.0
        self.manager.fastdds_shm_cleanup_enabled = True
        self.manager.fastdds_shm_cleanup_timeout_sec = 1.0
        self.manager.fastdds_force_udp = True
        self.manager.fastdds_udp_fallback_enabled = True
        self.manager.expected_workspace = Path('/home/a/3dnav')
        self.manager.running_processes = {}
        self.manager.process_lock = threading.RLock()
        self.manager.transition_lock = threading.RLock()
        self.manager.get_logger = lambda: self.logger
        self.manager._read_global_mapping_contract = lambda prefix: (
            Path(prefix) / 'share' / 'small_point_lio' /
            'global_mapping_contract.txt',
            self.manager.GLOBAL_MAPPING_CONTRACT,
        )
        self.manager._read_mapping_filter_contract = lambda prefix: (
            Path(prefix) / 'share' / 'octomap_server2' /
            'mapping_filter_contract.txt',
            self.manager.MAPPING_FILTER_CONTRACT,
        )

    def test_cleanup_runs_before_managed_launch_and_uses_udp_for_restart_safety(self):
        events = []

        def clean(*args, **kwargs):
            events.append(("clean", args, kwargs))
            return subprocess.CompletedProcess(args[0], 0, "cleaned")

        def start(command, **kwargs):
            events.append(("start", command, kwargs))
            return _Process()

        with mock.patch.object(self.module.subprocess, "run", side_effect=clean), \
                mock.patch.object(
                    self.module.subprocess, "Popen", side_effect=start), \
                mock.patch.dict(
                    self.module.os.environ,
                    {"FASTDDS_BUILTIN_TRANSPORTS": ""}, clear=False):
            self.module.os.environ.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
            success, _ = self.manager._start_mode("car_vel")

        self.assertTrue(success)
        self.assertEqual([event[0] for event in events], ["clean", "start"])
        self.assertEqual(events[0][1][0], ["fastdds", "shm", "clean"])
        self.assertEqual(
            events[1][2]["env"]["FASTDDS_BUILTIN_TRANSPORTS"], "UDPv4")

    def test_missing_cleaner_uses_udp_for_only_the_new_process_tree(self):
        captured = {}
        self.manager.fastdds_force_udp = False

        def start(command, **kwargs):
            captured.update(kwargs)
            return _Process()

        with mock.patch.object(
                self.module.subprocess, "run",
                side_effect=FileNotFoundError("fastdds")), \
                mock.patch.object(
                    self.module.subprocess, "Popen", side_effect=start), \
                mock.patch.dict(
                    self.module.os.environ,
                    {"FASTDDS_BUILTIN_TRANSPORTS": ""}, clear=False):
            self.module.os.environ.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
            success, _ = self.manager._start_mode("mapping")

        self.assertTrue(success)
        self.assertEqual(
            captured["env"]["FASTDDS_BUILTIN_TRANSPORTS"], "UDPv4")
        self.assertNotIn("FASTDDS_BUILTIN_TRANSPORTS", self.module.os.environ)
        self.assertTrue(any(
            level == "warn" and "UDPv4" in message
            for level, message in self.logger.messages
        ))

    def test_cleanup_timeout_uses_udp_without_deleting_shared_memory(self):
        self.manager.fastdds_force_udp = False
        with mock.patch.object(
                self.module.subprocess, "run",
                side_effect=subprocess.TimeoutExpired(
                    ["fastdds", "shm", "clean"], 1.0)), \
                mock.patch.dict(
                    self.module.os.environ,
                    {"FASTDDS_BUILTIN_TRANSPORTS": ""}, clear=False):
            self.module.os.environ.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
            environment = self.manager._prepare_fastdds_environment()

        self.assertEqual(
            environment["FASTDDS_BUILTIN_TRANSPORTS"], "UDPv4")
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertNotIn("/dev/shm/fastrtps_", source)
        self.assertNotIn("glob(", source)

    def test_force_udp_can_be_disabled_after_successful_cleanup(self):
        self.manager.fastdds_force_udp = False
        with mock.patch.object(
                self.module.subprocess, "run",
                return_value=subprocess.CompletedProcess(
                    ["fastdds", "shm", "clean"], 0, "cleaned")), \
                mock.patch.dict(
                    self.module.os.environ,
                    {"FASTDDS_BUILTIN_TRANSPORTS": ""}, clear=False):
            self.module.os.environ.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
            environment = self.manager._prepare_fastdds_environment()

        self.assertNotIn("FASTDDS_BUILTIN_TRANSPORTS", environment)

    def test_main_launch_exposes_fastdds_safety_parameters(self):
        launch = LAUNCH_PATH.read_text(encoding="utf-8")
        for parameter in (
                "fastdds_shm_cleanup_enabled",
                "fastdds_shm_cleanup_timeout_sec",
                "fastdds_force_udp",
                "fastdds_udp_fallback_enabled"):
            self.assertGreaterEqual(launch.count(f"'{parameter}'"), 2)

    def test_workspace_provenance_detects_old_overlay(self):
        self.manager.expected_workspace = Path('/home/a/3dnav')
        with mock.patch.object(
                self.module, '__file__',
                '/home/a/nav_ws/install/lzp_service/lib/python3.10/'
                'site-packages/lzp_service/launch_manager_service.py'), \
                mock.patch.dict(
                    self.module.os.environ,
                    {'AMENT_PREFIX_PATH': (
                        '/home/a/nav_ws/install/car_vel:'
                        '/home/a/nav_ws/install/lzp_service')},
                    clear=False):
            provenance = self.manager._workspace_provenance()

        self.assertFalse(provenance['module_from_expected'])
        self.assertFalse(provenance['overlay_has_expected'])
        self.assertEqual(len(provenance['stale_prefixes']), 2)

    def test_workspace_provenance_accepts_current_overlay(self):
        self.manager.expected_workspace = Path('/home/a/3dnav')
        with mock.patch.object(
                self.module, '__file__',
                '/home/a/3dnav/install/lzp_service/lib/python3.10/'
                'site-packages/lzp_service/launch_manager_service.py'), \
                mock.patch.dict(
                    self.module.os.environ,
                    {'AMENT_PREFIX_PATH': (
                        '/home/a/3dnav/install/car_vel:'
                        '/home/a/3dnav/install/lzp_service')},
                    clear=False):
            provenance = self.manager._workspace_provenance()

        self.assertTrue(provenance['module_from_expected'])
        self.assertTrue(provenance['overlay_has_expected'])
        self.assertFalse(provenance['stale_prefixes'])

    def test_mapping_start_refuses_package_from_old_workspace(self):
        def package_prefix(package_name):
            if package_name == 'small_point_lio':
                return '/home/a/nav_ws/install/small_point_lio'
            return f'/home/a/3dnav/install/{package_name}'

        with mock.patch.object(
                self.module, 'get_package_prefix', side_effect=package_prefix), \
                mock.patch.object(self.module.subprocess, 'Popen') as popen:
            success, message = self.manager._start_mode('mapping')

        self.assertFalse(success)
        self.assertIn('small_point_lio', message)
        self.assertIn('/home/a/nav_ws', message)
        popen.assert_not_called()

    def test_mapping_start_accepts_packages_from_current_workspace(self):
        captured = {}

        def package_prefix(name):
            if name == 'livox_ros_driver2':
                return '/opt/ros/humble'
            return f'/home/a/3dnav/install/{name}'

        def start(command, **kwargs):
            captured['command'] = command
            return _Process()

        self.manager.get_parameter = lambda name: type(
            'Parameter', (), {'value': 'bypass'})()

        with mock.patch.object(
                self.module, 'get_package_prefix',
                side_effect=package_prefix), \
                mock.patch.object(
                    self.module.subprocess, 'run',
                    return_value=subprocess.CompletedProcess(
                        ['fastdds', 'shm', 'clean'], 0, 'cleaned')), \
                mock.patch.object(
                    self.module.subprocess, 'Popen', side_effect=start):
            success, message = self.manager._start_mode('mapping')

        self.assertTrue(success)
        self.assertEqual(message, 'Started mapping.')
        self.assertIn(
            'dynamic_filter_mode:=bypass', captured['command'])

    def test_invalid_mapping_filter_mode_falls_back_to_protected(self):
        self.manager.get_parameter = lambda name: type(
            'Parameter', (), {'value': 'typo'})()
        self.assertEqual(
            self.manager._mapping_dynamic_filter_mode(), 'protected')
        self.assertTrue(any(
            level == 'warn' and 'mapping_dynamic_filter_mode' in message
            for level, message in self.logger.messages
        ))

    def test_mapping_start_refuses_vendor_package_from_stale_workspace(self):
        def package_prefix(package_name):
            if package_name == 'livox_ros_driver2':
                return '/home/a/nav_ws/install/livox_ros_driver2'
            return f'/home/a/3dnav/install/{package_name}'

        with mock.patch.object(
                self.module, 'get_package_prefix', side_effect=package_prefix), \
                mock.patch.object(self.module.subprocess, 'Popen') as popen:
            success, message = self.manager._start_mode('mapping')

        self.assertFalse(success)
        self.assertIn('livox_ros_driver2', message)
        popen.assert_not_called()

    def test_mapping_start_reports_missing_ament_environment(self):
        with mock.patch.object(
                self.module, 'get_package_prefix',
                side_effect=EnvironmentError('AMENT_PREFIX_PATH is unset')), \
                mock.patch.object(self.module.subprocess, 'Popen') as popen:
            success, message = self.manager._start_mode('mapping')

        self.assertFalse(success)
        self.assertIn('AMENT_PREFIX_PATH is unset', message)
        popen.assert_not_called()

    def test_mapping_start_refuses_current_prefix_with_stale_lio_build(self):
        self.manager._read_global_mapping_contract = lambda prefix: (
            Path(prefix) / 'share' / 'small_point_lio' /
            'global_mapping_contract.txt',
            'old_contract',
        )
        with mock.patch.object(self.module.subprocess, 'Popen') as popen:
            success, message = self.manager._start_mode('mapping')

        self.assertFalse(success)
        self.assertIn('small_point_lio build contract', message)
        self.assertIn('old_contract', message)
        popen.assert_not_called()

    def test_mapping_start_refuses_current_prefix_with_stale_filter_build(self):
        self.manager._read_mapping_filter_contract = lambda prefix: (
            Path(prefix) / 'share' / 'octomap_server2' /
            'mapping_filter_contract.txt',
            'old_filter_contract',
        )
        with mock.patch.object(self.module.subprocess, 'Popen') as popen:
            success, message = self.manager._start_mode('mapping')

        self.assertFalse(success)
        self.assertIn('octomap_server2 mapping filter build contract', message)
        self.assertIn('old_filter_contract', message)
        popen.assert_not_called()

    def test_mapping_restart_preflights_before_stopping_running_mode(self):
        process = _Process()
        self.manager.running_processes['mapping'] = process

        def package_prefix(package_name):
            if package_name == 'small_point_lio':
                return '/home/a/nav_ws/install/small_point_lio'
            return f'/home/a/3dnav/install/{package_name}'

        with mock.patch.object(
                self.module, 'get_package_prefix', side_effect=package_prefix), \
                mock.patch.object(
                    self.manager, '_stop_mode_locked') as stop_mode:
            success, message = self.manager._restart_mode('mapping')

        self.assertFalse(success)
        self.assertIn('small_point_lio', message)
        stop_mode.assert_not_called()
        self.assertIs(self.manager.running_processes['mapping'], process)

    def test_dependency_installer_provides_official_fastdds_cleaner(self):
        installer = INSTALL_SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertIn("fastdds-tools", installer)
        self.assertNotIn("rm -f /dev/shm", installer)

    def test_full_dependency_install_cannot_replace_cpu_pytorch_with_cuda(self):
        installer = INSTALL_SCRIPT_PATH.read_text(encoding="utf-8")
        constraints = CPU_CONSTRAINTS_PATH.read_text(encoding="utf-8")

        self.assertIn('CPU_CONSTRAINTS_FILE=', installer)
        self.assertIn('--constraint "${CPU_CONSTRAINTS_FILE}"', installer)
        self.assertIn(
            '--extra-index-url "${PYTORCH_CPU_INDEX_URL}"',
            installer,
        )
        self.assertIn("torch==2.10.0+cpu", constraints)
        self.assertIn("torchvision==0.25.0+cpu", constraints)


if __name__ == "__main__":
    unittest.main()
