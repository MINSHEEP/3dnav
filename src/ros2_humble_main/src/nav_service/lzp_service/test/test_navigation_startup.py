import sys
from pathlib import Path
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lzp_service.navigation_startup import StartupTracker, main


class StartupTests(unittest.TestCase):
    def test_monitor_initializes_with_ros_read_only_clients_property(self):
        class Node:
            @property
            def clients(self):
                return iter(())

            def __init__(self, name):
                self.create_client = Mock(return_value=Mock())
                self.declare_parameter = Mock()
                self.get_parameter = Mock(return_value=NS(value=60))
                self.create_publisher = Mock(return_value=Mock())
                self.create_subscription = Mock()
                self.create_timer = Mock()
                self.destroy_node = Mock()
        ros = NS(init=Mock(), spin=Mock(), ok=lambda: True, shutdown=Mock())
        modules = {'rclpy': ros, 'rclpy.node': NS(Node=Node),
            'rclpy.action': NS(ActionClient=Mock()), 'std_msgs.msg': NS(String=NS),
            'lifecycle_msgs.srv': NS(GetState=NS),
            'nav2_msgs.action': NS(NavigateToPose=NS),
            'lzp_interface.msg': NS(LaunchStatus=NS)}
        with patch.dict(sys.modules, modules):
            main()
        node = ros.spin.call_args.args[0]
        self.assertEqual(node.create_client.call_count, len(StartupTracker.REQUIRED))
        self.assertEqual(set(node._lifecycle_clients), set(StartupTracker.REQUIRED))
        node.destroy_node.assert_called_once()

    def test_missing_components_timeout_and_recovery(self):
        tracker = StartupTracker(timeout=60)
        tracker.observe_launch(True, 10, 0)
        tracker.observe_launch(True, 10, 61)
        result = tracker.snapshot(61, {}, False)
        self.assertEqual(result['state'], 'timeout')
        self.assertIn('bt_navigator', result['missing_nodes'])
        result = tracker.snapshot(62, {n: 'active' for n in tracker.REQUIRED}, True)
        self.assertEqual(result['state'], 'ready')

    def test_restart_cannot_inherit_previous_deadline_or_lifecycle(self):
        tracker = StartupTracker(timeout=60)
        tracker.observe_launch(True, 10, 0)
        tracker.observe_launch(True, 11, 70)
        result = tracker.snapshot(71, {}, False)
        self.assertEqual(result['state'], 'loading_nodes')
        self.assertEqual(result['elapsed_sec'], 1)
        self.assertEqual(result['pid'], 11)

    def test_process_stopped_and_status_stale_are_not_ready(self):
        tracker = StartupTracker()
        tracker.observe_launch(True, 10, 0)
        self.assertEqual(tracker.snapshot(5, {}, True)['state'], 'status_stale')
        tracker.observe_launch(False, 0, 6)
        self.assertEqual(tracker.snapshot(6, {}, True)['state'], 'stopped')

    def test_inactive_nodes_are_named_even_if_action_exists(self):
        tracker = StartupTracker()
        tracker.observe_launch(True, 10, 0)
        states = {n: 'active' for n in tracker.REQUIRED}
        states['map_server'] = 'unconfigured'
        result = tracker.snapshot(1, states, True)
        self.assertEqual(result['state'], 'activating')
        self.assertIn('map_server=unconfigured', result['message'])


if __name__ == '__main__':
    unittest.main()
