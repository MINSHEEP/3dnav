"""ROS services exposing safe host reboot and power-off operations."""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .system_power import SystemPowerController


class SystemPowerService(Node):
    """Expose two no-argument services backed by a fixed command allowlist."""

    def __init__(self):
        super().__init__('system_power_service')
        self.declare_parameter('enabled', True)
        self.declare_parameter('execution_delay_sec', 3.0)

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_publisher = self.create_publisher(
            String, '/system/power/status', status_qos)
        self._controller = SystemPowerController(
            enabled=self.get_parameter('enabled').value,
            delay_seconds=self.get_parameter('execution_delay_sec').value,
            status_callback=self._publish_status,
        )
        self._reboot_service = self.create_service(
            Trigger, '/system/reboot', self._handle_reboot)
        self._poweroff_service = self.create_service(
            Trigger, '/system/poweroff', self._handle_poweroff)
        # rosbridge subscribers can be volatile even when the publisher is
        # transient-local.  Republish the tiny status so a newly opened page
        # never depends on DDS history compatibility.
        self._status_timer = self.create_timer(
            1.0, self._publish_current_status)

        ready = self._controller.refresh_permissions()
        if ready:
            self.get_logger().info('System power services are ready')
        else:
            self.get_logger().warning(self._controller.status()['message'])

    def _handle_reboot(self, _request, response):
        return self._handle_action('reboot', response)

    def _handle_poweroff(self, _request, response):
        return self._handle_action('poweroff', response)

    def _handle_action(self, action, response):
        response.success, response.message = self._controller.request(action)
        if response.success:
            self.get_logger().warning(response.message)
        else:
            self.get_logger().error(response.message)
        return response

    def _publish_status(self, status):
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, sort_keys=True)
        self._status_publisher.publish(message)

    def _publish_current_status(self):
        self._publish_status(self._controller.status())


def main(args=None):
    rclpy.init(args=args)
    node = SystemPowerService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
