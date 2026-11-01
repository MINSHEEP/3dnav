"""Settings endpoint; saving never restarts a running LiDAR."""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lzp_interface.srv import SetLidarNetwork
from .lidar_network import installed_store, local_address_warning


class LidarNetworkBridge(Node):
    def __init__(self):
        super().__init__('lidar_network_bridge')
        self.store = installed_store()
        self.publisher = self.create_publisher(String, '/lidar/network_settings', 1)
        self.create_service(SetLidarNetwork, '/lidar/set_network', self.configure)
        self.create_timer(2.0, self.publish)

    def status(self):
        try:
            config = self.store.load()
            return {'config': config, 'error': '',
                    'warning': local_address_warning(config['host_ip']),
                    'effective_policy': 'next_start'}
        except Exception as error:
            return {'config': None, 'error': str(error), 'effective_policy': 'next_start'}

    def publish(self):
        self.publisher.publish(String(data=json.dumps(self.status(), ensure_ascii=False)))

    def configure(self, request, response):
        try:
            if request.operation == 'save':
                self.store.save(request.lidar_ip, request.host_ip, request.expected_version)
                response.message = '已保存；下次启动建图或导航的雷达驱动时生效，当前运行不受影响'
            elif request.operation == 'get':
                response.message = '已刷新保存的配置（不代表当前运行地址）'
            else:
                raise ValueError('未知操作')
            response.success = True
        except Exception as error:
            response.success, response.message = False, str(error)
        response.status_json = json.dumps(self.status(), ensure_ascii=False)
        self.publish()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LidarNetworkBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
