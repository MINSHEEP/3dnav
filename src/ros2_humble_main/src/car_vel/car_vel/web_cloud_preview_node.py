"""Demand-driven browser cloud topics, independent of mapping and terrain."""
import json
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from lzp_interface.msg import CurrentMap
from car_vel.web_cloud_preview import CLOUD_TOPICS, PROFILES, thin_cloud


class WebCloudPreview(Node):
    def __init__(self):
        super().__init__('web_cloud_preview')
        self.inputs, self.latest, self.outputs, self.sent = {}, {}, {}, {}
        self.previews, self.errors, self.published = {}, {}, {}
        self.static_topics = {'/global_localization_map'}
        self.status_at = float('-inf')
        self.saved_map_lock = threading.RLock()
        self.saved_map_current, self.saved_map_identity = {}, {}
        self.saved_map_http_status = {'ready': False, 'error': 'not_started'}
        self.pose = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.static_qos = QoSProfile(depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in CLOUD_TOPICS:
            for profile in PROFILES:
                self.outputs[topic, profile] = self.create_publisher(
                    PointCloud2, '/web_preview/' + profile + topic,
                    self.static_qos if topic in self.static_topics else qos)
        self.status_publisher = self.create_publisher(String, '/web_preview/status', self.static_qos)
        self.create_subscription(Odometry, '/Odometry', self._odom, self.input_qos)
        self.create_subscription(CurrentMap, '/current_map_info', self._saved_map_current, self.static_qos)
        self.create_subscription(String, '/global_localization_map/status', self._saved_map_identity, self.static_qos)
        # Saved maps publish once per load. Keep a reliable historical reader
        # even before DDS has discovered the browser's output subscription.
        for topic in self.static_topics:
            if topic in CLOUD_TOPICS:
                self._subscribe_input(topic)
        self.create_timer(0.25, self._tick)

    def _saved_map_current(self, message):
        with self.saved_map_lock:
            self.saved_map_current = dict(map_name=message.map_name, load_id=message.load_id) if message.success else {}

    def _saved_map_identity(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                value = {}
        except (ValueError, TypeError):
            value = {}
        with self.saved_map_lock:
            self.saved_map_identity = value

    def saved_map_state(self):
        with self.saved_map_lock:
            current, identity = self.saved_map_current, self.saved_map_identity
            if not current or any(current.get(key) != identity.get(key) for key in ('map_name', 'load_id')):
                return {}
            return dict(identity)

    def _subscribe_input(self, topic):
        self.inputs[topic] = self.create_subscription(PointCloud2, topic,
            lambda message, topic=topic: self._receive(topic, message),
            self.static_qos if topic in self.static_topics else self.input_qos)

    def _odom(self, message):
        position = message.pose.pose.position
        self.pose = (message.header.frame_id, position.x, position.y, time.monotonic())

    def _receive(self, topic, message):
        if topic in self.static_topics:
            # Do not let an unused profile retain a whole previous source map.
            for profile in PROFILES:
                self.previews.pop((topic, profile), None)
        self.latest[topic] = (message, time.monotonic())

    def _tick(self):
        now = time.monotonic()
        for topic in CLOUD_TOPICS:
            try:
                self._tick_topic(topic, now)
            except Exception as error:
                # A layout/discovery error in one display must not stop all
                # input subscriptions or hide which branch has failed.
                detail = f'{type(error).__name__}: {error}'
                if self.errors.get(topic) != detail:
                    self.get_logger().warning(f'网页点云 {topic}：{detail}')
                self.errors[topic] = detail
        if now - self.status_at >= 2.0:
            self._publish_status(now)

    def _tick_topic(self, topic, now):
        demanded = [profile for profile in PROFILES
                    if self.outputs[topic, profile].get_subscription_count() > 0]
        if not demanded:
            if topic in self.inputs and topic not in self.static_topics:
                self.destroy_subscription(self.inputs.pop(topic))
            if topic not in self.static_topics:
                self.latest.pop(topic, None)
            for profile in PROFILES:
                self.sent.pop((topic, profile), None)
            return
        if topic not in self.inputs:
            self._subscribe_input(topic)
        if topic not in self.latest:
            return
        message, received = self.latest[topic]
        for profile in demanded:
            key = topic, profile
            previous = self.sent.get(key)
            period = 0.5 if topic == '/cloud_mapping_dynamic' else PROFILES[profile][2]
            if previous and (now - previous[1] < period or
                    (topic not in self.static_topics and previous[0] == received)):
                continue
            center = None
            if (topic == '/cloud_mapping_dynamic' and self.pose
                    and self.pose[0] == message.header.frame_id and now - self.pose[3] < 1.0):
                center = self.pose[1:3]
            cached = self.previews.get(key)
            if topic in self.static_topics and cached and cached[0] is message:
                preview = cached[1]
            else:
                preview = thin_cloud(message, profile, center=center)
                if topic in self.static_topics:
                    self.previews[key] = message, preview
            # Old rosbridge VOLATILE readers need a new publication after
            # reconnect. Static replay is bounded and reuses the encoding.
            self.outputs[key].publish(preview)
            self.sent[key] = received, now
            self.published[key] = self.published.get(key, 0) + 1
            self.errors.pop(topic, None)

    def _publish_status(self, now):
        topics = {}
        for topic in CLOUD_TOPICS:
            latest = self.latest.get(topic)
            topics[topic] = {
                'input_subscribed': topic in self.inputs,
                'input_policy': 'reliable_transient_local' if topic in self.static_topics
                                else 'best_effort_volatile',
                'has_frame': latest is not None,
                'source_age_seconds': round(now - latest[1], 3) if latest else None,
                'width': int(latest[0].width) if latest else None,
                'viewers': {p: self.outputs[topic, p].get_subscription_count() for p in PROFILES},
                'published': {p: self.published.get((topic, p), 0) for p in PROFILES},
                'error': self.errors.get(topic, ''),
            }
        self.status_publisher.publish(String(data=json.dumps({
            'topics': topics, 'saved_map_http': self.saved_map_http_status}, ensure_ascii=False)))
        self.status_at = now


def main(args=None):
    rclpy.init(args=args)
    node = WebCloudPreview()
    server = None
    try:
        try:
            from car_vel.saved_map_http import SavedMapRepository, SavedMapServer, DEFAULT_DIRECTORY
            directory = node.declare_parameter('saved_map_directory', DEFAULT_DIRECTORY).value
            port = int(node.declare_parameter('saved_map_port', 8769).value)
            bind = node.declare_parameter('saved_map_bind', '0.0.0.0').value
            server = SavedMapServer((bind, port), SavedMapRepository(directory, node.saved_map_state))
            threading.Thread(target=server.serve_forever, daemon=True, name='saved-map-http').start()
            node.saved_map_http_status = {'ready': True, 'port': port, 'error': ''}
            node.get_logger().info(f'完整保存地图只读HTTP：{bind}:{port}；源目录 {directory}')
        except (OSError, ImportError) as error:
            node.saved_map_http_status = {'ready': False, 'error': str(error)}
            node.get_logger().error(f'完整地图HTTP未启动，建图实时预览继续工作：{error}')
        rclpy.spin(node)
    finally:
        if server:
            server.shutdown()
            server.server_close()
        node.destroy_node()
        rclpy.shutdown()
