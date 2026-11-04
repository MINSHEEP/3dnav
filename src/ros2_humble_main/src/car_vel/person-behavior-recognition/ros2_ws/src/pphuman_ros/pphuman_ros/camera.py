import os
import subprocess
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .common import IMAGE_QOS, Lease, image_message, json_message, node_lease, parameter


def device_busy(device):
    # Read-only owner check. Never send fuser -k, signals, or open another device.
    completed = subprocess.run(['fuser', device], capture_output=True, text=True, timeout=3)
    if completed.returncode not in (0, 1):
        raise RuntimeError('无法检查摄像头拥有者: ' + completed.stderr.strip())
    return completed.stdout.strip()


class Camera(Node):
    def __init__(self):
        super().__init__('pphuman_camera')
        self.lease = node_lease('camera')
        p = lambda name, default: parameter(self, name, default)
        self.device = os.path.realpath(p('device', '/dev/video0'))
        self.device_lease = Lease('camera:' + self.device)
        self.width, self.height = int(p('width', 1280)), int(p('height', 720))
        self.fps = float(p('fps', 30.0))
        self.fourcc = p('fourcc', 'MJPG')
        self.topic = p('image_topic', '/pp_human/image_raw')
        self.frame_id = p('frame_id', 'usb_camera')
        self.retry = max(.5, float(p('reconnect_seconds', 2.0)))
        if self.width <= 0 or self.height <= 0 or self.fps <= 0 or len(self.fourcc) != 4:
            raise ValueError('Invalid camera dimensions/fps/fourcc')
        self.publisher = self.create_publisher(Image, self.topic, IMAGE_QOS)
        self.status = self.create_publisher(String, '/pp_human/camera_status', 1)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.state = 'checking_ownership'
        self.last_frame = 0
        self.thread = threading.Thread(target=self.capture, name='usb-capture', daemon=True)
        self.thread.start()
        self.timer = self.create_timer(1 / self.fps, self.tick)
        self.health_timer = self.create_timer(1.0, self.health)

    def capture(self):
        # Allow DDS discovery before checking the topic's existing publisher.
        if self.stop.wait(1.0):
            return
        while not self.stop.is_set():
            cap = None
            try:
                if self.count_publishers(self.topic) > 1:
                    raise RuntimeError('输入话题已有发布者，拒绝重复采集')
                owners = device_busy(self.device)
                if owners:
                    raise RuntimeError('摄像头已被进程占用: ' + owners)
                cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
                if not cap.isOpened():
                    raise RuntimeError('摄像头打开失败: ' + self.device)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self.get_logger().info(f'USB采集 {self.device}: {cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x'
                                       f'{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} @ {cap.get(cv2.CAP_PROP_FPS):.1f}')
                while not self.stop.is_set():
                    ok, frame = cap.read()
                    if not ok:
                        raise RuntimeError('USB读取失败，等待重连')
                    stamp = self.get_clock().now().nanoseconds
                    with self.lock:
                        self.latest = (frame, stamp)
                        self.last_frame = time.monotonic()
                        self.state = 'streaming'
            except Exception as exc:
                with self.lock:
                    self.latest = None
                    self.state = str(exc)
                self.get_logger().warning(str(exc))
            finally:
                if cap is not None:
                    cap.release()
            self.stop.wait(self.retry)

    def tick(self):
        with self.lock:
            latest, self.latest = self.latest, None
        if latest:
            image, stamp = latest
            self.publisher.publish(image_message(image, stamp, self.frame_id))

    def health(self):
        with self.lock:
            state = self.state
            age = time.monotonic() - self.last_frame if self.last_frame else None
        self.status.publish(json_message(dict(component='camera', state=state,
            device=self.device, topic=self.topic, last_frame_age_seconds=age,
            connected=age is not None and age < 2.0, video_saved=False)))

    def destroy_node(self):
        self.stop.set()
        self.thread.join(timeout=3)
        self.device_lease.close()
        self.lease.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Camera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
