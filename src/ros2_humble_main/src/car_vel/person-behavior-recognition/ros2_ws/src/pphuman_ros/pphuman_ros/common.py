from array import array
import fcntl
import hashlib
import json
import os
from pathlib import Path

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

IMAGE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.BEST_EFFORT)


def project_root():
    return Path(os.environ.get('PPHUMAN_ROOT', '/home/a/PP-HumanV2')).resolve()


class Lease:
    def __init__(self, identity):
        key = hashlib.sha256(identity.encode()).hexdigest()[:24]
        self.path = Path('/tmp') / ('pphuman-' + key + '.lock')
        self.file = self.path.open('a+')
        try:
            fcntl.flock(self.file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.file.close()
            raise RuntimeError('已有拥有者，拒绝重复启动: ' + identity)
        self.file.seek(0)
        self.file.truncate()
        self.file.write(str(os.getpid()))
        self.file.flush()

    def close(self):
        self.file.close()


def node_lease(role):
    return Lease(str(project_root()) + ':' + os.environ.get('ROS_DOMAIN_ID', '0') + ':' + role)


def decode_image(msg):
    channels = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        raise ValueError('Unsupported encoding: ' + msg.encoding)
    if msg.height < 1 or msg.width < 1 or msg.step < msg.width * channels or len(msg.data) < msg.step * msg.height:
        raise ValueError('Invalid image geometry/step/data')
    pixels = np.frombuffer(msg.data, np.uint8, count=msg.height * msg.step).reshape(msg.height, msg.step)
    pixels = pixels[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding == 'mono8':
        pixels = np.repeat(pixels, 3, axis=2)
    elif msg.encoding in ('rgb8', 'rgba8'):
        pixels = pixels[:, :, :3][:, :, ::-1]
    else:
        pixels = pixels[:, :, :3]
    return np.ascontiguousarray(pixels)


def image_message(image, stamp_ns, frame_id):
    msg = Image()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(int(stamp_ns), 1_000_000_000)
    msg.header.frame_id = frame_id
    msg.height, msg.width = image.shape[:2]
    msg.encoding, msg.is_bigendian, msg.step = 'bgr8', 0, msg.width * 3
    # array('B') avoids generated ROS setters validating millions of Python ints.
    msg.data = array('B', image.tobytes())
    return msg


def json_message(value):
    return String(data=json.dumps(value, ensure_ascii=False, allow_nan=False))


def parameter(node, name, default):
    return node.declare_parameter(name, default).value
