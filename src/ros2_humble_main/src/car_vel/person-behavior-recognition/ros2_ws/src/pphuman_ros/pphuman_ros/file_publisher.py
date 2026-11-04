"""Explicit offline source; never opens a camera or saves frames."""
import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .common import IMAGE_QOS, image_message, node_lease


class FilePublisher(Node):
    def __init__(self, args):
        super().__init__('pphuman_file_publisher')
        self.lease = node_lease('file_publisher')
        self.args, self.count = args, 0
        self.loop_index = 0
        self.cap, self.still = None, None
        self.publisher = self.create_publisher(Image, args.topic, IMAGE_QOS)
        if args.file:
            path = Path(args.file).resolve(strict=True)
            if not path.is_file():
                raise ValueError('--file 仅接受普通文件，不能是摄像头设备')
            self.still = cv2.imread(str(path)) if path.suffix.lower() in ('.png', '.jpg', '.jpeg', '.bmp', '.webp') else None
            if self.still is None:
                self.cap = cv2.VideoCapture(str(path), cv2.CAP_FFMPEG)
                if not self.cap.isOpened():
                    raise ValueError('Cannot read offline file: ' + str(path))
        source_fps = self.cap.get(cv2.CAP_PROP_FPS) if self.cap else 10.0
        if not math.isfinite(source_fps) or source_fps <= 0:
            source_fps = 10.0
            self.get_logger().warning('视频源 FPS 不可用，默认 10Hz；请用 --fps 明确设置')
        effective_fps = args.fps if args.fps is not None else source_fps
        if self.cap and args.fps is not None and abs(args.fps - source_fps) > .1:
            self.get_logger().warning(f'视频源 {source_fps:.3f} FPS，指定 {args.fps:.3f} Hz 将改变播放/动作速度')
        self.get_logger().info(f'离线源发布频率 {effective_fps:.3f} Hz，不保存视频')
        self.timer = self.create_timer(1 / effective_fps, self.publish)

    def publish(self):
        if self.count_publishers(self.args.topic) > 1:
            raise RuntimeError('话题已有发布者，离线源拒绝混流')
        if self.args.frames and self.count >= self.args.frames:
            raise KeyboardInterrupt
        if self.args.synthetic:
            frame = np.zeros((self.args.height, self.args.width, 3), np.uint8)
            cv2.rectangle(frame, (20 + self.count % 100, 80), (130 + self.count % 100, 220), (0, 180, 240), 3)
            cv2.putText(frame, f'SYNTHETIC TRANSPORT TEST {self.count}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
        elif self.still is not None:
            frame = self.still
        else:
            ok, frame = self.cap.read()
            if not ok:
                if not self.args.loop:
                    raise KeyboardInterrupt
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.loop_index += 1
                ok, frame = self.cap.read()
                if not ok:
                    raise RuntimeError('Offline video restart failed')
        self.publisher.publish(image_message(frame, self.get_clock().now().nanoseconds, f'offline_test_loop_{self.loop_index}'))
        self.count += 1

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        self.lease.close()
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--synthetic', action='store_true')
    source.add_argument('--file')
    parser.add_argument('--loop', action='store_true')
    parser.add_argument('--topic', default='/pp_human/image_raw')
    parser.add_argument('--fps', type=float, default=None, help='视频默认源 FPS；合成/静态图默认 10Hz。显式覆盖会改变视频播放速度。')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=360)
    parser.add_argument('--frames', type=int, default=0)
    args, ros_args = parser.parse_known_args()
    if (args.fps is not None and (not math.isfinite(args.fps) or args.fps <= 0)) or args.width < 1 or args.height < 1 or args.frames < 0:
        parser.error('fps/width/height 必须为正数，frames 不能为负数')
    rclpy.init(args=ros_args)
    node = None
    try:
        node = FilePublisher(args)
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
