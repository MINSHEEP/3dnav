import os
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .common import IMAGE_QOS, decode_image, node_lease, parameter
from .overlay import Overlay


class PreviewClosed(Exception):
    """An explicit preview-only close, not a request to stop recognition."""


class View(Node):
    def __init__(self):
        super().__init__('pphuman_view')
        if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
            raise RuntimeError('预览需要图形桌面。请在目标机桌面运行，或在已配置 X11 的终端运行。')
        self.lease = node_lease('view')
        self.topic = parameter(self, 'annotated_topic', '/pp_human/annotated_image')
        self.stale = float(parameter(self, 'stale_seconds', 3.0))
        self.display_fps = max(1, min(60, float(parameter(self, 'display_fps', 20.0))))
        self.display_max_width = max(0, int(parameter(self, 'display_max_width', 1280)))
        self.overlay = Overlay(parameter(self, 'font_path', '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'))
        self.image = self.overlay.unavailable(text='等待识别结果；Q / Esc 仅关闭预览')
        self.stale_image = self.overlay.unavailable()
        self.pending_message = None
        self.received = 0
        self.sub = self.create_subscription(Image, self.topic, self.on_image, IMAGE_QOS)
        self.timer = self.create_timer(1 / self.display_fps, self.show)
        self.title = 'PP-HumanV2 | Q / Esc: close preview only'
        cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
        self.window_observed = False
        self.window_query_warning = False
        self.close_requested = False
        self.gtk_window = None
        self.bind_gtk_close()
        self.get_logger().info('预览已启动；Q / Esc 或关闭窗口只关闭预览。全部停止请在 start_all 终端按 Ctrl+C。')

    def bind_gtk_close(self):
        gui_line = next((line for line in cv2.getBuildInformation().splitlines() if line.strip().startswith('GUI:')), '')
        if 'GTK' not in gui_line:
            return
        try:
            import gi
            gi.require_version('Gtk', '3.0')
            from gi.repository import Gtk
            for window in Gtk.Window.list_toplevels():
                if window.get_title() == self.title:
                    self.gtk_window = window
                    window.connect('destroy', self.on_window_destroy)
                    return
            self.get_logger().warning('GTK 关闭事件尚未连接；可按 Q / Esc 关闭预览。')
        except (ImportError, ValueError) as exc:
            self.get_logger().warning(f'GTK 关闭事件不可用（{exc}）；可按 Q / Esc 关闭预览。')

    def on_window_destroy(self, widget):
        self.close_requested = True

    def on_image(self, msg):
        self.pending_message = msg
        self.received = time.monotonic()

    def show(self):
        # Check before imshow: otherwise HighGUI recreates a window the user closed.
        self.check_window_closed()
        if self.pending_message is not None:
            self.image = decode_image(self.pending_message)
            self.pending_message = None
            if self.display_max_width and self.image.shape[1] > self.display_max_width:
                height = max(1, round(self.image.shape[0] * self.display_max_width / self.image.shape[1]))
                self.image = cv2.resize(self.image, (self.display_max_width, height), interpolation=cv2.INTER_AREA)
        image = self.image if time.monotonic() - self.received < self.stale else self.stale_image
        cv2.imshow(self.title, image)
        key = cv2.waitKey(1) & 0xff
        if key in (27, ord('q'), ord('Q')):
            raise PreviewClosed('用户按 Q / Esc 关闭预览；采集和识别继续运行')
        self.check_window_closed()

    def check_window_closed(self):
        if self.close_requested:
            raise PreviewClosed('用户关闭预览窗口；采集和识别继续运行')
        # OpenCV 4.5.4 GTK3 returns -1 for WND_PROP_VISIBLE even for a live window.
        # getWindowImageRect is supported here. Never infer close before observing it.
        try:
            _, _, width, height = cv2.getWindowImageRect(self.title)
        except cv2.error:
            width = height = -1
        if width > 0 and height > 0:
            self.window_observed = True
        elif self.window_observed and (width < 0 or height < 0):
            raise PreviewClosed('用户关闭预览窗口；采集和识别继续运行')
        elif not self.window_observed and not self.window_query_warning:
            # Startup/unmapped/unsupported is not evidence of a user closing anything.
            self.window_query_warning = True
            self.get_logger().info('窗口尚未返回有效区域，等待显示；Q / Esc 始终可关闭预览。')

    def destroy_node(self):
        try:
            cv2.destroyWindow(self.title)
            cv2.waitKey(1)
        except cv2.error:
            pass
        self.gtk_window = None
        self.lease.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = View()
        rclpy.spin(node)
    except PreviewClosed as exc:
        if node:
            node.get_logger().info(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
