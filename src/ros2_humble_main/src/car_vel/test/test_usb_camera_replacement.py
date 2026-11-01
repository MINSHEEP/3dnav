#!/usr/bin/env python3
"""Regression contract for the fixed USB camera product mode."""

from pathlib import Path
import unittest


SOURCE_ROOT = Path(__file__).resolve().parents[2]
CAR_VEL_ROOT = SOURCE_ROOT / 'car_vel'
CAMERA_ROOT = CAR_VEL_ROOT / 'hikvision_camera'
WEB_ROOT = CAR_VEL_ROOT / 'car_vel' / 'web_frontend'
INSPECTION_MANAGER = (
    SOURCE_ROOT / 'nav_service' / 'lzp_service' / 'lzp_service'
    / 'inspection_task_manager.py'
)
DEFAULT_USB_CAMERA_DEVICE = (
    '/dev/v4l/by-id/'
    'usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0'
)


class UsbCameraReplacementTests(unittest.TestCase):

    def test_launch_defaults_to_v4l2_without_hikvision_credentials(self):
        launch = (CAR_VEL_ROOT / 'launch' / 'launch_manager.launch.py').read_text(
            encoding='utf-8')

        self.assertIn("'camera_device'", launch)
        self.assertIn(
            f"DEFAULT_USB_CAMERA_DEVICE = '{DEFAULT_USB_CAMERA_DEVICE}'",
            launch,
        )
        self.assertIn('default_value=DEFAULT_USB_CAMERA_DEVICE', launch)
        self.assertNotIn("default_value='/dev/video0'", launch)
        self.assertIn("'camera_width'", launch)
        self.assertIn("default_value='1280'", launch)
        self.assertIn("'camera_height'", launch)
        self.assertIn("default_value='720'", launch)
        self.assertIn("'camera_fps'", launch)
        self.assertIn("default_value='30'", launch)
        self.assertNotIn('HIKVISION_CAMERA_PASSWORD', launch)
        self.assertNotIn("'camera_host'", launch)
        self.assertNotIn("'camera_username'", launch)

    def test_standalone_gateway_uses_the_same_non_thermal_camera_default(self):
        gateway = (CAMERA_ROOT / 'camera_web.py').read_text(encoding='utf-8')

        self.assertIn(
            f'DEFAULT_USB_CAMERA_DEVICE = "{DEFAULT_USB_CAMERA_DEVICE}"',
            gateway,
        )
        self.assertIn(
            'parser.add_argument("--usb-device", '
            'default=DEFAULT_USB_CAMERA_DEVICE',
            gateway,
        )
        self.assertNotIn(
            'parser.add_argument("--usb-device", default="/dev/video0"',
            gateway,
        )

    def test_camera_and_yolo_use_one_v4l2_owner_and_local_mjpeg(self):
        gateway = (CAMERA_ROOT / 'camera_web.py').read_text(encoding='utf-8')
        yolo = (CAMERA_ROOT / 'yolo_hk.py').read_text(encoding='utf-8')
        voice_yolo = (CAMERA_ROOT / 'yolo_hk_voice.py').read_text(
            encoding='utf-8')

        self.assertIn('class V4L2MjpegSource', gateway)
        self.assertIn('camera_type == "usb"', gateway)
        self.assertIn('"backend": "v4l2"', gateway)
        self.assertIn('"ptz": False', gateway)
        self.assertIn('class MjpegSource', yolo)
        self.assertIn('"mjpeg": MjpegSource', yolo)
        self.assertIn('choices=("mjpeg", "jpeg", "rtsp", "v40", "auto")',
                      voice_yolo)
        self.assertIn('args.capture_backend != "mjpeg"', voice_yolo)
        self.assertIn('--mjpeg-url', gateway)
        self.assertIn('/api/stream', gateway)
        self.assertIn('--max-stream-clients", type=int, default=6', gateway)
        self.assertIn('YOLO 本身会占用一个连接', gateway)

    def test_product_pages_expose_fixed_visible_and_thermal_cameras(self):
        camera = (WEB_ROOT / 'camera.html').read_text(encoding='utf-8')
        remote = (WEB_ROOT / 'remote_control.html').read_text(encoding='utf-8')
        inspection = (WEB_ROOT / 'inspection.html').read_text(encoding='utf-8')
        backend_page = (CAMERA_ROOT / 'camera_control.html').read_text(
            encoding='utf-8')

        self.assertIn('USB相机', camera)
        self.assertIn('启动识别', camera)
        self.assertIn('查看识别画面', camera)
        self.assertIn('/api/detection', camera)
        self.assertIn('/api/thermal/stream.mjpg', camera)
        self.assertIn('默认高温报警阈值', camera)
        self.assertIn('打开当前帧', camera)
        for removed in ('data-motion=', 'data-ir=', 'wiper-on', '云台速度'):
            self.assertNotIn(removed, camera)
        self.assertNotIn('data-camera-motion=', remote)
        self.assertNotIn('data-camera-ir=', remote)
        self.assertIn('/api/thermal/stream.mjpg', remote)
        self.assertNotIn('direction-choices', inspection)
        self.assertNotIn('变倍倍率（留空保持当前）', inspection)
        self.assertIn('本次任务高温报警阈值', inspection)
        self.assertIn('/api/thermal/stream.mjpg', inspection)
        self.assertIn('USB 固定摄像头', backend_page)
        for removed in ('data-motion=', 'data-ir=', 'wiper-on', '云台速度'):
            self.assertNotIn(removed, backend_page)

    def test_thermal_sdk_has_one_owner_and_browser_uses_same_origin_proxy(self):
        thermal_web = (CAMERA_ROOT / 'thermal_web.py').read_text(encoding='utf-8')
        thermal_client = (CAMERA_ROOT / 'thermal_client.py').read_text(encoding='utf-8')
        gateway = (CAMERA_ROOT / 'camera_web.py').read_text(encoding='utf-8')

        self.assertIn('class HCUsbProducer', thermal_web)
        self.assertIn('demo.HCUsbSDK', thermal_web)
        self.assertNotIn('HCUsbSDK', thermal_client)
        self.assertIn('class ThermalHttpClient', thermal_client)
        self.assertIn('open_thermal_stream', gateway)
        self.assertIn('stream_thermal_mjpeg', gateway)
        self.assertIn('"/api/thermal/config"', gateway)

        launch = (CAR_VEL_ROOT / 'launch' / 'launch_manager.launch.py').read_text(
            encoding='utf-8')
        self.assertIn("'thermal_enabled'", launch)
        self.assertIn("default_value='/opt/hik_thermal_linux64_bundle'", launch)
        self.assertIn("'thermal_camera_web'", launch)
        self.assertIn("'--listen', '127.0.0.1'", launch)
        self.assertIn("'--thermal-url', LaunchConfiguration('thermal_api_url')", launch)

        for page in WEB_ROOT.glob('*.html'):
            content = page.read_text(encoding='utf-8')
            if 'href="camera.html"' in content:
                self.assertIn('href="camera.html">USB相机</a>', content)

    def test_new_inspection_templates_and_runtime_are_fixed_view(self):
        manager = INSPECTION_MANAGER.read_text(encoding='utf-8')

        self.assertIn("'directions': ['current']", manager)
        self.assertIn("'zoom_ratio': None", manager)
        checkpoint = manager.split('def _run_checkpoint(', 1)[1].split(
            'def _drive_inspection_future(', 1)[0]
        self.assertNotIn('self._camera.move(', checkpoint)
        self.assertNotIn('self._camera.set_zoom(', checkpoint)
        self.assertIn("'direction': 'current'", checkpoint)


if __name__ == '__main__':
    unittest.main()
