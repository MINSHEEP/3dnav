#!/usr/bin/env python3
"""Deployment contract for CPU-only SFace/HyperLPR recognition."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class RecognitionDeploymentTests(unittest.TestCase):

    def test_requirements_and_installer_supply_models_and_runtime_selfcheck(self):
        requirements = (ROOT / 'requirements-hikvision-camera.txt').read_text(
            encoding='utf-8').lower()
        usb_requirements = (ROOT / 'requirements-usb-camera.txt').read_text(
            encoding='utf-8').lower()
        installer = (ROOT / 'scripts' / 'install_hikvision_camera_deps.sh').read_text(
            encoding='utf-8')
        for content in (requirements, usb_requirements):
            self.assertIn('hyperlpr3', content)
            self.assertIn('onnxruntime', content)
        self.assertIn('face_recognition_sface_2021dec.onnx', installer)
        self.assertIn('0ba9fbfa01b5270c96627c4ef784da859931e02f04419c829e83484087c34e79', installer)
        self.assertIn('cv2.FaceRecognizerSF.create', installer)
        self.assertIn('LicensePlateCatcher', installer)
        self.assertIn('onnxruntime', installer)

    def test_usb_documentation_describes_matching_and_independent_database(self):
        documentation = (ROOT / 'USB_CAMERA.md').read_text(encoding='utf-8')
        self.assertNotIn('目前只检测位置并画框，不做人脸身份比对', documentation)
        self.assertIn('~/.ros/robot_navigation/recognition.db', documentation)
        self.assertIn('recognition_records.html', documentation)
        self.assertIn('/home/a/3dnav', documentation)

    def test_face_registration_request_allows_bounded_image_payloads(self):
        gateway = (ROOT / 'hikvision_camera' / 'camera_web.py').read_text(
            encoding='utf-8')
        self.assertIn('path == "/api/recognition/person/upsert"', gateway)
        self.assertIn('32 * 1024 * 1024', gateway)
        self.assertIn('self._read_json(max_bytes)', gateway)


if __name__ == '__main__':
    unittest.main()
