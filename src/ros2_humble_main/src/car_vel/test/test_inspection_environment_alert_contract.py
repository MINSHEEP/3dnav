import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_PACKAGES = ROOT.parent
LZP_SERVICE = (
    WORKSPACE_PACKAGES / 'nav_service' / 'lzp_service' / 'lzp_service')
LZP_INTERFACE = WORKSPACE_PACKAGES / 'nav_service' / 'lzp_interface'


class InspectionEnvironmentAlertContractTests(unittest.TestCase):
    def test_ros_contract_is_generated_and_wired(self):
        cmake = (LZP_INTERFACE / 'CMakeLists.txt').read_text(encoding='utf-8')
        service = (
            LZP_INTERFACE / 'srv' / 'SetInspectionEnvironmentConfig.srv'
        ).read_text(encoding='utf-8')
        manager = (LZP_SERVICE / 'inspection_task_manager.py').read_text(
            encoding='utf-8')
        self.assertIn('SetInspectionEnvironmentConfig.srv', cmake)
        self.assertIn('float64 co2_threshold_ppm', service)
        self.assertIn("'/air_quality/m701'", manager)
        self.assertIn("'/inspection/environment_config'", manager)
        self.assertIn("'/inspection/set_environment_config'", manager)
        self.assertIn("payload.get('warming_up') is not True", manager)
        self.assertIn('age <= self._environment_sample_max_age', manager)
        self.assertIn(
            'self._m701_sample_sequence != '
            'self._last_environment_sample_sequence', manager)

    def test_fixed_speech_chain_has_no_arbitrary_text_endpoint(self):
        manager = (LZP_SERVICE / 'inspection_task_manager.py').read_text(
            encoding='utf-8')
        camera = (ROOT / 'hikvision_camera' / 'camera_web.py').read_text(
            encoding='utf-8')
        yolo = (ROOT / 'hikvision_camera' / 'yolo_hk.py').read_text(
            encoding='utf-8')
        voice = (ROOT / 'hikvision_camera' / 'yolo_hk_voice.py').read_text(
            encoding='utf-8')
        self.assertIn("'/api/inspection/speech'", manager)
        self.assertIn('def inspection_speech', camera)
        self.assertIn('"thermal_high", "co2_high"', camera)
        self.assertIn("'/inspection/speech'", yolo)
        self.assertIn('self.environment_speech(key)', yolo)
        self.assertIn('self._pending_environment_speech', yolo)
        self.assertIn('"环境温度异常"', voice)
        self.assertIn('"二氧化碳浓度异常"', voice)
        self.assertIn('def announce_texts', voice)

    def test_settings_page_exposes_persisted_co2_threshold(self):
        page = (ROOT / 'car_vel' / 'web_frontend' / 'settings.html').read_text(
            encoding='utf-8')
        common = (
            ROOT / 'car_vel' / 'web_frontend' / 'robot_ui_common.js'
        ).read_text(encoding='utf-8')
        launch = (ROOT / 'launch' / 'launch_manager.launch.py').read_text(
            encoding='utf-8')
        self.assertIn('id="co2-threshold-input"', page)
        self.assertIn('min="400" max="10000"', page)
        self.assertIn("'/inspection/environment_config'", page)
        self.assertIn("'/inspection/set_environment_config'", page)
        self.assertIn('setInspectionEnvironmentConfig', common)
        self.assertIn("'environment_alert_config_path'", launch)


if __name__ == '__main__':
    unittest.main()
