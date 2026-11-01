"""Optional compatibility test against the actual cyber1_vc source.

Set CYBER1_VC_TEST_ROOT to its directory. Cloud/audio/UDP constructors are
replaced before creating the application; this test never opens a device.
"""

import base64
import copy
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_vel.voice_debug_server import DebugController, PHRASES, WebVoiceInput
from car_vel.voice_debug_navigation import DebugNavigation

VOICE_ROOT = Path(os.environ.get("CYBER1_VC_TEST_ROOT", str(Path.home() / "cyber1_vc")))


@unittest.skipUnless((VOICE_ROOT / "robot" / "app.py").is_file(), "cyber1_vc source not supplied")
class ExistingVoiceProgramTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(VOICE_ROOT))
        # Import-only dependencies; no cloud SDK operation is called.
        for dependency in ("requests", "websocket"):
            if dependency not in sys.modules and importlib.util.find_spec(dependency) is None:
                sys.modules[dependency] = types.ModuleType(dependency)
        from robot.app import RobotApp
        from robot.defaults import DEFAULTS
        cls.app_type = type("WebRobotApp", (WebVoiceInput, RobotApp), {})
        cls.defaults = DEFAULTS

    def setUp(self):
        cfg = copy.deepcopy(self.defaults)
        cfg.update(enable_local_music=False, enable_local_voice_stop=False,
                   enable_keyword_interrupt=False, enable_vision=False,
                   audio_settings_file="", auto_stand_on_start=False)
        with patch("robot.app.DogActionSender"), patch("robot.app.AlsaPlayer"):
            self.app = self.app_type(cfg)
        self.app.ws = Mock()
        self.app.ws_connected.set()
        self.app.player.is_playback_active.return_value = False
        self.app.web_renew(time.monotonic() + 10)

    def test_real_program_accepts_both_texts_and_plays_responses_with_mic_off(self):
        self.app.cfg["enable_keyword_interrupt"] = True
        self.app._keyword_turn_gate.interrupt()
        for number, text in enumerate(PHRASES.values()):
            self.app.web_text(text)
            packet = json.loads(self.app.ws.send.call_args.args[0])
            self.assertEqual(packet["contentType"], "TEXT")
            self.assertEqual(packet["content"], {"input": text})
            round_id = f"test-round-{number}"
            self.app._handle({"code": 200, "contentType": "EVENT", "mid": f"server-{number}", "content": {
                "roundId": round_id, "eventType": "CALL_AGENT_START_EVENT", "eventData": {"input": text}}})
            self.app._handle({"code": 200, "contentType": "AGENT", "content": {
                "roundId": round_id, "content": f"回答{number}"}})
            self.app._handle({"code": 200, "contentType": "TTS", "content": {
                "roundId": round_id, "audioBase64": base64.b64encode(b"\x00\x00").decode("ascii")}})
            self.assertEqual(self.app._web_reply, f"回答{number}")
            self.assertFalse(self.app.web_microphone_enabled)
        self.assertEqual(self.app.player.enqueue.call_count, 2)

    def test_real_microphone_callback_and_final_upload_are_both_gated(self):
        pcm = b"\x00\x00" * 960
        self.app._mic_callback(pcm)
        self.app.ws.send.assert_not_called()
        self.app.web_microphone(True)
        self.app._mic_callback(pcm)
        packet = json.loads(self.app.ws.send.call_args.args[0])
        self.assertEqual(packet["contentType"], "AUDIO")
        self.assertEqual(base64.b64decode(packet["content"]["audioBase64"]), pcm)
        self.app.ws.send.reset_mock()
        self.app.web_microphone(False)
        self.app._mic_callback(pcm)
        self.assertFalse(self.app._send_audio_frame(pcm))
        self.app.ws.send.assert_not_called()
        self.app.web_text(PHRASES["first"])
        self.assertEqual(json.loads(self.app.ws.send.call_args.args[0])["contentType"], "TEXT")

    def test_existing_program_sender_accepts_explicit_obstacle_mode_initialization(self):
        self.app.web_initialize_motion_mode()
        self.app.action_sender.stop_motion.assert_called_once_with()
        self.app.action_sender._send.assert_called_once_with({
            "msg_type": 3, "cmd_code": 2, "para": 0,
        })
        self.app.action_sender.stand_up.assert_not_called()
        self.app.action_sender.lie_down.assert_not_called()

    def test_navigation_preparation_can_stand_through_the_real_action_policy(self):
        from robot.motion import DogActionSender
        # Keep the actual stand_up -> action-policy call chain; replace only IO.
        sender = object.__new__(DogActionSender)
        sender._work_context = threading.local()
        sender._can_send_motion = Mock(return_value=True)
        sender.capture_generation = Mock(return_value=1)
        sender._motion_mode, sender._motion_mode_time = 1, time.monotonic()
        sender._movement_active = False
        sender.runtime_state = types.SimpleNamespace(snapshot=lambda: {'state': 'LYING'})
        sender._send = Mock()
        sender._speak = Mock()

        def confirm_stand(*_args):
            sender._motion_mode, sender._motion_mode_time = 2, time.monotonic()
            return True

        sender._set_posture = Mock(side_effect=confirm_stand)
        sender._action_policy_callback = self.app._action_policy
        self.app.action_sender = sender
        self.app._web_navigation = True
        controller = DebugController(self.app)
        runtime = types.SimpleNamespace(start=Mock(side_effect=RuntimeError('ROS_START_REACHED')))
        navigation = DebugNavigation(controller, runtime)
        with self.assertRaisesRegex(RuntimeError, 'ROS_START_REACHED'):
            navigation._prepare(navigation.generation, threading.Event())
        sender._set_posture.assert_called_once()
        self.assertFalse(hasattr(sender._work_context, 'web_navigation_stand'))


if __name__ == "__main__":
    unittest.main()
