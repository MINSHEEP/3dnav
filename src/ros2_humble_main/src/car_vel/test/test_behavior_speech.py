import importlib.util
from pathlib import Path
import threading
import unittest
from unittest.mock import patch

MODULE = Path(__file__).resolve().parents[1] / 'hikvision_camera' / 'behavior_speech.py'
spec = importlib.util.spec_from_file_location('behavior_speech', MODULE)
speech = importlib.util.module_from_spec(spec)
spec.loader.exec_module(speech)


class BehaviorSpeechTests(unittest.TestCase):
    def test_no_audio_backend_is_reported_without_breaking_recognition(self):
        with patch.object(speech.shutil, 'which', return_value=None):
            service = speech.BehaviorSpeechService()
        try:
            self.assertFalse(service.status()['ready'])
            self.assertFalse(service.announce('fall'))
        finally:
            service.close()

    def test_bundled_assets_use_local_player(self):
        with patch.object(speech.shutil, 'which', side_effect=lambda name: '/usr/bin/ffplay' if name == 'ffplay' else None):
            service = speech.BehaviorSpeechService()
        try:
            self.assertTrue(service.status()['ready'])
            for key in speech.PHRASES:
                command = service._command(key)
                self.assertEqual(command[0], '/usr/bin/ffplay')
                self.assertTrue(Path(command[-1]).is_file())
        finally:
            service.close()

    def test_only_fixed_phrases_and_cooldown(self):
        played = threading.Event()
        class Process:
            def wait(self, timeout=None):
                played.set()
                return 0
            def poll(self):
                return 0
        with patch.object(speech.shutil, 'which', side_effect=lambda name: '/usr/bin/espeak-ng' if name == 'espeak-ng' else None), \
                patch.object(speech.subprocess, 'Popen', return_value=Process()) as popen:
            service = speech.BehaviorSpeechService()
            try:
                self.assertFalse(service.announce('arbitrary shell text'))
                self.assertTrue(service.announce('fall'))
                self.assertFalse(service.announce('fall'))
                self.assertTrue(played.wait(1.0))
                self.assertEqual(popen.call_args.args[0][-1], '检测到疑似人员跌倒')
                self.assertNotIn('shell', popen.call_args.kwargs)
                service.reset()
                self.assertTrue(service.announce('fall'))
            finally:
                service.close()


if __name__ == '__main__':
    unittest.main()
