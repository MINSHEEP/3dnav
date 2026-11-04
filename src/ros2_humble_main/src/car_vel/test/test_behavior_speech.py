import importlib.util
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

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

    def test_smoking_uses_fixed_suspected_behavior_phrase(self):
        with patch.object(speech.shutil, 'which', side_effect=lambda name: '/usr/bin/espeak-ng' if name == 'espeak-ng' else None):
            service = speech.BehaviorSpeechService()
        try:
            self.assertEqual(service._command('smoking')[-1], '检测到疑似人员吸烟')
            self.assertIsNone(service._command('cigarettes'))
        finally:
            service.close()

    def test_inspection_smoking_plays_exact_notice_with_cooldown_and_long_deadline(self):
        played = threading.Event()
        process = Mock()
        process.poll.return_value = 0
        def complete(timeout=None):
            played.set()
            return 0
        process.wait.side_effect = complete
        with patch.object(speech.shutil, 'which', side_effect=lambda name: '/usr/bin/espeak-ng' if name == 'espeak-ng' else None), \
                patch.object(speech.subprocess, 'Popen', return_value=process) as popen:
            service = speech.BehaviorSpeechService()
            try:
                self.assertTrue(service.announce('smoking', inspection=True))
                self.assertFalse(service.announce('smoking', inspection=True))
                self.assertTrue(played.wait(1))
                self.assertEqual(popen.call_args.args[0][-1], '吸烟有害健康，请不要在公共场合吸烟，谢谢配合')
                process.wait.assert_called_with(timeout=30.0)
                self.assertNotIn('shell', popen.call_args.kwargs)
                self.assertEqual(service._command('smoking')[-1], '检测到疑似人员吸烟')
                self.assertEqual(service._command('fall', inspection=True)[-1], '检测到疑似人员跌倒')
            finally:
                service.close()

    def test_inspection_notice_never_reuses_old_smoking_audio(self):
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(speech.shutil, 'which', side_effect=lambda name: '/usr/bin/' + name if name in ('espeak-ng', 'ffplay') else None):
            root = Path(directory)
            (root / 'behavior_smoking.mp3').write_bytes(b'old manual speech')
            service = speech.BehaviorSpeechService(audio_dir=root)
            try:
                self.assertEqual(service._command('smoking')[-1], str(root / 'behavior_smoking.mp3'))
                self.assertEqual(service._command('smoking', inspection=True)[-1], '吸烟有害健康，请不要在公共场合吸烟，谢谢配合')
                asset = root / 'behavior_smoking_public_notice_v1.mp3'
                asset.write_bytes(b'new notice')
                self.assertEqual(service._command('smoking', inspection=True)[-1], str(asset))
            finally:
                service.close()

    def test_patrol_notices_survive_mode_reset_and_play_for_each_event(self):
        started = threading.Event()
        second_played = threading.Event()
        finish = threading.Event()
        first_process = Mock()
        first_process.poll.return_value = None
        second_process = Mock()
        second_process.poll.return_value = 0

        def play_first(timeout):
            started.set()
            finish.wait(2)
            return 0

        def play_second(timeout):
            second_played.set()
            return 0

        first_process.wait.side_effect = play_first
        second_process.wait.side_effect = play_second
        with patch.object(speech.shutil, 'which', return_value='/usr/bin/espeak-ng'), \
                patch.object(speech.subprocess, 'Popen', side_effect=[first_process, second_process]) as popen:
            service = speech.BehaviorSpeechService()
            try:
                self.assertTrue(service.announce_patrol_safety())
                self.assertTrue(started.wait(1))
                self.assertTrue(service.announce_patrol_safety())
                service.reset()
                first_process.terminate.assert_not_called()
                finish.set()
                self.assertTrue(second_played.wait(1))
                self.assertEqual([call.args[0][-1] for call in popen.call_args_list],
                                 ['正在执行巡检任务，请注意安全'] * 2)
            finally:
                finish.set()
                service.close()


if __name__ == '__main__':
    unittest.main()
