"""Exercise the debug input gate and HTTP boundary without hardware or cloud."""

from collections import deque
from contextlib import contextmanager
import http.client
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_vel.voice_debug_server import (
    DebugController, DebugError, DebugServer, PHRASES, WebVoiceInput,
)


class Sender:
    MODE_STANDING = 2

    def __init__(self):
        self._motion_mode = 2
        self._motion_mode_time = time.monotonic()
        self._generation = 0
        self._work_context = threading.local()
        self.velocities = []
        self.front_fresh = self.back_fresh = True
        self.front = self.back = 100
        self.stands = self.lies = 0

    def capture_generation(self):
        return self._generation

    def _can_send_motion(self, allow_fault=False):
        return getattr(self._work_context, "generation", self._generation) == self._generation

    def _send_velocity(self, *velocity):
        self.velocities.append(velocity)

    def stop_motion(self):
        self._generation += 1
        self._send_velocity(0, 0, 0)

    def exit_idle_mode(self):
        pass

    def has_fresh_front_distance(self, _age):
        return self.front_fresh

    def has_fresh_back_distance(self, _age):
        return self.back_fresh

    def get_front_distance_cm(self):
        return self.front

    def get_back_distance_cm(self):
        return self.back

    def stand_up(self):
        self.stands += 1
        return True

    def lie_down(self):
        self.lies += 1
        return True


class BaseVoice:
    def __init__(self):
        self.cfg = {"safe_distance_cm": 35}
        self._stop = threading.Event()
        self.ws_connected = threading.Event()
        self.ws_connected.set()
        self.sent = []
        self._keyword_lock = threading.RLock()
        self._speech_preroll_lock = threading.Lock()
        self._speech_preroll = deque()
        self.uploads = set()
        self._keyword_turn_gate = SimpleNamespace(
            note_upload=self.uploads.add, forget_upload=self.uploads.discard)
        self.ws = SimpleNamespace(send=lambda raw: self.sent.append(json.loads(raw)))
        self.uid = "test-user"
        self.action_sender = Sender()
        self.callbacks = []
        self.callback_entered = None
        self.callback_resume = None
        self.threads = []

    def _mic_callback(self, pcm):
        self.callbacks.append(pcm)
        if self.callback_entered:
            self.callback_entered.set()
            self.callback_resume.wait(2)
        self._send_audio_frame(pcm)

    def _send_audio_frame(self, pcm):
        self.sent.append({"contentType": "AUDIO", "pcm": pcm})
        return True

    def _handle_message(self, data):
        pass

    def _reset_round_state(self):
        pass

    def _clear_pending_motion(self):
        pass

    def _cancel_action_sequence(self):
        pass

    def _cancel_visual_follow(self, speak=False):
        pass

    def _make_thread(self, **kwargs):
        thread = threading.Thread(target=kwargs["target"], daemon=True)
        self.threads.append(thread)
        return thread


class App(WebVoiceInput, BaseVoice):
    pass


class DebugInputTests(unittest.TestCase):
    def setUp(self):
        self.app = App()
        self.now = [100.0]
        self.controller = DebugController(self.app, clock=lambda: self.now[0])
        self.token = self.controller.dispatch("/api/voice-debug/session", {})["session"]

    def send(self, path, **payload):
        return self.controller.dispatch("/api/voice-debug/" + path, {"session": self.token, **payload})

    def test_startup_explicitly_selects_obstacle_mode_after_stopping(self):
        actions = []
        self.app.action_sender.stop_motion = lambda: actions.append("stop")
        self.app.action_sender._send = lambda payload: actions.append(payload)
        self.app.web_initialize_motion_mode()
        self.assertEqual(actions, ["stop", {"msg_type": 3, "cmd_code": 2, "para": 0}])
        self.assertEqual(self.app.action_sender.stands, 0)
        self.assertEqual(self.app.action_sender.lies, 0)

    def test_repeated_startup_initialization_does_not_toggle_to_high_speed(self):
        commands = []
        self.app.action_sender._send = lambda payload: commands.append(payload)
        self.app.web_initialize_motion_mode()
        self.app.web_initialize_motion_mode()
        self.assertEqual(commands, [{"msg_type": 3, "cmd_code": 2, "para": 0}] * 2)

    def test_stopping_program_does_not_send_a_mode_command(self):
        commands = []
        self.app.action_sender._send = lambda payload: commands.append(payload)
        self.app._stop.set()
        with self.assertRaises(RuntimeError):
            self.app.web_initialize_motion_mode()
        self.assertEqual(commands, [])

    def test_microphone_is_off_by_default_but_both_exact_texts_are_sent(self):
        self.app._mic_callback(b"closed")
        self.assertFalse(self.app._send_audio_frame(b"direct-bypass"))
        self.send("phrase", phrase="first")
        self.send("phrase", phrase="second")
        self.assertEqual(self.app.callbacks, [])
        self.assertEqual([message["contentType"] for message in self.app.sent], ["TEXT", "TEXT"])
        self.assertEqual([message["content"]["input"] for message in self.app.sent], list(PHRASES.values()))
        self.assertEqual(len(self.app.uploads), 2, "text also authorizes a strict keyword response round")

    def test_audio_only_flows_while_enabled_and_old_preroll_is_discarded(self):
        self.app._speech_preroll.append(b"stale")
        self.send("microphone", enabled=True)
        self.assertFalse(self.app._speech_preroll)
        self.app._mic_callback(b"live")
        self.send("phrase", phrase="first")
        self.send("microphone", enabled=False)
        self.app._mic_callback(b"closed-again")
        self.send("phrase", phrase="second")
        self.assertEqual(self.app.callbacks, [b"live"])
        self.assertEqual([message["contentType"] for message in self.app.sent], ["AUDIO", "TEXT", "TEXT"])
        self.assertFalse(self.app.web_microphone_enabled)

    def test_gate_serializes_off_with_an_inflight_audio_callback(self):
        self.send("microphone", enabled=True)
        self.app.callback_entered = threading.Event()
        self.app.callback_resume = threading.Event()
        callback = threading.Thread(target=self.app._mic_callback, args=(b"inflight",))
        callback.start()
        self.assertTrue(self.app.callback_entered.wait(1))
        closed = threading.Event()
        closer = threading.Thread(target=lambda: (self.app.web_microphone(False), closed.set()))
        closer.start()
        self.assertFalse(closed.wait(0.02))
        self.app.callback_resume.set()
        callback.join(1)
        closer.join(1)
        self.assertTrue(closed.is_set())
        self.app._mic_callback(b"after-off")
        self.assertEqual(self.app.callbacks, [b"inflight"])

    def test_audio_gate_closes_at_lease_deadline_without_waiting_for_watchdog(self):
        self.send("microphone", enabled=True)
        self.app.web_renew(time.monotonic() - 1)
        self.app._mic_callback(b"expired")
        self.assertFalse(self.app._send_audio_frame(b"expired-direct"))
        self.assertEqual(self.app.sent, [])

    def test_response_capture_uses_existing_agent_protocol_field(self):
        self.app._handle_message({"code": 200, "contentType": "AGENT", "content": {"content": "你好，我是歪歪。"}})
        self.assertEqual(self.controller.status()["last_reply"], "你好，我是歪歪。")

    def test_repeated_button_presses_each_send_and_failure_is_reported(self):
        self.send("phrase", phrase="first")
        self.send("phrase", phrase="first")
        self.assertEqual(len(self.app.sent), 2)
        self.app.ws_connected.clear()
        with self.assertRaises(DebugError) as error:
            self.send("phrase", phrase="second")
        self.assertEqual(error.exception.status, 503)

    def test_move_stops_on_missing_refresh_and_late_packets_cannot_restart_it(self):
        self.send("move", direction="forward", speed=0.4, sequence=1)
        self.controller.tick()
        self.assertEqual(self.app.action_sender.velocities[-1], (0.4, 0, 0))
        self.send("stop", sequence=3)
        self.send("move", direction="forward", speed=0.4, sequence=2)
        self.controller.tick()
        self.assertEqual(self.app.action_sender.velocities[-1], (0, 0, 0))
        self.send("move", direction="turn_right", speed=0.4, sequence=4)
        self.controller.tick()
        self.assertEqual(self.app.action_sender.velocities[-1], (0, 0, -0.4))
        self.now[0] += 0.41
        self.controller.tick()
        self.assertEqual(self.app.action_sender.velocities[-1], (0, 0, 0))

    def test_remote_directions_use_all_six_axes(self):
        expected = [(0.4, 0, 0), (-0.4, 0, 0), (0, 0.4, 0), (0, -0.4, 0), (0, 0, 0.4), (0, 0, -0.4)]
        for sequence, (direction, velocity) in enumerate(zip(
                ["forward", "backward", "left", "right", "turn_left", "turn_right"], expected), 1):
            self.send("move", direction=direction, speed=0.4, sequence=sequence)
            self.controller.tick()
            self.assertEqual(self.app.action_sender.velocities[-1], velocity)

    def test_safety_interruption_does_not_resume_when_heartbeat_returns(self):
        self.send("move", direction="forward", sequence=1)
        self.app.action_sender.stop_motion()
        self.controller.tick()
        self.send("heartbeat")
        self.controller.tick()
        self.assertIsNone(self.controller.motion)
        self.assertEqual(self.app.action_sender.velocities[-1], (0, 0, 0))

    def test_manual_forward_and_backward_ignore_stale_range(self):
        sender = self.app.action_sender
        sender.front_fresh = sender.back_fresh = False
        for sequence, (direction, velocity) in enumerate([
                ("forward", (0.4, 0, 0)), ("backward", (-0.4, 0, 0))], 1):
            self.send("move", direction=direction, speed=0.4, sequence=sequence)
            self.controller.tick()
            self.assertEqual(sender.velocities[-1], velocity)

    def test_manual_motion_does_not_stop_when_range_becomes_too_close(self):
        sender = self.app.action_sender
        for sequence, (direction, velocity) in enumerate([
                ("forward", (0.4, 0, 0)), ("backward", (-0.4, 0, 0))], 1):
            sender.front = sender.back = 100
            self.send("move", direction=direction, speed=0.4, sequence=sequence)
            self.controller.tick()
            sender.front = sender.back = 1
            self.controller.tick()
            self.assertEqual(sender.velocities[-1], velocity)
            self.assertIsNotNone(self.controller.motion)

    def test_nonstanding_posture_still_blocks_manual_motion(self):
        self.app.action_sender._motion_mode = 1
        with self.assertRaises(DebugError):
            self.send("move", direction="forward", sequence=1)

    def test_hardware_motion_guard_still_blocks_manual_motion(self):
        self.app.action_sender._can_send_motion = lambda **kwargs: False
        with self.assertRaises(DebugError):
            self.send("move", direction="forward", sequence=1)

    def test_expired_page_turns_off_mic_and_motion_and_allows_new_owner(self):
        self.send("microphone", enabled=True)
        self.send("move", direction="left", sequence=1)
        self.controller.tick()
        self.now[0] += 2.1
        self.controller.tick()
        self.assertFalse(self.app.web_microphone_enabled)
        self.assertEqual(self.app.action_sender.velocities[-1], (0, 0, 0))
        with self.assertRaises(DebugError):
            self.send("phrase", phrase="first")
        new = self.controller.dispatch("/api/voice-debug/session", {})["session"]
        self.assertNotEqual(new, self.token)

    def test_another_tab_cannot_claim_session_or_use_controls(self):
        with self.assertRaises(DebugError):
            self.controller.dispatch("/api/voice-debug/session", {})
        with self.assertRaises(DebugError):
            self.controller.dispatch("/api/voice-debug/microphone", {"session": "other", "enabled": True})

    def test_both_posture_buttons_call_the_existing_program(self):
        for command in ("stand", "lie_down"):
            self.send("posture", command=command)
            self.app.threads[-1].join(1)
            self.assertFalse(self.controller.posture_busy)
        self.assertEqual(self.app.action_sender.stands, 1)
        self.assertEqual(self.app.action_sender.lies, 1)

    def test_stop_before_posture_worker_starts_cancels_command_and_clears_busy(self):
        work = []
        self.app._make_thread = lambda **kwargs: SimpleNamespace(start=lambda: work.append(kwargs["target"]))
        self.send("posture", command="stand")
        self.assertTrue(self.controller.posture_busy)
        self.send("stop", sequence=1)
        work[0]()
        self.assertFalse(self.controller.posture_busy)
        self.assertEqual(self.app.action_sender.stands, 0)


@contextmanager
def http_service(controller):
    frontend = Path(__file__).resolve().parents[1] / "car_vel" / "web_frontend"
    server = DebugServer(("127.0.0.1", 0), controller, frontend)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server.server_port
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1)


class HttpTests(unittest.TestCase):
    def test_real_http_route_sends_text_while_microphone_is_off(self):
        app = App()
        with http_service(DebugController(app)) as port:
            connection = http.client.HTTPConnection("127.0.0.1", port, timeout=2)

            def post(path, payload, origin=None):
                headers = {"Content-Type": "application/json"}
                if origin:
                    headers["Origin"] = origin
                connection.request("POST", "/api/voice-debug/" + path, json.dumps(payload), headers)
                response = connection.getresponse()
                data = json.loads(response.read())
                return response.status, data

            status, data = post("session", {})
            self.assertEqual(status, 200)
            token = data["session"]
            for phrase in PHRASES:
                status, _ = post("phrase", {"session": token, "phrase": phrase}, "http://127.0.0.1:8000")
                self.assertEqual(status, 200)
            self.assertFalse(app.web_microphone_enabled)
            self.assertEqual([message["content"]["input"] for message in app.sent], list(PHRASES.values()))
            status, _ = post("microphone", {"session": token, "enabled": True}, "https://unrelated.example")
            self.assertEqual(status, 403)
            connection.request("GET", "/voice_debug.html")
            response = connection.getresponse()
            html = response.read().decode("utf-8")
            self.assertEqual(response.status, 200)
            self.assertIn(f'data-service-port="{port}"', html)
            self.assertIn("第一句", html)
            connection.request("GET", "/../config/settings.json")
            response = connection.getresponse()
            response.read()
            self.assertEqual(response.status, 404)
            connection.close()

    def test_preview_does_not_create_a_robot_or_accept_input(self):
        controller = DebugController()
        self.assertTrue(controller.status()["preview"])
        with self.assertRaises(DebugError) as error:
            controller.dispatch("/api/voice-debug/session", {})
        self.assertEqual(error.exception.status, 503)


if __name__ == "__main__":
    unittest.main()
