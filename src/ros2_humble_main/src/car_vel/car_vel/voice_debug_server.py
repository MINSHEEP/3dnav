"""Opt-in web adapter for the existing cyber1_vc application.

Run through scripts/start_voice_debug.py using cyber1_vc's Python environment.
The adapter never edits that project or starts any of the normal web pages.
"""

from __future__ import annotations

import argparse
from collections import deque
from contextlib import contextmanager
import importlib
import json
import math
import os
from pathlib import Path
import secrets
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
import uuid


PHRASES = {"first": "歪歪帮我介绍一下", "second": "歪歪下一步介绍实验班"}
DIRECTIONS = {
    "forward": (1, 0, 0), "backward": (-1, 0, 0),
    "left": (0, 1, 0), "right": (0, -1, 0),
    "turn_left": (0, 0, 1), "turn_right": (0, 0, -1),
}
SESSION_TIMEOUT = 2.0
MOTION_TIMEOUT = 0.4
# Same protocol values as dog_remote_bridge, without importing the ROS node.
CMD_SET_MOTION_MODEL = 2
OBSTACLE_MOTION_MODEL = 0
ASSETS = {
    "/": ("voice_debug.html", "text/html; charset=utf-8"),
    "/voice_debug.html": ("voice_debug.html", "text/html; charset=utf-8"),
    "/voice_debug.css": ("voice_debug.css", "text/css; charset=utf-8"),
    "/voice_debug.js": ("voice_debug.js", "text/javascript; charset=utf-8"),
}


class DebugError(Exception):
    def __init__(self, message, status=409):
        super().__init__(message)
        self.status = status


class WebVoiceInput:
    """Mixin keeps the microphone gate separate from TEXT and playback."""

    def __init__(self, *args, **kwargs):
        self._web_input_lock = threading.RLock()
        self._web_mic_enabled = False
        self._web_deadline = 0.0
        self._web_manual = False
        self._web_navigation = False
        self._web_reply = ""
        self._web_last_input = ""
        self._web_waiting_reply_until = 0.0
        super().__init__(*args, **kwargs)

    def web_initialize_motion_mode(self):
        """Select obstacle gait explicitly once, after the UDP handshake."""
        if self._stop.is_set():
            raise RuntimeError("调试程序正在停止，未设置越障模式。")
        self.action_sender.stop_motion()
        self.action_sender._send({
            "msg_type": 3,
            "cmd_code": CMD_SET_MOTION_MODEL,
            "para": OBSTACLE_MOTION_MODEL,
        })
        print("[调试初始化] 已发送越障模式设置命令（model=0）")

    @property
    def web_microphone_enabled(self):
        with self._web_input_lock:
            return self._web_mic_enabled and time.monotonic() < self._web_deadline

    def web_renew(self, deadline):
        with self._web_input_lock:
            self._web_deadline = deadline

    def web_microphone(self, enabled):
        with self._web_input_lock:
            self._web_mic_enabled = enabled
            # Closed-gate audio must never become the next utterance's preroll.
            with self._speech_preroll_lock:
                self._speech_preroll.clear()
                self._speech_gate_was_closed = False
            self._pulse_reset_recognizer = True

    def _mic_callback(self, pcm_data):
        with self._web_input_lock:
            if self.web_microphone_enabled:
                return super()._mic_callback(pcm_data)

    def _send_audio_frame(self, pcm_data):
        with self._web_input_lock:
            if not self.web_microphone_enabled:
                return False
            return super()._send_audio_frame(pcm_data)

    def web_text(self, text):
        # JoyInside's TEXT input triggers the agent, independently of AUDIO.
        # Protocol: https://joyinside.com/handbook/part5-附录/b-protocol-fields.html
        with self._keyword_lock:
            if self._stop.is_set() or not self.ws or not self.ws_connected.is_set():
                raise DebugError("语音程序尚未连接，请稍后重试。", 503)
            mid = str(uuid.uuid4())
            self._reset_round_state()
            self._last_user_text = text
            self._web_last_input = text
            self._web_reply = ""
            self._last_user_activity_at = time.monotonic()
            self._network_resume_pending = False
            self._drop_old_tts = False
            self._suppress_repetitive_reply = False
            self._speech_interrupt_active = False
            self.asr_time = time.time()
            self._keyword_turn_gate.note_upload(mid)
            self._web_waiting_reply_until = time.monotonic() + 30.0
            try:
                self.ws.send(json.dumps({
                    "mid": mid, "contentType": "TEXT", "uid": self.uid,
                    "content": {"input": text},
                }, ensure_ascii=False))
            except Exception as exc:
                self._web_waiting_reply_until = 0.0
                self._keyword_turn_gate.forget_upload(mid)
                raise DebugError("文本发送失败，请确认语音连接后重试。", 503) from exc

    def _handle_message(self, data):
        result = super()._handle_message(data)
        if data.get("code") == 200:
            content = data.get("content", {})
            if data.get('contentType') == 'TTS':
                self._web_waiting_reply_until = 0.0
            if data.get("contentType") == "ASR" and content.get("text"):
                self._web_last_input = content["text"]
            elif data.get("contentType") == "AGENT" and content.get("content"):
                self._web_reply = str(content["content"])[-4000:]
        return result

    def _should_nod_for_tts(self, proactive):
        # A debug text response must not silently start a physical gesture.
        return False

    def web_voice_busy(self):
        player = getattr(self, 'player', None)
        return bool(time.monotonic() < self._web_waiting_reply_until
                    or getattr(self, '_tts_round_active', False)
                    or (player and player.is_playback_active()))

    def _action_policy(self, action_name):
        navigation_stand = (action_name == '站立' and self._web_navigation
                            and not self._web_manual and getattr(
                                self.action_sender._work_context, 'web_navigation_stand', False))
        if ((self._web_manual or self._web_navigation) and not navigation_stand
                and action_name not in ("停止", "停下", "趴下")):
            return False, "页面正在遥控或导航，请先停止移动"
        return super()._action_policy(action_name)


class DebugController:
    """One page lease, a bounded held-motion lease, and one robot owner."""

    def __init__(self, app=None, *, clock=time.monotonic):
        self.app = app
        self.clock = clock
        self.lock = threading.RLock()
        self.owner = ""
        self.deadline = 0.0
        self.motion = None
        self.motion_deadline = 0.0
        self.motion_generation = None
        self.motion_sequence = 0
        self.posture_busy = False
        self.error = ""
        self.events = deque(maxlen=40)
        self.event_id = 0
        self.navigation = None
        self.phrases = PHRASES

    def log(self, text):
        self.event_id += 1
        self.events.append({"id": self.event_id, "text": text})

    def _stop_motion(self, reason='已停止导航，可手动遥控。'):
        self.motion = None
        if self.navigation:
            self.navigation.cancel(reason)
        if self.app:
            self.app._web_manual = False
            self.app._startup_cancelled = True
            self.app._clear_pending_motion()
            self.app._cancel_action_sequence()
            self.app._cancel_visual_follow(speak=False)
            self.app.action_sender.stop_motion()

    def _release(self, reason='页面已释放控制会话，麦克风已关闭，运动已停止。'):
        had_owner = bool(self.owner)
        if self.app:
            self.app.web_microphone(False)
            self.app.web_renew(0.0)
            self._stop_motion(reason)
        self.owner = ""
        self.deadline = 0.0
        if had_owner:
            self.log(reason)

    def _expire(self):
        if self.owner and self.clock() >= self.deadline:
            self._release("页面连接超时，麦克风已关闭，运动已停止。")

    def _require_owner(self, token):
        self._expire()
        if not isinstance(token, str) or not self.owner or not secrets.compare_digest(token, self.owner):
            raise DebugError("调试会话已失效，请重新连接页面。", 403)

    def _motion_guard(self):
        """Check robot readiness; manual debug movement has no range veto."""
        sender = self.app.action_sender
        if self.error or not sender._can_send_motion():
            raise DebugError("机器人存在故障或动作已被停止。")
        if (sender._motion_mode != sender.MODE_STANDING
                or time.monotonic() - sender._motion_mode_time > 2.0):
            raise DebugError("请先站立，并确认机器人状态在线。")

    def dispatch(self, path, payload):
        if path == "/api/voice-debug/phrase":
            # A slow cloud send must not hold the motion watchdog's lock.
            with self.lock:
                self._require_owner(payload.get("session"))
                phrase = payload.get("phrase")
                if not isinstance(phrase, str) or phrase not in PHRASES:
                    raise DebugError("无效的句子按钮。", 400)
            self.app.web_text(PHRASES[phrase])
            with self.lock:
                self.log("已发送：" + PHRASES[phrase])
            return {"ok": True}
        with self.lock:
            self._expire()
            if not self.app:
                raise DebugError("当前为预览模式，未连接机器人和语音程序。", 503)
            if path == "/api/voice-debug/session":
                if self.error:
                    raise DebugError(self.error, 503)
                if self.owner:
                    raise DebugError("另一个页面正在使用调试功能，请先关闭该页面。")
                self.owner = secrets.token_urlsafe(24)
                self.motion_sequence = 0
                self.deadline = self.clock() + SESSION_TIMEOUT
                self.app.web_microphone(False)
                self.app.web_renew(time.monotonic() + SESSION_TIMEOUT)
                self.log("调试页面已连接，麦克风默认关闭。")
                return {"session": self.owner}
            self._require_owner(payload.get("session"))
            if (path in ("/api/voice-debug/move", "/api/voice-debug/stop")
                    or path.startswith('/api/voice-debug/navigation/')):
                sequence = payload.get("sequence")
                if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 1:
                    raise DebugError("运动请求序号无效。", 400)
                if sequence <= self.motion_sequence:
                    return {"ok": True, "ignored": True}
                self.motion_sequence = sequence
            if path == "/api/voice-debug/heartbeat":
                self.deadline = self.clock() + SESSION_TIMEOUT
                self.app.web_renew(time.monotonic() + SESSION_TIMEOUT)
            elif path == "/api/voice-debug/microphone":
                enabled = payload.get("enabled")
                if not isinstance(enabled, bool):
                    raise DebugError("麦克风状态必须为开或关。", 400)
                self.app.web_microphone(enabled)
                self.log("麦克风已开启。" if enabled else "麦克风已关闭；文本按钮仍可使用。")
            elif path == "/api/voice-debug/move":
                direction = payload.get("direction")
                speed = payload.get("speed", 0.4)
                if (not isinstance(direction, str) or direction not in DIRECTIONS
                        or isinstance(speed, bool) or not isinstance(speed, (int, float))
                        or not math.isfinite(speed) or not 0.1 <= speed <= 0.6):
                    raise DebugError("无效的方向或速度。", 400)
                if self.posture_busy:
                    raise DebugError("姿态切换中，请稍候。")
                self._motion_guard()
                if self.motion is None:
                    self._stop_motion()
                    self.app.action_sender.exit_idle_mode()
                    self.app._web_manual = True
                    self.motion_generation = self.app.action_sender.capture_generation()
                self.motion = (direction, float(speed))
                self.motion_deadline = self.clock() + MOTION_TIMEOUT
            elif path == "/api/voice-debug/stop":
                self._stop_motion()
            elif path == "/api/voice-debug/posture":
                command = payload.get("command")
                if command not in ("stand", "lie_down"):
                    raise DebugError("无效的姿态命令。", 400)
                if self.posture_busy:
                    raise DebugError("姿态切换中，请稍候。")
                self._stop_motion()
                self.posture_busy = True

                def run_posture():
                    try:
                        sender = self.app.action_sender
                        # _make_thread inherits the generation before starting.
                        if not sender._can_send_motion(allow_fault=command == "lie_down"):
                            return
                        ok = sender.stand_up() if command == "stand" else sender.lie_down()
                        with self.lock:
                            self.log(("站立" if command == "stand" else "趴下") + ("已确认。" if ok else "未完成，请检查状态。"))
                    except Exception:
                        with self.lock:
                            self.log("姿态命令失败，请检查机器人连接。")
                    finally:
                        with self.lock:
                            self.posture_busy = False

                # Always enter the target to clear posture_busy, even if STOP
                # invalidates the captured command before the thread starts.
                sender = self.app.action_sender
                generation = sender.capture_generation()

                def guarded_posture():
                    sender._work_context.generation = generation
                    run_posture()

                self.app._make_thread(target=guarded_posture, name="web-posture").start()
            elif path.startswith('/api/voice-debug/navigation/'):
                if not self.navigation:
                    raise DebugError('导航功能尚未配置。', 503)
                action = path.rsplit('/', 1)[1]
                try:
                    if action in ('prepare', 'route1', 'route2'):
                        if self.posture_busy or self.motion:
                            raise DebugError('请先停止遥控并等待姿态切换完成。')
                        if action != 'prepare' and not self.app.ws_connected.is_set():
                            raise DebugError('语音程序尚未连接，导航讲解暂不可用。', 503)
                        self._stop_motion()
                        self.navigation.start(action)
                    elif action == 'anchor':
                        self.navigation.save_anchor(payload)
                    elif action == 'reset-anchor':
                        self._stop_motion()
                        self.navigation.reset_anchor()
                    else:
                        raise DebugError('导航接口不存在。', 404)
                except (RuntimeError, ValueError, KeyError) as exc:
                    raise DebugError(str(exc)) from exc
            elif path == "/api/voice-debug/release":
                reasons = {'hidden': '页面切到后台，已关闭麦克风并停止导航与运动。',
                           'pagehide': '页面已离开，已关闭麦克风并停止导航与运动。',
                           'unload': '页面正在关闭或刷新，已关闭麦克风并停止导航与运动。'}
                reason = payload.get('reason')
                self._release(reasons.get(reason) if isinstance(reason, str) and reason in reasons
                              else '页面已释放控制会话，麦克风已关闭，运动已停止。')
            else:
                raise DebugError("接口不存在。", 404)
            return {"ok": True}

    def tick(self):
        with self.lock:
            self._expire()
            if self.navigation:
                self.navigation.tick()
            if not self.motion:
                return
            try:
                if (self.clock() >= self.motion_deadline
                        or self.app.action_sender.capture_generation() != self.motion_generation):
                    self._stop_motion()
                    return
                direction, speed = self.motion
                self._motion_guard()
                x, y, yaw = DIRECTIONS[direction]
                self.app.action_sender._send_velocity(x * speed, y * speed, yaw * speed)
            except Exception as exc:
                self._stop_motion()
                self.log(str(exc) if isinstance(exc, DebugError) else "运动连接异常，已停止。")

    def status(self):
        with self.lock:
            self._expire()
            app = self.app
            sender = app.action_sender if app else None
            return {
                "preview": app is None, "ready": bool(app) and not self.error,
                "error": self.error,
                "voice_connected": bool(app and app.ws_connected.is_set()),
                "microphone_enabled": bool(app and app.web_microphone_enabled),
                "posture_busy": self.posture_busy,
                "robot_online": bool(sender and time.monotonic() - sender._motion_mode_time <= 2.0),
                "posture": ({0: "趴下（待复位）", 1: "趴下", 2: "站立", 3: "低姿态", 4: "摔倒"}.get(sender._motion_mode, "未知") if sender else "未连接"),
                "last_input": app._web_last_input if app else "",
                "last_reply": app._web_reply if app else "",
                "events": list(self.events),
                "navigation": self.navigation.snapshot() if self.navigation else None,
            }


class DebugServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, controller, frontend):
        self.controller = controller
        self.frontend = Path(frontend)
        super().__init__(address, DebugHandler)


class DebugHandler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(3.0)

    def log_message(self, *args):
        pass

    def _origin(self):
        origin = self.headers.get("Origin")
        if not origin:
            return None
        try:
            parsed = urlsplit(origin)
            host = urlsplit("http://" + self.headers.get("Host", "")).hostname
            if parsed.scheme in ("http", "https") and parsed.hostname == host:
                return origin
        except ValueError:
            pass
        raise DebugError("不允许此页面来源访问调试接口。", 403)

    def _reply(self, status, data, content_type="application/json; charset=utf-8"):
        body = data if isinstance(data, bytes) else json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "same-origin")
        try:
            origin = self._origin()
            if origin:
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
        except DebugError:
            pass
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        try:
            self._origin()
            self.send_response(204)
            origin = self._origin()
            if origin:
                self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Content-Length", "0")
            self.end_headers()
        except DebugError as exc:
            self._reply(exc.status, {"error": str(exc)})

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/voice-debug/status":
            self._reply(200, self.server.controller.status())
        elif path == '/api/voice-debug/navigation/map':
            navigation = self.server.controller.navigation
            if navigation and navigation.map_cache:
                self._reply(200, navigation.map_cache)
            else:
                self._reply(409, {'error': '地图尚未加载。'})
        elif path in ASSETS:
            filename, mime = ASSETS[path]
            try:
                data = (self.server.frontend / filename).read_bytes()
                if filename == "voice_debug.html":
                    data = data.replace(b'data-service-port=""', f'data-service-port="{self.server.server_port}"'.encode("ascii"))
                self._reply(200, data, mime)
            except OSError:
                self._reply(404, {"error": "页面资源不存在。"})
        else:
            self._reply(404, {"error": "页面不存在。"})

    def do_POST(self):
        try:
            self._origin()
            if self.headers.get_content_type() != "application/json":
                raise DebugError("请发送 JSON 请求。", 415)
            try:
                size = int(self.headers.get("Content-Length", "0"))
            except ValueError as exc:
                raise DebugError("请求长度无效。", 400) from exc
            if not 0 < size <= 4096:
                raise DebugError("请求长度无效。", 400)
            try:
                payload = json.loads(self.rfile.read(size))
            except (ValueError, UnicodeError) as exc:
                raise DebugError("请求内容无效。", 400) from exc
            if not isinstance(payload, dict):
                raise DebugError("请求必须为对象。", 400)
            result = self.server.controller.dispatch(urlsplit(self.path).path, payload)
            self._reply(200, result)
        except DebugError as exc:
            self._reply(exc.status, {"error": str(exc)})
        except Exception:
            self._reply(500, {"error": "调试服务处理失败，请查看服务终端。"})


@contextmanager
def voice_project_lock(root):
    # Same lock as cyber1_vc/main.py; never compete with its running controller.
    import fcntl
    directory = root / "data"
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / "controller.lock").open("a+", encoding="utf-8") as stream:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            raise RuntimeError("原语音程序正在运行，请先停止 main.py，再启动调试服务。") from exc
        try:
            yield
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def find_frontend():
    source = Path(__file__).parent / "web_frontend"
    if (source / "voice_debug.html").is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("car_vel")) / "web_frontend"


def main(argv=None):
    parser = argparse.ArgumentParser(description="独立语音遥控调试页面（不修改其他页面）")
    parser.add_argument("--voice-project", default=os.environ.get("CYBER1_VC_ROOT", str(Path.home() / "cyber1_vc")))
    parser.add_argument("--config", help="cyber1_vc 配置文件路径")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8768)
    parser.add_argument("--preview", action="store_true", help="仅预览页面，不打开声卡、云端或机器人")
    parser.add_argument("--check", action="store_true", help="只检查配置与依赖，不连接硬件")
    parser.add_argument('--nav-port', type=int, default=9091, help='独立导航连接端口')
    parser.add_argument('--nav-domain', type=int, default=73, help='独立导航 ROS domain')
    args = parser.parse_args(argv)
    if not 1 <= args.nav_port <= 65535 or args.nav_port == args.port:
        parser.error('导航连接端口无效或与页面端口重复。')
    if not 1 <= args.nav_domain <= 232:
        parser.error('独立导航 ROS domain 必须在1至232之间。')
    frontend = find_frontend()
    if args.preview:
        app = None
    else:
        root = Path(args.voice_project).expanduser().resolve()
        if not (root / "robot" / "app.py").is_file():
            parser.error("找不到语音项目，请用 --voice-project 指定 cyber1_vc 目录。")
        sys.path.insert(0, str(root))
        settings = importlib.import_module("robot.settings")
        cfg = settings.configure(args.config, root=root)
        # The page's stand button is the only startup posture trigger.
        cfg["auto_stand_on_start"] = False
        errors = settings.validate_settings(cfg)
        environment_errors, warnings = settings.inspect_environment(cfg)
        for warning in warnings:
            print("[能力提示] " + warning)
        if errors or environment_errors:
            for error in errors + environment_errors:
                print("[检查失败] " + error, file=sys.stderr)
            return 2
        if args.check:
            print("[检查通过] 配置与依赖就绪；未连接机器人、声卡或云端。")
            return 0
        app = None

    from contextlib import ExitStack
    with ExitStack() as stack:
        if not args.preview:
            stack.enter_context(voice_project_lock(root))
            from car_vel.body_lock import acquire_body_lock
            ownership = acquire_body_lock()
            stack.callback(ownership.close)
            robot_app = importlib.import_module("robot.app").RobotApp
            app_type = type("WebRobotApp", (WebVoiceInput, robot_app), {})
            app = app_type(cfg)
            stack.callback(app.cleanup)
            app.web_initialize_motion_mode()
            app.action_sender.start_ultrasonic_rx()
        controller = DebugController(app)
        if app or args.preview:
            from car_vel.voice_debug_navigation import DebugNavigation, NavigationRuntime
            source_package = Path(__file__).resolve().parent.parent
            launch_file = source_package / 'launch/voice_debug_navigation.launch.py'
            if not launch_file.is_file() and app:
                from ament_index_python.packages import get_package_share_directory
                launch_file = Path(get_package_share_directory('car_vel')) / 'launch/voice_debug_navigation.launch.py'
            # In a source checkout the outer directory is the deployed workspace.
            workspace = next((parent for parent in source_package.parents
                              if (parent / 'src/ros2_humble_main').is_dir()), Path.home() / '3dnav')
            controller.navigation = DebugNavigation(controller, NavigationRuntime(
                workspace, launch_file, args.nav_port, args.nav_domain))
            if app:
                stack.callback(controller.navigation.close)
        server = DebugServer((args.host, args.port), controller, frontend)
        stack.callback(server.server_close)
        stopped = threading.Event()

        def shutdown(*_args):
            stopped.set()

        for number in (signal.SIGINT, signal.SIGTERM):
            old = signal.signal(number, shutdown)
            stack.callback(signal.signal, number, old)

        def run_voice():
            try:
                app.run()
                if not stopped.is_set():
                    raise RuntimeError("语音程序已停止。")
            except Exception:
                with controller.lock:
                    controller.error = "语音程序已停止，请检查终端并重启调试服务。"
                    controller._release()
                    controller.log(controller.error)
                import traceback
                traceback.print_exc()

        if app:
            voice_thread = threading.Thread(target=run_voice, name="debug-voice", daemon=True)
            voice_thread.start()
        http_thread = threading.Thread(target=server.serve_forever, name="debug-http", daemon=True)
        http_thread.start()
        print(f"[调试页面] http://127.0.0.1:{args.port}/voice_debug.html")
        print("[麦克风] 默认关闭；两个句子按钮独立于麦克风。")
        try:
            while not stopped.wait(0.05):
                controller.tick()
        finally:
            with controller.lock:
                controller._release()
            server.shutdown()
            http_thread.join(timeout=2.0)
            if app:
                app.request_shutdown()
                voice_thread.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
