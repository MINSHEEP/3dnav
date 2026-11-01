#!/usr/bin/env python3
"""USB/MJPEG YOLO 告警与本地语音播报脚本。

默认语音输出到运行脚本的本地电脑；显式选择 ``--voice-output device`` 或 ``both``
时才通过海康语音对讲接口输出到设备。设备输出链路为：
``YOLO 告警 -> 文本音频 -> G711 编码 -> NET_DVR_VoiceComSendData -> 设备音频输出``。
本地链路直接使用 ``ffplay`` 播放固定 MP3，不依赖设备外接音箱。

SDK 没有“传中文字符串让设备自己朗读”的通用接口，所以这里先把报警词合成为
8kHz 单声道 PCM，再按海康语音对讲示例的 G711 帧格式送入设备。

默认从相机主服务的本机 MJPEG 地址取流，避免重复打开 USB 设备。旧的
RTSP/JPEG/V40 与设备扬声器输出仅作为显式海康兼容模式保留。
"""

from __future__ import annotations

import argparse
import asyncio
import os
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ElementTree
from ctypes import CFUNCTYPE, POINTER, Structure, addressof, byref, c_int, c_uint, c_ubyte, c_void_p, create_string_buffer
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np
from ultralytics import YOLO

from multimodel_detection import MultiModelDetector
from recognition_core import RecognitionDatabase
from recognition_runtime import (
    DEFAULT_EVIDENCE_DIR, DEFAULT_SFACE_MODEL, HyperLPRPlateRecognizer, RecognitionRuntime,
    SFaceFeatureExtractor)
from yolo_hk import DeviceInfoV30, HikvisionJpegSource, HikvisionRtspSource, HikvisionV40Source, RtspSource, YoloMJPEGServer, draw_status, ensure_player_library_path, open_capture_source


ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL = ROOT / "models" / "cigarette_multiclass_best.pt"
DEFAULT_FACE_MODEL = ROOT / "models" / "face_detection_yunet_2023mar.onnx"
DEFAULT_PLATE_MODEL = ROOT / "models" / "license_plate_best.pt"
DEFAULT_RECOGNITION_DB = Path.home() / ".ros" / "robot_navigation" / "recognition.db"
DEFAULT_HOST = "192.168.1.19"
# SDK、模型与固定语音都从迁移包目录解析。
DEFAULT_SDK_DIR = ROOT / "sdk"
DEFAULT_SPEECH_AUDIO_DIR = ROOT
DEFAULT_TTS_VOICE = "zh-CN-XiaoxiaoNeural"

# G711 语音对讲常用帧：8kHz、单声道，20ms G711 为 160 字节。
VOICE_SAMPLE_RATE = 8000
G711_FRAME_BYTES = 160
VOICE_FRAME_SECONDS = 0.02
VOICE_READY_TIMEOUT_SECONDS = 1.0

# 只播报这三类，其他类别直接忽略。
TARGET_LABEL_ORDER = ("fire", "smoke", "cigarettes")
MIN_CONFIRM_CONFIDENCE = 0.50
TARGET_LABEL_CN = {
    "fire": "火灾",
    "smoke": "烟雾",
    "cigarettes": "烟头",
}

ENVIRONMENT_SPEECH = {
    "thermal_high": "环境温度异常",
    "co2_high": "二氧化碳浓度异常",
}

LOCAL_SPEECH_FILES = {
    "火灾": "火焰.mp3",
    "烟雾": "烟雾.mp3",
    "烟头": "香烟.mp3",
}

DEVICE_AUDIO_ENC_NAMES = {
    0: "G722/G722.1",
    1: "G711_U",
    2: "G711_A",
    5: "MP2L2",
    6: "G726",
    7: "AAC",
    8: "PCM",
    9: "G722.1.C",
    12: "AAC_LC",
    13: "AAC_LD",
    14: "Opus",
    15: "MP3",
    16: "ADPCM",
}


class CompressionAudio(Structure):
    """SDK 的 NET_DVR_COMPRESSION_AUDIO，只取语音发送前需要打印和判断的字段。"""

    _fields_ = [
        ("byAudioEncType", c_ubyte),
        ("byAudioSamplingRate", c_ubyte),
        ("byAudioBitRate", c_ubyte),
        ("byRes", c_ubyte * 4),
        ("bySupport", c_ubyte),
    ]


class AudioChannel(Structure):
    """SDK 的 NET_DVR_AUDIO_CHANNEL，用于 V50 音频压缩查询。"""

    _fields_ = [("dwChannelNum", c_uint), ("byRes", c_ubyte * 32)]


VoiceDataCallback = CFUNCTYPE(None, c_int, c_void_p, c_uint, c_ubyte, c_void_p)


def normalize_label(label: str) -> str:
    """把模型类别名压成统一小写形式，兼容不同训练命名。"""
    return label.strip().lower()


def classify_alarm_label(label: str) -> str | None:
    """将模型类别名映射到统一告警类型。"""
    key = normalize_label(label)
    if "fire" in key:
        return "fire"
    if "smoke" in key:
        return "smoke"
    if "cigarette" in key or "butt" in key:
        return "cigarettes"
    return None


def extract_alarm_keys(result: Any) -> list[str]:
    """从单帧 YOLO 结果中提取需要播报的类别，保持固定优先级顺序。"""
    return [key for key, _score in extract_alarm_detections(result)]


def extract_alarm_detections(result: Any) -> list[tuple[str, float]]:
    """从单帧 YOLO 结果中提取需要播报的类别及最高置信度。"""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return []

    detected: dict[str, float] = {}
    for box in boxes:
        class_id = int(box.cls[0].cpu().item())
        class_name = str(result.names[class_id])
        alarm_key = classify_alarm_label(class_name)
        if alarm_key is None:
            continue
        score = float(box.conf[0].cpu().item())
        if score < MIN_CONFIRM_CONFIDENCE:
            continue
        # 同一类在一帧里可能出现多次，只保留置信度最高的一条。
        if score > detected.get(alarm_key, 0.0):
            detected[alarm_key] = score

    return [(key, detected[key]) for key in TARGET_LABEL_ORDER if key in detected]


def choose_g711_law(voice_law: str, enc_type: int) -> str:
    """选择发送给设备的 G711 法则。

    auto 模式优先跟随设备当前对讲编码：
    1 -> G711_U，2 -> G711_A。这样可以贴合设备当前配置，
    避免“登录成功但播报静默”的格式不匹配问题。
    """
    if voice_law in ("u", "a"):
        return voice_law
    if enc_type == 2:
        return "a"
    if enc_type == 1:
        return "u"
    return "u"


def parse_two_way_audio_config(payload: bytes) -> dict[str, str]:
    """解析 ISAPI 双向音频配置，用于区分内置扬声器与 LineOut。"""
    root = ElementTree.fromstring(payload)

    def text_of(name: str) -> str:
        node = root.find(f"{{*}}{name}")
        return node.text.strip() if node is not None and node.text else ""

    return {
        "enabled": text_of("enabled"),
        "codec": text_of("audioCompressionType"),
        "volume": text_of("speakerVolume"),
        "output_type": text_of("audioOutputType"),
    }


def set_two_way_audio_enabled(payload: bytes, enabled: bool) -> bytes:
    """仅修改 ISAPI 双向音频 XML 的 enabled 字段，保留设备其余配置。"""
    root = ElementTree.fromstring(payload)
    node = root.find("{*}enabled")
    if node is None:
        raise ValueError("ISAPI 音频配置缺少 enabled 字段")
    node.text = "true" if enabled else "false"
    return ElementTree.tostring(root, encoding="utf-8", xml_declaration=True)


def build_alarm_message(alarm_keys: Iterable[str], prefix: str = "检测到") -> str:
    """把告警类别拼成日志文本；实际播报会拆成短语缓存后送设备。"""
    words = [TARGET_LABEL_CN[key] for key in alarm_keys if key in TARGET_LABEL_CN]
    if not words:
        return ""
    return f"{prefix}{'、'.join(words)}"


def format_alarm_summary(alarm_detections: Iterable[tuple[str, float]]) -> str:
    """把识别结果转成控制台可读的简短字符串。"""
    parts = []
    for key, score in alarm_detections:
        label = TARGET_LABEL_CN.get(key, key)
        parts.append(f"{label}({score:.2f})")
    return "、".join(parts)


def _uint8(value: int) -> int:
    """登录结构里 BYTE 用 c_byte 定义时可能带符号，这里统一转无符号。"""
    return int(value) & 0xFF


def resolve_voice_channel(args: argparse.Namespace, device_info: DeviceInfoV30) -> int:
    """自动计算语音通道；NVR 的 IP 通道优先用 byStartDTalkChan 偏移。"""
    if args.voice_channel > 0:
        return args.voice_channel

    video_channel = max(1, int(args.channel))
    start_dchan = _uint8(device_info.byStartDChan)
    start_talk_channel = _uint8(device_info.byStartDTalkChan)
    if start_talk_channel > 0:
        # NVR 的预览通道可能是 byStartDChan 开始的 SDK 通道号，例如 33；
        # 语音通道需要先还原成 IP 通道索引，再叠加 byStartDTalkChan。
        ip_channel_index = video_channel - start_dchan + 1 if start_dchan > 0 and video_channel >= start_dchan else video_channel
        return start_talk_channel + max(1, ip_channel_index) - 1

    # 部分设备只返回音频输入起始通道；没有对讲起始通道时用它兜底。
    start_voice_in = _uint8(device_info.byStartVoiceInChanNo)
    if start_voice_in > 0:
        return start_voice_in + video_channel - 1

    # 单 IPC/普通设备的语音对讲通道通常从 1 开始。
    return video_channel


class AlarmSpeechPolicy:
    """按类别做播报节流，避免同一目标每帧都重复喊。"""

    def __init__(self, cooldown_seconds: float) -> None:
        self.cooldown_seconds = cooldown_seconds
        self._last_spoken: dict[str, float] = {}

    def pick_due(self, alarm_keys: Iterable[str], now: float | None = None) -> list[str]:
        """返回当前帧里真正需要播报的类别，并同步更新最近播报时间。"""
        timestamp = time.monotonic() if now is None else now
        due: list[str] = []
        seen: set[str] = set()
        for key in alarm_keys:
            if key in seen:
                continue
            seen.add(key)
            last = self._last_spoken.get(key, 0.0)
            if timestamp - last >= self.cooldown_seconds:
                due.append(key)
        for key in due:
            self._last_spoken[key] = timestamp
        return due


class AlarmSpeechRenderer:
    """优先使用本地固定音频，必要时回退到 edge-tts。"""

    def __init__(self, audio_dir: Path | None, voice: str, rate: str, volume: str, ffmpeg: str) -> None:
        self.audio_dir = audio_dir.expanduser().resolve() if audio_dir is not None else None
        self.voice = voice
        self.rate = rate
        self.volume = volume
        self._edge_tts = None
        self._edge_tts_cmd = ""
        self._backend = "local" if self.audio_dir is not None else "tts"
        resolved_ffmpeg = self._resolve_command(ffmpeg)
        if resolved_ffmpeg is None:
            raise FileNotFoundError(f"找不到 ffmpeg: {ffmpeg}")
        self.ffmpeg = resolved_ffmpeg
        if self.audio_dir is None:
            self._ensure_tts_backend()
        print(f"[语音] 音频模式={self._backend}" + (f"，素材目录={self.audio_dir}" if self.audio_dir is not None else ""))

    @staticmethod
    def _resolve_command(command: str) -> str | None:
        """优先找显式路径，其次找 PATH，再按当前 conda 环境的常见位置兜底。"""
        if command and (os.path.isabs(command) or os.sep in command):
            candidate = Path(command)
            return str(candidate) if candidate.is_file() else None

        resolved = shutil.which(command)
        if resolved:
            return resolved

        candidates: list[Path] = []
        python_exe = Path(sys.executable).resolve()
        candidates.append(python_exe.parent / command)

        conda_prefix = Path(sys.prefix).resolve()
        candidates.append(conda_prefix / "bin" / command)
        candidates.append(conda_prefix.parent / "bin" / command)
        candidates.append(conda_prefix.parent.parent / "bin" / command)

        conda_exe = os.environ.get("CONDA_EXE")
        if conda_exe:
            candidates.append(Path(conda_exe).resolve().parent / command)

        home_bin = Path.home() / ".local" / "bin" / command
        candidates.append(home_bin)

        for candidate in candidates:
            if candidate.is_file():
                return str(candidate)
        return None

    def _ensure_tts_backend(self) -> None:
        """按需初始化 edge-tts，避免本地音频模式也强制依赖在线 TTS。"""
        if self._edge_tts is not None or self.audio_dir is not None:
            return
        try:
            import edge_tts
        except ImportError:
            edge_tts = None

        if edge_tts is not None:
            self._edge_tts = edge_tts
            self._backend = "tts-module"
            print("[语音] 使用 edge-tts Python 模块回退")
            return

        resolved_cli = self._resolve_command("edge-tts")
        if resolved_cli is None:
            raise RuntimeError("未安装 edge-tts：既找不到 Python 模块，也找不到 edge-tts 命令")
        self._edge_tts_cmd = resolved_cli
        self._backend = "tts-cli"
        print(f"[语音] 使用 edge-tts 命令回退: {self._edge_tts_cmd}")

    def _local_audio_path(self, text: str) -> Path | None:
        """按固定中文词条查找本地音频素材。"""
        if self.audio_dir is None:
            return None
        filename = LOCAL_SPEECH_FILES.get(text)
        if filename:
            candidate = self.audio_dir / filename
            if candidate.is_file():
                return candidate
            raise FileNotFoundError(f"本地语音素材缺失: {candidate}")
        candidate = self.audio_dir / f"{text}.mp3"
        return candidate if candidate.is_file() else None

    async def _save_mp3(self, text: str, output: Path) -> None:
        """edge-tts 是异步接口，这里只负责生成临时 MP3。"""
        if self._edge_tts is None:
            raise RuntimeError("edge-tts 模块未初始化")
        communicate = self._edge_tts.Communicate(text, self.voice, rate=self.rate, volume=self.volume)
        await communicate.save(str(output))

    def _save_mp3_cli(self, text: str, output: Path) -> None:
        """使用 edge-tts 命令行生成 MP3，避免 Python 模块缺失导致脚本不可用。"""
        if not self._edge_tts_cmd:
            raise RuntimeError("edge-tts 命令未初始化")
        subprocess.run(
            [
                self._edge_tts_cmd,
                "-t",
                text,
                "-v",
                self.voice,
                "--rate",
                self.rate,
                "--volume",
                self.volume,
                "--write-media",
                str(output),
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _convert_audio(self, input_path: Path, law: str, output: Path) -> None:
        """把本地音频或 TTS 产物统一转成 8kHz 单声道 G711。"""
        codec = "pcm_alaw" if law == "a" else "pcm_mulaw"
        raw_format = "alaw" if law == "a" else "mulaw"
        subprocess.run(
            [
                self.ffmpeg,
                "-y",
                "-loglevel",
                "error",
                "-i",
                str(input_path),
                "-ac",
                "1",
                "-ar",
                str(VOICE_SAMPLE_RATE),
                "-acodec",
                codec,
                "-f",
                raw_format,
                str(output),
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def render_g711(self, text: str, law: str) -> bytes:
        """返回 8kHz、单声道、原始 G711 数据，后续直接按 20ms 切帧。"""
        with tempfile.TemporaryDirectory(prefix="hik_voice_") as tmp:
            tmp_dir = Path(tmp)
            mp3_path = tmp_dir / "speech.mp3"
            g711_path = tmp_dir / "speech.g711"
            try:
                local_path = self._local_audio_path(text)
                if local_path is not None:
                    self._convert_audio(local_path, law, g711_path)
                elif self.audio_dir is not None:
                    # 固定音频模式下，未配置的词条保持静音，避免回退到在线 TTS。
                    return b""
                else:
                    self._ensure_tts_backend()
                    if self._edge_tts is not None:
                        asyncio.run(self._save_mp3(text, mp3_path))
                    else:
                        self._save_mp3_cli(text, mp3_path)
                    self._convert_audio(mp3_path, law, g711_path)
            except Exception as exc:
                raise RuntimeError(f"语音合成或转码失败: {text}") from exc

            return g711_path.read_bytes()


class LocalComputerSpeaker:
    """在运行 YOLO 的电脑上播放固定语音，不依赖相机音频硬件。"""

    def __init__(
            self, audio_dir: Path | None, player: str, speech_prefix: str,
            tts_voice: str, tts_rate: str, tts_volume: str) -> None:
        if audio_dir is None:
            raise RuntimeError("本地语音播放需要提供 speech-audio-dir")
        self.audio_dir = audio_dir.expanduser().resolve()
        resolved = AlarmSpeechRenderer._resolve_command(player)
        if resolved is None:
            raise FileNotFoundError(f"找不到本地音频播放器: {player}")
        self.player = resolved
        self.speech_prefix = speech_prefix
        self.tts_voice = tts_voice
        self.tts_rate = tts_rate
        self.tts_volume = tts_volume
        self._lock = threading.Lock()
        self._active: subprocess.Popen[bytes] | None = None
        self._closed = False
        self._missing: set[str] = set()
        print(f"[语音] 本地电脑播放: {self.player}，素材目录={self.audio_dir}", flush=True)

    async def _save_missing_audio(self, text: str, output: Path) -> None:
        import edge_tts
        communicate = edge_tts.Communicate(
            text, self.tts_voice, rate=self.tts_rate, volume=self.tts_volume)
        await asyncio.wait_for(communicate.save(str(output)), timeout=15.0)

    def _synthesize_missing_audio(self, text: str, path: Path) -> Path | None:
        """Generate a missing fixed phrase once, then reuse it offline."""
        temporary = path.with_name(f".{path.stem}.tmp.mp3")
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            asyncio.run(self._save_missing_audio(text, temporary))
            if not temporary.is_file() or temporary.stat().st_size == 0:
                raise RuntimeError("TTS 未生成音频")
            temporary.replace(path)
            print(f"[语音] 已自动生成并缓存本地素材: {path}", flush=True)
            return path
        except Exception as error:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
            print(
                f"[语音] 自动生成本地素材失败，继续播放类别词: {text}: {error}",
                file=sys.stderr, flush=True)
            return None

    def _audio_path(
            self, text: str, *, synthesize_missing: bool = False) -> Path | None:
        """按固定素材名称查找音频；缺失词条只跳过，不中断检测。"""
        filename = LOCAL_SPEECH_FILES.get(text, f"{text}.mp3")
        path = self.audio_dir / filename
        if path.is_file():
            return path
        if (
            (text == self.speech_prefix or synthesize_missing)
            and text and text not in self._missing
        ):
            cache_path = (
                Path.home() / ".cache" / "robot_inspection_speech" / filename)
            if cache_path.is_file():
                return cache_path
            generated = self._synthesize_missing_audio(text, cache_path)
            if generated is not None:
                return generated
        if text not in self._missing:
            self._missing.add(text)
            print(f"[语音] 本地素材缺失，跳过: {path}", file=sys.stderr, flush=True)
        return None

    def _play(self, path: Path) -> None:
        """同步等待当前短语播放结束；调用方位于独立语音队列线程。"""
        with self._lock:
            if self._closed:
                return
            process = subprocess.Popen(
                [
                    self.player,
                    "-hide_banner",
                    "-loglevel",
                    "error",
                    "-nodisp",
                    "-autoexit",
                    "-vn",
                    str(path),
                ],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._active = process
        try:
            return_code = process.wait()
            if return_code != 0 and not self._closed:
                raise RuntimeError(f"本地音频播放器退出码 {return_code}: {path.name}")
        finally:
            with self._lock:
                if self._active is process:
                    self._active = None

    def announce(self, alarm_keys: Iterable[str]) -> None:
        """按前缀和目标词顺序播放本地素材；不存在的前缀自动跳过。"""
        segments = [self.speech_prefix, *(TARGET_LABEL_CN[key] for key in alarm_keys if key in TARGET_LABEL_CN)]
        for text in segments:
            path = self._audio_path(text)
            if path is not None:
                self._play(path)

    def announce_texts(self, texts: Iterable[str]) -> None:
        """Play exact fixed phrases without the object-detection prefix."""
        for text in texts:
            path = self._audio_path(str(text), synthesize_missing=True)
            if path is not None:
                self._play(path)

    def close(self) -> None:
        """终止正在播放的子进程，避免退出时残留音频进程。"""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            process = self._active
        if process is not None and process.poll() is None:
            try:
                process.terminate()
                process.wait(timeout=1.0)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    process.kill()
                except OSError:
                    pass


class CompositeSpeechOutput:
    """把同一条告警转发到多个语音输出。"""

    def __init__(self, outputs: Iterable[Any]) -> None:
        self.outputs = list(outputs)

    def announce(self, alarm_keys: Iterable[str]) -> None:
        keys = tuple(alarm_keys)
        for output in self.outputs:
            try:
                output.announce(keys)
            except Exception as error:  # 一个输出失败不能影响另一个输出。
                print(f"[语音] 输出失败: {error}", file=sys.stderr, flush=True)

    def announce_texts(self, texts: Iterable[str]) -> None:
        phrases = tuple(texts)
        for output in self.outputs:
            try:
                output.announce_texts(phrases)
            except Exception as error:
                print(f"[语音] 输出失败: {error}", file=sys.stderr, flush=True)

    def close(self) -> None:
        for output in self.outputs:
            try:
                output.close()
            except Exception as error:
                print(f"[语音] 输出关闭失败: {error}", file=sys.stderr, flush=True)


class HikvisionDeviceSpeaker:
    """通过海康语音对讲转发接口把 G711 音频帧送到设备音频输出。"""

    def __init__(self, source: HikvisionV40Source | HikvisionJpegSource | RtspSource, args: argparse.Namespace) -> None:
        if not hasattr(source, "sdk") or source.sdk is None or source.user_id < 0:
            raise RuntimeError("设备尚未登录，不能启动设备语音播报")

        self.source = source
        self.sdk = source.sdk
        self.user_id = source.user_id
        self.args = args
        self.voice_channel = resolve_voice_channel(args, source.device_info)
        self.voice_handle = -1
        self._voice_ready = threading.Event()
        self._received_voice_packets = 0
        # 必须持有回调对象，防止 ctypes 回收后 SDK 调用悬空函数指针。
        self._voice_callback = VoiceDataCallback(self._on_voice_data)
        self._segment_cache: dict[str, list[bytes]] = {}
        self._silence_frame: bytes | None = None
        self.renderer = AlarmSpeechRenderer(args.speech_audio_dir, args.tts_voice, args.tts_rate, args.tts_volume, args.ffmpeg)

        self._bind_sdk_functions()
        compress = self._query_audio_compress()
        self.g711_law = self._select_g711_law(compress)
        try:
            self._log_output_path()
            self._open_voice_session()
            self._prime_cache()
        except Exception:
            self.close()
            raise

    def _bind_sdk_functions(self) -> None:
        """声明语音接口 ABI；视频取流脚本只声明了预览相关接口。"""
        self.sdk.NET_DVR_StartVoiceCom_MR_V30.argtypes = [c_int, c_uint, VoiceDataCallback, c_void_p]
        self.sdk.NET_DVR_StartVoiceCom_MR_V30.restype = c_int
        self.sdk.NET_DVR_VoiceComSendData.argtypes = [c_int, c_void_p, c_uint]
        # Linux HCNetSDK.h 将 BOOL 定义为 int（4 字节），不能按 ctypes.c_bool
        # 绑定返回值；返回值仍按 0/非 0 判断。
        self.sdk.NET_DVR_VoiceComSendData.restype = c_int
        self.sdk.NET_DVR_StopVoiceCom.argtypes = [c_int]
        self.sdk.NET_DVR_StopVoiceCom.restype = c_int
        self.sdk.NET_DVR_GetCurrentAudioCompress.argtypes = [c_int, POINTER(CompressionAudio)]
        self.sdk.NET_DVR_GetCurrentAudioCompress.restype = c_int
        self.sdk.NET_DVR_GetCurrentAudioCompress_V50.argtypes = [c_int, POINTER(AudioChannel), POINTER(CompressionAudio)]
        self.sdk.NET_DVR_GetCurrentAudioCompress_V50.restype = c_int

    def _sdk_error(self, prefix: str) -> str:
        """组合 SDK 错误码，便于现场排障。"""
        code = int(self.sdk.NET_DVR_GetLastError())
        return f"{prefix}: {code}"

    def _on_voice_data(self, _handle: int, _buffer: int, _size: int, _audio_flag: int, _user: object) -> None:
        """不保存设备麦克风音频，但用回调确认对讲会话已就绪。"""
        self._received_voice_packets += 1
        self._voice_ready.set()

    def _query_two_way_audio_config(self) -> dict[str, str]:
        """通过 ISAPI 读取实际音频输出类型；不修改设备配置。"""
        host = str(self.args.host)
        http_port = int(getattr(self.args, "http_port", 80))
        netloc = f"[{host}]" if ":" in host and not host.startswith("[") else host
        if http_port != 80:
            netloc = f"{netloc}:{http_port}"
        url = f"http://{netloc}/ISAPI/System/TwoWayAudio/channels/{self.voice_channel}"

        password_manager = urllib.request.HTTPPasswordMgrWithDefaultRealm()
        password_manager.add_password(None, url, self.args.username, self.args.password)
        # urllib 不识别 NO_PROXY 中的 CIDR；局域网设备必须显式绕过桌面代理，
        # 否则代理会改变 Digest 的 request-uri，最终表现为错误的 HTTP 401。
        opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({}),
            urllib.request.HTTPDigestAuthHandler(password_manager),
        )
        request = urllib.request.Request(url, headers={"Accept": "application/xml"})
        timeout = min(max(float(getattr(self.args, "connect_timeout_ms", 3000)) / 1000.0, 0.5), 3.0)
        with opener.open(request, timeout=timeout) as response:
            return parse_two_way_audio_config(response.read(64 * 1024))

    def _set_two_way_audio_enabled(self, enabled: bool) -> None:
        """通过明确命令行参数才修改设备双向音频开关。

        默认不改设备配置；开启 ``--voice-enable-output`` 后才会发送 PUT。
        """
        host = str(self.args.host)
        http_port = int(getattr(self.args, "http_port", 80))
        netloc = f"[{host}]" if ":" in host and not host.startswith("[") else host
        if http_port != 80:
            netloc = f"{netloc}:{http_port}"
        url = f"http://{netloc}/ISAPI/System/TwoWayAudio/channels/{self.voice_channel}"
        password_manager = urllib.request.HTTPPasswordMgrWithDefaultRealm()
        password_manager.add_password(None, url, self.args.username, self.args.password)
        opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({}),
            urllib.request.HTTPDigestAuthHandler(password_manager),
        )
        timeout = min(max(float(getattr(self.args, "connect_timeout_ms", 3000)) / 1000.0, 0.5), 3.0)
        with opener.open(urllib.request.Request(url, headers={"Accept": "application/xml"}), timeout=timeout) as response:
            payload = response.read(64 * 1024)
        request = urllib.request.Request(
            url,
            data=set_two_way_audio_enabled(payload, enabled),
            headers={"Content-Type": "application/xml", "Accept": "application/xml"},
            method="PUT",
        )
        with opener.open(request, timeout=timeout) as response:
            response.read(64 * 1024)

    def _log_output_path(self) -> None:
        """打印音频物理输出路径，避免把 LineOut 误当内置扬声器。"""
        audio_channels = _uint8(self.source.device_info.byAudioChanNum)
        print(f"[语音] 登录信息返回 {audio_channels} 个对讲通道")
        try:
            config = self._query_two_way_audio_config()
        except (OSError, ValueError, ElementTree.ParseError, urllib.error.URLError) as error:
            print(f"[语音] ISAPI 音频输出查询失败，继续使用 SDK: {error}", file=sys.stderr)
            return

        output_type = config["output_type"] or "未知"
        codec = config["codec"] or "未知"
        volume = config["volume"] or "未知"
        enabled = config["enabled"] or "未知"
        print(f"[语音] 设备输出={output_type}，配置编码={codec}，音量={volume}，enabled={enabled}")
        if output_type.lower() == "lineout":
            print("[语音] 提示: 该设备返回 LineOut，需连接有源音箱或功放；SDK 发送成功不代表机身会直接发声")
        if enabled.lower() == "false":
            if bool(getattr(self.args, "voice_enable_output", False)):
                print("[语音] 正在按命令行参数启用 ISAPI 双向音频输出")
                try:
                    self._set_two_way_audio_enabled(True)
                    config = self._query_two_way_audio_config()
                    print(f"[语音] ISAPI 双向音频已更新，enabled={config.get('enabled') or '未知'}")
                except (OSError, ValueError, ElementTree.ParseError, urllib.error.URLError) as error:
                    print(f"[语音] 启用 ISAPI 双向音频失败: {error}", file=sys.stderr)
            else:
                print("[语音] 提示: ISAPI 双向音频当前 enabled=false；请在设备音频/对讲设置中确认已启用输出")

    def _query_audio_compress(self) -> CompressionAudio:
        """查询当前音频编码；失败不阻塞播报，后续仍按 G711 尝试。"""
        compress = CompressionAudio()
        channel = AudioChannel()
        channel.dwChannelNum = self.voice_channel
        if self.sdk.NET_DVR_GetCurrentAudioCompress_V50(self.user_id, byref(channel), byref(compress)):
            return compress
        if self.sdk.NET_DVR_GetCurrentAudioCompress(self.user_id, byref(compress)):
            return compress
        print(f"[语音] 查询设备音频编码失败，将按 G711U 尝试: {self.sdk.NET_DVR_GetLastError()}", file=sys.stderr)
        return compress

    def _select_g711_law(self, compress: CompressionAudio) -> str:
        """根据命令行或设备返回值选择 U-law/A-law。

        auto 模式优先跟随设备当前的对讲编码；如果设备没有返回可识别的
        G711 编码，再回退到 U-law。手动传入 --voice-law u/a 时则强制覆盖。
        """
        enc_type = int(compress.byAudioEncType)
        enc_name = DEVICE_AUDIO_ENC_NAMES.get(enc_type, f"未知({enc_type})")
        law = choose_g711_law(self.args.voice_law, enc_type)
        if self.args.voice_law == "auto":
            if enc_type in (1, 2):
                print(f"[语音] 设备返回 {enc_name}，自动按 G711_{law.upper()} 发送")
            else:
                print("[语音] 设备未返回可直接复用的 G711 编码，自动回退到 G711_U")
        print(f"[语音] 设备音频编码={enc_name}，发送格式=G711_{law.upper()}，语音通道={self.voice_channel}")
        return law

    def _open_voice_session(self) -> None:
        """打开设备语音转发会话；句柄有效后才能 VoiceComSendData。"""
        self.voice_handle = int(
            self.sdk.NET_DVR_StartVoiceCom_MR_V30(self.user_id, self.voice_channel, self._voice_callback, None)
        )
        if self.voice_handle < 0:
            raise RuntimeError(self._sdk_error("设备语音转发启动失败"))
        ready = self._voice_ready.wait(VOICE_READY_TIMEOUT_SECONDS)
        state = "已收到设备音频回调" if ready else "未收到设备音频回调"
        print(f"[语音] 设备语音转发启动成功，句柄={self.voice_handle}，{state}")

    def _silence(self) -> bytes:
        """返回一个 20ms 静音 G711 帧，用于短语间隔。"""
        if self._silence_frame is None:
            silence_byte = 0xD5 if self.g711_law == "a" else 0xFF
            self._silence_frame = bytes([silence_byte]) * G711_FRAME_BYTES
        return self._silence_frame

    def _frames_for_text(self, text: str) -> list[bytes]:
        """合成并缓存短语，避免每次报警都访问 TTS 服务。"""
        if text in self._segment_cache:
            return self._segment_cache[text]

        g711 = self.renderer.render_g711(text, self.g711_law)
        frames: list[bytes] = []
        for offset in range(0, len(g711), G711_FRAME_BYTES):
            frame = g711[offset : offset + G711_FRAME_BYTES]
            if len(frame) < G711_FRAME_BYTES:
                frame += self._silence()[: G711_FRAME_BYTES - len(frame)]
            frames.append(frame)
        if not frames:
            frames.append(self._silence())
        self._segment_cache[text] = frames
        return frames

    def _prime_cache(self) -> None:
        """预生成固定报警短语，避免第一次识别到目标时才卡在 TTS 网络请求。"""
        print("[语音] 正在预热设备播报音频素材...")
        started = time.perf_counter()
        for text in [
            self.args.speech_prefix,
            *(TARGET_LABEL_CN[key] for key in TARGET_LABEL_ORDER),
            *ENVIRONMENT_SPEECH.values(),
        ]:
            self._frames_for_text(text)
        print(f"[语音] 音频素材预热完成: {time.perf_counter() - started:.2f} 秒")

    def announce(self, alarm_keys: Iterable[str]) -> None:
        """按“检测到 + 目标词”的短语序列向设备发送 G711 帧。"""
        segments = [self.args.speech_prefix, *(TARGET_LABEL_CN[key] for key in alarm_keys if key in TARGET_LABEL_CN)]
        if len(segments) <= 1:
            return

        frames: list[bytes] = []
        for index, segment in enumerate(segments):
            if index > 0:
                frames.append(self._silence())
            frames.extend(self._frames_for_text(segment))

        # 末尾追加两帧静音，避免设备端播放末尾被截断。
        frames.extend([self._silence(), self._silence()])
        self._send_frames(frames)

    def announce_texts(self, texts: Iterable[str]) -> None:
        """Send exact fixed phrases without the object-detection prefix."""
        frames: list[bytes] = []
        for text in texts:
            if frames:
                frames.append(self._silence())
            frames.extend(self._frames_for_text(str(text)))
        if not frames:
            return
        frames.extend([self._silence(), self._silence()])
        self._send_frames(frames)

    def _send_frames(self, frames: Iterable[bytes]) -> None:
        """按实时 20ms 节奏发送，过快发送会导致设备端播报断续或吞帧。"""
        started = time.monotonic()
        deadline = started
        sent_frames = 0
        sent_bytes = 0
        for frame in frames:
            buffer = create_string_buffer(frame)
            if not self.sdk.NET_DVR_VoiceComSendData(self.voice_handle, c_void_p(addressof(buffer)), len(frame)):
                raise RuntimeError(self._sdk_error(f"NET_DVR_VoiceComSendData 失败，已发 {sent_frames} 帧"))
            sent_frames += 1
            sent_bytes += len(frame)
            deadline += VOICE_FRAME_SECONDS
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        elapsed = time.monotonic() - started
        print(
            f"[语音] SDK 发送完成: {sent_frames} 帧/{sent_bytes} 字节/{elapsed:.2f} 秒，"
            f"设备回调 {self._received_voice_packets} 包",
            flush=True,
        )

    def close(self) -> None:
        """停止设备语音会话。"""
        if self.voice_handle >= 0:
            if not self.sdk.NET_DVR_StopVoiceCom(self.voice_handle):
                print(self._sdk_error("停止设备语音转发失败"), file=sys.stderr)
            self.voice_handle = -1


class SpeechQueueService:
    """后台串行语音队列，防止本地播放或设备发送阻塞 YOLO 推理循环。"""

    def __init__(self, output: Any) -> None:
        self._output = output
        self._queue: queue.Queue[tuple[str, tuple[str, ...]]] = queue.Queue()
        self._stop = threading.Event()
        self._closed = False
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def announce(self, alarm_keys: Iterable[str]) -> None:
        """把需要播报的类别放入队列。"""
        keys = tuple(alarm_keys)
        if not keys or self._closed:
            return
        self._queue.put(('alarms', keys))

    def announce_environment(self, key: str) -> None:
        """Queue one allow-listed environment phrase exactly as requested."""
        phrase = ENVIRONMENT_SPEECH.get(str(key))
        if phrase is None:
            raise ValueError('环境语音告警类型无效')
        if self._closed:
            raise RuntimeError('语音输出已关闭')
        self._queue.put(('texts', (phrase,)))

    def _run(self) -> None:
        while not self._stop.is_set() or not self._queue.empty():
            try:
                kind, values = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if kind == 'texts':
                    self._output.announce_texts(values)
                else:
                    self._output.announce(values)
            except Exception as exc:  # pragma: no cover - 硬件链路异常不能拖垮检测循环
                print(f"[语音] 播放失败: {exc}", file=sys.stderr)

    def close(self) -> None:
        """等待队列收尾并关闭设备语音会话。"""
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        self._worker.join(timeout=10.0)
        self._output.close()


def run(args: argparse.Namespace) -> None:
    """启动相机取流、多模型检测和本地语音播报。"""
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"模型不存在: {model_path}")
    if model_path.suffix.lower() == ".rknn":
        raise ValueError("本脚本使用 Ultralytics PT/ONNX 模型，不支持 RKNN")
    if not args.disable_face and not args.face_model.is_file():
        raise FileNotFoundError(f"人脸模型不存在: {args.face_model}")
    if not args.disable_plate and not args.plate_model.is_file():
        raise FileNotFoundError(f"车牌模型不存在: {args.plate_model}")

    primary_model = YOLO(str(model_path), task="detect")
    face_detector = None
    if not args.disable_face:
        if not hasattr(cv2, "FaceDetectorYN"):
            raise RuntimeError("当前 OpenCV 不支持 FaceDetectorYN，无法启用人脸检测")
        face_detector = cv2.FaceDetectorYN.create(
            str(args.face_model), "", (320, 320), args.face_conf,
            0.3, 5000)
    plate_model = (
        None if args.disable_plate
        else YOLO(str(args.plate_model), task="detect")
    )
    detector = MultiModelDetector(
        primary_model,
        face_detector=face_detector,
        plate_model=plate_model,
        primary_conf=args.conf,
        primary_iou=args.iou,
        primary_imgsz=args.imgsz,
        inspection_primary_imgsz=args.inspection_imgsz,
        device=args.device,
        face_conf=args.face_conf,
        face_max_width=args.face_max_width,
        plate_conf=args.plate_conf,
        plate_imgsz=args.plate_imgsz,
        transit_aux_fps=args.aux_transit_fps,
        manual_aux_fps=args.aux_manual_fps,
        display_ttl_seconds=args.aux_display_ttl,
    )
    # 先做空帧预热，减少正式检测首帧卡顿。
    detector.warmup()
    recognition_database = RecognitionDatabase(args.recognition_db)
    recognition = RecognitionRuntime(
        recognition_database,
        face_extractor=(
            None if args.disable_face or args.disable_recognition
            else SFaceFeatureExtractor(args.sface_model)),
        plate_recognizer=(
            None if args.disable_plate or args.disable_recognition
            else HyperLPRPlateRecognizer()),
        evidence_dir=args.recognition_evidence_dir,
    )
    recognition_status = recognition.model_status()
    print(
        "识别匹配层: "
        f"SFace={'就绪' if recognition_status['sface']['available'] else '不可用'}，"
        f"HyperLPR3={'就绪' if recognition_status['hyperlpr3']['available'] else '不可用'}，"
        f"数据库={recognition_database.path}", flush=True)

    source: Any | None = None
    voice_source: HikvisionV40Source | HikvisionJpegSource | None = None
    web_server: YoloMJPEGServer | None = None
    speech: SpeechQueueService | None = None
    policy = AlarmSpeechPolicy(args.cooldown)
    processed = 0
    smoothed_fps = 0.0
    window_open = False

    try:
        if args.web_port > 0:
            web_server = YoloMJPEGServer(args.web_host, args.web_port)
            web_server.inspection.set_recognition_status(recognition_status)
            web_server.start()
            print(f"网页结果流: http://{args.web_host}:{args.web_port}/", flush=True)
        source, frame = open_capture_source(args)
        print(
            f"原始输出画面尺寸: {frame.shape[1]}x{frame.shape[0]}；"
            f"主 YOLO 行进 imgsz={args.imgsz}/精检 {args.inspection_imgsz}，"
            f"车牌 imgsz={args.plate_imgsz}；"
            f"行进辅助检测 {args.aux_transit_fps:g} FPS",
            flush=True,
        )

        # 本地电脑播放是默认路径；只有显式选择 device/both 才创建 SDK 语音会话。
        outputs: list[Any] = []
        if args.voice_output in ("local", "both"):
            try:
                outputs.append(LocalComputerSpeaker(
                    args.speech_audio_dir, args.audio_player, args.speech_prefix,
                    args.tts_voice, args.tts_rate, args.tts_volume))
            except Exception as error:
                print(f"[语音] 本地播放初始化失败，继续 YOLO: {error}", file=sys.stderr, flush=True)
        if args.voice_output in ("device", "both"):
            try:
                assert source is not None
                # RTSP 不需要 SDK 登录。设备语音仍需要独立 JPEG SDK 会话，
                # 避免与 RTSP 视频路线耦合；设备无对讲时只禁用设备输出。
                voice_source = source if getattr(source, "sdk", None) is not None else HikvisionJpegSource(args)
                if voice_source is not source:
                    voice_source.connect()
                outputs.append(HikvisionDeviceSpeaker(voice_source, args))
            except Exception as error:
                if voice_source is not None and voice_source is not source:
                    try:
                        voice_source.release()
                    except Exception as release_error:
                        print(f"[语音] 独立 SDK 会话释放失败: {release_error}", file=sys.stderr, flush=True)
                voice_source = None
                print(f"[语音] 设备输出初始化失败，继续其他输出: {error}", file=sys.stderr, flush=True)
        if outputs:
            output = outputs[0] if len(outputs) == 1 else CompositeSpeechOutput(outputs)
            speech = SpeechQueueService(output)
            print(f"[语音] 输出模式: {args.voice_output}", flush=True)
        else:
            print("[语音] 没有可用输出，已禁用播报但继续 YOLO", file=sys.stderr, flush=True)
        if web_server is not None:
            web_server.set_speech_service(speech)
        if speech is not None and args.announce_test:
            try:
                print(f"[语音] 联调测试播报: {args.speech_prefix}{TARGET_LABEL_CN[args.announce_test]}")
                speech.announce([args.announce_test])
            except Exception as error:
                print(f"[语音] 联调播报失败，继续 YOLO: {error}", file=sys.stderr, flush=True)

        if args.show and not args.no_window:
            cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
            window_open = True
        elif args.no_window:
            print("OpenCV 窗口已禁用；按 Ctrl+C 退出")

        while args.max_frames == 0 or processed < args.max_frames:
            if web_server is not None and not web_server.wait_for_inference_turn():
                frame = source.read(args.frame_timeout)
                continue
            raw_frame = frame.copy()
            started = time.perf_counter()
            mode, task_id = (
                web_server.inspection.context()
                if web_server is not None else ("manual", "")
            )
            analysis = detector.analyze(frame, mode=mode)
            public_detections = recognition.process(
                raw_frame, analysis.public_detections(), task_id=task_id)
            elapsed = max(time.perf_counter() - started, 1e-9)
            fps = 1.0 / elapsed
            smoothed_fps = fps if smoothed_fps == 0.0 else smoothed_fps * 0.9 + fps * 0.1

            alarm_detections = analysis.alarm_detections()
            alarm_keys = [key for key, _score in alarm_detections]
            if alarm_detections:
                print(f"[YOLO] 识别结果: {format_alarm_summary(alarm_detections)}", flush=True)
            due_keys = policy.pick_due(alarm_keys)
            if due_keys and speech is not None:
                message = build_alarm_message(due_keys, args.speech_prefix)
                print(f"[语音] 播报: {message}", flush=True)
                speech.announce(due_keys)

            annotated_frame = raw_frame.copy()
            object_count = detector.annotate(annotated_frame, analysis)
            recognition.annotate(annotated_frame, public_detections)
            draw_status(
                annotated_frame, smoothed_fps, object_count,
                source.frames.dropped_count())
            if web_server is not None:
                options = [int(cv2.IMWRITE_JPEG_QUALITY), args.web_quality]
                annotated_ok, annotated_jpeg = cv2.imencode(
                    ".jpg", annotated_frame, options)
                raw_ok, raw_jpeg = cv2.imencode(".jpg", raw_frame, options)
                if annotated_ok:
                    web_server.publish(
                        annotated_jpeg.tobytes(),
                        inference_fps=smoothed_fps,
                        capture_fps=source.frames.capture_fps(),
                        frame_width=annotated_frame.shape[1],
                        frame_height=annotated_frame.shape[0],
                    )
                if raw_ok and annotated_ok:
                    web_server.publish_inspection_frame(
                        raw_jpeg.tobytes(),
                        annotated_jpeg.tobytes(),
                        public_detections,
                    )
            if args.show and not args.no_window:
                cv2.imshow(args.window_name, annotated_frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")) or cv2.getWindowProperty(args.window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break

            processed += 1
            frame = source.read(args.frame_timeout)
    except KeyboardInterrupt:
        print("收到退出信号")
    finally:
        if speech is not None:
            try:
                speech.close()
            except Exception as error:
                print(f"[语音] 关闭失败: {error}", file=sys.stderr, flush=True)
        if voice_source is not None and voice_source is not source:
            try:
                voice_source.release()
            except Exception as error:
                print(f"[语音] 独立 SDK 会话释放失败: {error}", file=sys.stderr, flush=True)
        if web_server is not None:
            try:
                web_server.close()
            except Exception as error:
                print(f"[网页] 结果流关闭失败: {error}", file=sys.stderr, flush=True)
        try:
            if source is not None:
                source.release()
        except Exception as error:
            print(f"[视频] 释放取流资源失败: {error}", file=sys.stderr, flush=True)
        if window_open:
            try:
                cv2.destroyAllWindows()
            except Exception as error:
                print(f"[显示] 窗口关闭失败: {error}", file=sys.stderr, flush=True)
        recognition_database.close()
        print(f"结束，共处理 {processed} 帧")


def parse_args() -> argparse.Namespace:
    """解析运行参数，默认使用本地电脑播放固定告警音频。"""
    parser = argparse.ArgumentParser(
        description="USB 相机 YOLO 告警与语音播报",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="YOLO PT/ONNX 模型路径")
    parser.add_argument("--face-model", type=Path, default=DEFAULT_FACE_MODEL, help="YuNet 人脸 ONNX 模型路径")
    parser.add_argument("--plate-model", type=Path, default=DEFAULT_PLATE_MODEL, help="车牌 YOLO PT/ONNX 模型路径")
    parser.add_argument("--sface-model", type=Path, default=DEFAULT_SFACE_MODEL, help="OpenCV SFace ONNX 模型路径")
    parser.add_argument("--recognition-db", type=Path, default=DEFAULT_RECOGNITION_DB, help="独立人车识别数据库路径")
    parser.add_argument("--recognition-evidence-dir", type=Path, default=DEFAULT_EVIDENCE_DIR, help="人车识别证据目录")
    parser.add_argument("--disable-recognition", action="store_true", help="保留人脸/车牌检测框但禁用匹配与事件记录")
    parser.add_argument("--disable-face", action="store_true", help="禁用人脸检测")
    parser.add_argument("--disable-plate", action="store_true", help="禁用车牌检测")
    parser.add_argument("--sdk-dir", type=Path, default=DEFAULT_SDK_DIR, help="HCNetSDK 库文件目录")
    parser.add_argument("--host", default=DEFAULT_HOST, help="海康设备 IP 或主机名")
    parser.add_argument("--sdk-port", type=int, default=8000, help="HCNetSDK 服务端口")
    parser.add_argument("--http-port", type=int, default=80, help="ISAPI HTTP 端口，用于查询音频输出类型")
    parser.add_argument("--username", default="admin", help="海康设备用户名")
    parser.add_argument("--password", default=os.environ.get("HIKVISION_PASSWORD"), help="设备密码，也可设置 HIKVISION_PASSWORD")
    parser.add_argument("--channel", type=int, default=1, help="预览视频通道")
    parser.add_argument("--stream", type=int, choices=(0, 1), default=0, help="0 主码流，1 子码流")
    parser.add_argument("--link-mode", type=int, choices=range(8), default=0, help="V40 链路：0 TCP，1 UDP，2 组播，3 RTP，4 RTSP/RTP，5 RTSP/HTTP，6 HRUDP，7 RTSP/HTTPS")
    parser.add_argument("--capture-backend", choices=("mjpeg", "jpeg", "rtsp", "v40", "auto"), default="mjpeg", help="采集路线；USB 默认复用主相机服务 MJPEG 流")
    parser.add_argument("--mjpeg-url", default="http://127.0.0.1:8766/api/stream", help="主相机服务的 MJPEG 输入地址")
    parser.add_argument("--rtsp-url", default=os.environ.get("HIKVISION_RTSP_URL", ""), help="自定义 RTSP 地址；为空时按通道和码流自动生成")
    parser.add_argument("--rtsp-port", type=int, default=554, help="RTSP 服务端口")
    parser.add_argument("--rtsp-transport", choices=("tcp", "udp"), default="tcp", help="RTSP 传输方式；TCP 更稳定")
    parser.add_argument("--rtsp-open-timeout-ms", type=int, default=5000, help="RTSP 建连超时毫秒")
    parser.add_argument("--rtsp-read-timeout-ms", type=int, default=5000, help="RTSP 读取超时毫秒")
    parser.add_argument("--rtsp-reconnect-delay", type=float, default=1.0, help="RTSP 断线首次重连等待秒数")
    parser.add_argument("--rtsp-reconnect-max-delay", type=float, default=8.0, help="RTSP 断线重连最大等待秒数")
    parser.add_argument("--connect-timeout-ms", type=int, default=3000, help="SDK 单次登录连接超时毫秒")
    parser.add_argument("--connect-retries", type=int, default=1, help="SDK 登录连接尝试次数")
    parser.add_argument("--frame-timeout", type=float, default=10.0, help="等待首帧或最新帧的超时秒数")
    parser.add_argument("--conf", type=float, default=0.50, help="检测置信度阈值；达到 50% 才确认")
    parser.add_argument("--iou", type=float, default=0.7, help="NMS 的 IoU 阈值")
    parser.add_argument("--imgsz", type=int, default=640, help="主 YOLO 推理输入尺寸")
    parser.add_argument("--inspection-imgsz", type=int, default=1280, help="到点精检主 YOLO 输入尺寸")
    parser.add_argument("--face-conf", type=float, default=0.60, help="人脸确认阈值")
    parser.add_argument("--face-max-width", type=int, default=640, help="YuNet 输入最大宽度")
    parser.add_argument("--plate-conf", type=float, default=0.25, help="车牌确认阈值")
    parser.add_argument("--plate-imgsz", type=int, default=640, help="车牌 YOLO 推理输入尺寸")
    parser.add_argument("--aux-transit-fps", type=float, default=3.0, help="行进中人脸/车牌最高检测频率")
    parser.add_argument("--aux-manual-fps", type=float, default=5.0, help="手动预览时人脸/车牌最高检测频率")
    parser.add_argument("--aux-display-ttl", type=float, default=0.60, help="限频帧之间辅助检测框保留秒数")
    parser.add_argument("--device", default="cpu", help="推理设备，例如 cpu 或 CUDA 编号 0")
    parser.add_argument("--show", action="store_true", help="显示 OpenCV 检测窗口")
    parser.add_argument("--no-window", action="store_true", help="不打开 OpenCV 窗口，适合网页后台启动")
    parser.add_argument("--window-name", default="USB Camera YOLO Detection", help="OpenCV 窗口标题")
    parser.add_argument("--web-host", default="127.0.0.1", help="YOLO 结果流监听地址")
    parser.add_argument("--web-port", type=int, default=0, help="YOLO 结果流监听端口，0 表示不启用网页流")
    parser.add_argument("--web-quality", type=int, default=95, help="YOLO 结果流 JPEG 质量 1-100")
    parser.add_argument("--speech-audio-dir", type=Path, default=DEFAULT_SPEECH_AUDIO_DIR, help="本地固定语音素材目录")
    parser.add_argument("--max-frames", type=int, default=0, help="最多处理帧数；0 表示持续运行")
    parser.add_argument("--cooldown", type=float, default=8.0, help="同一类目标的播报冷却时间秒数")
    parser.add_argument("--voice-channel", type=int, default=0, help="语音对讲通道；0 表示按登录信息自动推导")
    parser.add_argument("--voice-law", choices=("auto", "u", "a"), default="auto", help="G711 压缩律；auto 会跟随设备当前语音编码")
    parser.add_argument("--voice-output", choices=("local", "device", "both", "none"), default="local", help="语音输出位置；默认本机，device/both 才访问设备音频接口")
    parser.add_argument("--audio-player", default="ffplay", help="本地电脑音频播放器，默认 ffplay")
    parser.add_argument("--voice-enable-output", action="store_true", help="显式通过 ISAPI 将双向音频 enabled 设为 true；默认不修改设备配置")
    parser.add_argument("--speech-prefix", default="检测到", help="播报前缀")
    parser.add_argument("--announce-test", choices=TARGET_LABEL_ORDER, default="", help="启动后立即播报一次，用于联调测试")
    parser.add_argument("--tts-voice", default=DEFAULT_TTS_VOICE, help="edge-tts 中文语音名称")
    parser.add_argument("--tts-rate", default="+0%", help="edge-tts 语速，例如 +0%%、-10%%、+20%%")
    parser.add_argument("--tts-volume", default="+0%", help="edge-tts 音量，例如 +0%%、+20%%、-20%%")
    parser.add_argument("--ffmpeg", default="ffmpeg", help="ffmpeg 命令名或绝对路径，用于转码到 8k PCM")
    args = parser.parse_args()

    args.model = args.model.expanduser().resolve()
    args.face_model = args.face_model.expanduser().resolve()
    args.plate_model = args.plate_model.expanduser().resolve()
    args.sface_model = args.sface_model.expanduser().resolve()
    args.recognition_db = args.recognition_db.expanduser().resolve()
    args.recognition_evidence_dir = args.recognition_evidence_dir.expanduser().resolve()
    args.recognition_evidence_dir.mkdir(parents=True, exist_ok=True)
    args.sdk_dir = args.sdk_dir.expanduser().resolve()
    if args.speech_audio_dir is not None:
        args.speech_audio_dir = args.speech_audio_dir.expanduser().resolve()
    # 该开关明确表示要使用设备输出；未指定输出位置时自动切换到 device。
    if args.voice_enable_output and args.voice_output == "local":
        args.voice_output = "device"
    needs_hikvision_login = (
        args.capture_backend != "mjpeg"
        or args.voice_output in ("device", "both")
        or args.voice_enable_output
    )
    if needs_hikvision_login and not args.password:
        parser.error("请通过 HIKVISION_PASSWORD 或 --password 提供设备密码")
    if args.capture_backend == "mjpeg" and not str(args.mjpeg_url).strip():
        parser.error("MJPEG 采集地址不能为空")
    if not 1 <= args.sdk_port <= 65535 or not 1 <= args.http_port <= 65535 or args.channel <= 0:
        parser.error("SDK/HTTP 端口必须在 1 到 65535，视频通道必须大于 0")
    if args.connect_timeout_ms <= 0 or args.connect_retries <= 0 or args.frame_timeout <= 0:
        parser.error("连接超时、连接次数和帧超时必须大于 0")
    if not 1 <= args.rtsp_port <= 65535 or args.rtsp_open_timeout_ms <= 0 or args.rtsp_read_timeout_ms <= 0:
        parser.error("RTSP 端口和超时参数无效")
    if args.rtsp_reconnect_delay <= 0 or args.rtsp_reconnect_max_delay < args.rtsp_reconnect_delay:
        parser.error("RTSP 重连等待参数无效")
    if not 0.0 <= args.conf <= 1.0 or not 0.0 <= args.iou <= 1.0:
        parser.error("conf 和 iou 必须在 0 到 1 之间")
    if not 0.0 <= args.face_conf <= 1.0 or not 0.0 <= args.plate_conf <= 1.0:
        parser.error("face-conf 和 plate-conf 必须在 0 到 1 之间")
    if (
        args.imgsz <= 0 or args.imgsz % 32 != 0
        or args.inspection_imgsz <= 0 or args.inspection_imgsz % 32 != 0
        or args.max_frames < 0
    ):
        parser.error("imgsz/inspection-imgsz 必须是 32 的正整数倍，max-frames 不能小于 0")
    if args.plate_imgsz <= 0 or args.plate_imgsz % 32 != 0 or args.face_max_width <= 0:
        parser.error("plate-imgsz 必须是 32 的正整数倍，face-max-width 必须大于 0")
    if args.aux_transit_fps < 0 or args.aux_manual_fps < 0 or args.aux_display_ttl < 0:
        parser.error("辅助检测频率和显示保留时间不能小于 0")
    if args.cooldown < 0 or args.voice_channel < 0:
        parser.error("cooldown 和 voice-channel 不能小于 0")
    if not 0 <= args.web_port <= 65535:
        parser.error("web-port 必须在 0 到 65535 之间")
    if not 1 <= args.web_quality <= 100:
        parser.error("web-quality 必须在 1 到 100 之间")
    return args


if __name__ == "__main__":
    arguments = parse_args()
    # RTSP 视频路线不需要加载 PlayM4；跳过原生播放器库可降低网页 YOLO
    # 进程与 OpenCV-FFmpeg 的动态库冲突。JPEG/V40 路线仍使用官方库路径。
    if (
        arguments.capture_backend in {"jpeg", "v40", "auto"}
        or arguments.voice_output in {"device", "both"}
    ):
        ensure_player_library_path(arguments.sdk_dir)
    run(arguments)
