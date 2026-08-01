import pyaudio
import numpy as np
import time
import threading
import queue
import requests
import tempfile
import wave
import os


CHUNK = 1024          # 每帧样本数
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000          # 采样率
SILENCE_THRESHOLD = 500  # 音量阈值，低于这个值认为是静音
SILENCE_MAX_DURATION = 2.0  # 静音超过多少秒认为句子结束
ASR_API = "http://10.0.1.14:10252/asr"  # 替换成你的 ASR 接口


class AudioRecorder:
    def __init__(self):
        self.audio_interface = pyaudio.PyAudio()
        self.stream = self.audio_interface.open(format=FORMAT,
                                                channels=CHANNELS,
                                                rate=RATE,
                                                input=True,
                                                frames_per_buffer=CHUNK)
        self.frames = []
        self.queue = queue.Queue()
        self.running = False

    def start(self):
        self.running = True
        threading.Thread(target=self._record_loop, daemon=True).start()
        threading.Thread(target=self._process_loop, daemon=True).start()

    def _record_loop(self):
        last_sound_time = time.time()
        while self.running:
            data = self.stream.read(CHUNK, exception_on_overflow=False)
            audio_np = np.frombuffer(data, dtype=np.int16)
            volume = np.abs(audio_np).mean()

            if volume > SILENCE_THRESHOLD:
                self.frames.append(data)
                last_sound_time = time.time()
            else:
                # 静音超过阈值且有缓存音频
                if self.frames and (time.time() - last_sound_time) > SILENCE_MAX_DURATION:
                    self.queue.put(list(self.frames))
                    self.frames = []

    def _process_loop(self):
        while self.running:
            try:
                audio_chunk = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self.send_to_asr(audio_chunk)

    def send_to_asr(self, frames):

        tmp_wav_path = os.path.join(tempfile.gettempdir(), f"tmp_audio_{int(time.time()*1000)}.wav")
        wf = wave.open(tmp_wav_path, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(self.audio_interface.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()  # 先关闭文件

        try:
        # 发送给 ASR
            with open(tmp_wav_path, 'rb') as f:
                files = {'file': (os.path.basename(tmp_wav_path), f, 'audio/wav')}
                try:
                    resp = requests.post(ASR_API, files=files)
                    if resp.status_code == 200:
                        data = resp.json()
                        text = data.get('result', {}).get('text', '')
                        text_nospecial = data.get('result', {}).get('text_nospecial', '')
                        print("识别结果:", text)
                        print("去除特殊字符:", text_nospecial)
                        if "小狗" in text:
                            print("触发事件: 发现关键词 '小狗'!")
                    else:
                        print("ASR 请求失败:", resp.status_code)
                except Exception as e:
                    print("ASR 请求异常:", e)
        finally:
        # 删除临时文件
            try:
                os.remove(tmp_wav_path)
            except Exception as e:
                print("删除临时文件失败:", e)


    def stop(self):
        self.running = False
        self.stream.stop_stream()
        self.stream.close()
        self.audio_interface.terminate()


if __name__ == "__main__":
    recorder = AudioRecorder()
    print("开始收音，检测到静音句子会发送给 ASR...")
    recorder.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        recorder.stop()
        print("结束收音")