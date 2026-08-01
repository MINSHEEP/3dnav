from fastapi import FastAPI, Body
from pydantic import BaseModel
import requests
import tempfile
import os
import uvicorn
from pydub import AudioSegment
from pydub.playback import play



app = FastAPI()

# 配置你的 TTS 服务地址
TTS_API_URL = "http://10.0.1.14:10253/tts/generate/v1"

class TTSRequest(BaseModel):
    text: str

@app.post("/speak")
def speak(request: TTSRequest):
    # 向 TTS 接口发送请求
    try:
        print(request.text)
        schema = {"id":"male_02", "text": request.text}
        print(schema)
        response = requests.post(TTS_API_URL, data=schema)
        response.raise_for_status()
    except Exception as e:
        return {"success": False, "error": f"TTS 接口调用失败: {str(e)}"}

    # 保存音频文件到临时目录
    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix=".wmv") as tmp_audio:
            tmp_audio.write(response.content)
            tmp_audio_path = tmp_audio.name

        # 播放音频

        audio = AudioSegment.from_file(tmp_audio_path)
        play(audio)
        os.remove(tmp_audio_path)
    except Exception as e:
        return {"success": False, "error": f"音频播放失败: {str(e)}"}


    return {"success": True, "message": "播放完成"}

if __name__ == '__main__':
    uvicorn.run("tts:app", host='0.0.0.0', port=9090, workers=1)