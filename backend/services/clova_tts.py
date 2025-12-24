import os
import requests

CLOVA_TTS_URL = "https://naveropenapi.apigw.ntruss.com/tts-premium/v1/tts"

def synthesize_tts_mp3(text: str, speaker: str = "nara", speed: int = 0) -> bytes:
    key_id = os.environ["NCP_API_KEY_ID"]
    key = os.environ["NCP_API_KEY"]

    headers = {
        "X-NCP-APIGW-API-KEY-ID": key_id,
        "X-NCP-APIGW-API-KEY": key,
        "Content-Type": "application/x-www-form-urlencoded; charset=UTF-8",
    }
    data = {
        "speaker": speaker,
        "speed": str(speed),
        "format": "mp3",
        "text": text,
    }

    r = requests.post(CLOVA_TTS_URL, headers=headers, data=data, timeout=15)
    r.raise_for_status()
    return r.content
