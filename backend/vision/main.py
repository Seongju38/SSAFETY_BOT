import time
import threading
from fastapi import FastAPI
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware

from dotenv import load_dotenv
load_dotenv()

from vision.fall_detector import run_detector, get_latest_jpeg, get_fall_event_id
from robot.server.services.clova_tts import synthesize_tts_mp3

app = FastAPI()

# 개발 편의용 CORS (배포 땐 도메인 제한 추천)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 최신 음성 캐시
_latest_audio_id = None
_latest_audio_mp3 = None

@app.on_event("startup")
def _startup():
    # fall detector 백그라운드 실행
    t = threading.Thread(target=run_detector, kwargs={"show_local_window": False}, daemon=True)
    t.start()

def mjpeg_generator():
    while True:
        frame = get_latest_jpeg()
        if frame is None:
            time.sleep(0.05)
            continue
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(0.03)

@app.get("/stream")
def stream():
    return StreamingResponse(
        mjpeg_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )

def _generate_alert_audio(alert_id: int):
    global _latest_audio_id, _latest_audio_mp3
    text = "쓰러짐이 감지되었습니다. 관리자 화면에서 확인해주세요."
    mp3 = synthesize_tts_mp3(text=text, speaker="nara", speed=0)
    _latest_audio_id = alert_id
    _latest_audio_mp3 = mp3

@app.get("/alert/last")
def alert_last():
    # fall_detector가 올려준 최신 이벤트 ID를 기준으로, 오디오 생성 여부를 판단
    event_id = get_fall_event_id()
    return {"eventId": event_id, "audioId": _latest_audio_id}

@app.get("/alert/audio")
def alert_audio():
    if _latest_audio_mp3 is None:
        return JSONResponse({"ok": False, "reason": "no audio yet"}, status_code=404)

    def gen():
        yield _latest_audio_mp3

    return StreamingResponse(gen(), media_type="audio/mpeg")

@app.post("/alert/trigger")
def alert_trigger():
    # 테스트용 (넘어짐 없이도 음성 생성 테스트 가능)
    fake_id = int(time.time())
    _generate_alert_audio(fake_id)
    return {"ok": True, "audioId": fake_id}

@app.post("/alert/from-fall")
def alert_from_fall():
    # 실제로는 fall_detector 이벤트 발생 시 이걸 자동으로 호출하거나,
    # 아래 "폴링 감지" 방식으로 오디오 생성해도 됨.
    event_id = get_fall_event_id()
    if event_id is None:
        return {"ok": False, "reason": "no fall event"}
    _generate_alert_audio(event_id)
    return {"ok": True, "audioId": event_id}
