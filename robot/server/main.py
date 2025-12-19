import time
import threading
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware

from vision.fall_detector import main, get_latest_jpeg

app = FastAPI()

# 개발 편의용 CORS (배포 땐 도메인 제한 추천)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.on_event("startup")
def _startup():
    # fall detector를 백그라운드로 실행
    t = threading.Thread(target=main, kwargs={"show_local_window": False}, daemon=True)
    t.start()

def mjpeg_generator():
    while True:
        frame = get_latest_jpeg()
        if frame is None:
            time.sleep(0.05)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )

        # 너무 빨리 보내면 CPU 많이 씀 → 적당히 제한
        time.sleep(0.03)  # 약 30fps 느낌

@app.get("/stream")
def stream():
    return StreamingResponse(
        mjpeg_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )
