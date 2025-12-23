import time
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
from robodk.robolink import Robolink, ITEM_TYPE_CAMERA

app = FastAPI()

RDK = Robolink()

# 방금 만든 2D 카메라 아이템 이름
CAM_NAME = "D435_SIM_CAM"

cam = RDK.Item(CAM_NAME, ITEM_TYPE_CAMERA)
if not cam.Valid():
    raise RuntimeError(f"❌ RoboDK에서 카메라 '{CAM_NAME}'를 찾지 못함 (이름 확인 필요)")

cam.setParam("Open", 1)

def mjpeg_generator(fps: int = 10):
    boundary = b"--frame\r\n"
    delay = 1.0 / fps

    while True:
        # RoboDK가 JPEG bytes를 주는 경우가 많음
        frame_bytes = RDK.Cam2D_Snapshot("", cam)

        if frame_bytes:
            yield (
                boundary
                + b"Content-Type: image/jpeg\r\n\r\n"
                + frame_bytes
                + b"\r\n"
            )
        time.sleep(delay)

@app.get("/stream")
def stream():
    return StreamingResponse(
        mjpeg_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )