import threading
import time
from typing import Optional, Literal

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

from robodk.robolink import ITEM_TYPE_CAMERA
from robodk_controller import RoboDKController

DiseaseKey = Literal["EXH", "COL", "BLE", "CHR"]

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ctrl = RoboDKController(rdk_path=None)

# ================= STREAM =================
CAM_NAME = "D435_SIM_CAM"
cam = ctrl.RDK.Item(CAM_NAME, ITEM_TYPE_CAMERA)
if not cam.Valid():
    raise RuntimeError(f"❌ RoboDK에서 카메라 '{CAM_NAME}'를 찾지 못함")
cam.setParam("Open", 1)

def mjpeg_generator(fps: int = 10):
    boundary = b"--frame\r\n"
    delay = 1.0 / fps
    while True:
        frame_bytes = ctrl.RDK.Cam2D_Snapshot("", cam)
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

# ================= STATE =================
STATE = {
    "fallen": False,
    "fall_reason": "ok",
    "fall_event_id": 0,
    "fall_started_ms": None,
    "last_pose": None,

    "busy": False,
    "last_dispatch": None,

    # ✅ stage(음성 트리거용)
    "dispatch_stage": "IDLE",   # IDLE|PICKING|LOADING|NAVIGATING|ARRIVED|DONE|ERROR
    "dispatch_stage_id": 0,     # stage 바뀔 때마다 +1
    "dispatch_disease": None,
}

# FALL 튜닝
TILT_TH = 60.0
Z_TH_MM = 250.0
CONFIRM_SEC = 0.8
RECOVER_SEC = 1.2
USE_ABS = True   # 쓰러짐 감지가 안 맞으면 False로 바꿔(Pose 기준)

_candidate_since: Optional[float] = None
_normal_since: Optional[float] = None

def set_stage(stage: str):
    STATE["dispatch_stage"] = stage
    STATE["dispatch_stage_id"] += 1

def fall_watch_loop(poll_hz: float = 10.0):
    global _candidate_since, _normal_since
    dt = 1.0 / poll_hz

    while True:
        try:
            info = ctrl.detect_person_fallen(
                tilt_deg_threshold=TILT_TH,
                z_threshold_mm=Z_TH_MM,
                use_z=True,
                use_abs=USE_ABS,
            )

            now = time.time()
            candidate = bool(info["fallen"])
            prev_fallen = STATE["fallen"]

            if candidate:
                _normal_since = None
                if _candidate_since is None:
                    _candidate_since = now
                fallen_now = True if (now - _candidate_since) >= CONFIRM_SEC else prev_fallen
            else:
                _candidate_since = None
                if _normal_since is None:
                    _normal_since = now
                fallen_now = False if (now - _normal_since) >= RECOVER_SEC else prev_fallen

            STATE["fallen"] = fallen_now
            STATE["fall_reason"] = info.get("reason", "unknown")
            STATE["last_pose"] = {k: info[k] for k in ["x", "y", "z", "roll", "pitch", "yaw"]}

            if (not prev_fallen) and fallen_now:
                STATE["fall_event_id"] += 1
                STATE["fall_started_ms"] = int(now * 1000)

        except Exception:
            pass

        time.sleep(dt)

threading.Thread(target=fall_watch_loop, daemon=True).start()

@app.get("/status")
def status():
    return STATE

# ================= DISPATCH =================
class DispatchReq(BaseModel):
    disease_key: DiseaseKey
    fallen: bool = True

@app.post("/dispatch")
def dispatch(req: DispatchReq):
    if STATE["busy"]:
        raise HTTPException(status_code=409, detail="Robot is busy")

    def run_job():
        STATE["busy"] = True
        STATE["dispatch_disease"] = req.disease_key
        STATE["last_dispatch"] = None

        try:
            # 1) 박스 집기 시작
            set_stage("PICKING")
            ctrl.pick_box(req.disease_key)

            # 2) 터틀봇에 적재
            set_stage("LOADING")
            ctrl.load_box_on_turtlebot(req.disease_key)

            # 3) 이동 시작
            set_stage("NAVIGATING")
            ctrl.navigate_turtlebot_to_person()

            # 4) 도착(시뮬에서는 navigate 완료 = 도착)
            set_stage("ARRIVED")
            time.sleep(0.2)

            set_stage("DONE")
            STATE["last_dispatch"] = {"ok": True, "disease": req.disease_key}

        except Exception as e:
            set_stage("ERROR")
            STATE["last_dispatch"] = {"ok": False, "error": str(e)}
        finally:
            STATE["busy"] = False

    threading.Thread(target=run_job, daemon=True).start()
    return {"ok": True, "started": True}
