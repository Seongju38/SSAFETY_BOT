import cv2
import numpy as np
# import pyrealsense2 as rs
from ultralytics import YOLO
import threading

import time

# -----------------------------
# MJPEG 스트리밍용 공유 프레임
# -----------------------------
_latest_jpeg = None
_lock = threading.Lock()

def get_latest_jpeg():
    with _lock:
        return _latest_jpeg

def _set_latest_frame_bgr(frame_bgr):
    """BGR 이미지를 JPEG bytes로 변환해서 최신 프레임으로 저장"""
    global _latest_jpeg
    ok, buf = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if ok:
        with _lock:
            _latest_jpeg = buf.tobytes()

# ------------------------------------
# posture 판단 함수 (사람 한 명 기준)
# ------------------------------------
def get_posture(kpts, bbox):
    """
    kpts: (num_kpts, 2) keypoints (x, y)
    bbox: [x1, y1, x2, y2]
    """
    x1, y1, x2, y2 = bbox
    w = x2 - x1
    h = y2 - y1

    # bbox 비율로 대략적인 자세 판단
    # 세로가 훨씬 길면 standing, 가로가 훨씬 길면 lying
    if w > h * 1.2:
        posture = "lying"
    else:
        posture = "standing"

    return posture

# ------------------------------------
# fall 감지 시 eventId 올려 주는 전역 상태 추가
# ------------------------------------
_fall_event_id = None
_last_fall_time = 0.0
_COOLDOWN_SEC = 5.0

def get_fall_event_id():
    return _fall_event_id

def _mark_fall_event():
    global _fall_event_id, _last_fall_time
    now = time.time()
    if now - _last_fall_time < _COOLDOWN_SEC:
        return
    _last_fall_time = now
    _fall_event_id = int(now)  # 간단하게 timestamp를 ID로 사용

def run_detector(show_local_window: bool = True, cam_index: int = 0):
    # -----------------------
    # 1) RealSense 설정
    # -----------------------
    # pipeline = rs.pipeline()
    # config = rs.config()
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # pipeline.start(config)

    # -----------------------
    # 1) Webcam 설정
    # ----------------------- 
    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError(...)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)


    # -----------------------
    # 2) YOLO Pose 모델 로드
    # -----------------------
    model = YOLO("yolov8n-pose.pt")  # 로컬에 없으면 자동 다운로드

    print("Press Q to quit")

    try:
        while True:
            # RealSense frame 받기
            # frames = pipeline.wait_for_frames()
            # color_frame = frames.get_color_frame()
            # if not color_frame:
            #     continue

            # frame = np.asanyarray(color_frame.get_data())

            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            # YOLO Pose 추론
            results = model(frame, verbose=False)

            annotated = frame.copy()

            if len(results) > 0:
                r = results[0]

                # 기본 skeleton + bbox 그린 이미지부터 시작
                annotated = r.plot()

                boxes = r.boxes.xyxy.cpu().numpy() if r.boxes is not None else []
                kpts_all = r.keypoints.xy.cpu().numpy() if r.keypoints is not None else []

                for box, kpts in zip(boxes, kpts_all):
                    posture = get_posture(kpts, box)

                    x1, y1, x2, y2 = box.astype(int)
                    # posture에 따른 색
                    if posture == "standing":
                        color = (0, 255, 0)      # 초록
                        label = "STAND"
                    else:
                        color = (0, 0, 255)      # 빨강
                        label = "FALL"
                        _mark_fall_event()

                    # 두꺼운 테두리로 박스 다시 강조
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 3)

                    # 사람 위에 텍스트 표시
                    cv2.putText(
                        annotated,
                        label,
                        (x1, max(y2 , 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        color,
                        2,
                    )

            # 웹으로 보낼 최신 프레임 업데이트
            _set_latest_frame_bgr(annotated)

            # show_local_window = True / False 로 로컬 창 확인 Test
            if show_local_window:
                cv2.imshow("Multi-person Fall Detector", annotated)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    finally:
        # pipeline.stop()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run_detector()
