import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# -----------------------
# 1) RealSense 설정
# -----------------------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# -----------------------
# 2) YOLO Pose 모델 로드
# -----------------------
model = YOLO("yolov8n-pose.pt")  # pretrained pose model

print("Press Q to quit")

try:
    while True:
        # RealSense frame 받기
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())

        # YOLO Pose 추론
        results = model(frame, stream=True)

        for r in results:
            annotated = r.plot()  # skeleton + bbox 그린 frame

        cv2.imshow("YOLO-Pose RealSense", annotated)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
