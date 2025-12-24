import json
import numpy as np
import cv2
import pyrealsense2 as rs
from pathlib import Path

# 설정 파일 경로
LOAD_PATH = Path("homography_xy.json")
SAVE_PATH = Path("homography_dual.json")

# 사용자가 제공한 데이터 (판 좌표 -> 로봇 TCP 실측 mm)
# 왼쪽: 판(Plate) 좌표 (mm), 오른쪽: 로봇 TCP 실측 (mm)
CALIB_DATA = [
    ([180.0, 0.0],    [123.2, -3.1]),
    ([180.0, 60.0],   [126.1, 3.9]),
    ([300.0, 0.0],    [241.2, -6.0]),
    ([300.0, 40.0],   [244.1, 27.8]),
    ([300.0, -120.0], [244.9, -102.6]),
    ([180.0, -120.0], [132.9, -90.1]),
    ([60.0, -200.0], [43.1, -143.5]),
    ([100.0, -200.0], [73.1, -148.8]),
]

def main():
    # 1. 기존 판(Plate) 행렬 로드
    with open(LOAD_PATH, 'r') as f:
        h_plate_data = json.load(f)
        H_plate = np.array(h_plate_data['H'])

    # 2. 판 좌표 -> 로봇 TCP 좌표 행렬(H_robot) 계산
    # 이 행렬은 판 mm를 넣으면 로봇 실측 mm를 뱉습니다.
    src_pts = np.array([d[0] for d in CALIB_DATA], dtype=np.float64)
    dst_pts = np.array([d[1] for d in CALIB_DATA], dtype=np.float64)
    H_robot, _ = cv2.findHomography(src_pts, dst_pts)

    # 3. 통합 JSON 저장
    dual_data = {
        "H_plate": H_plate.tolist(), # 픽셀 -> 판 (시각화용)
        "H_robot": H_robot.tolist()  # 판 -> 로봇 (실제 이동용)
    }
    with open(SAVE_PATH, 'w') as f:
        json.dump(dual_data, f, indent=2)
    
    print(f"✅ 통합 행렬 저장 완료: {SAVE_PATH}")
    print("H_plate: 픽셀 -> 판 mm")
    print("H_robot: 판 mm -> 로봇 실측 mm")

if __name__ == "__main__":
    main()