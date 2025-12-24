import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np

def clamp_hsv(h, s, v):
    return (max(0, min(179, h)), max(0, min(255, s)), max(0, min(255, v)))

class EmergencyColorTargetSelector(Node):
    def __init__(self):
        super().__init__('emergency_color_target_selector')

        self.bridge = CvBridge()

        # 선택된 상황(웹 입력)
        self.selected_item = "출혈"  # 기본값(테스트용)

        self.sub_img = self.create_subscription(
            Image, 'camera/camera/color/image_raw', self.on_image, 10
        )
        self.sub_sel = self.create_subscription(
            String, '/selected_item', self.on_selected, 10
        )

        self.pub_debug = self.create_publisher(Image, '/target_debug_image', 10)

        # HSV 임계값(초기값: 현장 조명에 따라 튜닝 필요)
        # 빨강은 2구간(0~10) + (170~179)
        self.hsv_ranges = {
            "탈진": [  # 빨강
                ((0, 120, 70), (10, 255, 255)),
                ((170, 120, 70), (179, 255, 255)),
            ],
            "충돌/골절": [  # 노랑
                ((20, 120, 70), (35, 255, 255)),
            ],
            "출혈": [  # 연두(그린-옐로우)
                ((35, 80, 70), (85, 255, 255)),
            ],
            "기저질환 의심": [  # 파랑
                ((90, 120, 70), (130, 255, 255)),
            ],
        }

        # 작은 노이즈 제거를 위한 최소 면적(px^2)
        self.min_area = 800  # 라벨 크기에 맞게 조정

        self.get_logger().info("EmergencyColorTargetSelector started.")
        self.get_logger().info("Publish /selected_item (std_msgs/String) to switch target.")
        self.get_logger().info("Debug image: /target_debug_image")

    def on_selected(self, msg: String):
        if msg.data in self.hsv_ranges:
            self.selected_item = msg.data
            self.get_logger().info(f"Selected item set to: {self.selected_item}")
        else:
            self.get_logger().warn(f"Unknown selected_item: {msg.data}. Keep: {self.selected_item}")

    def build_mask(self, hsv_img, ranges):
        mask_total = np.zeros(hsv_img.shape[:2], dtype=np.uint8)
        for (lo, hi) in ranges:
            lo = clamp_hsv(*lo)
            hi = clamp_hsv(*hi)
            mask = cv2.inRange(hsv_img, np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8))
            mask_total = cv2.bitwise_or(mask_total, mask)
        return mask_total

    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        ranges = self.hsv_ranges.get(self.selected_item, None)
        if ranges is None:
            return

        mask = self.build_mask(hsv, ranges)

        # morphology로 노이즈 감소
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # 컨투어 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        target = None
        best_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
            if area > best_area:
                best_area = area
                target = cnt

        debug = bgr.copy()

        if target is not None:
            x, y, w, h = cv2.boundingRect(target)
            cx = int(x + w / 2)
            cy = int(y + h / 2)

            cv2.rectangle(debug, (x, y), (x + w, y + h), (255, 255, 255), 2)
            cv2.circle(debug, (cx, cy), 6, (255, 255, 255), -1)
            cv2.putText(
                debug,
                f"{self.selected_item} area={int(best_area)} center=({cx},{cy})",
                (x, max(0, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            self.get_logger().info(f"[TARGET] {self.selected_item} center=({cx},{cy}) area={int(best_area)}")
        else:
            cv2.putText(
                debug,
                f"{self.selected_item}: target not found",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2
            )

        # 디버그 이미지 publish
        out_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        out_msg.header = msg.header
        self.pub_debug.publish(out_msg)

def main():
    rclpy.init()
    node = EmergencyColorTargetSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
