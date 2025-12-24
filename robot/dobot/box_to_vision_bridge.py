#!/usr/bin/env python3
import socket, json, threading, cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HOST = "0.0.0.0"
PORT = 20000

class VisionBridgeNode(Node):
    def __init__(self):
        super().__init__("vision_bridge_node")
        self.pub = self.create_publisher(String, "/dobot_cmd_json", 10)
        
        # 1. 이중 호모그래피 행렬 로드 (Pixel -> Plate, Plate -> Robot)
        try:
            with open('homography_dual.json', 'r') as f:
                dual_h = json.load(f)
                self.H_plate = np.array(dual_h['H_plate'])
                self.H_robot = np.array(dual_h['H_robot'])
            self.get_logger().info("Dual Homography matrices loaded ✅")
        except Exception as e:
            self.get_logger().error(f"Failed to load dual H-matrix: {e}")
            exit()

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        threading.Thread(target=self.vision_loop, daemon=True).start()

    def transform_dual(self, u, v):
        """픽셀 -> 판 좌표 -> 로봇 실측 좌표 이중 변환"""
        p_pixel = np.array([u, v, 1.0])
        # 1단계: Pixel -> Plate mm
        p_plate = np.dot(self.H_plate, p_pixel)
        px, py = p_plate[0]/p_plate[2], p_plate[1]/p_plate[2]

        # 2단계: Plate mm -> Robot Actual mm
        p_plate_mm = np.array([px, py, 1.0])
        p_robot = np.dot(self.H_robot, p_plate_mm)
        rx, ry = p_robot[0]/p_robot[2], p_robot[1]/p_robot[2]
        
        return (float(px), float(py)), (float(rx), float(ry))

    def vision_loop(self):
        try:
            while rclpy.ok():
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame: continue

                img = np.asanyarray(color_frame.get_data())
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                m1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
                m2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
                mask = m1 + m2

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    c = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(c) > 1000:
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                            (px, py), (rx, ry) = self.transform_dual(u, v)
                            
                            self.publish_json({
                                "type": "boxes",
                                "boxes": { "red": {"x": rx, "y": ry} }
                            })

                            cv2.circle(img, (u, v), 5, (0, 255, 0), -1)
                            cv2.putText(img, f"Plate: {px:.1f}, {py:.1f}", (u, v-30), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                            cv2.putText(img, f"Robot: {rx:.1f}, {ry:.1f}", (u, v-10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                cv2.imshow("Detection (Dual-H Applied)", img)
                if cv2.waitKey(1) == ord('q'): break
        finally:
            self.pipeline.stop()

    def publish_json(self, obj: dict):
        msg = String()
        msg.data = json.dumps(obj, ensure_ascii=False)
        self.pub.publish(msg)

def run_tcp_server(node):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(5)
    while rclpy.ok():
        conn, _ = s.accept()
        try:
            data = conn.recv(4096)
            if data:
                obj = json.loads(data.decode())
                if obj.get("type") == "exhaustion":
                    node.publish_json({"type": "load", "condition": "탈진"})
                else:
                    node.publish_json(obj)
                conn.sendall(b'{"ok":true}\n')
        except: pass
        finally: conn.close()

def main():
    rclpy.init()
    node = VisionBridgeNode()
    threading.Thread(target=run_tcp_server, args=(node,), daemon=True).start()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()