#!/usr/bin/env python3
import os, time, json, threading, socket, signal, subprocess
import cv2
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl


HOST, PORT = "0.0.0.0", 20000
MAP_YAML = "/home/ssafy/SSAFETY_BOT/robot/turtlebot/project_final_map.yaml"

def yaw_to_quat(yaw: float):
    import math
    return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))

class IntegratedController(Node):
    def __init__(self):
        super().__init__("integrated_controller")
        self.group = ReentrantCallbackGroup()

        # ---- publishers/subscribers ----
        self.cmd_pub = self.create_publisher(String, "/dobot_cmd_json", 10)
        self.cmd_sub = self.create_subscription(String, "/dobot_cmd_json", self.on_cmd, 10, callback_group=self.group)

        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # ---- Dobot ----
        self.ptp_client = ActionClient(self, PointToPoint, "PTP_action", callback_group=self.group)
        self.suction_cli = self.create_client(SuctionCupControl, "dobot_suction_cup_service", callback_group=self.group)

        # ---- 상태 ----
        self.boxes = {}
        self.is_dobot_running = False
        self.nav2_proc = None

        # ---- Homography ----
        self.H_plate = None
        self.H_robot = None
        self.load_homography("/home/ssafy/homography_dual.json")  # 경로는 네 환경에 맞게

        # ---- RealSense ----
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(cfg)

        # ---- background threads ----
        threading.Thread(target=self.vision_loop, daemon=True).start()
        threading.Thread(target=self.run_tcp_server, daemon=True).start()

        # ---- Nav2 auto launch ----
        self.start_nav2()

        self.get_logger().info("✅ Integrated Controller Ready")

    # ----------------- Nav2 -----------------
    def start_nav2(self):
        os.environ.setdefault("TURTLEBOT3_MODEL", "waffle_pi")
        os.environ.setdefault("ROS_DOMAIN_ID", "36")

        cmd = ["ros2", "launch", "turtlebot3_navigation2", "navigation2.launch.py", f"map:={MAP_YAML}"]
        self.nav2_proc = subprocess.Popen(cmd)
        self.get_logger().info("🚀 Nav2 launched")
        time.sleep(8)  # 필요하면 토픽/서비스로 준비 확인하도록 개선 가능

    def publish_initialpose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx,qy,qz,qw = yaw_to_quat(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.init_pub.publish(msg)

    def publish_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        qx,qy,qz,qw = yaw_to_quat(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.goal_pub.publish(msg)
        self.get_logger().info(f"🎯 Goal published: {x:.2f}, {y:.2f}")

    # ----------------- Homography -----------------
    def load_homography(self, path):
        try:
            with open(path, "r") as f:
                dual = json.load(f)
            self.H_plate = np.array(dual["H_plate"])
            self.H_robot = np.array(dual["H_robot"])
            self.get_logger().info("✅ Homography loaded")
        except Exception as e:
            self.get_logger().error(f"Homography load failed: {e}")
            raise

    def transform_dual(self, u, v):
        p = np.array([u, v, 1.0])
        pp = self.H_plate @ p
        px, py = pp[0]/pp[2], pp[1]/pp[2]
        pr = self.H_robot @ np.array([px, py, 1.0])
        rx, ry = pr[0]/pr[2], pr[1]/pr[2]
        return (float(px), float(py)), (float(rx), float(ry))

    # ----------------- Vision -----------------
    def vision_loop(self):
        while rclpy.ok():
            frames = self.pipeline.wait_for_frames()
            cf = frames.get_color_frame()
            if not cf:
                continue
            img = np.asanyarray(cf.get_data())
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            m1 = cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255]))
            m2 = cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255]))
            mask = m1 + m2

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 1000:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        u = int(M["m10"]/M["m00"]); v = int(M["m01"]/M["m00"])
                        (px,py),(rx,ry) = self.transform_dual(u,v)

                        # boxes 업데이트 publish
                        self.publish_json({"type":"boxes","boxes":{"red":{"x":rx,"y":ry}}})

                        cv2.circle(img,(u,v),5,(0,255,0),-1)
                        cv2.putText(img,f"Robot: {rx:.1f},{ry:.1f}",(u,v-10),
                                    cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)

            cv2.imshow("Detection", img)
            if cv2.waitKey(1) == ord("q"):
                break

        self.pipeline.stop()

    def publish_json(self, obj):
        msg = String()
        msg.data = json.dumps(obj, ensure_ascii=False)
        self.cmd_pub.publish(msg)

    # ----------------- TCP server -----------------
    def run_tcp_server(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(5)
        self.get_logger().info(f"TCP server listening {HOST}:{PORT}")

        while rclpy.ok():
            conn, _ = s.accept()
            try:
                data = conn.recv(4096)
                if data:
                    obj = json.loads(data.decode())
                    # 예: exhaustion 이벤트면 Dobot+Turtlebot 트리거
                    if obj.get("type") == "exhaustion":
                        self.publish_json({"type":"load","condition":"탈진"})
                    else:
                        self.publish_json(obj)
                    conn.sendall(b'{"ok":true}\n')
            except Exception:
                pass
            finally:
                conn.close()

    # ----------------- Dobot logic -----------------
    def on_cmd(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get("type") == "boxes":
                self.boxes.update(data["boxes"])
            elif data.get("type") == "load" and not self.is_dobot_running:
                # 여기서 "탈진" 감지 시: Dobot 작업 + Turtlebot 이동을 동시에 트리거 가능
                self.start_dobot_sequence()
                # 예시: 고정 목적지로 터틀봇 이동
                # (낙상 위치를 map좌표로 만들면 여기 goal에 넣으면 됨)
                self.publish_goal(1.0, 0.0, 0.0)
        except Exception as e:
            self.get_logger().error(f"cmd parse error: {e}")

    def start_dobot_sequence(self):
        red = self.boxes.get("red")
        if not red:
            self.get_logger().warn("No red box yet")
            return
        self.is_dobot_running = True

        rx, ry = float(red["x"]), float(red["y"])
        self.get_logger().info(f"🚀 Dobot start target {rx:.1f}, {ry:.1f}")

        # TODO: 여기엔 네 tasks 리스트를 그대로 넣으면 됨.
        # 지금은 “시작했다” 구조만 보여주는 골격.
        self.is_dobot_running = False


def main():
    os.environ.setdefault("ROS_DOMAIN_ID", "36")

    rclpy.init()
    node = IntegratedController()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        # nav2 종료
        if node.nav2_proc:
            node.nav2_proc.send_signal(signal.SIGINT)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
