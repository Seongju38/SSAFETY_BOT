#!/usr/bin/env python3
import sys, json, time, rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl

class DobotExecutor(Node):
    def __init__(self):
        super().__init__('dobot_executor')
        self.group = ReentrantCallbackGroup()

        # 통합 행렬 로드 (상수 좌표 변환용)
        try:
            with open('homography_dual.json', 'r') as f:
                dual_h = json.load(f)
                self.H_robot = np.array(dual_h['H_robot'])
            self.get_logger().info("✅ H_robot Matrix loaded for constant coordinate correction")
        except Exception as e:
            self.get_logger().error(f"Failed to load dual H-matrix: {e}")
            sys.exit(1)

        self._action_client = ActionClient(self, PointToPoint, 'PTP_action', callback_group=self.group)
        self.suction_cli = self.create_client(SuctionCupControl, 'dobot_suction_cup_service', callback_group=self.group)
        self.sub = self.create_subscription(String, '/dobot_cmd_json', self.on_cmd, 10, callback_group=self.group)

        while not self.suction_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Suction service not available, waiting...')

        self.boxes = {}
        self.is_running = False
        self.get_logger().info('✅ Dobot Executor Ready (Dual-H & Constant Correction Applied)')

    def plate_to_robot(self, px, py):
        """판 기준 mm 좌표를 로봇 실측 mm 좌표로 변환"""
        p_plate = np.array([px, py, 1.0])
        p_robot = np.dot(self.H_robot, p_plate)
        rx, ry = p_robot[0]/p_robot[2], p_robot[1]/p_robot[2]
        return float(rx), float(ry)

    def on_cmd(self, msg):
        try:
            data = json.loads(msg.data)
            if data["type"] == "boxes":
                self.boxes.update(data["boxes"])
            elif data["type"] == "load" and not self.is_running:
                self.start_sequence()
        except Exception as e:
            self.get_logger().error(f"JSON Error: {e}")

    def start_sequence(self):
        red_box = self.boxes.get("red")
        if not red_box:
            self.get_logger().warn("❌ Red box coordinate not found!")
            return

        self.is_running = True
        # 1. 비전 인식 좌표 (이미 VisionBridge에서 로봇 좌표로 변환됨)
        rx_target, ry_target = float(red_box['x']), float(red_box['y'])
        rx_target+=10
        ry_target-=10
        # 2. 상수 좌표 변환 (Plate mm -> Robot Actual mm)
        p1_x, p1_y = self.plate_to_robot(180.0, 0.0)
        # p2_x, p2_y = self.plate_to_robot(240.0, 0.0)
        home_x, home_y = self.plate_to_robot(180.0, 0.0)

        self.get_logger().info(f"🚀 Job Start! Target: {rx_target:.2f}, {ry_target:.2f}")
        
        # [Task List] - 모든 좌표가 로봇 실측 체계로 보정됨
        self.tasks = [
            ["move", [rx_target, ry_target, 50.0, 0.0], 2],   # 박스 위 접근
            ["move", [rx_target, ry_target, -50.0, 0.0], 1],  # 하강 (Z -10)
            ["suction", True],
            ["move", [rx_target, ry_target, 50.0, 0.0], 1],   # 상승
            ["move", [p1_x, p1_y, 80.0, 0.0], 2],             # 지점 1 (교정된 240)
            ["move", [201, -28.7, 80.0, 0.0], 2],             # 지점 2 (교정된 300)
            ["move", [201, -28.7, 67.0, 0.0], 2],             # x: 0.2010498809814453
                                                              # y: -0.028708534240722658
                                                              # z: 0.0669716796875
            ["suction", False],
            ["move", [201, -28.7, 80.0, 0.0], 2], 
            ["move", [p1_x, p1_y, 80.0, 0.0], 2],
            ["move", [home_x, home_y, 0.0, 0.0], 2]         # 홈 복귀 (교정된 180)
        ]
        self.current_step = 0
        self.execute_next_task()

    def execute_next_task(self):
        if self.current_step >= len(self.tasks):
            self.get_logger().info("🏁 All Tasks Finished!")
            self.is_running = False
            return

        task_type, params, *m_type = self.tasks[self.current_step]
        if task_type == "move":
            self.send_move_goal(params, m_type[0] if m_type else 1)
        elif task_type == "suction":
            self.send_suction_request(params)

    def send_move_goal(self, target, m_type):
        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = [float(t) for t in target]
        goal_msg.motion_type = m_type
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal Rejected! 좌표 범위를 확인하세요.')
            self.is_running = False
            return
        goal_handle.get_result_async().add_done_callback(self.task_done_callback)

    def send_suction_request(self, state):
        req = SuctionCupControl.Request()
        req.enable_suction = state
        self.suction_cli.call_async(req).add_done_callback(self.task_done_callback)

    def task_done_callback(self, _future):
        self.current_step += 1
        self.execute_next_task()

def main():
    rclpy.init()
    node = DobotExecutor()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()