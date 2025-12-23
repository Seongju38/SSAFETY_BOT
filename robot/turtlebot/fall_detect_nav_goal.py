#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, PointStamped
from std_msgs.msg import Bool


def yaw_to_quat(yaw: float) -> Quaternion:
    """2D yaw(rad) -> geometry_msgs/Quaternion"""
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    return q


def dist2(x1, y1, x2, y2) -> float:
    return (x1 - x2) ** 2 + (y1 - y2) ** 2


class FallToGoalNavigator(Node):
    """
    - 시작 시 출발지 initialpose를 1회 publish (AMCL/Localization에서 사용)
    - fall 위치 토픽(/fall_pose or /fall_point) 수신 시 NavigateToPose goal 전송
    - 중복 goal 방지: 쿨다운 + 마지막 goal과의 최소거리 필터
    """

    def __init__(self):
        super().__init__("fall_to_goal_navigator")

        # ===== Parameters =====
        # start pose (출발지: map frame 기준)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_yaw", 0.0)  # rad

        # goal behavior
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("cooldown_sec", 5.0)       # 연속 탐지 스팸 방지
        self.declare_parameter("min_goal_dist", 0.4)      # 마지막 goal과 너무 가까우면 무시(m)
        self.declare_parameter("auto_send_on_start", True)

        # topics
        self.declare_parameter("fall_pose_topic", "/fall_pose")     # PoseStamped (권장)
        self.declare_parameter("fall_point_topic", "/fall_point")   # PointStamped (대안)
        self.declare_parameter("initialpose_topic", "/initialpose")

        self.start_x = float(self.get_parameter("start_x").value)
        self.start_y = float(self.get_parameter("start_y").value)
        self.start_yaw = float(self.get_parameter("start_yaw").value)

        self.goal_frame = str(self.get_parameter("goal_frame").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.min_goal_dist = float(self.get_parameter("min_goal_dist").value)
        self.auto_send_on_start = bool(self.get_parameter("auto_send_on_start").value)

        self.fall_pose_topic = str(self.get_parameter("fall_pose_topic").value)
        self.fall_point_topic = str(self.get_parameter("fall_point_topic").value)
        self.initialpose_topic = str(self.get_parameter("initialpose_topic").value)

        # ===== Publishers/Subscribers =====
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, self.initialpose_topic, 10)

        self.fall_pose_sub = self.create_subscription(
            PoseStamped, self.fall_pose_topic, self.on_fall_pose, 10
        )
        self.fall_point_sub = self.create_subscription(
            PointStamped, self.fall_point_topic, self.on_fall_point, 10
        )

        # (선택) 상태 표시용 토픽
        self.nav_active_pub = self.create_publisher(Bool, "/fall_nav_active", 10)

        # ===== Nav2 Action Client =====
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # ===== Internal state =====
        self.last_goal_time = 0.0
        self.last_goal_xy = None  # (x, y)
        self.nav_in_progress = False
        self.current_goal_handle = None

        # 시작 시 initial pose 설정
        self.timer_once = self.create_timer(1.0, self._on_startup_timer)

        self.get_logger().info("FallToGoalNavigator ready. Waiting fall detections...")

    def _on_startup_timer(self):
        # 1회만 실행
        self.timer_once.cancel()

        if self.auto_send_on_start:
            self.publish_initial_pose(self.start_x, self.start_y, self.start_yaw)
            self.get_logger().info(
                f"Published initial pose: x={self.start_x:.2f}, y={self.start_y:.2f}, yaw={self.start_yaw:.2f}rad"
            )

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.goal_frame  # 보통 "map"

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quat(float(yaw))

        # 너무 빡세게 정확 covariance 넣을 필요는 없지만, localization에서 참고함
        # (대충 작은 값)
        cov = [0.0] * 36
        cov[0] = 0.05   # x
        cov[7] = 0.05   # y
        cov[35] = 0.1   # yaw
        msg.pose.covariance = cov

        self.initialpose_pub.publish(msg)

    def on_fall_point(self, msg: PointStamped):
        # Point만 들어오면 yaw는 0으로 처리 (필요하면 추후 로직 추가)
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = msg.point.z
        pose.pose.orientation = yaw_to_quat(0.0)
        self._handle_fall_goal(pose)

    def on_fall_pose(self, msg: PoseStamped):
        self._handle_fall_goal(msg)

    def _handle_fall_goal(self, pose_msg: PoseStamped):
        # frame 체크(가능하면 map이어야 함)
        if pose_msg.header.frame_id == "":
            self.get_logger().warn("Fall pose has empty frame_id. Ignored.")
            return

        # goal_frame 강제(원하면 detector 쪽에서 map으로 변환해서 주는 게 제일 깔끔)
        # 여기서는 그냥 frame_id를 goal_frame으로 바꿔버리진 않고, 동일 프레임 가정.
        if pose_msg.header.frame_id != self.goal_frame:
            self.get_logger().warn(
                f"Fall pose frame_id='{pose_msg.header.frame_id}' != goal_frame='{self.goal_frame}'. "
                f"Detector should publish in '{self.goal_frame}'. Ignored for safety."
            )
            return

        now = time.time()

        # 쿨다운
        if (now - self.last_goal_time) < self.cooldown_sec:
            return

        gx = float(pose_msg.pose.position.x)
        gy = float(pose_msg.pose.position.y)

        # 마지막 goal과 너무 가까우면 무시
        if self.last_goal_xy is not None:
            if dist2(gx, gy, self.last_goal_xy[0], self.last_goal_xy[1]) < (self.min_goal_dist ** 2):
                return

        # 진행 중이면 새 goal 무시(원하면 cancel 후 재전송으로 바꿀 수 있음)
        if self.nav_in_progress:
            self.get_logger().info("Navigation already in progress. Ignoring new fall goal.")
            return

        self.last_goal_time = now
        self.last_goal_xy = (gx, gy)

        self.send_nav_goal(pose_msg)

    def send_nav_goal(self, goal_pose: PoseStamped):
        self.get_logger().info(
            f"Sending Nav2 goal -> x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}"
        )

        self.nav_active_pub.publish(Bool(data=True))
        self.nav_in_progress = True

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Nav2 action server (/navigate_to_pose) not available!")
            self.nav_active_pub.publish(Bool(data=False))
            self.nav_in_progress = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        send_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.on_feedback)
        send_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected.")
            self.nav_active_pub.publish(Bool(data=False))
            self.nav_in_progress = False
            return

        self.get_logger().info("Nav2 goal accepted.")
        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        # 너무 시끄러우면 주석 처리
        fb = feedback_msg.feedback
        # fb.current_pose, fb.distance_remaining 등
        if hasattr(fb, "distance_remaining"):
            self.get_logger().debug(f"Distance remaining: {fb.distance_remaining:.2f}m")

    def on_result(self, future):
        result = future.result().result
        status = future.result().status  # GoalStatus

        # status 값은 action_msgs/msg/GoalStatus 참고
        if status == 4:  # SUCCEEDED
            self.get_logger().info("Navigation SUCCEEDED (arrived at fall location).")
        else:
            self.get_logger().warn(f"Navigation finished with status={status}.")

        self.nav_active_pub.publish(Bool(data=False))
        self.nav_in_progress = False
        self.current_goal_handle = None


def main():
    rclpy.init()
    node = FallToGoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
