from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple, Dict

from robodk import robolink
from robodk import robomath

import time

@dataclass
class DiseasePlan:
    box_name: str
    pick_target: str


class RoboDKController:
    """
    - RoboDK 실행 중이면 붙어서 제어
    - 필요하면 rdk_path로 .rdk를 열 수도 있음
    - 아이템 이름은 .rdk 트리 이름과 반드시 일치해야 함
    """

    def __init__(self, rdk_path: Optional[str] = None):
        self.RDK = robolink.Robolink()

        # (선택) rdk 파일을 자동으로 열기
        if rdk_path:
            self.RDK.AddFile(rdk_path)

        # ====== TODO: rdk 트리 이름으로 맞추기 ======
        self.robot_name = "RB16-900E"
        self.tool_name = "Tool 6"
        self.tb_name = "turtlebot3_waffle"
        self.person_name = "person"
        self.place_on_tb_target = "T_PLACE_ON_TB"

        # 질병 키 -> (박스, 픽 타겟)
        self.plans: Dict[str, DiseasePlan] = {
            "CPR": DiseasePlan("box_red", "T_PICK_BOX_RED"),
            "BURN": DiseasePlan("box_yellow", "T_PICK_BOX_YELLOW"),
            "BLEEDING": DiseasePlan("box_green", "T_PICK_BOX_GREEN"),
        }

        # Items
        self.robot = self._req(self.robot_name, robolink.ITEM_TYPE_ROBOT)
        self.tool = self._req(self.tool_name, robolink.ITEM_TYPE_TOOL)
        self.tb = self._req(self.tb_name, robolink.ITEM_TYPE_OBJECT)
        self.person = self._req(self.person_name, robolink.ITEM_TYPE_OBJECT)
        self.t_place_on_tb = self._req(self.place_on_tb_target, robolink.ITEM_TYPE_TARGET)

    def _req(self, name: str, item_type: int):
        it = self.RDK.Item(name, item_type)
        if not it.Valid():
            raise RuntimeError(f"[RoboDK] Item not found/invalid: name='{name}', type={item_type}")
        return it

    def _movej_target(self, target_name: str):
        tgt = self._req(target_name, robolink.ITEM_TYPE_TARGET)
        self.robot.MoveJ(tgt)

    def _set_person_pose_mat(self, pose_mat):
        """
        pose_mat: 4x4 homogeneous matrix (list or robomath.Mat)
        """
        if isinstance(pose_mat, robomath.Mat):
            pose = pose_mat
        else:
            pose = robomath.Mat(pose_mat)

        self.person.setPose(pose)

    def _set_person_fallen(self, fallen: bool):
        # pose = self.person.Pose()
        pose0 = self.person.Pose()
        x, y, z = pose0.Pos()
        if fallen:
            new_pose = robomath.transl(x, y, z) * robomath.rotx(-90 * robomath.pi / 180.0)
            
            self.person.setPose(new_pose)
        else:
            self.person.setPose(pose0)

    def _attach_to(self, child_item, parent_item):
        # "집기/적재"의 핵심: parent-child 관계를 바꿔서 붙임
        child_item.setParentStatic(parent_item)

    def move_turtlebot_near_person(self, offset_xyz=(0, 900, 0)):
        # 최종 목적지 (사람 근처)
        px, py, pz = self.person.Pose().Pos()
        ox, oy, oz = offset_xyz
        final_xyz = (px + ox, py + oy, pz + oz)

        WAYS = ["WAY1_F", "WAY2_F", "WAY3_F", "WAY4_F", "WAY5_F"]

        # 문 순서대로 통과
        for door_name in WAYS:
            door = self.RDK.Item(door_name, robolink.ITEM_TYPE_FRAME)
            dx, dy, dz = door.Pose().Pos()
            self.move_turtlebot_smooth((dx, dy, dz))

        # 마지막 목적지
        self.move_turtlebot_smooth(final_xyz)

    def move_turtlebot_smooth(
        self,
        target_xyz: Tuple[float, float, float],
        steps: int = 50,
        dt: float = 0.05,
    ):
        """
        TurtleBot을 현재 위치 → target_xyz까지 부드럽게 이동
        """
        start_pose = self.tb.Pose()
        sx, sy, sz = start_pose.Pos()
        tx, ty, tz = target_xyz

        for i in range(1, steps + 1):
            a = i / steps
            x = sx + a * (tx - sx)
            y = sy + a * (ty - sy)
            z = sz + a * (tz - sz)

            self.tb.setPose(robomath.transl(x, y, z))
            time.sleep(dt)

    def move_turtlebot_via_door(
        self,
        door_xyz,
        target_xyz,
    ):
        # 1. 문 앞으로 이동
        self.move_turtlebot_smooth(door_xyz)

        # 2. 문 통과 (조금 더 안쪽)
        dx, dy, dz = door_xyz
        door_pass_xyz = (dx + 0.6, dy, dz)  # 문 두께만큼
        self.move_turtlebot_smooth(door_pass_xyz)

        # 3. 최종 목적지
        self.move_turtlebot_smooth(target_xyz)

    def dispatch_emergency_box(
        self,
        disease_key: str,
        person_pose=None, 
        fallen: bool = True,
    ):
        if disease_key not in self.plans:
            raise ValueError(f"Unknown disease_key='{disease_key}'. available={list(self.plans.keys())}")

        plan = self.plans[disease_key]
        box = self._req(plan.box_name, robolink.ITEM_TYPE_OBJECT)

        # (가짜 데이터) 사람 위치/쓰러짐 상태
        if person_pose:
            self._set_person_pose_mat(person_pose)
        self._set_person_fallen(fallen)

        # 1) 박스 픽
        self._movej_target(plan.pick_target)
        time.sleep(0.5)
        self._attach_to(box, self.tool)          # 로봇이 박스 집기
        time.sleep(0.5)

        # 2) 터틀봇 적재
        self.robot.MoveJ(self.t_place_on_tb)
        time.sleep(0.5)
        self._attach_to(box, self.tb)            # 터틀봇에 박스 적재
        time.sleep(0.5)

        # 3) 터틀봇 이동(시뮬)
        self.move_turtlebot_near_person()

        return {"ok": True, "disease": disease_key, "box": plan.box_name}