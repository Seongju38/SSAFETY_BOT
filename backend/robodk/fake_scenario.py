import time
from pathlib import Path
from robodk_controller import RoboDKController

ROOT = Path(__file__).resolve().parents[2]  # project-root
RDK_PATH = str(ROOT / "simulation" / "AI_industry_site.rdk")

def main():
    # RoboDK가 열려있으면 rdk_path=None로 둬도 됨.
    ctrl = RoboDKController(rdk_path=None)

    # 가짜 이벤트: 쓰러짐 감지됨 -> CPR 키트 보내기
    fake_person_pose = [
        [ 1.000000,  0.000000, -0.984808, 1631.2 ],
        [ 0.000000,  1.000000,  0.000000, -1059.5 ],
        [ 0.984808,  0.000000,  1.000000,  520.0 ],
        [ 0.000000,  0.000000,  0.000000,    1.0 ]
    ]

    print("[FAKE] fall detected (pose)")
    time.sleep(1.0)

    print("[FAKE] user clicked: dispatch CPR box")
    res = ctrl.dispatch_emergency_box("CPR", person_pose=fake_person_pose, fallen=True)
    print("RESULT:", res)

if __name__ == "__main__":
    main()
