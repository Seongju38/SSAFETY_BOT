# 🛡️ SSAFETY BOT

### AI 기반 산업현장 응급처치 물품 운송 시스템

> **RealSense × AI Fall Detection × Dobot × TurtleBot3 Autonomous Navigation**  
> 사람이 없는 현장에서, 로봇이 대신 응급 물품을 전달합니다.

## 📌 프로젝트 개요 (Overview)

**SSAFETY BOT**은 자동화 산업현장·무인 작업장·실버타운 등에서 **쓰러짐 감지 → 관리자 판단 → 물품 자동 운송 → 환자 도착**까지 수행하는 AI·로봇 융합 기반 응급 대응 시스템입니다.

구현 기술:

- RealSense D435i 기반 실시간 사람 감지
- YOLO-Pose / ST-GCN 기반 쓰러짐(Fall) 판단
- Dobot Magician Pick & Place 자동 물품 적재
- TurtleBot3 자율주행을 통한 환자 위치 운송

## 🎯 문제 정의 (Problem Statement)

자동화 산업현장에서는 다음 문제 상황이 발생할 가능성이 높습니다:

- 작업 도중 **탈진** 또는 **실신**
- 기계 장비와 **충돌**
- 고온 장비로 인한 **화상**
- **기저질환**으로 인한 쓰러짐
- 무인·야간 작업 시 주변에 **도움 요청자가 없음**

이를 해결하기 위해  
**AI 기반 감지 + 로봇 자동 운송 시스템**이 필요합니다.

## 🧠 시스템 아키텍처 (System Architecture)

```
[RealSense D435i]
        ↓
[AI Fall Detection – YOLO-Pose / ST-GCN]
        ↓
[서버 알림 → 관리자 화면]
        ↓
[관리자 응급 물품 선택]
        ↓
[Dobot Pick & Place]
        ↓
[TurtleBot3 자율주행 이동]
        ↓
[환자에게 응급 물품 전달]
```

## 🔍 기능 상세 (Features)

### 1. 쓰러짐 감지 (AI Fall Detection)

- YOLOv8-Pose로 Skeleton Keypoint 실시간 추출
- ST-GCN(pretrained)으로 포즈 시퀀스 분석
- 응급도 판단 결과:

| 상태 | 색상 | 기준               |
| ---- | ---- | ------------------ |
| 정상 | 🟩   | 서 있음·이동       |
| 주의 | 🟨   | 비정상 자세        |
| 위급 | 🟥   | 쓰러짐·움직임 없음 |

> 간편 버전: 바운딩박스 + 자세 기울기 판단 방식 가능

---

### 2. 관리자 알림 & 상황 판단

- 응급 상태 감지 시 서버로 경고 전송
- 관리자 Dashboard에서 실시간 상태 확인
- 응급 유형 선택:
  - 탈진
  - 충돌 사고
  - 화상
  - 기저질환 의심

---

### 3. Dobot Magician – 응급 물품 Pick & Place

인식 가능한 물품:

- 부목
- 붕대
- 물
- 수액팩
- 소독액
- 반창고

기능:

- YOLO 기반 물품 인식
- 미리 세팅된 좌표로 Pick & Place 수행
- TurtleBot 적재 위치까지 이동해 전달

---

### 4. TurtleBot3 – 자율주행

- Navigation2 기반 Path Planning
- 미리 구축된 맵에서 환자 위치로 이동
- Lidar + Depth 기반 장애물 회피
- 도착 시 관리자에게 알림

## 🛠 기술 스택 (Tech Stack)

| 분야         | 기술                               |
| ------------ | ---------------------------------- |
| AI Detection | YOLOv8-Pose, ST-GCN                |
| Camera       | Intel RealSense D435i              |
| Robot Arm    | Dobot Magician                     |
| Mobile Robot | TurtleBot3 Waffle Pi (ROS2 Humble) |
| Navigation   | Nav2, SLAM, AMCL                   |
| Backend      | Python FastAPI or Flask            |
| Frontend     | Web Dashboard                      |
| 기타         | OpenCV, PyTorch, pyrealsense2      |

---

## 🗂️ 프로젝트 구조 (Repository Structure)

```
SSAFETY-BOT/
├── vision/ # RealSense + Fall Detection
├── server/ # 관리자 알림 서버
├── ui/ # Dashboard Web UI
├── dobot/ # Dobot 제어 코드
├── turtlebot/ # Nav2 자율주행 코드
├── configs/ # YOLO / ST-GCN 모델 설정
├── README.md
```

## 📽️ 데모 영상

(추후 업로드 예정)

## 🏁 기대 효과 (Expected Benefits)

- 응급상황 발생 시 초기 대응 시간을 크게 단축
- 무인 자동화 산업 현장의 안전성 상승
- 실버타운·요양시설·물류센터 등 다양한 환경에 적용 가능
- AI × Robot 융합 프로젝트로 높은 실용성과 기술적 임팩트 제공

## 👥 팀 구성 (Team)

- 팀원 1: 박세진
- 팀원 2: 이성주
