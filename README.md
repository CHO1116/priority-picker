# 📦 Priority Picker

주문 우선순위 기반 물류 피킹 로봇 셀. Doosan E0509, RH-P12-RN-A 그리퍼, Intel RealSense, ROS 2를 사용한다.

뒤섞인 상품(색상 큐브)을 3D 비전으로 인식한 뒤 **긴급 주문 → 집기 쉬운 것 → 가까운 것** 순서로 출고 박스에 담는다.

- 프로젝트 개요·시나리오: [docs/README.md](docs/README.md)
- 데이터 흐름도: [docs/데이터흐름도.md](docs/데이터흐름도.md)
- 기능 명세서: [docs/기능명세서.md](docs/기능명세서.md)
- 일정·역할: [docs/일정_역할분담.md](docs/일정_역할분담.md)

## 패키지 구조

| 패키지 | 담당 | 내용 |
|---|---|---|
| `picker_interfaces` | 조영준 | 커스텀 메시지(`DetectedCube`, `Order`)와 액션(`PickPlace`) |
| `picker_vision` | 이준희 | `vision_node`(실제 카메라), `fake_world`(가짜 작업대), `calibrate`(캘리브레이션) |
| `picker_planner` | 조영준 | `order_manager`, `task_planner`, 우선순위 로직 `priority.py` |
| `picker_control` | 류창표 | `robot_controller`(`/pick_place` 액션 서버), 로봇·그리퍼 추상화 `motion.py` |
| `picker_bringup` | 조영준 | 런치 파일과 `config/*.yaml`(자세, 비전, 캘리브레이션, 주문) |

## 개발 환경

- Ubuntu 24.04, ROS 2 Jazzy, Python 3.12
- `doosan-robot2` (Jazzy 지원 브랜치), `realsense2_camera` (4.x, 토픽이 `/camera/camera/...` 형태)
- 의존 패키지 설치:

```bash
sudo apt install ros-jazzy-cv-bridge ros-jazzy-message-filters ros-jazzy-realsense2-camera python3-opencv python3-yaml python3-pytest
```

## 빌드

`--symlink-install`을 쓰면 config YAML을 고친 뒤 다시 빌드하지 않아도 된다.

```bash
cd ~/robo_ws && colcon build --symlink-install && source install/setup.bash
```

## 실행

### 1) 시뮬레이션 (로봇·카메라 없이 전체 흐름 확인)

```bash
ros2 launch picker_bringup picker.launch.py mode:=sim
```

전략 비교 실험을 하려면 strategy를 바꿔 실행한다. 결과는 `results/summary.csv`에 누적된다.

```bash
ros2 launch picker_bringup picker.launch.py mode:=sim strategy:=random seed:=1
```

### 2) 실제 로봇

터미널 1: RealSense (컬러와 깊이 정렬)

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

터미널 2: 두산 로봇 드라이버. 실습실에서 쓰는 명령과 IP를 확인한다.

```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=<로봇IP> model:=e0509
```

터미널 3: Priority Picker

```bash
ros2 launch picker_bringup picker.launch.py mode:=real gripper_mode:=drl
```

### 3) 단계별 단독 테스트

비전 결과 확인 (rqt에서 `/vision/debug_image` 선택):

```bash
ros2 run rqt_image_view rqt_image_view
```

카메라 좌표 수집 (캘리브레이션용):

```bash
ros2 run picker_vision vision_node --ros-args -p vision_config:=src/picker_bringup/config/vision.yaml -p calibration_file:=src/picker_bringup/config/calibration.yaml -p log_camera_coords:=true
```

캘리브레이션 계산:

```bash
ros2 run picker_vision calibrate --input points.csv --output src/picker_bringup/config/calibration.yaml
```

### 4) 단위 테스트 (ROS 없이 실행 가능)

```bash
python3 -m pytest src/picker_planner/test src/picker_vision/test
```

## ⚠️ 실제 장비에서 반드시 확인할 것 (TODO)

- [ ] `motion.py`의 `DrlGripper`: 실습실 그리퍼 예제로 열기, 닫기, 벌림 폭 읽기 구현 (류창표)
- [ ] `motion.py`의 `DsrRobot`: 설치된 doosan-robot2의 서비스 이름과 필드 확인
  - `ros2 interface show dsr_msgs2/srv/MoveLine`
- [ ] `poses.yaml`: 스캔 자세, 박스 위치, 그리퍼가 아래를 보는 자세 `[rx, ry, rz]`, 작업 영역을 티칭으로 측정
- [ ] 45° 돌아간 큐브로 yaw 회전 방향(부호) 확인
- [ ] `vision.yaml`: RealSense 토픽 이름, ROI, HSV 범위를 현장 조명에 맞게 조정
- [ ] `calibration.yaml`: Day 3 캘리브레이션 결과로 교체
- [ ] 시연 속도 30% 이하로 설정하고 비상정지 담당자 지정
