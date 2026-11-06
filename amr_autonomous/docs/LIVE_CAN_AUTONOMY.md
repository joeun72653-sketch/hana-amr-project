# 실제 CANopen 저속 AUTO 주행 — 설치/실행 절차

**상태:** GitHub 코드 연결 완료. IPC 및 실물 주행 결과는 아직 확인되지 않았습니다.
**대상:** Ubuntu 24.04, ROS2 Jazzy, domain 31, repo `feature/amr-qr-autonomous-prep-20261105`.

## 1. 실제 연결

휴대폰 `http://<IPC_IP>:8000/` 지도 → AUTO 선택 → 30RPM 설정 → 목적지 클릭 → AUTO 시작 → `/navigate_to_pose` → `/mapex/cmd_vel` (`TwistStamped`) → `real_mapex_cmd_vel_to_rpm` → `/mapex/wheel_rpm_{left,right}_requested` → `auto_rpm_limiter.py` → `/amr_auto/wheel_rpm_{left,right}_limited` → `guarded_motor_mux.py` → **독점 토픽** `/amr_mux/wheel_rpm_{left,right}` + `/amr_mux/motion_authorized` → `amr_ros_wheel_rpm_can.py --mux-input` → USB FT232R CANopen Node 1/2.

- 기존 `/wheel_rpm_left/right` 직결 수동 Publisher는 전용 CAN 노드에서 구독하지 않습니다.
- AUTO 시작은 모터·Depth·Mux 상태가 최신이며, 라이브 `map→base_link` TF, 출발점/계획 일치, 실제 정지가 확인된 경우에만 허용됩니다.
- `--live-motor-output`와 `--allow-physical-auto`는 실구동 Launch에서만 지정합니다. `run_all.sh`는 계속 미리보기 전용입니다.
- 기본 사용자 AUTO 상한은 30 RPM, UI에서 30~1000 RPM(10 단위). 이 범위와 별도의 하위 소프트웨어 상한은 설정하지 않았습니다. CAN 드라이버는 양 바퀴 각각 절대 1000 RPM을 넘는 입력을 래치 차단합니다.
- 모터축 RPM ↔ 차체 속도는 바퀴 반경 0.075m, 트랙 0.615m, 기어비 15.0 기준입니다. 드라이브 실제 0x60FF 단위, 회전 부호, 인코더, 제동거리 실측 후 범위 확대가 필요합니다.

## 2. 시작 전 안전 조건

필수: 독립 하드웨어 E-Stop 실제 작동, CAN 통신 단절 드라이브 측 정지, 양 바퀴 정지 피드백, Depth 경고 실시간, LiDAR 장애물/Costmap, 리프트 정지 인터락, AMCL/ICP 최신 TF, 빈 시험구역, 근접 감시자. 실제 운전 시 영상만 보고 접근하지 않습니다.

실구동 프로그램 `safe_start()`는 Node 1·2의 드라이브를 Operation Enabled 상태로 만들 수 있습니다. STOP/Ctrl+C만을 기계적 비상정지로 간주하지 않습니다. **통신이 끊겼을 때는 독립적인 하드웨어 E-Stop을 즉시 사용합니다.**

## 3. 설치

현재 개별 실행한 LiDAR, ICP, Map Server, AMCL, Nav2, RViz, 웹 8000/8771, mode dry run, 기존 CAN/Web 브리지를 각각 Ctrl+C로 종료하고, 새 터미널에서:

```bash
cd ~/hana-amr-project-autonomous
git status --short
git pull --ff-only
bash ~/hana-amr-project-autonomous/amr_autonomous/run_live.sh
```

터미널의 실제 모터 활성화 확인 질문에서 `START`를 입력해야 CAN 드라이버가 시작됩니다. 인증 토큰을 요청하는 것은 아니며, 로컬 운전자 확인 절차입니다.

기본은 AUTO 주행 실행 준비이며 휴대폰에서 버튼을 누르기 전에는 AUTO 목표를 보내지 않습니다. 기존 MANUAL 웹 명령 연결은 기본 비활성. 별도 검증 후에만 `manual_motion:=true` 인자를 사용합니다. **동일한 `/amr_mux` 출력 발행 노드를 두 개 시작하지 마세요.**

## 4. 휴대폰 주행 시험

1. RViz의 Fixed Frame을 map으로 설정하고 `2D Pose Estimate`로 실제 위치/방향을 초기화한 다음 위치 추정을 확인합니다.
2. 휴대폰 기존 웹 `http://<IPC_IP>:8000/` 접속. 지도 iframe이 나타나면 AUTO 선택. `AUTO RPM=30` 입력하고 **RPM 적용**.
3. 목적지를 가까운 통행 가능 공간에 클릭. 웹과 RViz 경로 생성 및 출발점 확인.
4. 화면 안의 **AUTO 목표까지 주행 시작**을 누른 뒤 확인.
5. 실제 이동 중에는 휴대폰 화면을 열어 둡니다. 앱 백그라운드, 서버 통신 단절, Depth 장애, TF/CAN 이상, MANUAL 전환 시 목표가 취소되고 모터 권한이 해제되도록 구성했습니다.
6. 목표에 도착하면 NavigateToPose 완료 결과 후 모터 0RPM/실제 정지를 확인합니다. 웹 **AUTO 중지**는 별도 버튼입니다.

## 5. IPC 진단

다른 ROS2 터미널에서:

```bash
export ROS_DOMAIN_ID=31 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.bash
source ~/real_amr/install/setup.bash

ros2 topic echo /amr_mux/status --once --field data
ros2 topic echo /amr_motor_state --once --field data
ros2 topic info /amr_mux/wheel_rpm_left -v
ros2 topic info /amr_mux/wheel_rpm_right -v
ros2 topic info /amr_mux/motion_authorized -v
ros2 topic echo /amr_auto/rpm_limit_status --once --field data
ros2 topic info /amr/depth_obstacle -v
ros2 lifecycle get /controller_server
curl -s http://127.0.0.1:8771/api/status | python3 -m json.tool
```

`/amr_mux/wheel_rpm_left/right`에는 **Publisher가 `/amr_guarded_motor_mux` 하나만** 있어야 합니다. CAN node는 전용 `--mux-input`로만 구독하며, 기존 수동 입력 Publisher를 무시합니다. `reason`이 `DEPTH_NOT_CLEAR_OR_STALE`, `LIFT_NOT_CONFIRMED_STOPPED`, `MOTOR_STATE_MISSING_OR_FAULT`, `DUPLICATE_CAN_OWNER`, `NO_FRESH_MODE_SESSION`이면 **자동 구동을 강제로 진행하지 말고 해당 신호부터 점검**하세요.

`max_rpm=30`이면 15:1 기어 기준 약 0.0157m/s라서 실제 로봇이 정지마찰을 극복하지 못하거나 Nav2 진행률 검사가 실패할 수 있습니다. 이러한 경우 로그를 분석해야 하며 안전 제한을 임의 우회하지 않습니다.

## 6. 보안 / 현재 한계

사용자 요청에 따라 휴대폰 AUTO UI는 인증키/토큰 입력이 없습니다. **실제 이동을 명령할 수 있으므로 AMR :8000/:8771은 반드시 신뢰된 격리 시험망에만 노출**하고 외부 인터넷 포트포워딩/공용 와이파이 노출을 금지하세요.

`/mapex/cmd_vel`은 Nav2 제어 신호로, 외부 Nav2 목표/토픽 발행과 다른 원격 컨트롤러의 중복 실행을 허용하지 마세요. 현재 실물 검증 전의 코드이므로, 구동에 실패하면 원인을 확인하고 변경 후 재시험해야 합니다.
