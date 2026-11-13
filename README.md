# HANA AMR — 실행 명령어 및 운영 가이드

Ubuntu ROS 2 Jazzy 기반 AMR의 **CANopen 모터, Windows WebSocket, 웹 관제, IMU, Lift, 카메라** 실행 순서를 정리한 문서입니다.

> **기준:** 2026-10-27. 현재 GitHub에 올라온 파일과, AMR Ubuntu PC에서 별도로 실행해 온 로컬 파일을 구분했습니다. 아래 명령은 *각기 다른 터미널*에서 실행하며, 파일/장치 경로/네트워크 주소가 실제 환경과 일치하는지 먼저 확인하세요.

## 0. 실행 전 확인

AMR 바퀴가 의도치 않게 움직이지 않도록 기체를 고정하고 독립 비상정지, 모터 드라이브의 통신 단절 정지, CANopen Watchdog, Lift 인터록을 확인합니다. **ARM/DISARM, Fault 및 정지 검증을 우회하지 마세요.**

Ubuntu에서 중복 실행을 확인합니다.

```bash
pgrep -af 'one_command_launch.py|amr_ws_to_ros|amr_ros_wheel_rpm_can|amr_depth_obstacle.py|amr_openvino_camera_depth.py|lift_websocket.py'
hostname -I
ls -l /dev/serial/by-id/
```

- 예전 Ubuntu 웹 IP: `192.168.0.29` (DHCP/설정에 따라 달라질 수 있음)
- 예전 Windows WebSocket 서버 IP: `192.168.0.31`, 포트 `8765`
- ROS 2: Jazzy, `ROS_DOMAIN_ID=31`
- CANopen uCAN USB VCP: 코드 기본 `115200` baud (**CAN 버스의 250/500 kbps와 다름**)

## 1. Windows — WebSocket 서버

**Windows CMD**에서 실행합니다. 이 Windows 파일은 현재 GitHub 저장소에 포함되지 않았습니다.

```cmd
cd C:\amr_server
python amr_server_8direction.py --bench-motion
```

- Windows 서버: `ws://192.168.0.31:8765`
- 단일 Operator / 단일 Robot 세션을 처리합니다.
- `Invalid/duplicate role`이 발생하면 이미 연결 중인 Robot Bridge나 Operator가 있는지 점검합니다.
- `--bench-motion`은 시험용 옵션이며 모터와 안전장치의 준비 상태를 보장하지 않습니다.

## 2. Ubuntu — CANopen 모터 Operation Enable

**터미널 A.** 두 모터의 목표/실제 RPM이 0이고 안전 조건이 충족된 상태에서만 안내에 따라 ENABLE합니다.

```bash
cd ~/amr_motor_control
python3 -u amr_enable_motors.py
```

`[SUCCESS] 양쪽 모터 Operation Enabled` 등을 확인한 후 ENABLE 프로그램을 종료합니다. 같은 uCAN 시리얼 포트를 CANopen 제어 프로세스와 동시에 열지 않습니다.

## 3. Ubuntu — CANopen 모터 노드

**터미널 B.** GitHub에 포함된 기본 파일은 다음과 같습니다.

```bash
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=31
source /opt/ros/jazzy/setup.bash
cd ~/amr_remote_client
python3 -u amr_ros_wheel_rpm_can.py
```

`[READY]` 메시지와 모터 상태를 확인합니다.

**CAN RPM 명령 처리(2026-11-11 변경):** 좌우 0x60FF 명령은 두 노드에
연속 송신한 후 각각의 CAN ACK를 받습니다. 정상 주행 중 잠깐 발생한
`(0, 0)` 명령은 즉시 양쪽 0 RPM 목표를 전송하지만, 실제 바퀴 RPM이
완전히 0이 될 때까지 다음 주행 명령을 차단하지 않습니다. 실제 정지는
엔코더로 비동기 확인하며 `/amr_motor_state.stop_verify_count`는 기존처럼
양쪽 모터가 실제 정지한 뒤에만 증가합니다. **웹 DISARM/Fault
래치 해제 및 역회전 시에는 물리 정지 확인을 계속 요구**합니다.
기존에 비활성화한 350ms `WATCHDOG_TIMEOUT` 래치는 다시 활성화하지 않았습니다.
비상정지 및 드라이브 통신 단절 보호장치를 해제하는 변경이 아닙니다.
실차 적용 전 바퀴를 지면에서 띄운 상태로 좌우 동시 출발, 짧은 0 RPM
후 재출발, 역회전, Ctrl+C 및 통신 단절을 검증하세요.

**로컬 Normal 버전:** 이전에 Ubuntu PC에서 생성한 `amr_ros_wheel_rpm_can_normal.py`를 사용하는 경우 **위 명령 대신** 다음을 실행합니다.

```bash
python3 -u ~/amr_remote_client/amr_ros_wheel_rpm_can_normal.py
```

> `*_normal.py`는 현재 GitHub 저장소에 **없으며**, AMR Ubuntu PC에 별도로 존재하는 파일입니다. 일반 버전과 기본 버전을 **동시에 실행하지 마세요**.

## 4. Ubuntu — 주행 WebSocket Robot Bridge

**터미널 C.** GitHub 기본 버전:

```bash
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=31
source /opt/ros/jazzy/setup.bash
cd ~/amr_remote_client
python3 -u amr_ws_to_ros.py --bench-motion
```

**Lift 연동 버전(저장소에 포함):** 위 Bridge **대신** 다음 파일을 실행합니다.

```bash
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=31
source /opt/ros/jazzy/setup.bash
cd ~/amr_remote_client
python3 -u amr_ws_to_ros_lift.py --bench-motion
```

**로컬 Normal 버전:** Ubuntu에 `amr_ws_to_ros_normal.py`가 존재한다면 다음을 사용할 수도 있습니다(현재 GitHub에는 없음).

```bash
python3 -u ~/amr_remote_client/amr_ws_to_ros_normal.py --bench-motion
```

**Bridge는 위 세 가지 중 하나만 실행합니다.** 정상 연결 시 `[CONNECTED] Windows Server`를 확인합니다. Lift 연동 파일이 Lift 인터록과 정상 연동하는지는 실제 로컬 환경에서 검증해야 합니다.

## 5. Ubuntu — 웹 관제 페이지

**터미널 D.**

```bash
cd ~/amr_websocket
python3 -m http.server 8000 --bind 0.0.0.0
```

- 일반 관제: `http://192.168.0.29:8000/` 또는 `/index.html`
- Lift 포함 관제: `http://192.168.0.29:8000/index_lift.html`

브라우저를 여는 PC/휴대폰이 AMR 웹 서버에 접근할 수 있어야 합니다. 8000번 포트가 이미 사용 중이라면 웹 서버를 중복 실행하지 마세요.

## 6. Ubuntu — Lift WebSocket 상승·하강

**터미널 E.** GitHub 위치는 `amr_lift/lift_websocket.py`, 기존 Ubuntu 운영 위치는 `~/amr_lift/lift_websocket.py`입니다.

```bash
cd ~/amr_lift
source .venv/bin/activate
python -u lift_websocket.py
```

- Lift WebSocket: `ws://192.168.0.29:8767` (실제 AMR IP 기준)
- 서버 시작 로그에 **일회성 인증 토큰**이 표시됩니다. 토큰은 공개 저장소/보고서에 붙여 넣지 말고 Lift 관제 화면의 토큰 입력란에만 입력하세요.
- 실제 Lift 시리얼 장치, Modbus 설정, 모터 정지/인터록, 끝단 스위치 상태가 맞는지 확인해야 합니다.

## 7. 카메라 — **둘 중 하나만** 실행

한 대의 Orbbec Gemini 335L을 두 Python 프로세스가 동시에 점유하지 않도록 주의하세요.

### A. Depth 전용 거리 측정/1m 보조 전진 차단

**로컬 파일:** `~/amr_websocket/amr_depth_obstacle.py` (현재 GitHub에는 없음)

```bash
export ROS_DOMAIN_ID=31
source /opt/ros/jazzy/setup.bash
source ~/yolo_openvino_env/bin/activate
env LD_LIBRARY_PATH="$HOME/yolo_openvino_env/lib/python3.12/site-packages/pyorbbecsdk:$LD_LIBRARY_PATH" \
python3 -u ~/amr_websocket/amr_depth_obstacle.py
```

진단 토픽 확인:

```bash
export ROS_DOMAIN_ID=31
source /opt/ros/jazzy/setup.bash
ros2 topic echo /amr/depth_obstacle
```

Depth 센서 판단값이 안전 등급의 정지 기능을 대체하지 않습니다. 로컬 코드의 `near=N/A` 허용 여부와 보정식을 실제로 확인해야 합니다.

### B. OpenVINO 객체인식 + 웹 MJPEG

**GitHub에 포함된 기본 파일:** `amr_websocket/amr_openvino_camera_depth.py`

```bash
cd ~/amr_websocket
source ~/yolo_openvino_env/bin/activate
python3 -u amr_openvino_camera_depth.py
```

- MJPEG 스트리밍: `http://192.168.0.29:8090/stream.mjpg`
- HTTP 상태: `http://192.168.0.29:8090/health`
- **중요:** Ubuntu에서 별도로 수정했던 *OpenVINO + Depth 토픽 통합판*이 GitHub 기본 파일과 동일하다고 가정하지 마세요. `/amr/depth_obstacle` 토픽을 발행하는지는 실제 로컬 소스를 확인해야 합니다.
- 카메라 드라이버 라이브러리 충돌이 발생할 경우 Depth 전용 실행에서 사용했던 `LD_LIBRARY_PATH` 설정을 검토합니다.

## ROS2 NumPy 호환성 사전 검사

통합 런처는 ROS2 환경을 로드할 때 `PYTHONNOUSERSITE=1`을 설정하고,
모터 Enable 전에 관리 프로세스와 동일한 Python 환경으로 `numpy`,
`rclpy`, ROS 메시지 타입을 import합니다. 성공하면 `[PYTHON] ROS/NumPy
import OK:` 로그에 NumPy 버전과 실제 import 위치를 출력합니다. 실패하면
모터 구동 전에 종료하며, NumPy ABI/패키지 경로 오류를 표시합니다.

이는 NumPy를 자동 설치·다운그레이드하는 기능이 아니라 충돌을 조기에
탐지하는 검사입니다. `--dry-run`에서는 하드웨어와 Python import 검사
모두 생략합니다.

## 새 IPC에서 웹 접속 IP 지정

`one_command_launch.py` 실행 시 `--host-ip`로 현재 IPC에 실제로 할당된
LAN IPv4 주소를 지정할 수 있습니다. 캘리브레이션 서버(8877) 바인딩 주소와
휴대폰 접속 안내 URL(8000)에 모두 적용됩니다.

```bash
python3 -u one_command_launch.py --skip-camera --web-drive --host-ip 192.168.0.250
```

기존 `AMR_CAL_HOST` 환경변수도 계속 지원하며 `--host-ip`가 우선합니다.
둘 다 지정하지 않으면 이전 기본값 `192.168.0.35`를 유지합니다.
주소는 반드시 해당 PC에 할당되어 있어야 합니다(`ip -4 addr`로 확인).
이 옵션은 Windows WebSocket 서버(`192.168.0.31:8765`)의 주소는 변경하지 않습니다.

## CAN USB 장치 자동 탐색 (통합 런처)

`one_command_launch.py`는 실행 시 `/dev/serial/by-id/`에서
`FTDI_FT232R`가 이름에 포함된 장치를 대소문자 구분 없이 찾습니다.
선택된 장치 경로를 모터 Enable 프로세스의 `--port`와 CAN ROS 노드의
`AMR_CAN_PORT` 환경 변수에 동일하게 전달합니다. USB 일련번호가 바뀌어도
통합 런처를 사용할 때는 코드를 수정하지 않아도 됩니다.

장치가 없거나 여러 개 발견되거나 권한이 없으면 시작을 중단합니다.
FT232R이라는 이름만으로 CAN 어댑터임을 보장할 수 없으므로 여러 어댑터를
연결한 환경에서는 장치 식별을 별도로 구성해야 합니다.
독립 실행하는 모터 Enable / CAN 노드에는 런처의 자동 탐색이 적용되지 않습니다.

## MicroStrain USB 장치 자동 탐색 (통합 런처)

`one_command_launch.py`는 실행할 때마다 `/dev/serial/by-id/`에서 이름에
`Microstrain`이 포함된 장치를 대소문자 구분 없이 탐색합니다.
USB 시리얼 번호가 변경되어도 원래 `amr_imu/config/microstrain.yml`은 변경하지 않고,
`~/amr_one_command_logs/microstrain_runtime.yml`에 실행 전용 설정을 생성하여
MicroStrain ROS 드라이버에 전달합니다. `--log-dir`을 지정한 경우 해당 디렉터리를 사용합니다.

연결된 장치가 없거나 둘 이상 있거나 읽기/쓰기 권한이 없으면 안전하게 시작을 중단합니다.
`--skip-imu` 또는 `--external-imu-driver`를 사용하면 자동 탐색하지 않습니다.
**직접 실행하는 `ros2 launch microstrain_inertial_driver ...`에는 이 자동 탐색이 적용되지 않습니다.**

## 8. IMU (필요할 때)

**터미널 F — MicroStrain 드라이버:**

```bash
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=31
source /opt/ros/jazzy/setup.bash
ros2 launch microstrain_inertial_driver microstrain_launch.py params_file:=$HOME/amr_imu/config/microstrain.yml
```

**터미널 G — IMU Roll/Pitch/Yaw 노드:**

```bash
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=31
source /opt/ros/jazzy/setup.bash
python3 -u ~/amr_imu/imu_rpy_node.py
```

**터미널 H — IMU 웹 전송:**

```bash
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=31
source /opt/ros/jazzy/setup.bash
python3 -u ~/amr_imu/amr_imu_ws.py
```

IMU 웹소켓은 기존 설정 기준 Ubuntu `8766` 포트를 사용합니다. MicroStrain 드라이버와 구성 파일은 별도로 설치되어야 합니다.

## 9. ROS2 상태 확인 / 트러블슈팅

```bash
export ROS_DOMAIN_ID=31
source /opt/ros/jazzy/setup.bash

ros2 topic list -t
ros2 topic echo --once /amr_motor_state
ros2 topic echo --once /amr_motion_authorized
ros2 topic info /amr/depth_obstacle -v
pgrep -af 'amr_ws_to_ros|amr_ros_wheel_rpm_can|one_command_launch.py|lift_websocket.py'
```

- `/amr_motor_state`는 `std_msgs/String`에 담긴 JSON 형식입니다.
- `web_authorized=false`와 `status=DISARMED`가 계속되면 Windows 서버의 Operator/Robot 세션 및 Bridge의 ARM 로그를 확인합니다. 보호 기능을 무력화해서 해결하지 않습니다.
- `Invalid/duplicate role`: Windows 서버에 동일한 Robot/Operator 역할이 이미 접속되어 있을 수 있습니다.
- `Address already in use`: 8000/8090/8767 등의 포트를 기존 프로세스가 점유 중인지 확인합니다.
- 두 개의 모터 제어 노드 또는 Bridge를 중복 실행하지 마세요.
- 실제 CAN 장치 `/dev/serial/by-id/...` 경로와 `BENCH_BYPASS_DISARM` 및 Watchdog 설정을 검토한 뒤 동작 시험을 진행하세요.

## 10. 코드 파일 구분

| 파일 | GitHub 포함 여부 | 설명 |
| --- | --- | --- |
| `amr_motor_control/amr_enable_motors.py` | 예 | 모터 Operation Enable |
| `amr_remote_client/amr_ros_wheel_rpm_can.py` | 예 | CANopen 모터 노드 |
| `amr_remote_client/amr_ws_to_ros.py` | 예 | 기본 WebSocket Robot Bridge |
| `amr_remote_client/amr_ws_to_ros_lift.py` | 예 | Lift 연동 Bridge |
| `amr_lift/lift_websocket.py` | 예 | Lift WebSocket 서버 |
| `amr_websocket/index_lift.html` | 예 | Lift 통합 관제 페이지 |
| `amr_remote_client/fix_amr_normal_arm.py` | 예 | Normal 로컬 Bridge 중복 STOP 개선용 패치 |
| `amr_remote_client/*_normal.py` | **아니오** | Ubuntu 로컬 일반 주행용 파일 |
| `amr_websocket/amr_depth_obstacle.py` | **아니오** | Ubuntu 로컬 Depth 전용 파일 |
| Windows `C:\amr_server\amr_server_8direction.py` | **아니오** | Windows 중계 서버 |

> `fix_amr_normal_arm.py`는 실행 중인 Bridge 자체가 아니라, **기존 `amr_ws_to_ros_normal.py`에 수정 사항을 적용하는 패치 스크립트**입니다. 적용 전 로컬 원본을 백업하고 현재 내용이 일치하는지 확인하세요.

## 11. 원격 네트워크를 사용하지 않는 One Command 실행

`one_command_launch.py --local-ros-only`는 이 런처가 시작하는 프로세스에서
**외부 LAN/NIC 통신을 사용하지 않고 같은 PC의 ROS2만 사용**할 때 선택합니다.

```bash
cd ~/hana-amr-github
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=31
source /opt/ros/jazzy/setup.bash

python3 -u one_command_launch.py --local-ros-only
```

- 유지: MicroStrain IMU 드라이버, `/imu/data`, RPY 변환 `/amr/imu/rpy`, 모터 자동 Enable, CANopen 및 ROS2 모터 토픽.
- 미실행: HTML Web UI(8000), 보정 HTTP 서버(8877), Windows WebSocket 브리지(8765 연결), IMU WebSocket 서버(8766), 카메라 HTTP(8090).
- DDS는 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, 빈 `ROS_STATIC_PEERS` 및 저장소의 전용 Fast DDS/Cyclone DDS 설정으로 Linux 루프백(`lo`)에 제한합니다. 선택한 RMW가 두 종류 이외면 안전하게 시작을 거부합니다.
- `--web-drive` 및 기존 프로세스에 의존하는 `--external-imu-driver`는 이 모드와 조합할 수 없습니다.
- `--dry-run`을 붙여 시작 구성을 모터 접속 없이 확인할 수 있습니다.
- **적용 범위는 이 런처가 시작한 프로세스뿐입니다.** 이미 실행 중인 ROS2 노드나 운영체제의 다른 프로그램 트래픽은 차단하지 않습니다. 독립 실행한 자율주행/센서 노드도 루프백 DDS 설정을 공유해야 LAN 사용이 없어집니다. 실제 외부 NIC 패킷 0건을 엄격히 보증하려면 네트워크 네임스페이스/방화벽 분리와 패킷 캡처 검증이 별도로 필요합니다.
- 이 모드는 ROS2 통신만 로컬로 제한합니다. CAN 모터 구동 자체의 안전장치, 물리 비상정지, 주행권한 조건은 제거하지 않습니다.


## 12. 기존 AMR 실행·테스트 코드 목록 (Codex 분석용)

> **정리 기준: 2026-10-31.** 이전 AMR 작업 대화에서 실행·테스트 대상으로 언급된 Python 파일 목록입니다. **GitHub 확인**은 이 문서 작성 시 `main` 브랜치에서 아래 기재한 경로의 파일을 직접 조회한 결과이며, **경로 미확인**은 지정 경로에서 찾지 못했다는 뜻입니다(IPC 로컬에 없다는 뜻은 아님). 실제 실행 이력 및 파일의 최신 버전은 IPC에서 별도로 확인해야 합니다.

| 구분 | 실행·테스트 파일 (조회 경로) | 역할 | GitHub `main` 조회 결과 |
| --- | --- | --- | --- |
| 통합 실행 | `one_command_launch.py` | AMR 시스템 통합 실행 및 시작 구성 | **확인** |
| CANopen | `amr_motor_control/amr_enable_motors.py` | 모터 Operation Enable | **확인** |
| CANopen | `amr_motor_control/amr_keyboard_safe.py` | 안전 키보드 제어 및 초기화 로직 | **확인** |
| CANopen | `amr_motor_control/amr_encoder_probe.py` | 엔코더 정보 확인 | 경로 미확인 |
| ROS2·모터 | `amr_remote_client/amr_ros_wheel_rpm_can.py` | CANopen·ROS2 RPM 상태 연동 | **확인** |
| ROS2·웹 | `amr_remote_client/amr_ws_to_ros.py` | WebSocket 주행 명령의 ROS2 연동 | **확인** |
| ROS2·리프트 | `amr_remote_client/amr_ws_to_ros_lift.py` | 웹 주행과 리프트 연동 Bridge | **확인** |
| 리프트 | `amr_lift/lift_status.py` | Modbus/RS-232 리프트 상태 조회 | 경로 미확인 |
| 리프트 | `amr_lift/lift_down_limited.py` | 제한 범위 하강 테스트 | 경로 미확인 |
| 리프트 | `amr_lift/lift_down_bottom.py` | 최하단 하강 테스트 | 경로 미확인 |
| 리프트 | `amr_lift/lift_up_top.py` | 최상단 상승 테스트 | 경로 미확인 |
| 리프트 | `amr_lift/lift_websocket.py` | 웹 기반 리프트 제어 및 인터록 연동 | **확인** |
| 카메라 | `amr_websocket/amr_camera_mjpeg.py` | 카메라 MJPEG 스트리밍 | 경로 미확인 |
| 객체인식 | `amr_websocket/amr_openvino_camera.py` | OpenVINO 기반 객체인식 | 경로 미확인 |
| 객체인식·Depth | `amr_websocket/amr_openvino_camera_depth_integrated.py` | 객체인식 및 거리 측정 통합 | 경로 미확인 |
| Depth | `amr_websocket/amr_depth_obstacle.py` | 거리 측정 및 근접 장애물 차단 보조 | 경로 미확인 |
| Orbbec 테스트 | `amr_depth_test/test_orbbec_sdk.py` | Orbbec SDK RGB·Depth 실행 시험 | 경로 미확인 |
| Orbbec 테스트 | `amr_depth_test/test_rgb_depth_640.py` | RGB·Depth 동시 출력 프로파일 테스트 | 경로 미확인 |

### Codex 1차 코드 흐름·오류 분석 대상 (8개)

1. `one_command_launch.py` — 통합 프로세스 시작 순서 및 종료 처리
2. `amr_motor_control/amr_enable_motors.py` — CANopen Enable·SDO 응답 확인
3. `amr_motor_control/amr_keyboard_safe.py` — DISARM·안전 정지 및 수동 제어
4. `amr_remote_client/amr_ros_wheel_rpm_can.py` — RPM 수신·ROS2 상태 발행
5. `amr_remote_client/amr_ws_to_ros_lift.py` — 웹 명령·주행·리프트 Bridge
6. `amr_lift/lift_websocket.py` — Lift 인터록, RS-232 명령과 예외 처리
7. `amr_websocket/amr_openvino_camera_depth_integrated.py` — 객체인식·Depth 통합판(지정 경로 미확인, 로컬 확인 필요)
8. `amr_websocket/amr_depth_obstacle.py` — Depth 주행 차단 보조(지정 경로 미확인, 로컬 확인 필요)

**분석 시 주의:** 이 목록을 코드 업로드 완료 목록이나 가동 보증 목록으로 간주하지 마세요. Codex로 분석할 때는 IPC에서 실제 사용 중인 파일과 실행 로그를 먼저 대조하고, 초기에 **읽기 전용**으로 파일 구조·호출 관계·통신·로그를 점검합니다. 실제 모터 구동, Lift 동작, 인터록 우회나 하드웨어 명령 실행은 분석 단계에서 수행하지 않습니다.
