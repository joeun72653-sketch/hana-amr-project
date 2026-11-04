# AMR 버튼 해제 부드러운 감속 — 통합 코드 (실물 미검증)

기준: `amr-integrated-lift-target-accel-20261104` (가속/리프트 정상 동작을 확인한 이전 브랜치).
이 브랜치는 **수정 코드 보관용**입니다. 소프트 감속 기능의 실제 주행 및 정지거리는 아직 검증되지 않았습니다. 기존 브랜치와 `main`을 변경하지 않았습니다.

## 수정된 전체 실행 파일

| 환경 | 파일 | 역할 |
| --- | --- | --- |
| Windows | `windows/amr_server_8direction.py` | 웹에서 받은 `release`를 `normal_release`로 전달; 기존 STOP/통신 감시 유지 |
| Ubuntu | `amr_websocket/index.html` | 정상 전진·후진 버튼 해제에만 `release` 요청 |
| Ubuntu | `amr_remote_client/amr_ws_to_ros.py` | 일반 감속 명령 처리, 조건 검사 및 최종 STOP 검증 연계 |
| Ubuntu | `amr_remote_client/amr_ros_wheel_rpm_can.py` | 기존 가속 60 RPM/s 유지, 일반 감속 중 별도 제어 |
| Ubuntu | `amr_remote_client/amr_all_with_lift.launch.py` | 기존 통합 실행 파일 그대로 |
| Ubuntu | `amr_lift/lift_websocket.py` | 리프트 목표 Pulse 이동 ±20 허용, 기존 버전 그대로 |

## 설정

- 기존 가속: 60 RPM/s
- 기존 비영점 속도 감소: 60 RPM/s
- 전진·후진 버튼 정상 해제: 일반 감속 요청 400 RPM/s, 제한 시간 0.8초 (시험용, 검증된 안전 값 아님)
- 제자리 회전, 명시적 웹 STOP/Q/Space, DISARM, Depth 장애물 차단, 통신 결손: 원래 안전 정지 요청 유지하도록 설계
- Windows WebSocket 포트: 8765
- Ubuntu 리프트 WebSocket 포트: 8767

## 실행

Windows, 수정된 서버를 `C:\amr_server\amr_server_8direction.py`에 배치하고 기존 서버를 종료한 후:

```cmd
cd C:\amr_server
python amr_server_8direction.py --bench-motion
```

Ubuntu: 수정된 파일을 **기존 정상 파일 백업 후** 각 실행 경로에 배치하고:

```bash
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
ros2 launch ~/amr_remote_client/amr_all_with_lift.launch.py bench_motion:=true
```

## 주의

- Windows + Ubuntu + 웹 파일은 같은 버전이어야 합니다. 일부만 교체한 상태에서 실제 구동 금지.
- 바퀴를 안전하게 띄워 차체 고정, 독립 하드웨어 비상정지 준비, 기존 장애물 정지와 STOP 기능 검증 전 지면 주행 금지.
- 버스/드라이버 통신 장애에서는 소프트웨어 명령으로 물리적 정지를 보장할 수 없습니다.
- `COMMAND_TIMEOUT` 및 기존 인터록을 삭제하거나 우회하지 마세요.
- 이 브랜치는 완료된 검증 결과가 아니라 **코드 스냅샷**입니다. 실제 테스트 성공 여부에 따라 추후 수정이 필요할 수 있습니다.
