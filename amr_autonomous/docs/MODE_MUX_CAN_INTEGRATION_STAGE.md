# MANUAL/AUTO 단일 제어권 연동 준비 - 2026-11-06

## 현재 검증한 인터페이스

- 웹 선택: `/amr_web/requested_mode` (`std_msgs/String`), MANUAL/AUTO. 이것은 **요청**이지 제어권 승인이 아닙니다.
- AUTO: `/mapex/cmd_vel` (`geometry_msgs/TwistStamped`) → `real_mapex_cmd_vel_to_rpm` → `/mapex/wheel_rpm_{left,right}_requested` (`Float64`).
- 기존 MANUAL: `amr_remote_client/amr_ws_to_ros.py`에서 `/wheel_rpm_left`, `/wheel_rpm_right`, `/amr_motion_authorized`를 직접 발행할 수 있습니다.
- CAN: `amr_remote_client/amr_ros_wheel_rpm_can.py`에서 이 세 토픽과 Depth 차단을 구독합니다. 실행 시 `UCan.safe_start()`가 드라이버를 Operation Enabled로 전환할 수 있으므로 **진단을 위해 함부로 실행하지 않습니다**.
- 진단: `scripts/mode_mux_dry_run.py`는 웹 기존 CAN 입력(읽기 전용), AUTO 요청(읽기 전용), CAN 모터 상태(읽기 전용)를 관찰합니다. `/amr_web/mux_diagnostic`에 String JSON만 발행합니다.

## 이번 변경

`mode_mux_dry_run.py`에 모드 변경 때 입력 시각 폐기, 신선한 좌/우 0 RPM 쌍 요구, 실제 CAN stop_verified 피드백 없을 때 승인 거부를 추가했습니다. 이 검사는 **모의 로직 검증**이며 실제 하드웨어 모드 인터락이 아닙니다. 특히 구버전 수동 웹 브리지가 CAN 토픽에 직접 발행하는 상태에서는 진단 모드 선택이 이를 차단하지 않습니다.

### 테스트

```bash
cd ~/hana-amr-project-autonomous
git pull --ff-only

export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=31 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ~/real_amr/install/setup.bash
python3 -u ~/hana-amr-project-autonomous/amr_autonomous/scripts/mode_mux_dry_run.py
```

별도 터미널:
```bash
ros2 topic echo /amr_web/mux_diagnostic
ros2 topic info /wheel_rpm_left -v
ros2 topic info /amr_motion_authorized -v
```

현재 CAN 노드 미실행 상태라면 stop_verified=false가 계속 유지되는 것이 정상입니다. AUTO 요청 0RPM을 수신해도 실제 CAN 제어 권한이 생성되지는 않습니다.

## 실제 CAN 연결을 위한 필수 변경 (아직 미구현)

1. 기존 웹 브리지의 수동 명령 발행을 전용 `/amr_manual/wheel_rpm_left_requested`, `/amr_manual/wheel_rpm_right_requested`로 **분리**하면서 기존 웹 세션·Depth·리프트·정지 인터락을 그대로 유지합니다.
2. 최종 CAN 노드 앞의 물리적 RPM 출력은 **오직 하나의 승인된 노드**가 발행해야 합니다. 다른 모든 Publisher를 배제하고 토픽 연결을 검증해야 합니다.
3. MANUAL↔AUTO 변경 시 양 바퀴 0 RPM 목표+실제 정지 검증, 새 명령 이력, CAN 상태/Depth/LiDAR/리프트 인터락 freshness, E-stop 및 드라이브 자체 통신 끊김 정지를 확인합니다.
4. CAN 수신의 명시적 RPM 상한(모드별), 초기 샘플 누락 감시, 권한 TTL, 장애물 정지, 역방향 전환 시 0RPM 정지, fault 래치를 코드와 실물 테스트로 검증합니다.
5. AUTO 목표 1000RPM은 **검증 전 설계 목표**이며 실제 활성 상한이 아닙니다.

라이브 연결 테스트 시 바퀴 공중 부양/바퀴 접근 금지/하드웨어 비상정지 확인이 선행돼야 합니다. 이 문서와 진단 코드는 CAN 모터의 동작을 승인하지 않습니다.
