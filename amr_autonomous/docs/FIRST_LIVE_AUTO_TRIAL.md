# 실제 AMR 최저속 AUTO 주행: 첫 시험 체크리스트

**현 상태**: live 코드 구성 반영 / 실제 IPC 또는 모터 실물 검증은 아직 완료되지 않음.
**주의**: `run_live.sh`는 `UCan.safe_start()`를 호출해 물리 CAN 드라이버를 Operation Enabled로 전환할 수 있습니다. 사람이나 물체 주변에서는 실행하지 마세요.

## 1. 코드

- `amr_autonomous/amr_one_command_live.launch.py`: LiDAR, ICP, Map Server, AMCL, Nav2, lifecycle 검증, RViz, 휴대폰 웹, OpenVINO/Depth, Lift 상태 서버, CAN 모터 노드(`--mux-input`), 단일 mux, AUTO RPM limiter 실행
- `amr_autonomous/scripts/check_activate_nav2.py`: Planner/Controller/Behavior/BT 및 Costmaps가 모두 Active인지 확인; 실패 시 CAN 시작 중단
- `amr_autonomous/scripts/guarded_motor_mux.py`: 첫 시험 AUTO 30RPM **하드 상한**, 소스 timestamp, Depth, Lift, CAN 피드백, stop_verified, 모드 제어권 확인
- `amr_remote_client/amr_ros_wheel_rpm_can.py`: `--mux-input` 모드에서 /amr_mux/wheel_rpm_left/right, /amr_mux/motion_authorized만 구독. 350ms 수신 watchdog을 구현한 브랜치 버전 사용. 실물 코드의 `0x60FF` 드라이버 단위와 실제 모터축 RPM 대응 여부는 현장 확인 필요
- `amr_autonomous/map_preview_server.py`: `--allow-physical-auto`일 때만 NavigateToPose 시작/중지; 기본 실행은 preview-only

## 2. 시작 전에 필수 현장 확인

- 사람 없는 통제된 구역, 바퀴와 리프트 접근 금지, 확인된 하드웨어 E-Stop, 독립 드라이버 통신상실 정지 동작 확보
- 실제 로봇/IPC 모터 ID, 회전 방향, 속도 단위, 엔코더 부호 검증
- LiDAR 안전 스캐너와 Depth의 실제 장애물 차단, 센서 전원/케이블 단절 동작, 리프트 정지 인터락 확인
- 저장 지도와 실제 장애물/벽 위치 일치, AMCL/TF 최신성 확인, 노이즈 장애물 제거 전에 임의로 비용 영역 해제하지 않기
- 기존 단독 `:8000`, `:8771`, RViz, LiDAR, ICP, AMCL, Nav2, RPM, CAN 웹브리지 프로세스를 모두 종료해 중복 제어 경로가 없도록 하기
- 라이브 서버에는 토큰 로그인 기능이 없으므로 신뢰할 수 있는 **격리된 사설망**에서만 운영. WAN 포트 포워딩 금지

## 3. 코드 갱신

이전 `run_all.sh` 로컬 수정 때문에 pull 오류가 났다면 백업 후 해당 수정만 보관:
```bash
cd ~/hana-amr-project-autonomous
git status --short
cp amr_autonomous/run_all.sh ~/run_all_manual_backup_20261106.sh
git stash push -m "local run_all before live test" -- amr_autonomous/run_all.sh
git pull --ff-only
```
변경된 다른 파일이 표시되면 확인 후 별도로 처리합니다.

## 4. 검증 후에만 실제 Launch

먼저 Python 파일 문법 및 필수 파일 검사:
```bash
cd ~/hana-amr-project-autonomous
bash -n amr_autonomous/run_live.sh
/usr/bin/python3 -m py_compile amr_autonomous/scripts/guarded_motor_mux.py amr_autonomous/amr_one_command_live.launch.py amr_autonomous/map_preview_server.py
```

독립적인 하드웨어 비상정지와 드라이버 통신 단절 시 정지를 실제로 확인한 현장 감독자만:
```bash
bash ~/hana-amr-project-autonomous/amr_autonomous/run_live.sh
```
명령 후 `START`를 입력하면 실제 CAN safe_start가 시작될 수 있습니다. 이 단계에서 첫 실물 시험은 가능하면 바퀴를 지면에서 분리한 정비용 고정 장치에서 방향·RPM 피드백·중지 동작을 먼저 확인합니다.

휴대폰 관제에서 AUTO를 선택하고 목적지를 지정한 뒤 `AUTO 시작`으로 NavigateToPose를 요청합니다. 최초 명령은 mux 30RPM 상한을 통과해야 하고, 하나라도 안전 정보가 오래되거나 실패하면 실패 닫힘. 웹 STOP은 실제 E-Stop을 대체하지 않습니다.

## 5. 출력 검증 (읽기 전용)

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /behavior_server
ros2 lifecycle get /global_costmap/global_costmap
ros2 lifecycle get /local_costmap/local_costmap
ros2 topic info /amr_mux/wheel_rpm_left -v
ros2 topic info /amr_mux/wheel_rpm_right -v
ros2 topic info /amr_mux/motion_authorized -v
ros2 topic echo /amr_mux/status --once --field data
ros2 topic echo /amr_motor_state --once --field data
```

`/amr_mux/*` 각 RPM/authorization Publisher가 단 하나의 `amr_guarded_motor_mux`인지, CAN node가 해당 토픽만 구독하는지 확인합니다. `/amr/depth_obstacle` 구독 신호는 별도로 최신이어야 합니다.

**정지 조건:** 지도와 AMCL 불일치, 구동 휠 부호 반대, 비정상 가속, 장애물 감지 실패, 모터 피드백 불일치, stop_verified 거짓, 드라이버 통신 오류 시 즉시 실물 E-Stop을 사용하고 시험을 중단하세요.
