# HANA AMR | QR · LiDAR · SLAM · AMCL · Nav2 자율주행 준비

기준일: 2026-11-05 / Ubuntu 24.04 / ROS 2 Jazzy / ROS_DOMAIN_ID=31

이 폴더는 기존 **AMR_QR 세션에서 실제 사용한 명령**과 재현을 위한 점검 명령을 터미널·단계별로 모아 둔 자율주행 준비 자료입니다. 이 브랜치는 \`amr-integrated-lift-target-accel-20261104\`에서 분기했으므로 GitHub에 이미 있던 CAN/Web/Depth/Lift 코드도 그대로 포함됩니다.

> **핵심 주의:** \`~/real_amr/src/real_amr_lidar\`의 실제 ROS 2 원본은 이 GitHub 저장소에 아직 없습니다. 이 문서의 \`real_amr_lidar\` 명령은 **IPC에 설치된 기존 패키지**를 사용합니다. 실행 전 반드시 파일과 설치 상태를 검사하세요. 문서의 모든 경로/파라미터가 현재 IPC에서 검증된 것은 아닙니다.

## 시작 순서

1. 운영 터미널마다 \`source <이 저장소>/amr_autonomous/scripts/source_ros_env.sh\`
2. \`bash <이 저장소>/amr_autonomous/scripts/healthcheck.sh\` (읽기 전용)
3. \`python3 <이 저장소>/amr_autonomous/scripts/inspect_ipc_sources.py\` (읽기 전용)
4. **[전체 실행 순서 및 명령어](docs/AMR_QR_COMMANDS.md)**를 열어 QR/라이다/ICP/SLAM 또는 Map Server/AMCL/Nav2를 **중복 실행 없이** 순서대로 구동
5. **[기존 IPC 코드 원본 관리](docs/IPC_SOURCE_MANIFEST.md)**를 참고하여 미등록 ROS 2 원본을 별도 검토
6. \`python3 <이 저장소>/amr_autonomous/scripts/nav_readiness.py --timeout 10 --require scan --require odom --require map --require amcl\`로 센서·위치추정 수신 확인

## 새로 추가한 코드 파일

| 파일 | 동작 |
|---|---|
| \`scripts/source_ros_env.sh\` | ROS2 Jazzy, CycloneDDS, ROS_DOMAIN_ID=31 환경 로드 |
| \`scripts/healthcheck.sh\` | 노드·토픽·Lifecycle·TF 등 **읽기 전용** 상태 검사 |
| \`scripts/nav_readiness.py\` | Scan/Odom/Map/AMCL/CmdVel 메시지 수신 확인(**발행 없음**) |
| \`scripts/inspect_ipc_sources.py\` | 로컬 \`real_amr_lidar\` 실제 파일 확인, 선택 시 비공개 검토 폴더로 복사 |

## 이 브랜치가 상속하는 기존 코드

- \`amr_motor_control/amr_enable_motors.py\`, \`amr_motor_control/amr_keyboard_safe.py\`
- \`amr_remote_client/amr_ros_wheel_rpm_can.py\`, \`amr_remote_client/amr_ws_to_ros.py\`
- \`amr_remote_client/amr_all_with_lift.launch.py\`
- \`amr_websocket/amr_openvino_camera_depth.py\`, \`amr_lift/lift_websocket.py\`
- 루트 \`one_command_launch.py\` 및 관련 테스트

이는 기존 저장소 파일을 **상속**한다는 뜻이며, 실제 IPC에서 동작 중인 최신 파일과 내용이 같다는 보장은 아닙니다.

## 물리적 주행 안전

- 이 문서의 진단 스크립트는 비영(非零) RPM/CAN/주행 Goal을 발행하지 않습니다.
- \`/mapex/cmd_vel\`에서 RPM/CAN으로 연결하는 구간은 아직 별도 검증 대상입니다.
- 독립 하드웨어 E-stop, 드라이버 측 통신단절 정지, Depth 정지, ARM/DISARM/리프트 인터록을 유지하세요.
- \`bench_motion:=true\`는 물리 시험용으로만, 정지 검증 후 운영자가 명시적으로 선택하세요. 테스트를 위해 인터록을 우회하지 마세요.
- 센서 통합 Launch 및 Nav2 통합 Launch와 수동으로 개별 노드를 동시에 실행하면 토픽·TF·Lifecycle 중복이 발생할 수 있습니다.

## 관련 문서

- [터미널별 전체 명령어](docs/AMR_QR_COMMANDS.md)
- [IPC 원본 소스 파일 점검 및 업로드 준비](docs/IPC_SOURCE_MANIFEST.md)
