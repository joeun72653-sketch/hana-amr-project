# real_amr_lidar 실제 소스 반영 현황

작성일: 2026-11-05

## 확인된 사실

- GitHub `hana-amr-project`의 기존 `main` 및 `amr-integrated-lift-target-accel-20261104`에는 `real_amr_lidar` 디렉터리가 없습니다.
- IPC에서 기존에 사용한 경로는 `~/real_amr/src/real_amr_lidar`입니다.
- 현재 GitHub에 올라온 CAN/Web/Depth/Lift 관련 원본은 새 브랜치에 그대로 상속되었습니다.
- IPC의 `real_amr_lidar` 실제 코드는 ChatGPT가 IPC 파일 시스템에 직접 접근할 수 없어 **복사하지 않았습니다**. 아래 파일들은 경로가 확인되었지만 내용/버전은 GitHub에서 검증하지 못했습니다.

## IPC에서 확인할 파일

| IPC 상대 경로 (`~/real_amr/src/real_amr_lidar/`) | 역할 |
|---|---|
| `launch/real_amr_sensor_qr_test.launch.py` | QR·센서 통합 |
| `launch/real_amr_slam_mapping.launch.py` | SLAM |
| `launch/real_mapex_nav2.launch.py` | Nav2 |
| `launch/real_mapex_autonomous_mapping.launch.py` | 통합 맵핑 |
| `scripts/gls611_pose_node.py` | SICK GLS611 QR 위치 |
| `scripts/qr_sensor_test_geometry.py` | QR 기하 테스트 |
| `scripts/real_dual_lidar_scan_merge.py` | LiDAR 병합 |
| `scripts/real_lidar_icp_odometry.py` | ICP odometry |
| `scripts/real_mapex_cmd_vel_to_rpm.py` | Nav2 → Wheel RPM |
| `scripts/real_mapex_wheel_odometry.py` | 휠 odometry |
| `package.xml`, `setup.py`, `setup.cfg` (존재할 경우) | ROS2 패키지 구성 |

`ros2 pkg executables real_amr_lidar` 및 `ros2 launch real_amr_lidar <launch-file> --show-args`로 실제 설치된 이름을 확인하세요.

## IPC 터미널에서 원본 확인 (읽기 전용)

```bash
cd ~/real_amr
git status --short
git remote -v
find src/real_amr_lidar -maxdepth 3 -type f | sort
ros2 pkg executables real_amr_lidar

# 이 브랜치를 PC에 checkout한 뒤 실행
python3 /path/to/hana-amr-project/amr_autonomous/scripts/inspect_ipc_sources.py
```

## 검토본 만들기 (공개 업로드 아님)

```bash
python3 /path/to/hana-amr-project/amr_autonomous/scripts/inspect_ipc_sources.py --copy-for-review
```

이 명령은 확인된 파일만 `amr_autonomous/.ipc_src_review/`에 복사합니다. 이 폴더는 `.gitignore`로 제외됩니다. **원본을 변경하거나 GitHub에 업로드하지 않습니다.**

업로드 전 반드시 하드코딩된 IP, 장치 시리얼 번호, 토큰/비밀번호, 내부 호스트명, 개인 경로, 사내 설정, 민감 로그 등을 검토하고 제거한 뒤 안전한 공개용 소스를 별도로 추가하세요. `real_amr`가 독립적인 Git 저장소라면 Git 원본·히스토리를 보존하고 서브모듈/별도 저장소 연계를 검토하는 것이 좋습니다.

## 현재 남은 연동 확인

1. 지도 파일 `~/.ros/real_amr/mapex_maps/real_mapex_20261028_162015.yaml`과 `~/amr_maps/amr_map_01.yaml` 중 실제 사용할 버전 결정
2. `map → odom → base_link` TF 중복 발행 방지
3. `/mapex/cmd_vel` (`geometry_msgs/msg/TwistStamped`) → RPM 변환기 → CAN 브리지 subscriber 연결
4. Depth 1m 차단, Web 인터록, E-stop이 Nav2 경로 실행 중에도 우선 적용되는지 확인
5. 주행 Goal은 바퀴를 구동하지 않는 경로 계산 모드에서 먼저 검증
