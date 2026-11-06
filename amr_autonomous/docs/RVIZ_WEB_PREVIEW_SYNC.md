# RViz와 웹 지도 경로 미리보기 동기화 (2026-11-06)

웹에서 목적지 클릭 → Nav2 `ComputePathToPose` → 성공한 경로를 웹과 RViz에 동시에 표시합니다.

**주의: 두 표시 전용 토픽은 실제 자율주행 명령이 아닙니다.** `NavigateToPose`, `/goal_pose`, `/cmd_vel`, CAN 모터, 리프트 명령을 송신하지 않습니다.

## 새 ROS2 토픽

| 토픽 | 메시지 타입 | RViz Display |
|---|---|---|
| `/amr_web/preview_path` | `nav_msgs/msg/Path` | Path |
| `/amr_web/preview_goal` | `visualization_msgs/msg/Marker` | Marker |

`Fixed Frame = map`을 사용합니다. 두 Publisher는 RELIABLE + TRANSIENT_LOCAL QoS로 최근 경로를 유지합니다. RViz의 Path와 Marker에서 수신이 안 되면 Durability를 Transient Local로 설정합니다.

`/amr_web/preview_path`는 Nav2 `ComputePathToPose` 결과를 그대로 발행합니다. `/amr_web/preview_goal`은 지도상 목표 위치와 방향을 빨간 화살표로 표시합니다. RViz 2D Pose Estimate로 출발 위치를 변경하면 지난 미리보기 경로를 지웁니다.

## 실행 순서

먼저 LiDAR/ICP/Map Server/AMCL/Nav2 활성화를 유지합니다.

```bash
cd ~/hana-amr-project-autonomous
git pull --ff-only
```

기존 웹 미리보기 서버만 Ctrl+C로 중단 후 다시 실행:

```bash
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ~/real_amr/install/setup.bash
cd ~/hana-amr-project-autonomous/amr_autonomous
python3 -u map_preview_server.py
```

RViz에서 `Add → By display type → Path`를 추가하고 Topic을 `/amr_web/preview_path`로 지정합니다. 또 `Add → Marker`를 추가하고 Topic을 `/amr_web/preview_goal`로 지정합니다. 기존 RViz '2D Goal Pose'는 클릭하지 마세요. 별도 Nav2 주행 목표가 전송될 수 있습니다.

웹 지도에서 목적지 클릭 시 경로 계산이 자동 실행됩니다. 목표 방향을 수정한 경우에는 `Nav2 경로 미리보기` 버튼을 다시 누릅니다.

토픽 점검:

```bash
ros2 topic info /amr_web/preview_path -v
ros2 topic info /amr_web/preview_goal -v
ros2 topic echo /amr_web/preview_goal --once --qos-durability transient_local
```

만약 계산 실패라면 지도 비점유 셀인지, AMCL 또는 RViz 2D Pose Estimate 위치가 정확한지 확인하세요. 최신 위치가 아닌 상태에서도 현재 기능은 **경로 모양만 보는 미리보기용**이며 주행 승인에 사용할 수 없습니다.
