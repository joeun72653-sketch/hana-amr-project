# AMR_QR 전체 실행 순서 · 명령어 기록

**기록 기준:** 2026-11-05 / Ubuntu 24.04 / ROS2 Jazzy / `~/real_amr` / DDS CycloneDDS / Domain 31

**출처 구분:** AMR_QR 대화에서 확인된 명령을 우선 수록했습니다. 각 단계의 `--show-args`, `ros2 topic info`, `healthcheck.sh` 등은 재실행·진단을 위해 보완한 명령입니다. 실행 환경과 Launch 구성을 IPC에서 아직 재검증하지 않았으므로 **동시에 모두 실행하는 스크립트가 아닙니다.**

> 하드웨어 E-stop, CAN 드라이브 통신단절 정지, Depth 장애물 차단, ARM/DISARM/리프트 인터록을 우회하지 마세요. Nav2 경로 계산과 실물 바퀴 주행은 별도입니다.

## 0. 공통 환경 설정 — 사용하는 모든 ROS 터미널

```bash
cd ~/real_amr

export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1

source /opt/ros/jazzy/setup.bash
source ~/real_amr/install/setup.bash

echo "$ROS_DOMAIN_ID"
echo "$RMW_IMPLEMENTATION"
echo "$ROS_DISTRO"
ros2 pkg list | grep real_amr
ros2 pkg executables real_amr_lidar
```

편의 스크립트(새 브랜치 checkout 위치에 맞게 경로 수정):

```bash
source /path/to/hana-amr-project/amr_autonomous/scripts/source_ros_env.sh
bash /path/to/hana-amr-project/amr_autonomous/scripts/healthcheck.sh
```

기존 노드·중복 프로세스와 파일 확인:

```bash
pgrep -af 'ros2|rviz2|slam_toolbox|amcl|nav2|map_server'
ros2 node list
ros2 topic list -t
find ~/real_amr/src/real_amr_lidar -maxdepth 3 -type f | sort
```

## 1. 터미널 A — 기존 QR + LiDAR 통합 센서

```bash
cd ~/real_amr
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch real_amr_lidar real_amr_sensor_qr_test.launch.py include_sensors:=true
```

**주의:** 이 Launch 안에서 Merge나 ICP 노드를 띄우는 경우 다음 터미널의 같은 노드를 중복 실행하지 않습니다.

## 2. 터미널 B — QR 데이터 및 통신 확인

```bash
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ~/real_amr/install/setup.bash

ros2 node list
ros2 topic list -t
ros2 topic echo /qr/raw
ros2 topic echo /qr/status
ros2 topic echo /qr/pose
ros2 node info /gls611_pose
```

반복 `echo`는 각 터미널에서 Ctrl+C로 종료하세요. 확인용 단발 명령:

```bash
ros2 topic echo /qr/raw --once
ros2 topic echo /qr/status --once
ros2 topic echo /qr/pose --once
ros2 topic hz /qr/raw
ros2 topic info /qr/pose -v
ss -tnp state established
```

## 3. 터미널 C — 듀얼 LiDAR 병합 (필요할 때만)

```bash
source /opt/ros/jazzy/setup.bash
source ~/real_amr/install/setup.bash
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run real_amr_lidar real_dual_lidar_scan_merge \
  --ros-args \
  -r /scan_slam:=/scan_slam
```

병합된 센서 확인:

```bash
ros2 topic list | grep scan
ros2 topic info /scan_slam -v
ros2 topic hz /scan_slam
ros2 topic echo /scan_slam --once
ros2 topic echo /scan_slam --field header.frame_id --once
```

이전 기록: `/scan_slam`이 통합 LaserScan 토픽이며 해당 프레임은 `base_link`였습니다. 기존 기록에서 관측한 주파수는 약 68 Hz이나 현재 현장값은 측정해서 판단하세요.

## 4. 터미널 D — ICP Odometry (필요할 때만)

```bash
source /opt/ros/jazzy/setup.bash
source ~/real_amr/install/setup.bash
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run real_amr_lidar real_lidar_icp_odometry \
  --ros-args -p scan_topic:=/scan_slam
```

```bash
ros2 topic list | grep odom
ros2 topic echo /lidar_icp/odom --once
ros2 topic hz /lidar_icp/odom
ros2 topic info /lidar_icp/odom -v
ros2 run tf2_ros tf2_echo odom base_link
```

**TF 소유권:** `odom → base_link`는 ICP/Wheel EKF 등 **하나의 소스**만 발행해야 합니다.

## 5A. 새 지도 작성이 필요한 경우 — SLAM

아래는 지도 생성 단계이며, **기존 지도로 주행할 경우 5B로 이동**합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/real_amr/install/setup.bash
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 실제 지원 인자 확인
ros2 launch real_amr_lidar real_amr_slam_mapping.launch.py --show-args
ros2 launch real_amr_lidar real_mapex_autonomous_mapping.launch.py --show-args
```

별도 SLAM Launch 사용 예시(통합 실행과 중복하지 않음):

```bash
ros2 launch real_amr_lidar real_amr_slam_mapping.launch.py
```

RViz:

```bash
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ~/real_amr/install/setup.bash
rviz2
```

RViz 설정: `Fixed Frame=map`, `Map=/map` (Transient Local), `LaserScan=/scan_slam`, `TF=On`. **수동 주행 키보드 명령 원본은 이번 기록에서 확인되지 않았으므로 임의로 추가하지 않았습니다.**

지도 생성 여부와 저장(재실행 보완 명령):

```bash
ros2 topic list | grep map
ros2 topic info /map -v
ros2 run tf2_ros tf2_echo map odom

mkdir -p ~/amr_maps
ros2 run nav2_map_server map_saver_cli -f ~/amr_maps/amr_map_01
ls -lh ~/amr_maps/amr_map_01.*
```

11월 2일경 이전에 저장한 지도의 예: `~/amr_maps/amr_map_01.yaml`.

## 5B. 기존 지도로 자율주행할 경우 — Map Server

기존 AMR_QR 복구 과정에서 사용한 지도:
`~/.ros/real_amr/mapex_maps/real_mapex_20261028_162015.yaml`

**터미널 E — Map Server:**

```bash
cd ~/real_amr
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ls -lh ~/.ros/real_amr/mapex_maps/
cat ~/.ros/real_amr/mapex_maps/real_mapex_20261028_162015.yaml

ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:=$HOME/.ros/real_amr/mapex_maps/real_mapex_20261028_162015.yaml
```

**터미널 F — Map Server Lifecycle:**

```bash
export ROS_DOMAIN_ID=31
source /opt/ros/jazzy/setup.bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle get /map_server
ros2 topic info /map -v
ros2 topic echo /map --qos-durability transient_local --once
```

출력 상태 기대값: `active [3]`. Nav2 통합 Launch가 이미 Map Server를 포함한다면 수동 Map Server를 별도로 띄우지 않습니다.

## 6. AMCL 위치 추정

**터미널 G — AMCL:**

```bash
cd ~/real_amr
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run nav2_amcl amcl \
  --ros-args \
  -p use_sim_time:=false \
  -p global_frame_id:=map \
  -p odom_frame_id:=odom \
  -p base_frame_id:=base_link \
  -p tf_broadcast:=true \
  -r /scan:=/scan_slam
```

**터미널 H — Lifecycle + 위치 확인:**

```bash
export ROS_DOMAIN_ID=31
source /opt/ros/jazzy/setup.bash
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
ros2 lifecycle get /amcl
ros2 topic echo /amcl_pose --once
ros2 topic info /amcl_pose -v
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
```

RViz에서 **2D Pose Estimate**로 AMR 실제 위치/방향 초기화. Map→Odom TF 발행자는 SLAM과 AMCL을 동시에 사용하지 말고 하나만 선택합니다. 기존 AMR_QR 기록에서는 AMCL Active 및 `/amcl_pose` 확인까지 진행했습니다.

## 7. Nav2 실행

**터미널 I — Nav2:**

```bash
cd ~/real_amr
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch real_amr_lidar real_mapex_nav2.launch.py autostart:=true
```

**터미널 J — 상태 검사:**

```bash
export ROS_DOMAIN_ID=31
source /opt/ros/jazzy/setup.bash
source ~/real_amr/install/setup.bash

ros2 node list | grep -E 'planner_server|controller_server|bt_navigator|behavior_server|lifecycle_manager'
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /behavior_server
ros2 action list -t
ros2 topic list | grep costmap
ros2 topic info /global_costmap/costmap -v
ros2 topic info /local_costmap/costmap -v
ros2 topic list | grep plan
```

### 경로만 계산하는 테스트 (실제 바퀴 주행 아님)

기존 기록에서 `ComputePathToPose`가 `SUCCEEDED`를 반환했던 형태입니다. 좌표 `(0,0)`가 현재 지도에서 유효한지는 RViz에서 먼저 확인하세요.

```bash
ros2 action send_goal \
  /compute_path_to_pose \
  nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, use_start: false}" \
  --feedback
```

이전 결과 기록: `SUCCEEDED`, `error_code: 0`, `Path: 10 poses`.

## 8. Nav2 ↔ CAN 연동 상태만 확인 (비주행)

```bash
ros2 topic list | grep cmd_vel
ros2 topic type /mapex/cmd_vel
ros2 topic info /mapex/cmd_vel -v
ros2 node list | grep -E 'cmd_vel|rpm|motor|can'

ros2 topic list | grep rpm
ros2 topic info /mapex/wheel_rpm_left_requested -v
ros2 topic info /mapex/wheel_rpm_right_requested -v

ros2 topic echo /amr_motor_state --once
ros2 topic echo /amr_motion_authorized --once

pgrep -af 'amr_ws_to_ros|amr_ros_wheel_rpm_can|one_command_launch'
```

기존 AMR_QR 기록의 `/mapex/cmd_vel`: `geometry_msgs/msg/TwistStamped`, **Publisher 5 / Subscriber 0**. 이는 이전 측정값이며 현재 상황을 재확인해야 합니다. 0 subscriber라면 Nav2가 계획한 속도가 실제 모터로 전달되지 않습니다.

원본 코드/Launch 연결 점검:

```bash
sed -n '1,195p' ~/real_amr/src/real_amr_lidar/launch/real_mapex_nav2.launch.py

grep -Rn -A 32 -B 12 'real_mapex_cmd_vel_to_rpm' \
  ~/real_amr/src/real_amr_lidar/launch/real_mapex_autonomous_mapping.launch.py

sed -n '325,365p' \
  ~/real_amr/src/real_amr_lidar/launch/real_mapex_autonomous_mapping.launch.py

find ~/real_amr/src -iname '*cmd_vel*' -o -iname '*rpm*'
grep -RnE 'class .*CmdVel|cmd_vel_topic|wheel_rpm_left|wheel_rpm_right|lidar_safty_bitmask|motion_authorized' \
  ~/real_amr/src/real_amr_lidar/scripts --include='*.py' | head -100
```

실제 IPC 코드와 `geometry_msgs/TwistStamped` 유형의 일치, 속도·회전 부호, Subscriber, 0 명령, timeout, Depth 차단, E-stop 우선순위 등을 확인한 뒤 단계별 주행 검증을 진행하세요. `ros2 topic pub`를 이용한 비영(非零) RPM 직접 발행은 이 문서에 넣지 않았습니다.

## 9. 기존 통합 CAN/Web/Depth/Lift 코드 — 별도 참고

이 브랜치는 최근 통합 브랜치에서 생성했으므로 다음 코드가 이미 포함되어 있습니다.

- `amr_remote_client/amr_all_with_lift.launch.py`
- `amr_remote_client/amr_ros_wheel_rpm_can.py`
- `amr_remote_client/amr_ws_to_ros.py`
- `amr_websocket/amr_openvino_camera_depth.py`
- `amr_lift/lift_websocket.py`
- `amr_motor_control/amr_enable_motors.py`

IPC에 동일 파일이 있는지, 최신 소스인지 확인하고 **선택적으로** 별도 진단 모드로 실행할 수 있습니다.

```bash
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.bash

# 반드시 사용 중인 실제 IPC 파일과 차이가 없는지 확인
ls -l ~/amr_remote_client/amr_all_with_lift.launch.py

# 이 Launch의 기본 bench_motion:=false는 진단 모드 (GitHub 파일 기준)
ros2 launch ~/amr_remote_client/amr_all_with_lift.launch.py bench_motion:=false
```

통합 Launch가 CAN/Bridge/Web/카메라/리프트를 포함하므로 개별 프로세스와 중복 실행하지 않습니다. 실제 IPC의 다른 버전으로 대체되어 있을 수 있으며, 기본 bench 설정도 실행 전 `--show-args` 및 코드 확인이 필요합니다.

## 10. 최종 확인 명령

```bash
ros2 node list
ros2 topic list -t
ros2 action list -t

ros2 topic info /scan_slam -v
ros2 topic info /lidar_icp/odom -v
ros2 topic info /map -v
ros2 topic info /amcl_pose -v
ros2 topic info /mapex/cmd_vel -v

ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link

ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

자세한 자동 읽기 전용 점검:

```bash
bash /path/to/hana-amr-project/amr_autonomous/scripts/healthcheck.sh

python3 /path/to/hana-amr-project/amr_autonomous/scripts/nav_readiness.py \
  --timeout 10 --require scan --require odom --require map --require amcl
```

이 명령으로 데이터 수신을 검사하더라도 실제 물리적 주행 가능·안전이 보장되지는 않습니다.
