# 한 번에 실행하는 AMR 지도·자율주행 경로 미리보기 (2026-11-06)

## 용도

Ubuntu 24.04, ROS2 Jazzy, `~/real_amr` / `~/hana-amr-project-autonomous`, ROS_DOMAIN_ID=31. 기존 저장 지도 `~/.ros/real_amr/mapex_maps/real_mapex_20261028_162015.yaml` 기반.

다음 프로세스가 하나의 launch로 실행됩니다:
- `real_amr_sensors.launch.py start_qr:=false publish_qr_tf:=false rviz:=false` (SICK LiDAR 2대)
- `real_dual_lidar_scan_merge` (`/scan_slam`)
- `real_lidar_icp_odometry` (odom → base_link)
- `nav2_map_server map_server` + 라이프사이클 자동 configure/activate
- `nav2_amcl amcl` + 라이프사이클 자동 configure/activate
- `real_mapex_nav2.launch.py autostart:=true`
- `rviz2` (기존 `~/.rviz2/default.rviz`가 있으면 재사용)
- `real_mapex_cmd_vel_to_rpm` (**요청 RPM 진단만**, CAN 미연결)
- `mode_mux_dry_run.py` (read-only)
- `http.server :8000`에서 기존 휴대폰 UI
- `map_preview_server.py :8771`, 웹 목표 → Nav2 ComputePathToPose 경로 계산 → RViz Path/Marker 토픽

**중요:** CAN 모터 드라이버, 기존 실제 웹 RPM 브리지, 리프트 구동, NavigateToPose 주행 목표, 수동주행 ARM을 실행하지 않습니다. 휴대폰 웹의 기존 카메라·리프트·모터 정보는 각각의 별도 서버가 실행 중이지 않으면 연결되지 않을 수 있습니다.

## 1. 시작

기존에 수동으로 띄운 개별 센서·ICP·Map Server·AMCL·Nav2·RViz 및 `:8000`, `:8771` 서버, 진단 노드를 종료한 후 **새 터미널 하나**에서:

```bash
cd ~/hana-amr-project-autonomous
git pull --ff-only
bash ~/hana-amr-project-autonomous/amr_autonomous/run_all.sh
```

시작 시 기존 `:8000` 또는 `:8771` 포트가 열려 있으면 중복 실행을 방지하기 위해 중단합니다.

`run_all.sh`는 자동으로 ROS2 환경을 source하고, `amr_one_command_preview.launch.py`를 실행합니다. 종료는 해당 터미널에서 Ctrl+C.

직접 실행하고 싶으면:

```bash
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=31 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ~/real_amr/install/setup.bash
ros2 launch ~/hana-amr-project-autonomous/amr_autonomous/amr_one_command_preview.launch.py
```

## 2. 접속

- IPC: `http://127.0.0.1:8000/`
- 휴대폰: `http://<IPC_LAN_IP>:8000/` (기존 설정에서 흔히 사용한 주소 예: `http://192.168.0.29:8000/`; 실제 IP는 `hostname -I`로 확인)
- 관제 페이지 안에 지도 iframe 자동 표시 (지도 서버는 같은 IPC의 8771 포트).
- 두 포트는 신뢰할 수 있는 사설망에서만 접근되도록 네트워크 방화벽을 설정하세요. 테스트 서버에는 인증이 없고, 이 페이지는 기존 모터 제어 UI도 포함합니다. 네트워크를 인터넷에 노출하지 마세요.

## 3. 확인

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 topic info /scan_slam -v
ros2 topic info /amcl_pose -v
ros2 topic info /amr_web/preview_path -v
curl -s http://127.0.0.1:8771/api/status | python3 -m json.tool
```

기대: map_server, amcl, nav2 lifecycle active; 지도 연결. 화면에서 RViz **2D Pose Estimate**로 초기 위치를 지정하고, 휴대폰 웹 지도에서 목적지를 선택해 경로를 계산합니다. 초기 위치는 곧바로 검증된 실측 위치가 아니므로 실제 주행에 사용하지 마세요.

## 4. 선택 인자

```bash
bash ~/hana-amr-project-autonomous/amr_autonomous/run_all.sh rviz:=false
bash ~/hana-amr-project-autonomous/amr_autonomous/run_all.sh rpm_diagnostic:=false
```

실제 로봇 제어는 별도 검증된 운용 Launch에서만 진행합니다. `amr_all_with_lift.launch.py`는 CAN 드라이버를 초기화하므로 본 미리보기 Launch와 함께 실행하지 않습니다.

## 5. 문제 발생 시

- 포트 사용 중: `ss -lntp | grep -E ':8000|:8771'`; 기존 서버만 종료.
- map/amcl activate 실패: Launch는 Nav2를 이어서 시작하지 않고 종료합니다. `/scan_slam`, TF, 지도 YAML, lifecycle 오류를 점검.
- LiDAR launch 인자 오류: `ros2 launch real_amr_lidar real_amr_sensors.launch.py --show-args`
- RViz 경로 미표시: `Fixed Frame=map`, Path `/amr_web/preview_path`, Marker `/amr_web/preview_goal` 확인.
- 표시용으로 계산한 경로는 **실제 CAN 모터 명령이 아닙니다**.
