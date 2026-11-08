# AMR TurtleBot-style navigation baseline (ROS2 Jazzy)

This is a separate, non-actuating navigation baseline using the **existing saved map**:

`~/.ros/real_amr/mapex_maps/real_mapex_20261028_162015.yaml`

## Start

```bash
cd ~/hana-amr-project-autonomous
bash amr_turtlebot_style/run_navigation.sh
```

The launcher reuses the existing dual SICK LiDAR scan merger, ICP odometry, Map Server, AMCL, Nav2 and RViz launch graph. It deliberately does **not** launch CAN or authorize wheel motion. Do not run it simultaneously with the old run_all.sh (ports 8000/8771 and ROS node duplication).

By default it expects `~/.config/cyclonedds/amr_local.xml` to bind CycloneDDS to loopback and reduce office-network traffic. All cooperating ROS2 processes must use matching DDS configuration. If the system relies on remote ROS2 nodes, explicitly use `AMR_LOCAL_DDS=0` with the validated existing DDS network configuration instead.

## RViz workflow

1. Verify `map` fixed frame and loaded map.
2. Set initial localization with **2D Pose Estimate**.
3. Display `/plan` and `/amcl_pose`.
4. Set **2D Goal Pose** topic to `/goal_pose` for native Nav2 behavior (do not use the bench bridge's `/amr_rviz/goal_pose`).
5. Observe planned paths and `/mapex/cmd_vel`; no physical motion is enabled in this baseline.

## Next integration gate

Before physical floor navigation, validate wheel-encoder odometry and TF `odom -> base_link`, positive wheel RPM directions, real position/yaw feedback, and the front/rear/turn swept-area safety coverage. Do not force forward RPM when Nav2 commands rotation. Keep CAN watchdog, hardware E-stop, lift interlock, and depth safety intact.

The current existing Nav2 config uses `custom_amr_controller::StabilizedRotationShimController`. This baseline does not claim to fix repeated in-place rotation; that needs validated odometry and controller tuning before physical autonomous movement.
