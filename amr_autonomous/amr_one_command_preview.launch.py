#!/usr/bin/env python3
"""Single-command real AMR LiDAR/ICP/Map/AMCL/Nav2/RViz/phone preview launch.

IMPORTANT: PREVIEW-ONLY. Does not start CAN, motor WebSocket bridge,
NavigateToPose goals, arm/authorization, lift actuation, or robot movement.

Run after sourcing ROS2 Jazzy and ~/real_amr/install/setup.bash:
  ros2 launch ~/hana-amr-project-autonomous/amr_autonomous/amr_one_command_preview.launch.py

Do NOT run alongside the old individual nodes/phone_preview.launch.py.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

HOME = Path.home()
APP = HOME / "hana-amr-project-autonomous" / "amr_autonomous"
STATIC = HOME / "amr_websocket"
MAP = HOME / ".ros/real_amr/mapex_maps/real_mapex_20261028_162015.yaml"


def process(name, cmd, cwd=None, condition=None):
    return ExecuteProcess(
        cmd=cmd, name=name, cwd=str(cwd) if cwd else None,
        output="screen", emulate_tty=True, condition=condition,
        sigterm_timeout="5", sigkill_timeout="3")


def generate_launch_description():
    for path in (MAP, APP / "map_preview_server.py",
                 APP / "scripts" / "start_map_amcl_lifecycle.py",
                 APP / "scripts" / "mode_mux_dry_run.py",
                 APP / "scripts" / "check_activate_nav2.py",
                 APP / "scripts" / "wait_for_amcl_tf.py",
                 STATIC / "index.html"):
        if not path.is_file():
            raise FileNotFoundError(f"[AMR] required file missing: {path}")

    rviz_enabled = LaunchConfiguration("rviz")
    rpm_enabled = LaunchConfiguration("rpm_diagnostic")
    map_file = LaunchConfiguration("map_file")

    lidar = process("dual_sick_lidar", [
        "ros2", "launch", "real_amr_lidar", "real_amr_sensors.launch.py",
        "start_qr:=false", "publish_qr_tf:=false", "rviz:=false"])
    scan_merge = process("dual_lidar_scan_merge", [
        "ros2", "run", "real_amr_lidar", "real_dual_lidar_scan_merge",
        "--ros-args", "-r", "/scan_slam:=/scan_slam"])
    icp = process("lidar_icp_odometry", [
        "ros2", "run", "real_amr_lidar", "real_lidar_icp_odometry",
        "--ros-args", "-p", "scan_topic:=/scan_slam"])
    map_server = process("real_map_server", [
        "ros2", "run", "nav2_map_server", "map_server",
        "--ros-args", "-p", ["yaml_filename:=", map_file]])
    amcl = process("amcl_localization", [
        "ros2", "run", "nav2_amcl", "amcl",
        "--ros-args", "-p", "use_sim_time:=false",
        "-p", "global_frame_id:=map", "-p", "odom_frame_id:=odom",
        "-p", "base_frame_id:=base_link", "-p", "tf_broadcast:=true",
        "-r", "/scan:=/scan_slam"])
    activate = process("activate_map_and_amcl", [
        "/usr/bin/python3", "-u",
        str(APP / "scripts" / "start_map_amcl_lifecycle.py"),
        "--timeout", "60"])
    wait_localization = process("wait_for_amcl_localization", [
        "/usr/bin/python3", "-u", str(APP / "scripts" / "wait_for_amcl_tf.py"),
        "--timeout", "600", "--max-age", "2"])
    nav2 = process("mapex_nav2", [
        "ros2", "launch", "real_amr_lidar",
        "real_mapex_nav2.launch.py", "autostart:=true"])
    nav2_verify = process("verify_nav2_lifecycle", [
        "/usr/bin/python3", "-u",
        str(APP / "scripts" / "check_activate_nav2.py"),
        "--timeout", "90", "--autostart-grace", "8"])
    rpm_converter = process("auto_rpm_request_diagnostic", [
        "ros2", "run", "real_amr_lidar", "real_mapex_cmd_vel_to_rpm"],
        condition=IfCondition(rpm_enabled))

    rviz_config = HOME / ".rviz2/default.rviz"
    rviz_cmd = ["rviz2"]
    if rviz_config.is_file():
        rviz_cmd += ["-d", str(rviz_config)]
    rviz = process("amr_rviz", rviz_cmd,
                   condition=IfCondition(rviz_enabled))
    phone = process("phone_dashboard_http_8000", [
        "/usr/bin/python3", "-u", "-m", "http.server", "8000",
        "--bind", "0.0.0.0"], cwd=STATIC)
    preview = process("map_preview_http_8771", [
        "/usr/bin/python3", "-u", str(APP / "map_preview_server.py"),
        "--bind", "0.0.0.0", "--port", "8771"], cwd=APP)
    auto_rpm = process("auto_user_rpm_limiter", [
        "/usr/bin/python3", "-u",
        str(APP / "scripts/auto_rpm_limiter.py")], cwd=APP)
    mux = process("mode_mux_read_only", [
        "/usr/bin/python3", "-u",
        str(APP / "scripts/mode_mux_dry_run.py")], cwd=APP)

    def after_lifecycle(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg="[AMR] Map/AMCL activation failed. Preview stopping."),
                EmitEvent(event=Shutdown(reason="Map/AMCL activation failed")),
            ]
        return [
            LogInfo(msg="[AMR] Map/AMCL ACTIVE. Starting RViz and web BEFORE Nav2."),
            rviz, phone, preview, auto_rpm, mux,
            LogInfo(msg="[AMR] RViz: Set the REAL robot pose using 2D Pose Estimate."),
            LogInfo(msg="[AMR] Web remains PREVIEW ONLY; no CAN or motor."),
            wait_localization,
        ]

    def after_localization(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg="[AMR] AMCL localization not ready. No Nav2/CAN; shutting down."),
                EmitEvent(event=Shutdown(reason="AMCL localization unavailable")),
            ]
        return [
            LogInfo(msg="[AMR] Fresh map->odom TF confirmed. Starting Nav2."),
            nav2,
            TimerAction(period=3.0, actions=[nav2_verify]),
        ]

    def after_nav2_verify(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg="[AMR] Nav2 activation failed; preview stopping."),
                EmitEvent(event=Shutdown(reason="Nav2 lifecycle not ready")),
            ]
        return [
            LogInfo(msg="[AMR] NAV2 READY. Starting request-only RPM diagnostic."),
            rpm_converter,
            LogInfo(msg="[AMR] PREVIEW ONLY - CAN/ARM/LIFT NOT STARTED"),
        ]

    return LaunchDescription([
        DeclareLaunchArgument("map_file", default_value=str(MAP),
                              description="Existing saved occupancy map YAML"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Start rviz2 with ~/.rviz2/default.rviz if available"),
        DeclareLaunchArgument("rpm_diagnostic", default_value="true",
                              description="Convert Nav2 cmd_vel to request-only RPM; NEVER to CAN"),
        SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "31"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("OPENBLAS_NUM_THREADS", "1"),
        SetEnvironmentVariable("OMP_NUM_THREADS", "1"),
        LogInfo(msg="[AMR] One command preview startup; do not run duplicate nodes"),
        lidar,
        TimerAction(period=2.0, actions=[scan_merge]),
        TimerAction(period=4.0, actions=[icp]),
        TimerAction(period=5.0, actions=[map_server]),
        TimerAction(period=6.0, actions=[amcl]),
        TimerAction(period=9.0, actions=[activate]),
        RegisterEventHandler(OnProcessExit(
            target_action=activate, on_exit=after_lifecycle)),
        RegisterEventHandler(OnProcessExit(
            target_action=wait_localization, on_exit=after_localization)),
        RegisterEventHandler(OnProcessExit(
            target_action=nav2_verify, on_exit=after_nav2_verify)),
    ])
