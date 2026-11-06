#!/usr/bin/env python3
"""Explicit LIVE AMR launch with guarded Nav2 -> mux-only CAN output.

This starts CAN safe_start() which can enable physical motor drives.
Requires independently tested E-stop, drive-side comms-loss stop, and
controlled clear driving space. Do not run alongside preview/legacy launches.
Launch from run_live.sh in a supervised setting.
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
REMOTE = HOME / "hana-amr-project-autonomous" / "amr_remote_client"
CAMERA = HOME / "amr_websocket" / "amr_openvino_camera_depth_integrated.py"
CAMERA_PY = HOME / "yolo_openvino_env" / "bin" / "python"
LIFT = HOME / "amr_lift" / "lift_websocket.py"
LIFT_PY = HOME / "amr_lift" / ".venv" / "bin" / "python"


def process(name, cmd, cwd=None, condition=None):
    return ExecuteProcess(
        cmd=cmd, name=name, cwd=str(cwd) if cwd else None,
        output="screen", emulate_tty=True, condition=condition,
        sigterm_timeout="5", sigkill_timeout="3")


def generate_launch_description():
    for path in (MAP, APP / "map_preview_server.py",
                 APP / "scripts" / "start_map_amcl_lifecycle.py",
                 APP / "scripts" / "mode_mux_dry_run.py",
                 STATIC / "index.html",
                 REMOTE / "amr_ros_wheel_rpm_can.py",
                 REMOTE / "amr_ws_to_ros.py",
                 APP / "scripts" / "guarded_motor_mux.py",
                 CAMERA, CAMERA_PY, LIFT, LIFT_PY):
        if not path.is_file():
            raise FileNotFoundError(f"[AMR] required file missing: {path}")

    rviz_enabled = LaunchConfiguration("rviz")
    manual_motion = LaunchConfiguration("manual_motion")
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
    nav2 = process("mapex_nav2", [
        "ros2", "launch", "real_amr_lidar",
        "real_mapex_nav2.launch.py", "autostart:=true"])
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
    preview = process("auto_map_http_8771", [
        "/usr/bin/python3", "-u", str(APP / "map_preview_server.py"),
        "--bind", "0.0.0.0", "--port", "8771",
        "--allow-physical-auto"], cwd=APP)
    auto_rpm = process("auto_user_rpm_limiter", [
        "/usr/bin/python3", "-u",
        str(APP / "scripts/auto_rpm_limiter.py")], cwd=APP)
    mux = process("motor_mux_live", [
        "/usr/bin/python3", "-u",
        str(APP / "scripts/guarded_motor_mux.py"),
        "--live-motor-output"], cwd=APP)
    can = process("can_mux_driver", [
        "/usr/bin/python3", "-u",
        str(REMOTE / "amr_ros_wheel_rpm_can.py"),
        "--mux-input"], cwd=REMOTE)
    camera = process("openvino_depth", [
        str(CAMERA_PY), "-u", str(CAMERA)], cwd=CAMERA.parent)
    lift = process("lift_state_websocket", [
        str(LIFT_PY), "-u", str(LIFT)], cwd=LIFT.parent)
    manual = process("manual_web_private_bridge", [
        "/usr/bin/python3", "-u", str(REMOTE / "amr_ws_to_ros.py"),
        "--private-output", "--bench-motion"],
        cwd=REMOTE, condition=IfCondition(manual_motion))

    def after_lifecycle(event, context):
        if event.returncode != 0:
            return [
                LogInfo(msg="[AMR] map/amcl lifecycle activation FAILED; shutting down"),
                EmitEvent(event=Shutdown(reason="map or AMCL failed to activate"))
            ]
        return [
            LogInfo(msg="[AMR] Map Server + AMCL ACTIVE. Starting Nav2/RViz/phone map."),
            nav2,
            TimerAction(period=2.0, actions=[rpm_converter, rviz, phone, preview, auto_rpm, mux, camera, lift]),
            TimerAction(period=5.0, actions=[can, manual]),
            LogInfo(msg="[AMR] phone: http://<IPC_LAN_IP>:8000/ and embedded map :8771"),
            LogInfo(msg="[AMR] LIVE CAN ENABLED: default AUTO 30 RPM / requested up to 1000 RPM"),
            RegisterEventHandler(OnProcessExit(
                target_action=can,
                on_exit=[EmitEvent(event=Shutdown(reason="CAN driver stopped"))])),
            RegisterEventHandler(OnProcessExit(
                target_action=mux,
                on_exit=[EmitEvent(event=Shutdown(reason="motor mux stopped"))])),
            RegisterEventHandler(OnProcessExit(
                target_action=preview,
                on_exit=[EmitEvent(event=Shutdown(reason="AUTO web server stopped"))])),
        ]

    return LaunchDescription([
        DeclareLaunchArgument("manual_motion", default_value="false",
                              description="Explicit opt-in for existing phone MANUAL RPM via private bridge"),
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
        LogInfo(msg="[AMR] LIVE physical CAN startup — supervised E-stop mandatory; do not run duplicate nodes"),
        lidar,
        TimerAction(period=2.0, actions=[scan_merge]),
        TimerAction(period=4.0, actions=[icp]),
        TimerAction(period=5.0, actions=[map_server]),
        TimerAction(period=6.0, actions=[amcl]),
        TimerAction(period=9.0, actions=[activate]),
        RegisterEventHandler(OnProcessExit(
            target_action=activate, on_exit=after_lifecycle)),
    ])
