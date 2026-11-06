#!/usr/bin/env python3
"""Phone dashboard + ROS2 map/path preview ONLY (no CAN/drive/lift start).

Usage:
  ros2 launch ~/hana-amr-project-autonomous/amr_autonomous/phone_preview.launch.py

Ensure Map Server, AMCL and Nav2 are already active if path planning is wanted.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, SetEnvironmentVariable

HOME = Path.home()
APP = HOME / "hana-amr-project-autonomous" / "amr_autonomous"
STATIC = HOME / "amr_websocket"


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "31"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        LogInfo(msg="[PHONE PREVIEW] http://<AMR_IP>:8000/ | Map :8771 | NO CAN/ARM/MOTION"),
        ExecuteProcess(
            cmd=["/usr/bin/python3", "-u", "-m", "http.server", "8000",
                 "--bind", "0.0.0.0"],
            cwd=str(STATIC), name="amr_phone_dashboard", output="screen"),
        ExecuteProcess(
            cmd=["/usr/bin/python3", "-u", str(APP / "map_preview_server.py"),
                 "--bind", "0.0.0.0", "--port", "8771"],
            cwd=str(APP), name="amr_map_preview", output="screen"),
        ExecuteProcess(
            cmd=["/usr/bin/python3", "-u",
                 str(APP / "scripts" / "mode_mux_dry_run.py")],
            cwd=str(APP), name="amr_mode_dry_run", output="screen"),
    ])
