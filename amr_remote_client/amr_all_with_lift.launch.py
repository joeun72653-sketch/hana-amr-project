#!/usr/bin/env python3
"""ROS 2 Jazzy: CAN motor + no-ramp bridge + OpenVINO/Depth + HTTP server.

Default: diagnostic mode, nonzero Web RPM commands disabled.
Lift control and S-Curve are intentionally NOT launched.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess

HOME = Path.home()
REMOTE = HOME / 'amr_remote_client'
CAMERA = HOME / 'amr_websocket'
SYSTEM_PYTHON = '/usr/bin/python3'
CAMERA_PYTHON = str(HOME / 'yolo_openvino_env' / 'bin' / 'python')
LIFT = HOME / 'amr_lift'
AUTONOMOUS = HOME / 'hana-amr-project-autonomous' / 'amr_autonomous'
LIFT_PYTHON = str(LIFT / '.venv' / 'bin' / 'python')


def make_process(label, python, script, extra=None, condition=None):
    return ExecuteProcess(
        cmd=[python, '-u', str(script)] + (extra or []),
        name=label,
        output='screen',
        emulate_tty=True,
        sigterm_timeout='5',
        sigkill_timeout='3',
        condition=condition,
    )


def generate_launch_description():
    bench_motion = LaunchConfiguration('bench_motion')
    with_map_preview = LaunchConfiguration('with_map_preview')

    can = make_process('can_motor', SYSTEM_PYTHON,
                       REMOTE / 'amr_ros_wheel_rpm_can.py')
    from launch.conditions import IfCondition, UnlessCondition
    # Default is deliberately safe: no nonzero RPM unless explicitly requested.
    bridge_diag = make_process(
        'web_drive_no_ramp', SYSTEM_PYTHON,
        REMOTE / 'amr_ws_to_ros.py',
        condition=UnlessCondition(bench_motion))
    # This is the ONLY bridge selected when bench_motion:=true.
    bridge_bench = make_process(
        'web_drive_no_ramp', SYSTEM_PYTHON,
        REMOTE / 'amr_ws_to_ros.py', ['--bench-motion'],
        condition=IfCondition(bench_motion))

    # Optional, preview-only map and command-state diagnostics.
    # Does NOT issue NavigateToPose or send motor RPM; uses a second HTTP port.
    from launch.conditions import IfCondition
    map_preview = make_process(
        'map_preview', SYSTEM_PYTHON,
        AUTONOMOUS / 'map_preview_server.py',
        ['--bind', '0.0.0.0', '--port', '8771'],
        condition=IfCondition(with_map_preview))
    mux_diagnostic = make_process(
        'mode_mux_dry_run', SYSTEM_PYTHON,
        AUTONOMOUS / 'scripts' / 'mode_mux_dry_run.py',
        condition=IfCondition(with_map_preview))

    camera = make_process('openvino_depth', CAMERA_PYTHON,
                          CAMERA / 'amr_openvino_camera_depth_integrated.py')
    lift = make_process(
        'lift_websocket', LIFT_PYTHON,
        LIFT / 'lift_websocket.py')

    http_server = ExecuteProcess(
        cmd=[SYSTEM_PYTHON, '-u', '-m', 'http.server', '8000', '--bind', '0.0.0.0'],
        cwd=str(CAMERA),
        name='amr_http_server',
        output='screen',
        emulate_tty=True,
        sigterm_timeout='5',
        sigkill_timeout='3',
    )

    actions = [
        DeclareLaunchArgument('with_map_preview', default_value='true',
                              description='Optional embedded phone map and dry-run mode observer, NO AUTO motor control'),
        DeclareLaunchArgument('bench_motion', default_value='false',
                              description='Explicit opt-in: bench_motion:=true enables nonzero wheel RPM'),
        LogInfo(msg=['[AMR] bench_motion=', bench_motion,
                     ' (true enables --bench-motion; false is diagnostic only)']),
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '31'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        can,
        http_server,
        TimerAction(period=2.0, actions=[camera]),
        TimerAction(period=4.0, actions=[bridge_diag, bridge_bench]),
        TimerAction(period=6.0, actions=[lift]),
        TimerAction(period=8.0, actions=[map_preview, mux_diagnostic]),
    ]
    for process, label in ((can, 'CAN motor'), (camera, 'OpenVINO/Depth'),
                           (http_server, 'HTTP server'),
                           (bridge_diag, 'Web drive'), (bridge_bench, 'Web drive'),
                           (map_preview, 'Map preview'), (mux_diagnostic, 'Mode diagnostic')):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason=f'{label} process exited'))],
        )))
    return LaunchDescription(actions)
