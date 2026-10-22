#!/usr/bin/env python3
"""Start and supervise the AMR-PC services from one terminal.

This launcher does not issue nonzero wheel commands, ARM, or calibration START.
The CAN worker owns its existing drive initialization and stop verification.
If a managed service fails, only processes started by this launcher are stopped.

Place this file in the root of hana-amr-project, then run:
    python3 one_command_launch.py --dry-run
    python3 one_command_launch.py
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import urlopen


ROOT = Path(__file__).resolve().parent
CAL_FLAG = Path.home() / '.amr_calibration_active'
CAMERA_MODEL = Path.home() / 'amr_openvino/models/epoch22_openvino_model'
CAN_SCRIPT = ROOT / 'amr_remote_client/amr_ros_wheel_rpm_can.py'
SCRIPTS = {
    'imu_rpy': ROOT / 'amr_imu/imu_rpy_node.py',
    'imu_ws': ROOT / 'amr_imu/amr_imu_ws.py',
    'camera': ROOT / 'amr_websocket/amr_openvino_camera_depth.py',
    'calibration': ROOT / 'amr_websocket/amr_calibration_manager.py',
    'can': CAN_SCRIPT,
    'bridge': ROOT / 'amr_remote_client/amr_ws_to_ros.py',
    'rpm_viewer': ROOT / 'amr_tools/actual_rpm_viewer.py',
    'encoder_viewer': ROOT / 'amr_tools/encoder_echo.py',
}


class LaunchError(RuntimeError):
    pass


@dataclass
class Child:
    name: str
    proc: subprocess.Popen
    log_path: Path
    ready_markers: tuple[str, ...] = ()
    fatal_markers: tuple[str, ...] = ()
    ready: threading.Event = field(default_factory=threading.Event)
    last_lines: list[str] = field(default_factory=list)
    fatal: str | None = None


def port_open(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.3):
            return True
    except OSError:
        return False


def http_get(url: str, timeout: float = 0.7) -> tuple[int, bytes] | None:
    try:
        with urlopen(url, timeout=timeout) as response:
            return response.status, response.read(8192)
    except (OSError, HTTPError, URLError, TimeoutError):
        return None


def source_ros_environment() -> dict[str, str]:
    base = Path('/opt/ros/jazzy/setup.bash')
    if not base.is_file():
        raise LaunchError(f'ROS 2 Jazzy setup missing: {base}')
    overlays = (
        Path.home() / 'custom_amr_ws/install/setup.bash',
        Path.home() / 'real_amr/install/setup.bash',
    )
    commands = ['set -e', f'source {shlex.quote(str(base))}']
    commands.extend(f'source {shlex.quote(str(path))}' for path in overlays if path.is_file())
    commands.extend(["printf '__AMR_ENV_START__\\0'", 'env -0'])
    result = subprocess.run(
        ['bash', '-c', '\n'.join(commands)], capture_output=True, check=False, timeout=15,
    )
    if result.returncode:
        raise LaunchError('Cannot source ROS environment: '
                          + result.stderr.decode(errors='replace')[-1800:])
    marker = b'__AMR_ENV_START__\0'
    if marker not in result.stdout:
        raise LaunchError('ROS environment output was malformed')
    _, raw = result.stdout.rsplit(marker, 1)
    env = {}
    for item in raw.split(b'\0'):
        if b'=' in item:
            key, value = item.split(b'=', 1)
            env[key.decode(errors='surrogateescape')] = value.decode(errors='surrogateescape')
    env['ROS_DOMAIN_ID'] = '31'
    env['PYTHONUNBUFFERED'] = '1'
    env['RCUTILS_COLORIZED_OUTPUT'] = '0'
    env['RCUTILS_LOGGING_BUFFERED_STREAM'] = '0'
    motor_dir = str(ROOT / 'amr_motor_control')
    env['PYTHONPATH'] = motor_dir + os.pathsep + env.get('PYTHONPATH', '')
    return env


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_can_import_source() -> None:
    """Current CAN code prepends ~/amr_motor_control to sys.path at runtime."""
    repo_helper = ROOT / 'amr_motor_control/amr_keyboard_safe.py'
    home_helper = Path.home() / 'amr_motor_control/amr_keyboard_safe.py'
    if not repo_helper.is_file():
        raise LaunchError(f'CAN helper missing: {repo_helper}')
    if home_helper.is_file() and sha256(home_helper) != sha256(repo_helper):
        raise LaunchError(
            'CAN imports ~/amr_motor_control/amr_keyboard_safe.py, which differs from '
            'the Git repository copy. In amr_ros_wheel_rpm_can.py, replace its '
            'sys.path.insert(...) argument with '\
            'str(Path(__file__).resolve().parent.parent / "amr_motor_control") '
            'and recheck the diff before launching.'
        )


def validate_inputs(args: argparse.Namespace) -> None:
    required = ['can']
    if not args.skip_bridge:
        required.append('bridge')
    if not args.skip_calibration:
        required.append('calibration')
    if not args.skip_imu:
        required += ['imu_rpy', 'imu_ws']
    if not args.skip_camera:
        required.append('camera')
    if args.with_viewers:
        required += ['rpm_viewer', 'encoder_viewer']
    for name in required:
        if not SCRIPTS[name].is_file():
            raise LaunchError(f'{name} script missing: {SCRIPTS[name]}')
    if not (ROOT / 'amr_websocket/index.html').is_file():
        raise LaunchError('Web UI index.html missing')
    if not (ROOT / 'amr_imu/config/microstrain.yml').is_file() and not args.skip_imu:
        raise LaunchError('MicroStrain config missing')
    if not args.skip_imu and not args.external_imu_driver:
        config_text = (ROOT / 'amr_imu/config/microstrain.yml').read_text(encoding='utf-8')
        port_match = re.search(r'^\s+port:\s*[\'\"]?([^\'\"\s#]+)', config_text, re.M)
        if not port_match:
            raise LaunchError('MicroStrain config has no port parameter')
        imu_port = Path(port_match.group(1))
        if not imu_port.exists():
            raise LaunchError(f'MicroStrain serial device is missing: {imu_port}')
        if not os.access(imu_port, os.R_OK | os.W_OK):
            raise LaunchError(f'MicroStrain serial device is not readable/writable: {imu_port}. '
                              'Check dialout/group permissions and reconnect the device.')
    if CAL_FLAG.exists():
        raise LaunchError(f'Calibration flag already exists: {CAL_FLAG}. '
                          'Check whether calibration is still running before clearing it.')
    if not args.skip_camera and not CAMERA_MODEL.is_dir():
        raise LaunchError(f'OpenVINO model missing: {CAMERA_MODEL}')
    verify_can_import_source()
    can_source = CAN_SCRIPT.read_text(encoding='utf-8')
    match = re.search(r"^CAN_PORT\s*=\s*['\"]([^'\"]+)['\"]", can_source, re.M)
    if not match or not Path(match.group(1)).exists():
        raise LaunchError(f'CAN serial device missing: {match.group(1) if match else "CAN_PORT"}')


class Supervisor:
    def __init__(self, env: dict[str, str], log_dir: Path):
        self.env = env
        self.log_dir = log_dir
        self.children: list[Child] = []
        self.stopping = False
        self.interrupted = False

    def request_stop(self, signum, frame):
        self.interrupted = True

    def _log_reader(self, child: Child) -> None:
        assert child.proc.stdout is not None
        with child.log_path.open('w', encoding='utf-8') as output:
            for raw in child.proc.stdout:
                line = raw.rstrip('\n')
                output.write(line + '\n')
                output.flush()
                child.last_lines.append(line)
                if len(child.last_lines) > 45:
                    child.last_lines.pop(0)
                print(f'[{child.name}] {line}', flush=True)
                if any(marker in line for marker in child.ready_markers):
                    child.ready.set()
                if any(marker in line for marker in child.fatal_markers):
                    child.fatal = line

    def start(self, name: str, argv: list[str], *, ready_markers=(), fatal_markers=()) -> Child:
        print(f'\n[START] {name}: {shlex.join(argv)}', flush=True)
        try:
            process = subprocess.Popen(
                argv, cwd=ROOT, env=self.env, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                errors='replace', bufsize=1, start_new_session=True,
            )
        except OSError as exc:
            raise LaunchError(f'Could not start {name}: {exc}') from exc
        child = Child(name, process, self.log_dir / f'{name}.log',
                      tuple(ready_markers), tuple(fatal_markers))
        self.children.append(child)
        threading.Thread(target=self._log_reader, args=(child,), daemon=True).start()
        return child

    def check(self) -> None:
        if self.interrupted:
            raise KeyboardInterrupt
        for child in self.children:
            if child.fatal:
                raise LaunchError(f'{child.name} reported: {child.fatal}')
            status = child.proc.poll()
            if status is not None:
                tail = '\n'.join(child.last_lines[-6:])
                raise LaunchError(f'{child.name} exited unexpectedly (code={status})\n{tail}')

    def until(self, name: str, predicate, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check()
            if predicate():
                print(f'[READY] {name}', flush=True)
                return
            time.sleep(0.25)
        self.check()
        raise LaunchError(f'{name} did not become ready in {timeout:g}s')

    def wait_log(self, child: Child, timeout: float):
        self.until(child.name, child.ready.is_set, timeout)

    def wait_topic(self, label: str, topic: str, msg_type: str,
                   timeout: float, best_effort=False):
        argv = ['ros2', 'topic', 'echo', topic, msg_type, '--once']
        if best_effort:
            argv += ['--qos-reliability', 'best_effort']
        try:
            probe = subprocess.Popen(argv, env=self.env, cwd=ROOT,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        except OSError as exc:
            raise LaunchError(f'{label} ROS probe could not start: {exc}') from exc
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                self.check()
                status = probe.poll()
                if status == 0:
                    print(f'[READY] {label}: {topic} message received', flush=True)
                    return
                if status is not None:
                    details = probe.stdout.read().decode(errors='replace')[-1000:]
                    raise LaunchError(f'{label} ROS probe failed (code={status}): {details}')
                time.sleep(0.3)
            raise LaunchError(f'{label}: no {topic} message within {timeout:g}s')
        finally:
            if probe.poll() is None:
                try:
                    os.killpg(probe.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    probe.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(probe.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    probe.wait(timeout=2)
            if probe.stdout:
                probe.stdout.close()

    def wait_http(self, label: str, url: str, check_body, timeout: float):
        def ready():
            result = http_get(url)
            return result is not None and result[0] == 200 and check_body(result[1])
        self.until(label, ready, timeout)

    @staticmethod
    def _send(child: Child, sig):
        # A launch parent may have exited while grandchildren remain in its
        # original session/process group. Signal the owned group in that case.
        try:
            os.killpg(child.proc.pid, sig)
        except ProcessLookupError:
            pass

    def shutdown(self):
        if self.stopping:
            return
        self.stopping = True
        print('\n[SHUTDOWN] Stopping owned processes. Confirm the physical stop externally.',
              flush=True)
        # Begin stopping command sources, calibration children, and CAN promptly.
        priority = ('bridge', 'calibration', 'can')
        ordered = [c for key in priority for c in self.children if c.name == key]
        ordered += [c for c in reversed(self.children) if c not in ordered]
        for child in ordered:
            self._send(child, signal.SIGINT)

        for child in ordered:
            if child.proc.poll() is not None:
                continue
            grace = 16 if child.name in ('can', 'calibration') else 8
            try:
                child.proc.wait(timeout=grace)
            except subprocess.TimeoutExpired:
                print(f'[WARN] {child.name} did not stop after SIGINT; sending SIGTERM',
                      flush=True)
                self._send(child, signal.SIGTERM)
                try:
                    child.proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    print(f'[CRITICAL] {child.name} unresponsive. '
                          'Hardware E-stop and drive state must be checked.', flush=True)
                    self._send(child, signal.SIGKILL)
                    try:
                        child.proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        pass
        print(f'[SHUTDOWN] Logs: {self.log_dir}', flush=True)


def choose_camera_python(requested: str | None, ros_python: str) -> str:
    if requested:
        path = Path(requested).expanduser()
        if not path.is_file():
            raise LaunchError(f'Camera Python not found: {path}')
        return str(path)
    for path in (ROOT / 'amr_websocket/.venv/bin/python',
                 Path.home() / 'amr_websocket/.venv/bin/python'):
        if path.is_file():
            return str(path)
    return ros_python


def main() -> int:
    parser = argparse.ArgumentParser(description='One-command real AMR startup supervisor')
    parser.add_argument('--dry-run', action='store_true', help='Print plan; never access hardware')
    parser.add_argument('--skip-imu', action='store_true', help='Do not start IMU services')
    parser.add_argument('--external-imu-driver', action='store_true',
                        help='Wait for an existing /imu/data publisher')
    parser.add_argument('--skip-camera', action='store_true')
    parser.add_argument('--skip-calibration', action='store_true')
    parser.add_argument('--skip-bridge', action='store_true',
                        help='Do not connect to the Windows motor WebSocket')
    parser.add_argument('--web-drive', action='store_true',
                        help='Explicitly allow the legacy Web WASD bridge to publish nonzero RPM')
    parser.add_argument('--with-viewers', action='store_true',
                        help='Also start actual-RPM and encoder text viewers')
    parser.add_argument('--camera-python', help='Interpreter with cv2, ultralytics, pyorbbecsdk')
    parser.add_argument('--log-dir', type=Path,
                        default=Path.home() / 'amr_one_command_logs')
    args = parser.parse_args()
    if args.web_drive and args.skip_bridge:
        parser.error('--web-drive conflicts with --skip-bridge')

    if args.dry_run:
        print('Repository:', ROOT)
        print('ROS: Jazzy, domain 31, custom_amr_ws and real_amr overlays if present')
        print('Stages:')
        if not args.skip_imu:
            print('  IMU driver (' + ('external' if args.external_imu_driver else 'launch')
                  + ') -> /imu/data -> RPY converter -> IMU WebSocket :8766')
        if not args.skip_camera:
            print('  OpenVINO + Orbbec camera -> /health :8090')
        print('  CANopen controller -> verified zero + /amr_motor_state')
        if not args.skip_bridge:
            print('  Windows WebSocket bridge -> ws://192.168.0.31:8765 '
                  + ('[WEB MOTION ENABLED]' if args.web_drive else '[DIAGNOSTIC ONLY]'))
        if not args.skip_calibration:
            print('  Calibration manager -> :8877 (no calibration automatically started)')
        print('  HTML dashboard -> :8000')
        if args.with_viewers:
            print('  Actual RPM + encoder text monitors')
        print('On any startup/runtime failure: stop launched process groups; '
              'CAN runs its existing stop verification on graceful exit.')
        return 0

    validate_inputs(args)
    env = source_ros_environment()
    if not shutil.which('ros2', path=env.get('PATH')):
        raise LaunchError('ros2 CLI is unavailable after sourcing setup.bash')
    py = shutil.which('python3', path=env.get('PATH')) or sys.executable
    cam_py = choose_camera_python(args.camera_python, py)
    # Bound ports are checked before CAN drive initialization begins.
    ports = [("127.0.0.1", 8000)]
    if not args.skip_imu:
        ports.append(("127.0.0.1", 8766))
    if not args.skip_camera:
        ports.append(("127.0.0.1", 8090))
    if not args.skip_calibration:
        ports.append((env.get('AMR_CAL_HOST', '192.168.0.22'),
                      int(env.get('AMR_CAL_PORT', '8877'))))
    for host, port in ports:
        if port_open(host, port):
            raise LaunchError(f'{host}:{port} already serves connections. '
                              'Do not start a duplicate service.')

    date = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    logs = args.log_dir.expanduser() / date
    logs.mkdir(parents=True, exist_ok=False)
    supervisor = Supervisor(env, logs)
    signal.signal(signal.SIGINT, supervisor.request_stop)
    signal.signal(signal.SIGTERM, supervisor.request_stop)
    try:
        if not args.skip_imu:
            if not args.external_imu_driver:
                config = ROOT / 'amr_imu/config/microstrain.yml'
                supervisor.start('imu_driver', ['ros2', 'launch', 'microstrain_inertial_driver',
                                               'microstrain_launch.py', f'params_file:={config}'],
                                 fatal_markers=('[FATAL]',
                                                'Failed to read configuration for node',
                                                'Failed to configure node base',
                                                'process has died'))
            supervisor.wait_topic('IMU driver', '/imu/data', 'sensor_msgs/msg/Imu',
                                  35, best_effort=True)
            supervisor.start('imu_rpy', [py, str(SCRIPTS['imu_rpy'])])
            supervisor.wait_topic('IMU RPY', '/amr/imu/rpy', 'std_msgs/msg/String', 15)
            supervisor.start('imu_ws', [py, str(SCRIPTS['imu_ws'])])
            supervisor.until('IMU WebSocket', lambda: port_open('127.0.0.1', 8766), 10)

        if not args.skip_camera:
            supervisor.start('camera', [cam_py, str(SCRIPTS['camera'])],
                             fatal_markers=('[CAMERA ERROR]', '[CAMERA] 종료'))
            supervisor.wait_http('AI camera', 'http://127.0.0.1:8090/health',
                                 lambda body: body == b'CAMERA_OK', 90)

        can = supervisor.start('can', [py, str(SCRIPTS['can'])],
                               ready_markers=('[READY] ROS wheel RPM topics',),
                               fatal_markers=('[CRITICAL CAN]', '[CRITICAL] CAN worker'))
        supervisor.wait_log(can, 25)

        if not args.skip_bridge:
            bridge_argv = [py, str(SCRIPTS['bridge'])]
            if args.web_drive:
                bridge_argv.append('--bench-motion')
            bridge = supervisor.start('bridge', bridge_argv,
                                      ready_markers=('[CONNECTED] Windows Server',),
                                      fatal_markers=('[DISCONNECTED]', '[ROS ERROR]', '[FAULT]'))
            supervisor.wait_log(bridge, 30)

        if not args.skip_calibration:
            supervisor.start('calibration', [py, str(SCRIPTS['calibration'])])
            host = env.get('AMR_CAL_HOST', '192.168.0.22')
            port = int(env.get('AMR_CAL_PORT', '8877'))
            url = f'http://{host}:{port}/status'
            supervisor.wait_http('Calibration manager', url,
                                 lambda body: json.loads(body).get('ready') is True, 12)

        supervisor.start('web_ui', [py, '-m', 'http.server', '8000',
                                    '--bind', '0.0.0.0', '--directory',
                                    str(ROOT / 'amr_websocket')])
        supervisor.wait_http('Web UI', 'http://127.0.0.1:8000/index.html',
                             lambda body: b'<html' in body.lower(), 10)

        if args.with_viewers:
            viewer = supervisor.start('rpm_viewer', [py, str(SCRIPTS['rpm_viewer'])],
                                      ready_markers=('[INFO] CANopen',))
            supervisor.wait_log(viewer, 8)
            supervisor.start('encoder_viewer', [py, str(SCRIPTS['encoder_viewer'])])
            supervisor.until('Encoder viewer', lambda: True, 1)

        ui_host = env.get('AMR_CAL_HOST', '192.168.0.22')
        print(f'\n[ALL READY] http://{ui_host}:8000', flush=True)
        print('[INFO] Press Ctrl+C to stop all managed services.', flush=True)
        print('[INFO] No ARM or nonzero RPM was sent by the launcher.', flush=True)
        next_health_check = time.monotonic() + 3
        failed_health_checks: dict[str, int] = {}
        while True:
            supervisor.check()
            if time.monotonic() >= next_health_check:
                checks = [('Web UI', 'http://127.0.0.1:8000/index.html',
                           lambda b: b'<html' in b.lower())]
                if not args.skip_camera:
                    checks.append(('Camera', 'http://127.0.0.1:8090/health',
                                   lambda b: b == b'CAMERA_OK'))
                if not args.skip_calibration:
                    host = env.get('AMR_CAL_HOST', '192.168.0.22')
                    port = int(env.get('AMR_CAL_PORT', '8877'))
                    checks.append(('Calibration manager', f'http://{host}:{port}/status',
                                   lambda b: json.loads(b).get('ready') is True))
                if not args.skip_imu:
                    checks.append(('IMU WebSocket', None,
                                   lambda _: port_open('127.0.0.1', 8766)))
                for label, url, test in checks:
                    result = http_get(url) if url else (200, b'')
                    try:
                        ok = result is not None and result[0] == 200 and test(result[1])
                        if ok and label == 'Calibration manager':
                            status = json.loads(result[1])
                            exit_code = status.get('exit_code')
                            if exit_code not in (None, 0) and '사용자' not in status.get('last_error', ''):
                                raise LaunchError(f'Calibration child exited with code={exit_code}; '
                                                  f'log={status.get("log_path", "")!r}')
                    except (ValueError, TypeError):
                        ok = False
                    failed_health_checks[label] = 0 if ok else failed_health_checks.get(label, 0) + 1
                    if failed_health_checks[label] >= 3:
                        raise LaunchError(f'{label} health check failed three consecutive times')
                next_health_check = time.monotonic() + 3
            time.sleep(0.25)
    finally:
        supervisor.shutdown()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('[INFO] Operator requested shutdown', flush=True)
        sys.exit(130)
    except LaunchError as error:
        print(f'\n[LAUNCH FAILED] {error}', file=sys.stderr, flush=True)
        sys.exit(1)
