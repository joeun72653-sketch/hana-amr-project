#!/usr/bin/env python3
"""Web AMR calibration launch manager and per-run text logger.

This web layer relies on the robot's independent, preexisting hardware/ROS2
safety functions. LAN allowlist and browser-origin checks are *not* authentication
and must not be regarded as a safety interlock.
"""
import ipaddress
import json
import os
import signal
import time
import subprocess
import threading
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

HOST = os.environ.get('AMR_CAL_HOST', '192.168.0.22')
PORT = int(os.environ.get('AMR_CAL_PORT', '8877'))
ORIGIN = os.environ.get('AMR_CAL_WEB_ORIGIN', f'http://{HOST}:8000').rstrip('/')
ALLOWED_NETS = tuple(ipaddress.ip_network(item.strip(), strict=False) for item in
                     os.environ.get('AMR_CAL_ALLOWED_NETS', '192.168.0.0/24,127.0.0.0/8').split(',') if item.strip())
CAL_FLAG = Path.home() / '.amr_calibration_active'
LOG_DIR = Path(os.environ.get('AMR_CAL_LOG_DIR', str(Path.home() / 'amr_calibration_logs'))).expanduser()
LAUNCH_COMMAND = ('ros2', 'launch', 'real_amr_lidar', 'real_amr_calibration.launch.py',
                  'start:=true', 'auto_reposition:=true')

lock = threading.RLock()
proc = None
log_handle = None
last_exit_code = None
last_error = ''
last_log_path = ''
stop_in_progress = False


def stamp():
    return datetime.now().astimezone().isoformat(timespec='seconds')


def package_exists(name):
    try:
        return subprocess.run(('ros2', 'pkg', 'prefix', name), stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL, timeout=4, check=False).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def refresh_process():
    global proc, log_handle, last_exit_code
    with lock:
        if proc is not None and proc.poll() is not None:
            last_exit_code = proc.returncode
            if log_handle is not None:
                try:
                    log_handle.write(f'\n===== LAUNCH EXIT {stamp()} code={last_exit_code} =====\n'.encode())
                finally:
                    log_handle.close()
                    log_handle = None
            proc = None
            CAL_FLAG.unlink(missing_ok=True)


def stop_launch(reason):
    global last_error, stop_in_progress
    with lock:
        refresh_process()
        if proc is None:
            return False
        if stop_in_progress:
            return True
        stop_in_progress = True
        target = proc
        last_error = reason
    try:
        try:
            os.killpg(target.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            target.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(target.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                target.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(target.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                target.wait(timeout=3)
    finally:
        with lock:
            stop_in_progress = False
            refresh_process()
    return True


def latest_log():
    with lock:
        if last_log_path and Path(last_log_path).is_file():
            return Path(last_log_path)
    if not LOG_DIR.exists():
        return None
    logs = list(LOG_DIR.glob('calibration_*.txt'))
    return max(logs, key=lambda p: p.stat().st_mtime_ns) if logs else None


def status_payload():
    refresh_process()
    installed = all(package_exists(pkg) for pkg in ('custom_amr_description', 'real_amr_lidar'))
    with lock:
        running = proc is not None
        pid = proc.pid if running else None
        err = last_error
        exit_code = last_exit_code
    log_path = latest_log()
    env_ready = os.environ.get('ROS_DOMAIN_ID') == '31'
    ready = installed and env_ready
    if running:
        message = '보정 Launch 실행 중. 출력을 TXT 파일에 실시간 저장합니다.'
    elif not installed:
        message = '필수 ROS2 패키지를 찾을 수 없습니다.'
    elif not env_ready:
        message = '관리 서버를 ROS_DOMAIN_ID=31 환경에서 재실행하세요.'
    else:
        message = '실행 환경 준비됨. 하드웨어 및 Launch의 독립 안전장치는 그대로 사용합니다.'
    return {'state': '실행 중' if running else '대기', 'ready': ready,
            'dependency_installed': installed, 'running': running, 'pid': pid,
            'exit_code': exit_code, 'log_path': str(log_path) if log_path else '',
            'log_filename': log_path.name if log_path else '',
            'last_error': err, 'message': message}


def tail_log(path, max_bytes=90000):
    if path is None:
        return ''
    with path.open('rb') as f:
        f.seek(0, 2)
        size = f.tell()
        f.seek(max(0, size - max_bytes), 0)
        data = f.read()
    return ('[최근 로그만 표시합니다. 전체 파일은 다운로드하세요.]\n' if size > max_bytes else '') + data.decode('utf-8', errors='replace')


class Handler(BaseHTTPRequestHandler):
    def send_payload(self, status, content, mime='application/json; charset=utf-8', download=None):
        body = content if isinstance(content, bytes) else content.encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Access-Control-Allow-Origin', ORIGIN)
        self.send_header('Vary', 'Origin')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        if download:
            self.send_header('Content-Disposition', f'attachment; filename="{download}"')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        if body:
            self.wfile.write(body)

    def reply(self, status, data):
        self.send_payload(status, json.dumps(data, ensure_ascii=False).encode('utf-8'))

    def network_allowed(self):
        try:
            ip = ipaddress.ip_address(self.client_address[0])
            return any(ip in network for network in ALLOWED_NETS)
        except ValueError:
            return False

    def origin_allowed(self):
        origin = self.headers.get('Origin', '')
        if origin:
            return origin == ORIGIN
        # No browser Origin: local machine CLI only. LAN clients must use the
        # designated web UI, reducing accidental cross-site commands.
        return ipaddress.ip_address(self.client_address[0]).is_loopback or self.client_address[0] == HOST

    def do_OPTIONS(self):
        if self.network_allowed() and self.headers.get('Origin') == ORIGIN:
            self.send_payload(204, b'')
        else:
            self.reply(403, {'error': 'client_or_origin_not_allowed'})

    def do_GET(self):
        if not self.network_allowed():
            return self.reply(403, {'error': 'client_network_not_allowed'})
        if urlparse(self.path).path == '/status':
            return self.reply(200, status_payload())
        if urlparse(self.path).path == '/log-tail':
            path = latest_log()
            if path is None:
                return self.reply(200, {'filename': '', 'text': '아직 저장된 보정 로그가 없습니다.'})
            return self.reply(200, {'filename': path.name, 'text': tail_log(path)})
        if urlparse(self.path).path == '/logs/latest':
            path = latest_log()
            if path is None:
                return self.reply(404, {'error': 'no_log_yet'})
            # File is outside the port-8000 HTML document root by default.
            return self.send_payload(200, path.read_bytes(),
                                     mime='text/plain; charset=utf-8', download=path.name)
        return self.reply(404, {'error': 'not_found'})

    def do_POST(self):
        global proc, log_handle, last_error, last_log_path, last_exit_code
        if not self.network_allowed():
            return self.reply(403, {'accepted': False, 'message': '허용되지 않은 네트워크 주소입니다.'})
        if not self.origin_allowed():
            return self.reply(403, {'accepted': False, 'message': '관제 웹 페이지에서 요청해야 합니다.'})
        if self.headers.get('Content-Type', '').split(';')[0].strip().lower() != 'application/json':
            return self.reply(415, {'accepted': False, 'message': 'JSON 요청만 허용합니다.'})
        length = int(self.headers.get('Content-Length', '0') or '0')
        if length > 1024:
            return self.reply(413, {'accepted': False, 'message': '요청 데이터가 너무 큽니다.'})
        if length:
            self.rfile.read(length)
        path = urlparse(self.path).path
        if path == '/preflight':
            payload = status_payload()
            payload.update(kind='calibration_preflight', launch_executed=False)
            return self.reply(200, payload)
        if path not in ('/start', '/stop'):
            return self.reply(404, {'error': 'not_found'})
        if path == '/stop':
            stopped = stop_launch('사용자 보정 중지 요청')
            return self.reply(200, {'kind': 'calibration_stop', 'accepted': True,
                                    'running': False, 'stopped_launch': stopped,
                                    'message': '보정 Launch 종료 요청 처리됨. 실제 구동 정지는 하드웨어에서 확인하세요.'})
        with lock:
            refresh_process()
            if proc is not None:
                return self.reply(409, {'accepted': False, 'running': True,
                                        'message': '보정 Launch가 이미 실행 중입니다.'})
            if not all(package_exists(n) for n in ('custom_amr_description', 'real_amr_lidar')):
                return self.reply(424, {'accepted': False, 'message': '필수 ROS2 패키지를 찾을 수 없습니다.'})
            if os.environ.get('ROS_DOMAIN_ID') != '31':
                return self.reply(423, {'accepted': False, 'message': 'ROS_DOMAIN_ID가 31이 아닙니다.'})
            handle = None
            try:
                LOG_DIR.mkdir(parents=True, exist_ok=True)
                name = datetime.now().strftime('calibration_%Y%m%d_%H%M%S_%f.txt')
                path = LOG_DIR / name
                handle = path.open('xb', buffering=0)
                handle.write((f'===== AMR CALIBRATION START {stamp()} =====\n'
                              f'Command: {" ".join(LAUNCH_COMMAND)}\n'
                              f'ROS_DOMAIN_ID: {os.environ.get("ROS_DOMAIN_ID", "UNSET")}\n'
                              f'===== ROS2 LAUNCH STDOUT / STDERR =====\n').encode('utf-8'))
                child_env = os.environ.copy()
                child_env['PYTHONUNBUFFERED'] = '1'
                child_env['RCUTILS_COLORIZED_OUTPUT'] = '0'
                child_env['RCUTILS_LOGGING_BUFFERED_STREAM'] = '0'
                CAL_FLAG.touch()
                time.sleep(0.35)
                newproc = subprocess.Popen(LAUNCH_COMMAND, stdout=handle,
                                           stderr=subprocess.STDOUT,
                                           start_new_session=True, env=child_env)
            except (OSError, ValueError) as exc:
                CAL_FLAG.unlink(missing_ok=True)
                if handle is not None:
                    handle.close()
                return self.reply(500, {'accepted': False, 'message': f'Launch 시작 실패: {exc}'})
            proc = newproc
            log_handle = handle
            last_log_path = str(path)
            last_exit_code = None
            last_error = ''
            print(f'[CAL] launch started PID={proc.pid} log={path}', flush=True)
            return self.reply(202, {'kind': 'calibration_start', 'accepted': True,
                                    'running': True, 'pid': proc.pid,
                                    'log_path': str(path), 'log_filename': path.name,
                                    'message': '보정 Launch 시작됨. TXT 로그 저장을 시작했습니다.'})

    def log_message(self, fmt, *args):
        print('[WEB]', self.address_string(), fmt % args, flush=True)


def main():
    def shutdown_handler(signum, frame):
        print(f'[CAL] signal {signum}: stopping owned ROS2 launch', flush=True)
        stop_launch('관리 서버 종료')
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, shutdown_handler)
    signal.signal(signal.SIGINT, shutdown_handler)
    print(f'[CAL] server http://{HOST}:{PORT}; allowed LAN={ALLOWED_NETS}; web origin={ORIGIN}', flush=True)
    print(f'[CAL] ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "UNSET")}', flush=True)
    print(f'[CAL] TXT logs: {LOG_DIR}', flush=True)
    print('[CAL] No control token. Access is restricted by LAN/IP and Origin, which are NOT authentication.', flush=True)
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()


if __name__ == '__main__':
    main()
