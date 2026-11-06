#!/usr/bin/env python3
"""Static AMR phone dashboard + authenticated preview-only map process starter.

This endpoint starts ONLY amr_autonomous/map_preview_server.py. It never
starts CAN, lift, NavigateToPose, publishes RPM or authorizes motion.
Restrict HTTP port 8000 and map port 8771 to a trusted private LAN.
"""
import hmac
import json
import os
import socket
import subprocess
import threading
import urllib.request
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HOME = Path.home()
STATIC_ROOT = HOME / "amr_websocket"
PREVIEW = HOME / "hana-amr-project-autonomous" / "amr_autonomous" / "map_preview_server.py"
TOKEN_FILE = HOME / ".amr_map_start_token"
LOG = HOME / ".amr_map_preview.log"
LISTEN_PORT = 8000
MAP_PORT = 8771


class StartMapServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, handler):
        super().__init__(address, handler)
        self.map_lock = threading.Lock()
        self.map_proc = None

    def is_map_up(self):
        try:
            with urllib.request.urlopen(
                f"http://127.0.0.1:{MAP_PORT}/api/status", timeout=0.45
            ) as response:
                state = json.loads(response.read(8192))
            return (response.status == 200
                    and state.get("motion_enabled") is False
                    and "map_ready" in state)
        except Exception:
            return False

    def start_map(self):
        with self.map_lock:
            if self.is_map_up():
                return "already_running"
            if self.map_proc is not None and self.map_proc.poll() is None:
                return "starting"
            if not PREVIEW.is_file():
                raise FileNotFoundError(f"Map server missing: {PREVIEW}")
            # Never accept a command, executable, filename, args or port
            # from the browser. Only a fixed preview-only executable runs.
            env = dict(os.environ)
            env["PYTHONNOUSERSITE"] = "1"
            env["ROS_DOMAIN_ID"] = "31"
            env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
            with LOG.open("ab") as log_file:
                self.map_proc = subprocess.Popen(
                    ["/usr/bin/python3", "-u", str(PREVIEW),
                     "--bind", "0.0.0.0", "--port", str(MAP_PORT)],
                    cwd=str(PREVIEW.parent),
                    env=env, stdin=subprocess.DEVNULL,
                    stdout=log_file, stderr=subprocess.STDOUT,
                    close_fds=True)
            return "starting"

    def server_close(self):
        with self.map_lock:
            if self.map_proc is not None and self.map_proc.poll() is None:
                self.map_proc.terminate()
                try:
                    self.map_proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.map_proc.kill()
                    self.map_proc.wait(timeout=2)
        super().server_close()


class Handler(SimpleHTTPRequestHandler):
    def send_json(self, code, obj):
        raw = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        if self.path.split("?", 1)[0] == "/api/map/status":
            return self.send_json(200, {
                "running": self.server.is_map_up(),
                "preview_only": True,
                "map_port": MAP_PORT
            })
        return super().do_GET()

    def do_POST(self):
        if self.path != "/api/map/start":
            return self.send_json(404, {"error": "Not found"})
        size = self.headers.get("Content-Length", "")
        if not size.isdecimal() or int(size) > 1024 or int(size) < 2:
            return self.send_json(400, {"error": "Invalid request"})
        try:
            body = json.loads(self.rfile.read(int(size)))
        except (ValueError, UnicodeDecodeError):
            return self.send_json(400, {"error": "Invalid JSON"})
        try:
            secret = TOKEN_FILE.read_text(encoding="utf-8").strip()
        except OSError:
            return self.send_json(503, {"error": "Map start token not configured"})
        supplied = body.get("token") if isinstance(body, dict) else None
        if (not secret or not isinstance(supplied, str) or
                not hmac.compare_digest(secret, supplied)):
            return self.send_json(403, {"error": "Invalid map start token"})
        try:
            result = self.server.start_map()
            return self.send_json(202, {"result": result, "preview_only": True})
        except OSError as exc:
            return self.send_json(503, {"error": type(exc).__name__})


def main():
    if not STATIC_ROOT.is_dir():
        raise FileNotFoundError(STATIC_ROOT)
    handler = partial(Handler, directory=str(STATIC_ROOT))
    server = StartMapServer(("0.0.0.0", LISTEN_PORT), handler)
    print(f"[AMR HTTP] static + map start control :{LISTEN_PORT}", flush=True)
    print("[AMR HTTP] map start is token protected, preview-only", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
