#!/usr/bin/env python3
"""Map/AMCL/Nav2 path-preview web server. Never commands motors or lift.

Ubuntu ROS2 Jazzy. This server reads OccupancyGrid and AMCL pose, offers
ComputePathToPose planning, and records an arrival lift *plan* for review.
It does not run NavigateToPose, publish cmd_vel, or send WebSocket commands.
"""
from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatus


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def coordinates(grid, x, y):
    ori = grid.info.origin
    angle = yaw(ori.orientation)
    dx, dy = x - ori.position.x, y - ori.position.y
    c, s = math.cos(angle), math.sin(angle)
    r = grid.info.resolution
    i = math.floor((c*dx + s*dy)/r)
    j = math.floor((-s*dx + c*dy)/r)
    return i, j


def parse_goal(obj):
    if not isinstance(obj, dict):
        raise ValueError("Expected JSON object")
    vals = (obj.get("x"), obj.get("y"), obj.get("yaw_deg", 0))
    if any(type(v) not in (float, int) or not math.isfinite(v) for v in vals):
        raise ValueError("x, y, yaw_deg must be finite numbers")
    x, y, degrees = (float(v) for v in vals)
    if abs(degrees) > 360:
        raise ValueError("Invalid yaw")
    return x, y, degrees


def parse_lift(obj):
    if not isinstance(obj, dict):
        raise ValueError("Expected lift object")
    mode = obj.get("mode", "none")
    if mode not in ("none", "up", "down", "target"):
        raise ValueError("lift.mode must be none/up/down/target")
    pulse = obj.get("pulse")
    if mode == "target" and (type(pulse) is not int or not 0 <= pulse <= 2000):
        raise ValueError("lift.pulse must be integer 0..2000")
    return {"mode": mode, "pulse": pulse if mode == "target" else None}


def wait_future(future, timeout):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout):
        raise TimeoutError("Nav2 planning response timed out")
    return future.result()


class MapNode(Node):
    def __init__(self):
        super().__init__("amr_qr_map_preview")
        self.lock = threading.Lock()
        self.grid = None
        self.pose = None
        self.pose_at = None
        self.preview = []
        self.arrival_lift_plan = {"mode": "none", "pulse": None}
        self.target = None
        map_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self.pose_callback, qos_profile_sensor_data)
        self.planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

    def map_callback(self, message):
        with self.lock:
            self.grid = message

    def pose_callback(self, message):
        p = message.pose.pose
        with self.lock:
            self.pose = [p.position.x, p.position.y, yaw(p.orientation)]
            self.pose_at = time.monotonic()

    def map_data(self):
        with self.lock:
            g = self.grid
            if g is None:
                return {"ready": False}
            p = g.info.origin
            return {
                "ready": True, "width": g.info.width, "height": g.info.height,
                "resolution": g.info.resolution,
                "origin": [p.position.x, p.position.y, yaw(p.orientation)],
                "data": list(g.data),
            }

    def status(self):
        with self.lock:
            pose = self.pose if self.pose_at and time.monotonic()-self.pose_at < 2 else None
            return {"pose": pose, "map_ready": self.grid is not None,
                    "planner_ready": self.planner.server_is_ready(),
                    "path": self.preview,
                    "destination": self.target,
                    "arrival_lift_plan": self.arrival_lift_plan,
                    "motion_enabled": False,
                    "detail": "Preview only. No motor or lift commands are sent."}

    def plan(self, message):
        goal = message.get("goal", {})
        x, y, a = parse_goal(goal)
        arrival = parse_lift(message.get("lift", {}))
        with self.lock:
            g = self.grid
            ok_pose = self.pose_at and time.monotonic()-self.pose_at < 2
        if g is None or not ok_pose:
            raise ValueError("Map/AMCL pose missing or stale")
        i, j = coordinates(g, x, y)
        w, h = g.info.width, g.info.height
        if not (0 <= i < w and 0 <= j < h) or g.data[j*w + i] != 0:
            raise ValueError("Destination is not a known free map cell")
        if not self.planner.wait_for_server(timeout_sec=2):
            raise ValueError("ComputePathToPose action unavailable")

        target = PoseStamped()
        target.header.frame_id = "map"
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y = x, y
        angle = math.radians(a)
        target.pose.orientation.z = math.sin(angle/2)
        target.pose.orientation.w = math.cos(angle/2)

        task = ComputePathToPose.Goal()
        task.goal = target
        task.use_start = False
        handle = wait_future(self.planner.send_goal_async(task), 3)
        if handle is None or not handle.accepted:
            raise ValueError("Nav2 refused path preview")
        result = wait_future(handle.get_result_async(), 20)
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            raise ValueError("Nav2 planning unsuccessful")
        poses = [[q.pose.position.x, q.pose.position.y] for q in result.result.path.poses]
        with self.lock:
            self.preview = poses
            self.target = {"x": x, "y": y, "yaw_deg": a}
            self.arrival_lift_plan = arrival
        return {"path": poses, "destination": self.target,
                "arrival_lift_plan": arrival, "message": "Planned only; no motion"}


class Handler(BaseHTTPRequestHandler):
    def respond(self, status, data):
        raw = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        url = urlparse(self.path).path
        node = self.server.node
        if url == "/":
            file = Path(__file__).parent / "web" / "autonomous.html"
            raw = file.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
        elif url == "/api/map":
            self.respond(200, node.map_data())
        elif url == "/api/status":
            self.respond(200, node.status())
        else:
            self.respond(404, {"error": "Not found"})

    def do_POST(self):
        if urlparse(self.path).path != "/api/plan":
            self.respond(404, {"error": "Not found"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if size < 1 or size > 4096:
                raise ValueError("Invalid request length")
            payload = json.loads(self.rfile.read(size))
            self.respond(200, self.server.node.plan(payload))
        except (ValueError, TimeoutError, TypeError) as error:
            self.respond(422, {"error": str(error)})
        except Exception as error:
            self.respond(500, {"error": type(error).__name__})

    def log_message(self, fmt, *args):
        print("[MAP PREVIEW]", fmt % args, flush=True)


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8771)
    parser.add_argument("--bind", default="127.0.0.1",
                        help="Default localhost only. Private test LAN only for 0.0.0.0.")
    args = parser.parse_args()
    rclpy.init()
    node = MapNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    server.node = node
    print(f"[MAP PREVIEW] http://{args.bind}:{args.port} - READ/PLAN ONLY", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
