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
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Float64
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
        self.pose_msg_stamp = None
        self.initial_pose = None
        self.initial_pose_at = None
        self.preview = []
        self.selected_mode = "MANUAL"
        # An operator-requested maximum; NOT an approved physical motor limit.
        self.auto_rpm_request_max = 30
        self.arrival_lift_plan = {"mode": "none", "pulse": None}
        self.target = None
        map_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        # Match AMCL's reliable/transient-local pose publisher: receive its last
        # sample even when localization does not publish again while stopped.
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self.pose_callback, map_qos)
        # RViz 2D Pose Estimate publishes /initialpose. Listen-only: do NOT
        # re-publish it or change the robot pose from this web application.
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                 self.initial_pose_callback,
                                 QoSProfile(depth=10,
                                            reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.VOLATILE))
        # Visualization-only topics for RViz, never /goal_pose or /cmd_vel.
        # Transient local keeps the most recent preview visible when RViz
        # subscribes after the web operator planned a path.
        preview_qos = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.preview_path_pub = self.create_publisher(
            NavPath, "/amr_web/preview_path", preview_qos)
        self.preview_goal_pub = self.create_publisher(
            Marker, "/amr_web/preview_goal", preview_qos)
        # UI selection is a separate read-only diagnostic signal; it MUST
        # never be interpreted as /amr_motion_authorized or an ARM request.
        self.mode_pub = self.create_publisher(
            String, "/amr_web/requested_mode", preview_qos)
        self.mode_pub.publish(String(data=self.selected_mode))
        self.auto_rpm_pub = self.create_publisher(
            Float64, "/amr_web/auto_max_rpm_requested", preview_qos)
        self.auto_rpm_pub.publish(Float64(data=float(self.auto_rpm_request_max)))
        self.create_subscription(PoseStamped, "/amr_web/rviz_goal_request",
                                 self.on_rviz_preview_goal, 10)
        self.planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

    def map_callback(self, message):
        with self.lock:
            self.grid = message

    def pose_callback(self, message):
        p = message.pose.pose
        with self.lock:
            self.pose = [p.position.x, p.position.y, yaw(p.orientation)]
            self.pose_at = time.monotonic()
            self.pose_msg_stamp = [message.header.stamp.sec,
                                   message.header.stamp.nanosec]

    def initial_pose_callback(self, message):
        if message.header.frame_id.lstrip("/") != "map":
            self.get_logger().warning("Ignoring initial pose not in map frame")
            return
        p = message.pose.pose
        with self.lock:
            self.initial_pose = [p.position.x, p.position.y, yaw(p.orientation)]
            self.initial_pose_at = time.monotonic()
            # Previously previewed route used an older localization origin.
            self.preview = []
            self.target = None
        self.clear_rviz_preview()

    def on_rviz_preview_goal(self, message):
        """Accept only RViz preview pose in map frame. No motion goal sent."""
        if message.header.frame_id.lstrip("/") != "map":
            self.get_logger().warning("RViz preview goal must have frame_id=map")
            return
        pose = message.pose
        values = (pose.position.x, pose.position.y, pose.orientation.x,
                  pose.orientation.y, pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(v) for v in values):
            return
        magnitude = sum(v*v for v in values[2:])
        if not 0.9 < magnitude < 1.1:
            return
        goal = {"x": pose.position.x, "y": pose.position.y,
                "yaw_deg": math.degrees(yaw(pose.orientation))}
        # Planning waits for a ROS action. Never wait inside the ROS callback.
        with self.lock:
            self.target = goal
            self.preview = []
        threading.Thread(target=self._plan_from_rviz, args=(goal,), daemon=True).start()

    def _plan_from_rviz(self, goal):
        try:
            self.plan({"goal": goal, "lift": {"mode": "none"}})
        except Exception as exc:
            self.get_logger().warning(f"RViz goal preview failed: {exc}")

    def clear_rviz_preview(self):
        empty = NavPath()
        empty.header.frame_id = "map"
        empty.header.stamp = self.get_clock().now().to_msg()
        self.preview_path_pub.publish(empty)
        delete = Marker()
        delete.header = empty.header
        delete.ns = "amr_web_preview"
        delete.id = 0
        delete.action = Marker.DELETE
        self.preview_goal_pub.publish(delete)

    def publish_rviz_preview(self, path, target):
        # Keep the Nav2 planned pose sequence and orientation exactly as
        # returned by ComputePathToPose, with an explicit map frame.
        path.header.frame_id = "map"
        self.preview_path_pub.publish(path)
        arrow = Marker()
        arrow.header.frame_id = "map"
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = "amr_web_preview"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = target.pose
        arrow.scale.x = 0.45
        arrow.scale.y = 0.12
        arrow.scale.z = 0.12
        arrow.color.r = 0.9
        arrow.color.g = 0.2
        arrow.color.b = 0.2
        arrow.color.a = 1.0
        self.preview_goal_pub.publish(arrow)

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

    def select_mode_preview(self, payload):
        """UI-only request: does NOT grant motor authority or publish ROS commands."""
        mode = payload.get("mode") if isinstance(payload, dict) else None
        if mode not in ("MANUAL", "AUTO"):
            raise ValueError("mode must be MANUAL or AUTO")
        with self.lock:
            self.selected_mode = mode
        self.mode_pub.publish(String(data=mode))
        return {"selected_mode": mode, "control_enabled": False,
                "message": "PREVIEW ONLY: no CAN/ARM/velocity command sent"}

    def set_auto_rpm_request(self, payload):
        """Store operator AUTO RPM request, without issuing any motor commands."""
        value = payload.get("rpm") if isinstance(payload, dict) else None
        if type(value) is not int or not 30 <= value <= 1000 or value % 10 != 0:
            raise ValueError("AUTO RPM must be an integer 30..1000 in steps of 10")
        with self.lock:
            self.auto_rpm_request_max = value
        self.auto_rpm_pub.publish(Float64(data=float(value)))
        return {"auto_rpm_requested_max": value, "motor_output_enabled": False,
                "message": "Setpoint request saved only; actual CAN output NOT enabled."}

    def status(self):
        with self.lock:
            age = time.monotonic() - self.pose_at if self.pose_at is not None else None
            # On a new RViz initialization request, display its requested map
            # location immediately. It is NOT a verified AMCL estimate.
            awaiting_amcl = (self.initial_pose_at is not None and
                             (self.pose_at is None or self.initial_pose_at > self.pose_at))
            display_pose = self.initial_pose if awaiting_amcl else self.pose
            display_age = ((time.monotonic() - self.initial_pose_at)
                           if awaiting_amcl else age)
            stale = awaiting_amcl or age is None or age >= 2
            return {"pose": display_pose,
                    "pose_source": "initialpose_pending_amcl" if awaiting_amcl else ("amcl" if self.pose is not None else "none"),
                    "pose_age_sec": round(display_age, 2) if display_age is not None else None,
                    "pose_stale": stale,
                    "pose_msg_stamp": self.pose_msg_stamp,
                    "map_ready": self.grid is not None,
                    "planner_ready": self.planner.server_is_ready(),
                    "path": self.preview,
                    "destination": self.target,
                    "arrival_lift_plan": self.arrival_lift_plan,
                    "selected_mode": self.selected_mode,
                    "auto_rpm_requested_max": self.auto_rpm_request_max,
                    "auto_rpm_effective_live_limit": None,
                    "control_enabled": False,
                    "motion_enabled": False,
                    "detail": "Preview only. No motor or lift commands are sent."}

    def plan(self, message):
        goal = message.get("goal", {})
        x, y, a = parse_goal(goal)
        arrival = parse_lift(message.get("lift", {}))
        with self.lock:
            g = self.grid
            # Preview only: explicitly use the last available map-frame pose.
            # An RViz initialpose that is newer than AMCL is merely an operator
            # estimate and must not be treated as a confirmed localization.
            initial_pending = (self.initial_pose_at is not None and
                               (self.pose_at is None or self.initial_pose_at > self.pose_at))
            start_pose = (list(self.initial_pose) if initial_pending
                          else list(self.pose) if self.pose is not None else None)
            start_source = ("rviz_initialpose_unverified" if initial_pending else
                            "amcl_last_known")
            age = (time.monotonic() - self.pose_at
                   if self.pose_at is not None else None)
        if g is None:
            raise ValueError("Map unavailable")
        if start_pose is None:
            raise ValueError("No AMCL pose or RViz initial pose received. Set 2D Pose Estimate first.")
        # Starting-point staleness is allowed ONLY for ComputePathToPose.
        # This service has no NavigateToPose, CAN, lift or motion API.
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
        task.start = PoseStamped()
        task.start.header.frame_id = "map"
        task.start.header.stamp = self.get_clock().now().to_msg()
        task.start.pose.position.x = start_pose[0]
        task.start.pose.position.y = start_pose[1]
        task.start.pose.orientation.z = math.sin(start_pose[2] / 2)
        task.start.pose.orientation.w = math.cos(start_pose[2] / 2)
        # Without use_start=True, Nav2 uses a live TF pose regardless of
        # the selected point shown in the preview web UI.
        task.use_start = True
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
        self.publish_rviz_preview(result.result.path, target)
        return {"path": poses, "destination": self.target,
                "arrival_lift_plan": arrival,
                "preview_start": start_pose,
                "preview_start_source": start_source,
                "preview_pose_age_sec": round(age, 2) if age is not None else None,
                "message": "Approximate preview from last/selected pose; NO motion. Verify localization before AUTO."}


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
        endpoint = urlparse(self.path).path
        if endpoint not in ("/api/plan", "/api/mode", "/api/auto-rpm"):
            self.respond(404, {"error": "Not found"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if size < 1 or size > 4096:
                raise ValueError("Invalid request length")
            payload = json.loads(self.rfile.read(size))
            if endpoint == "/api/mode":
                self.respond(200, self.server.node.select_mode_preview(payload))
            elif endpoint == "/api/auto-rpm":
                self.respond(200, self.server.node.set_auto_rpm_request(payload))
            else:
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
