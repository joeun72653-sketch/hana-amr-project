#!/usr/bin/env python3
"""RViz 2D Goal Pose -> Nav2 action with explicit raised-wheel CAN authorization.

BENCH ONLY. Requires run_all.sh with AMR_RVIZ_GOAL_BENCH=1 and the guarded
CAN mux with --bench-depth-directional. Does not bypass motor/lift/depth watchdogs.
Stop with Ctrl+C; no autonomous rear/side collision checking is provided.
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String
from action_msgs.msg import GoalStatus


class RvizBenchBridge(Node):
    def __init__(self):
        super().__init__("amr_rviz_bench_goal_bridge")
        self.state = "IDLE"
        self.goal = None
        self.handle = None
        self.started = None
        self.motor = None
        self.motor_at = None
        self.depth_at = None
        self.mode_pub = self.create_publisher(
            String, "/amr_web/requested_mode",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.arm_pub = self.create_publisher(Bool, "/amr_web/arm_heartbeat", 10)
        self.auto_pub = self.create_publisher(Bool, "/amr_auto/navigation_active", 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.create_subscription(String, "/amr_motor_state", self.on_motor, 10)
        self.create_subscription(Bool, "/amr/depth_obstacle",
                                 lambda m: setattr(self, "depth_at", time.monotonic()), 10)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.create_timer(0.05, self.tick)
        self.get_logger().warning(
            "RAISED WHEELS ONLY. RViz 2D Goal Pose accepted; Ctrl+C disarms. "
            "Never run simultaneously with web AUTO.")

    def on_motor(self, msg):
        try:
            self.motor = json.loads(msg.data)
            self.motor_at = time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_goal(self, msg):
        if self.state != "IDLE":
            self.get_logger().warning("Goal ignored: previous goal active; Ctrl+C to stop")
            return
        if msg.header.frame_id.lstrip("/") != "map":
            self.get_logger().error("RViz goal must be in map frame")
            return
        self.goal = msg
        self.started = time.monotonic()
        self.state = "PREARM"
        self.get_logger().info("Goal received; zero-RPM CAN handshake")

    def tick(self):
        now = time.monotonic()
        self.mode_pub.publish(String(data="AUTO"))
        fresh_motor = (self.motor_at is not None and now - self.motor_at < 0.45
                       and self.motor and self.motor.get("fault") is False)
        fresh_depth = self.depth_at is not None and now - self.depth_at < 0.30
        if self.state == "PREARM":
            if not fresh_motor or not fresh_depth:
                self.stop("Motor/depth status not fresh")
            elif now - self.started > 8.0:
                self.stop("CAN ARM handshake timed out")
            elif self.motor.get("armed") is True and self.motor.get("stop_verified") is True:
                if not self.client.server_is_ready():
                    self.stop("Nav2 action server unavailable")
                else:
                    self.state = "SENDING"
                    fut = self.client.send_goal_async(
                        NavigateToPose.Goal(pose=self.goal))
                    fut.add_done_callback(self.on_accepted)
        if self.state == "RUNNING" and (not fresh_motor or not fresh_depth):
            self.stop("Motor/depth status lost")
        if self.state in ("PREARM", "SENDING", "RUNNING"):
            self.arm_pub.publish(Bool(data=True))
        else:
            self.arm_pub.publish(Bool(data=False))
        self.auto_pub.publish(Bool(data=self.state == "RUNNING"))

    def on_accepted(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.stop("Nav2 rejected goal")
                return
            self.handle = handle
            self.state = "RUNNING"
            handle.get_result_async().add_done_callback(self.on_result)
            self.get_logger().info("Nav2 goal accepted; continuous AUTO authorization")
        except Exception as exc:
            self.stop(str(exc))

    def on_result(self, future):
        try:
            status = future.result().status
            self.stop("Goal result: " + ("SUCCEEDED" if status == GoalStatus.STATUS_SUCCEEDED else str(status)))
        except Exception as exc:
            self.stop("Goal error: " + str(exc))

    def stop(self, reason):
        self.get_logger().warning(reason)
        self.state = "STOPPED"
        self.auto_pub.publish(Bool(data=False))
        self.arm_pub.publish(Bool(data=False))
        if self.handle is not None:
            self.handle.cancel_goal_async()
            self.handle = None

    def shutdown(self):
        self.stop("Operator bridge shutdown")


def main():
    rclpy.init()
    node = RvizBenchBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
