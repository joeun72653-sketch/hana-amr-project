#!/usr/bin/env python3
"""Apply operator AUTO RPM 30..1000 to the Nav2 wheel-RPM *request* stream.

ROS2 Jazzy. This component does NOT publish to physical CAN motor topics,
does NOT grant motion permission, and does NOT start NavigateToPose.
Publishes an input candidate for the future single-owner motor arbiter.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64, String


class AutoRpmLimiter(Node):
    def __init__(self):
        super().__init__("amr_auto_rpm_limiter")
        self.requested_max_rpm = 30.0
        self.values = [None, None]
        self.arrival = [None, None]
        self.left = self.create_publisher(
            Float64, "/amr_auto/wheel_rpm_left_limited", 10)
        self.right = self.create_publisher(
            Float64, "/amr_auto/wheel_rpm_right_limited", 10)
        self.info = self.create_publisher(
            String, "/amr_auto/rpm_limit_status", 10)
        input_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Float64, "/amr_web/auto_max_rpm_requested",
            self.on_limit, input_qos)
        self.create_subscription(
            Float64, "/mapex/wheel_rpm_left_requested",
            lambda m: self.on_wheel(0, m.data), 10)
        self.create_subscription(
            Float64, "/mapex/wheel_rpm_right_requested",
            lambda m: self.on_wheel(1, m.data), 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().warning(
            "AUTO request limiter ready: default 30 RPM; configurable 30..1000 RPM. "
            "NO CAN/ARM/motion output")

    def on_limit(self, msg):
        value = float(msg.data)
        if not math.isfinite(value) or not 30 <= value <= 1000:
            self.get_logger().warning("Invalid AUTO RPM setting ignored")
            return
        self.requested_max_rpm = value
        # Require a new pair following any setting change, not old samples.
        self.arrival = [None, None]

    def on_wheel(self, side, value):
        if not math.isfinite(value):
            self.values[side] = None
            self.arrival[side] = None
            return
        self.values[side] = float(value)
        self.arrival[side] = time.monotonic()

    def tick(self):
        now = time.monotonic()
        fresh = all(t is not None and 0 <= now-t < 0.30 for t in self.arrival)
        if fresh:
            peak = max(abs(v) for v in self.values)
            factor = min(1.0, self.requested_max_rpm / peak) if peak else 1.0
            requested = [v * factor for v in self.values]
        else:
            requested = [0.0, 0.0]
        self.left.publish(Float64(data=requested[0]))
        self.right.publish(Float64(data=requested[1]))
        self.info.publish(String(data=(
            '{"requested_max_rpm":%.1f,"limited_left_rpm":%.3f,'
            '"limited_right_rpm":%.3f,"input_fresh":%s,'
            '"can_output_enabled":false}' % (
                self.requested_max_rpm, requested[0], requested[1],
                "true" if fresh else "false"))))


def main():
    rclpy.init()
    node = AutoRpmLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
