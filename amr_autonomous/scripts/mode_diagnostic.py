#!/usr/bin/env python3
"""Read-only AMR MANUAL/AUTO command observer for disconnected bench diagnostics.

This node publishes NO commands, no authorization, no RPM output, no CAN
frames, and never requests NavigateToPose. It only reports candidate inputs.
The web MANUAL/AUTO selection is a UI preference, NOT motor authority.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64, String


class ModeDiagnostic(Node):
    def __init__(self):
        super().__init__("amr_mode_diagnostic")
        self.mode = "UNKNOWN"
        self.sources = {"auto_left": None, "auto_right": None,
                        "can_input_left": None, "can_input_right": None}
        self.changed = {key: None for key in self.sources}
        mode_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/amr_web/requested_mode",
                                 self.on_mode, mode_qos)
        topics = {
            "auto_left": "/mapex/wheel_rpm_left_requested",
            "auto_right": "/mapex/wheel_rpm_right_requested",
            "can_input_left": "/wheel_rpm_left",
            "can_input_right": "/wheel_rpm_right",
        }
        for key, topic in topics.items():
            self.create_subscription(
                Float64, topic,
                lambda msg, k=key: self.on_rpm(k, msg.data), 10)
        self.create_timer(1.0, self.report)
        self.get_logger().warning(
            "READ ONLY - this node sends NO RPM, CAN, ARM, authorization, or navigation commands")

    def on_mode(self, msg):
        if msg.data in ("MANUAL", "AUTO"):
            self.mode = msg.data
        else:
            self.get_logger().warning("Unexpected mode value ignored")

    def on_rpm(self, key, value):
        now = time.monotonic()
        self.sources[key] = value
        self.changed[key] = now

    def report(self):
        parts = []
        for key, value in self.sources.items():
            age = None if self.changed[key] is None else time.monotonic()-self.changed[key]
            if value is None:
                status = "NO_DATA"
            elif not math.isfinite(value):
                status = "INVALID"
            elif age > 0.5:
                status = "STALE"
            elif abs(value) > (500 if key.startswith("can_input") else 1000):
                status = "OVER_TARGET_LIMIT"
            else:
                status = "FRESH"
            parts.append(f"{key}={value!r}({status})")
        self.get_logger().info("UI_REQUEST=%s | %s | NOT ACTUATING" %
                               (self.mode, " | ".join(parts)))


def main():
    rclpy.init()
    node = ModeDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
