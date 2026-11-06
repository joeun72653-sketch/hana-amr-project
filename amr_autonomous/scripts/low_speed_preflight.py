#!/usr/bin/env python3
"""Fail-closed AMR low-speed navigation preflight. READ ONLY.

No CAN, ARM, NavigateToPose goals, RPM, cmd_vel, or interlock writes.
Run from ROS Jazzy environment before any attempted autonomous motion.
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String


class Preflight(Node):
    def __init__(self):
        super().__init__("amr_low_speed_preflight")
        self.latest = {}
        self.received_at = {}
        for topic, kind in (
            ("/amr_motor_state", String),
            ("/amr/depth_obstacle", Bool),
            ("/amr_web/requested_mode", String),
            ("/amr_web/mux_diagnostic", String),
        ):
            q = QoSProfile(depth=1)
            if topic == "/amr_web/requested_mode":
                q.reliability = ReliabilityPolicy.RELIABLE
                q.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(kind, topic,
                                     lambda msg, name=topic: self.observe(name, msg), q)

    def observe(self, name, msg):
        self.latest[name] = msg.data
        self.received_at[name] = time.monotonic()

    def publishers(self, topic):
        return [p.node_name for p in self.get_publishers_info_by_topic(topic)]

    def fresh(self, topic, limit=0.6):
        return topic in self.received_at and 0 <= time.monotonic()-self.received_at[topic] < limit

    def report(self):
        checks = {}
        cmd = self.publishers("/mapex/cmd_vel")
        left = self.publishers("/wheel_rpm_left")
        right = self.publishers("/wheel_rpm_right")
        auth = self.publishers("/amr_motion_authorized")
        motor = self.latest.get("/amr_motor_state")
        try:
            state = json.loads(motor) if isinstance(motor, str) else {}
        except (TypeError, ValueError):
            state = {}
        stop = (self.fresh("/amr_motor_state") and
                state.get("stop_verified") is True and
                state.get("fault") is False and
                state.get("applied") == [0, 0] and
                state.get("actual") == [0, 0])
        depth = (self.fresh("/amr/depth_obstacle", 0.35)
                 and self.latest.get("/amr/depth_obstacle") is False)
        diagnostic = self.latest.get("/amr_web/mux_diagnostic")
        try:
            mux = json.loads(diagnostic) if isinstance(diagnostic, str) else {}
        except (TypeError, ValueError):
            mux = {}
        checks["nav2_cmd_vel_source_present"] = bool(cmd)
        checks["auto_rpm_conversion_source_present"] = (
            bool(self.publishers("/mapex/wheel_rpm_left_requested")) and
            bool(self.publishers("/mapex/wheel_rpm_right_requested")))
        checks["motor_state_recent_and_stopped"] = bool(stop)
        checks["depth_recent_and_clear"] = bool(depth)
        checks["exclusive_wheel_output_owner"] = (
            len(left) == 1 and len(right) == 1 and left == right
            and left[0] != "amr_ws_to_ros" if left and right else False)
        checks["authorization_publisher_known"] = bool(auth)
        checks["hardware_mux_present"] = False  # current code is DRY-RUN ONLY
        checks["safety_io_estop_verified"] = False  # cannot be proved via these topics
        checks["drive_communication_loss_stop_verified"] = False
        checks["localization_current_verified"] = False
        checks["controller_low_speed_limit_verified"] = False
        checks["can_wheel_watchdog_revalidated"] = False
        return {
            "ready_for_live_autonomous_motion": False,
            "reason": "No approved single-owner CAN mux or verified E-stop/limits yet",
            "requested_mode": self.latest.get("/amr_web/requested_mode"),
            "legacy_can_publishers": {"left": left, "right": right, "auth": auth},
            "nav2_cmd_publishers": cmd,
            "mux_diagnostic": mux,
            "checks": checks,
            "note": "READ-ONLY preflight; never enables motion. False means absent/unverified, not necessarily failed hardware.",
        }


def main():
    rclpy.init()
    node = Preflight()
    try:
        deadline = time.monotonic()+3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(json.dumps(node.report(), indent=2, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
