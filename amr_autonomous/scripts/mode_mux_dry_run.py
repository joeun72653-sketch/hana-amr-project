#!/usr/bin/env python3
"""Read-only ROS2 mode mux laboratory: NEVER commands wheels, CAN or authorization.

MANUAL source is an ISOLATED diagnostic request pair; not a live CAN input.
AUTO source is an RPM conversion candidate. Output stays on diagnostic topics;
there is deliberately NO publisher to /wheel_rpm_left, /wheel_rpm_right,
or /amr_motion_authorized. This is NOT a runtime motor controller.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64, String


class DryRunMux(Node):
    MAX_AGE_S = 0.30
    MANUAL_MAX = 500.0
    AUTO_DESIGN_MAX = 1000.0  # Not approved as a live motor speed

    def __init__(self):
        super().__init__("amr_mode_mux_dry_run")
        self.mode = "MANUAL"
        self.samples = {k: None for k in ("manual_left", "manual_right",
                                         "auto_left", "auto_right")}
        self.stamps = {k: None for k in self.samples}
        self.motor_status = None
        self.motor_stamp = None
        self.mode_change_at = time.monotonic()
        self.switch_pending = True
        self.switch_zero_seen = False
        self.last_log = None
        mode_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/amr_web/requested_mode", self.on_mode, mode_qos)
        for key, topic in (
            ("manual_left", "/amr_manual/wheel_rpm_left_requested"),
            ("manual_right", "/amr_manual/wheel_rpm_right_requested"),
            ("auto_left", "/mapex/wheel_rpm_left_requested"),
            ("auto_right", "/mapex/wheel_rpm_right_requested"),
        ):
            self.create_subscription(
                Float64, topic, lambda msg, source=key: self.on_rpm(source, msg.data), 10)
        self.create_subscription(String, "/amr_motor_state", self.on_motor_state, 10)
        self.diag_pub = self.create_publisher(String, "/amr_web/mux_diagnostic", 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().warning(
            "DRY RUN ONLY. This node never publishes physical motor commands or authorization.")

    def on_mode(self, msg):
        if msg.data not in ("MANUAL", "AUTO"):
            return
        if msg.data != self.mode:
            self.mode = msg.data
            self.mode_change_at = time.monotonic()
            # Never carry older commands across a mode change.
            self.switch_pending = True
            self.switch_zero_seen = False
            for key in self.stamps:
                self.stamps[key] = None

    def on_rpm(self, source, value):
        self.samples[source] = float(value)
        self.stamps[source] = time.monotonic()

    def on_motor_state(self, msg):
        try:
            state = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(state, dict):
            return
        self.motor_status = state
        self.motor_stamp = time.monotonic()

    def verified_stopped(self, now):
        s = self.motor_status
        return bool(s is not None and self.motor_stamp is not None
                    and now - self.motor_stamp <= 0.5
                    and s.get("fault") is False
                    and s.get("stop_verified") is True
                    and s.get("applied") == [0, 0]
                    and s.get("actual") == [0, 0])

    def tick(self):
        now = time.monotonic()
        left_key, right_key = (("manual_left", "manual_right") if
                               self.mode == "MANUAL" else ("auto_left", "auto_right"))
        keys = (left_key, right_key)
        limit = self.MANUAL_MAX if self.mode == "MANUAL" else self.AUTO_DESIGN_MAX
        fresh = all(self.stamps[k] is not None and
                    0 <= now - self.stamps[k] < self.MAX_AGE_S for k in keys)
        values = tuple(self.samples[k] for k in keys)
        valid = all(v is not None and math.isfinite(v) and abs(v) <= limit for v in values)
        # Diagnostics only; no transition is considered safe unless physical
        # motor stopped feedback is current and verified.
        stopped = self.verified_stopped(now)
        if not fresh:
            reason = "SOURCE_STALE_OR_MISSING"
        elif not valid:
            reason = "INVALID_OR_OVER_LIMIT"
        elif not stopped:
            # No candidate can pass until genuine CAN state is received and
            # both requested/applied wheel velocities are verified stopped.
            self.switch_pending = True
            self.switch_zero_seen = False
            reason = "PHYSICAL_STOP_NOT_VERIFIED"
        elif self.switch_pending:
            # A mode transition demands at least one fresh zero RPM pair from
            # the newly selected input AFTER the mode switch. This remains
            # diagnostic only; it does not grant real drive authorization.
            if all(abs(v) <= 1e-9 for v in values):
                self.switch_zero_seen = True
                self.switch_pending = False
                reason = "ZERO_HANDSHAKE_OBSERVED"
            else:
                reason = "WAIT_NEW_SOURCE_ZERO"
        else:
            reason = "CANDIDATE_ONLY"
        candidate = list(values) if reason == "CANDIDATE_ONLY" else [0.0, 0.0]
        info = {
            "requested_mode": self.mode,
            "candidate_rpm": candidate,
            "input_fresh": fresh,
            "input_valid": valid,
            "stop_verified": stopped,
            "switch_pending": self.switch_pending,
            "zero_handshake_observed": self.switch_zero_seen,
            "input_source": "isolated_manual_request" if self.mode == "MANUAL" else "mapex_auto_requested",
            "reason": reason,
            "motor_output_enabled": False,
            "control_authorized": False,
            "note": "Diagnostic only. Neither source is forwarded to CAN. Legacy web bridge is NOT mode-gated.",
        }
        self.diag_pub.publish(String(data=json.dumps(info)))
        summary = (self.mode, reason, tuple(candidate))
        if summary != self.last_log:
            self.get_logger().info(json.dumps(info))
            self.last_log = summary


def main():
    rclpy.init()
    node = DryRunMux()
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
