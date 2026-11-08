#!/usr/bin/env python3
"""Fail-closed MANUAL/AUTO to CAN arbitration for ROS2 Jazzy.

CAN topics ONLY when --live-motor-output is explicitly selected. The CAN node
must use --mux-input. Drive-side communication-loss stop and a hardware E-stop
are mandatory; Python cannot guarantee safe stopping on cable/power failure.
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Float64, String, Int32


class GuardedMotorMux(Node):
    AGE = 0.30
    STATUS_AGE = 0.45
    DEPTH_AGE = 0.30
    LIDAR_AGE = 0.35
    MAX_MANUAL = 500.0
    MAX_AUTO = 30.0  # First live trial HARD CAP; do not raise before hardware validation

    def __init__(self, live, bench_directional=False):
        super().__init__("amr_guarded_motor_mux")
        self.live = live
        self.bench_directional = bench_directional
        self.mode = "MANUAL"
        self.mode_at = time.monotonic()
        self.auto_limit = 30.0
        self.auto_limit_at = None
        self.auto_active = False
        self.auto_active_at = None
        self.manual_authorized = False
        self.manual_authorized_at = None
        self.web_arm = False
        self.web_arm_at = None
        self.depth_blocked = True
        self.depth_at = None
        self.lidar_mask = 255
        self.lidar_at = None
        self.motor = None
        self.motor_at = None
        self.request = {k: [None, None] for k in ("MANUAL", "AUTO")}
        self.request_at = {k: [None, None] for k in self.request}
        self.phase = "DISARMED"
        self.last_reason = None

        mode_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/amr_web/requested_mode", self.on_mode, mode_qos)
        self.create_subscription(Float64, "/amr_web/auto_max_rpm_requested",
                                 self.on_auto_limit, mode_qos)
        self.create_subscription(Bool, "/amr_auto/navigation_active",
                                 self.on_auto_active, 10)
        self.create_subscription(Bool, "/amr_manual/motion_authorized",
                                 self.on_manual_authorized, 10)
        self.create_subscription(Bool, "/amr_web/arm_heartbeat", self.on_web_arm, 10)
        self.create_subscription(Bool, "/amr/depth_obstacle", self.on_depth, 10)
        self.create_subscription(Int32, "/lidar_safty_bitmask", self.on_lidar_mask, 10)
        self.create_subscription(String, "/amr_motor_state", self.on_motor, 10)
        for mode, side, topic in (
            ("MANUAL", 0, "/amr_manual/wheel_rpm_left_requested"),
            ("MANUAL", 1, "/amr_manual/wheel_rpm_right_requested"),
            ("AUTO", 0, "/amr_auto/wheel_rpm_left_limited"),
            ("AUTO", 1, "/amr_auto/wheel_rpm_right_limited"),
        ):
            self.create_subscription(
                Float64, topic, lambda msg, m=mode, s=side:
                self.on_rpm(m, s, msg.data), 10)
        self.auth_pub = self.create_publisher(
            Bool, "/amr_mux/motion_authorized", 10)
        self.left_pub = self.create_publisher(Float64, "/amr_mux/wheel_rpm_left", 10)
        self.right_pub = self.create_publisher(Float64, "/amr_mux/wheel_rpm_right", 10)
        self.status_pub = self.create_publisher(String, "/amr_mux/status", 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().warning("Guarded CAN mux; live motor output=%s" % live)

    def reset(self):
        self.phase = "DISARMED"
        for mode in self.request_at:
            self.request_at[mode] = [None, None]

    def on_mode(self, msg):
        if msg.data not in ("MANUAL", "AUTO"):
            return
        if msg.data != self.mode:
            self.mode = msg.data
            self.mode_at = time.monotonic()
            self.reset()

    def on_auto_limit(self, msg):
        v = float(msg.data)
        if math.isfinite(v) and 30 <= v <= self.MAX_AUTO:
            if v != self.auto_limit:
                self.auto_limit = v
                self.auto_limit_at = time.monotonic()
                self.reset()

    def on_auto_active(self, msg):
        self.auto_active = msg.data is True
        self.auto_active_at = time.monotonic()

    def on_manual_authorized(self, msg):
        self.manual_authorized = msg.data is True
        self.manual_authorized_at = time.monotonic()

    def on_web_arm(self, msg):
        self.web_arm = msg.data is True
        self.web_arm_at = time.monotonic()

    def on_depth(self, msg):
        self.depth_blocked = msg.data is not False
        self.depth_at = time.monotonic()

    def on_lidar_mask(self, msg):
        self.lidar_mask = int(msg.data) if 0 <= int(msg.data) <= 255 else 255
        self.lidar_at = time.monotonic()

    def on_motor(self, msg):
        try:
            s = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(s, dict):
            self.motor = s
            self.motor_at = time.monotonic()

    def on_rpm(self, mode, side, value):
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            self.request_at[mode][side] = None
            return
        self.request[mode][side] = float(value)
        self.request_at[mode][side] = time.monotonic()

    def fresh(self, stamp, duration, now):
        return stamp is not None and 0 <= now - stamp < duration

    def stopped(self, now):
        m = self.motor
        return (self.fresh(self.motor_at, self.STATUS_AGE, now)
                and m.get("fault") is False
                and m.get("stop_verified") is True
                and m.get("applied") == [0, 0]
                and m.get("actual") == [0, 0])

    def safe_lift(self, now):
        p = Path.home() / ".amr_lift_interlock.json"
        try:
            state = json.loads(p.read_text())
        except (OSError, ValueError):
            return False
        return (isinstance(state, dict) and
                isinstance(state.get("time"), (int, float)) and
                0 <= now - state["time"] < 0.6 and
                state.get("state") in ("IDLE", "TOP", "BOTTOM") and
                state.get("verified_stopped") is True)

    def sole_publisher(self):
        # A second CAN command publisher is unsafe even if nominally idle.
        for topic in ("/amr_mux/wheel_rpm_left",
                      "/amr_mux/wheel_rpm_right",
                      "/amr_mux/motion_authorized"):
            owners = self.get_publishers_info_by_topic(topic)
            if len(owners) != 1 or owners[0].node_name != self.get_name():
                return False
        return True

    def publish_output(self, allowed, l=0.0, r=0.0):
        if not self.live:
            return
        self.auth_pub.publish(Bool(data=allowed))
        self.left_pub.publish(Float64(data=l if allowed else 0.0))
        self.right_pub.publish(Float64(data=r if allowed else 0.0))

    def tick(self):
        now = time.monotonic()
        data = list(self.request[self.mode])
        stamps = self.request_at[self.mode]
        max_rpm = self.MAX_MANUAL if self.mode == "MANUAL" else self.auto_limit
        valid = all(v is not None and math.isfinite(v) and abs(v) <= max_rpm
                    for v in data)
        fresh = all(self.fresh(t, self.AGE, now) and t > self.mode_at
                    for t in stamps)
        active = (
            self.fresh(self.manual_authorized_at, self.AGE, now)
            and self.manual_authorized if self.mode == "MANUAL" else
            self.fresh(self.auto_active_at, self.AGE, now) and self.auto_active)
        arm_hold = (self.fresh(self.web_arm_at, self.AGE, now)
                    and self.web_arm and self.mode == "AUTO")
        # Pre-arm authorizes only a zero handshake; it never authorizes motion.
        prearm = arm_hold and not active
        motor_fresh = self.fresh(self.motor_at, self.STATUS_AGE, now)
        motor_ok = motor_fresh and self.motor.get("fault") is False
        # Front-only depth cannot certify rear or swept-path clearance.
        # A blocked or stale reading must inhibit every direction until
        # independent directional sensing is integrated and validated.
        depth_ok = (self.fresh(self.depth_at, self.DEPTH_AGE, now)
                    and not self.depth_blocked)
        depth_fresh = self.fresh(self.depth_at, self.DEPTH_AGE, now)
        lidar_fresh = self.fresh(self.lidar_at, self.LIDAR_AGE, now)
        mask = self.lidar_mask if lidar_fresh else 255
        # Advisory bitmask never overrides depth interlock or grants motion.
        # Eight-direction availability is a command-gate diagnostic, not
        # proof of obstacle-free rear/side space. Bench mode ignores LiDAR.
        directions = ("forward", "reverse", "pivot_left", "pivot_right",
                      "forward_left", "forward_right", "reverse_left", "reverse_right")
        bits = (1, 2, 4, 8, 16, 32, 64, 128)
        directional_clearance = {
            name: (depth_ok if self.bench_directional else
                   depth_ok and lidar_fresh and not bool(mask & bit))
            for name, bit in zip(directions, bits)
        }
        if self.bench_directional and depth_fresh and self.depth_blocked:
            directional_clearance.update({
                "reverse": True, "pivot_left": True, "pivot_right": True,
            })
        directional_clearance.update({
            "lidar_mask": mask, "lidar_fresh": lidar_fresh,
            "reason": ("DEPTH_STALE" if not depth_fresh else
                       "DEPTH_BLOCKED_BENCH_REVERSE_PIVOT_ONLY"
                       if self.bench_directional and self.depth_blocked else
                       "DEPTH_CLEAR_BENCH_ALL_DIRECTIONS"
                       if self.bench_directional else
                       "DEPTH_BLOCKED_OR_STALE" if not depth_ok else
                       "LIDAR_STALE_OR_INVALID" if not lidar_fresh else
                       "ADVISORY_ONLY"),
        })
        # RAISED-WHEELS BENCH ONLY: front depth does not establish rear/side clearance.
        # Permit pure reverse or equal-and-opposite pivot requests, not arcs.
        left, right = data
        reverse = valid and left < -1e-9 and right < -1e-9
        pivot = valid and abs(left + right) < 1e-6 and (
            (left < -1e-9 and right > 1e-9) or
            (left > 1e-9 and right < -1e-9))
        bench_escape = self.bench_directional and (reverse or pivot)
        # Preserve Nav2-requested RPM continuously. Do not inject timed
        # reverse/pivot commands or invent a trajectory in the motor mux.
        no_conflicts = self.sole_publisher() if self.live else True
        lift_ok = self.safe_lift(now)
        reason = "OK"
        if not self.live:
            reason = "DRY_RUN"
        elif not active and not prearm:
            reason = "NO_FRESH_MODE_SESSION"
        elif not no_conflicts:
            reason = "DUPLICATE_CAN_OWNER"
        elif not motor_ok:
            reason = "MOTOR_STATE_MISSING_OR_FAULT"
        elif not depth_fresh:
            reason = "DEPTH_NOT_FRESH"
        elif not depth_ok and not (prearm or bench_escape):
            reason = "DEPTH_FORWARD_BLOCKED"
        elif not self.bench_directional and (not lidar_fresh or mask == 255):
            reason = "LIDAR_NOT_CLEAR_OR_STALE"
        elif not lift_ok:
            reason = "LIFT_NOT_CONFIRMED_STOPPED"
        elif not prearm and (not fresh or not valid):
            reason = "RPM_MISSING_OR_INVALID"
        if reason == "DEPTH_FORWARD_BLOCKED" and self.phase == "ACTIVE":
            # Keep the established AUTO session armed at ZERO while the
            # forward-only depth gate blocks a fresh Nav2 command.
            # Do not revoke CAN authorization and force a re-arm cycle.
            # All other faults/stale inputs still disarm as before.
            self.publish_output(True, 0.0, 0.0)
        elif reason != "OK":
            self.reset()
            self.publish_output(False)
        else:
            if prearm:
                # No Nav2 goal: ARM state only, always ZERO on both wheels.
                if not self.stopped(now):
                    reason = "PREARM_WAIT_PHYSICAL_STOP"
                    self.publish_output(False)
                    self.reset()
                else:
                    self.phase = "ZERO_HANDSHAKE"
                    self.publish_output(True, 0.0, 0.0)
            elif self.phase == "DISARMED":
                if not self.stopped(now):
                    reason = "WAIT_PHYSICAL_STOP"
                    self.publish_output(False)
                elif self.mode == "MANUAL" and any(abs(v) > 1e-9 for v in data):
                    # Manual must return to neutral before accepting movement.
                    reason = "WAIT_FRESH_ZERO_PAIR"
                    self.publish_output(False)
                else:
                    # Auto may already have a nonzero Nav2 command by the time
                    # a goal is accepted. The mux itself issues the mandatory
                    # zero handshake while CAN confirms both drives stopped.
                    self.phase = "ZERO_HANDSHAKE"
                    self.publish_output(True, 0.0, 0.0)
            elif self.phase == "ZERO_HANDSHAKE":
                if not self.motor.get("armed"):
                    self.publish_output(True, 0.0, 0.0)
                elif not self.motor.get("stop_verified"):
                    reason = "WAIT_DRIVER_STOP_VERIFICATION"
                    self.publish_output(False)
                    self.reset()
                else:
                    self.phase = "ACTIVE"
                    self.publish_output(True, 0.0, 0.0)
            elif self.phase == "ACTIVE":
                if not self.motor.get("armed") or not self.motor.get("stop_verified"):
                    reason = "MOTOR_NOT_ARMED"
                    self.publish_output(False)
                    self.reset()
                else:
                    self.publish_output(True, data[0], data[1])
        report = {
            "requested_mode": self.mode, "phase": self.phase,
            "requested_auto_max_rpm": self.auto_limit,
            "first_trial_hard_cap_rpm": self.MAX_AUTO,
            "input_rpm": data, "input_fresh": fresh,
            "motor_stop_verified": self.stopped(now),
            "depth_clear": depth_ok, "bench_directional": self.bench_directional,
            "bench_escape_candidate": bench_escape,
            "directional_clearance": directional_clearance, "lift_safe": lift_ok,
            "live_enabled": self.live, "prearm_zero_only": prearm,
            "reason": reason,
        }
        self.status_pub.publish(String(data=json.dumps(report)))
        if reason != self.last_reason:
            self.get_logger().info("MUX " + reason)
            self.last_reason = reason


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--live-motor-output", action="store_true",
                   help="Explicit hardware enable; requires separate CAN --mux-input node")
    p.add_argument("--bench-depth-directional", action="store_true",
                   help="Raised wheels ONLY: allow reverse/pivot with fresh blocked depth; no LiDAR requirement")
    args = p.parse_args()
    rclpy.init()
    node = GuardedMotorMux(args.live_motor_output, args.bench_depth_directional)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_output(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
