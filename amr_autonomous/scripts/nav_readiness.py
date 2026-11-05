#!/usr/bin/env python3
"""ROS2 Jazzy read-only Nav2 inputs probe for AMR_QR.

No publishers, actions, CAN calls, motor commands, or hardware control.
Collects whether expected ROS messages arrive within the timeout.

Usage:
  source /opt/ros/jazzy/setup.bash
  source ~/real_amr/install/setup.bash
  export ROS_DOMAIN_ID=31
  python3 nav_readiness.py --timeout 10 --require scan --require odom --require map --require amcl
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan


TOPICS = {
    "scan": "/scan_slam",
    "odom": "/lidar_icp/odom",
    "map": "/map",
    "amcl": "/amcl_pose",
    "cmd_vel": "/mapex/cmd_vel",
}


class ReadinessProbe(Node):
    """Subscribe only; do not emit velocity/authorization/goal commands."""

    def __init__(self) -> None:
        super().__init__("amr_qr_readiness_probe")
        self.received = dict.fromkeys(TOPICS, False)
        self.subscriptions_keepalive = []

        # Map Server usually publishes transient-local maps.
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        # Compatible with best-effort sensor publishers and reliable writers.
        sensor_qos = qos_profile_sensor_data

        definitions = (
            ("scan", LaserScan, sensor_qos),
            ("odom", Odometry, sensor_qos),
            ("map", OccupancyGrid, map_qos),
            ("amcl", PoseWithCovarianceStamped, sensor_qos),
            ("cmd_vel", TwistStamped, sensor_qos),
        )

        for name, msg_type, qos in definitions:
            sub = self.create_subscription(
                msg_type, TOPICS[name], self._callback(name), qos
            )
            self.subscriptions_keepalive.append(sub)

    def _callback(self, name: str):
        def record(_message) -> None:
            self.received[name] = True
        return record


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument(
        "--require", action="append", choices=tuple(TOPICS),
        default=[], help="Require a message from this topic (repeatable)"
    )
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    rclpy.init()
    node = ReadinessProbe()
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.25, max(0.0, deadline-time.monotonic())))
            if args.require and all(node.received[k] for k in args.require):
                break

        print("[AMR_QR READINESS] Message reception only; not motion approval.")
        for name, topic in TOPICS.items():
            state = "RECEIVED" if node.received[name] else "NOT SEEN"
            required = " (required)" if name in args.require else ""
            print(f"  {name:8} {topic:38} {state}{required}")

        missing = [name for name in args.require if not node.received[name]]
        if missing:
            print(f"[AMR_QR READINESS] Missing required: {', '.join(missing)}")
            return 2
        print("[AMR_QR READINESS] Requested messages received.")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
