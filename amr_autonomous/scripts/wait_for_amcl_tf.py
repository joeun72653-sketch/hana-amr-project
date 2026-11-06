#!/usr/bin/env python3
"""Wait for AMCL map->odom localization before launching Nav2 and any CAN node.

A saved occupancy map is not a pose estimate. The operator must use RViz
2D Pose Estimate against the actual robot pose. This script never sets a
pose automatically, never publishes any motor commands, and fails closed.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--max-age", type=float, default=2.0)
    args = parser.parse_args()
    if not 10 <= args.timeout <= 900 or not 0.1 <= args.max_age <= 10:
        parser.error("Invalid timeout or TF age")
    rclpy.init()
    node = Node("amr_wait_for_localization_tf")
    tf = Buffer()
    listener = TransformListener(tf, node, spin_thread=False)
    deadline = time.monotonic() + args.timeout
    last_log = -10.0
    success = False
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            elapsed = args.timeout - (deadline-time.monotonic())
            if elapsed-last_log >= 3:
                print("[LOCALIZATION] Waiting for RViz 2D Pose Estimate / fresh map->odom TF...",
                      flush=True)
                last_log = elapsed
            try:
                t = tf.lookup_transform("map", "odom", Time())
                stamp = t.header.stamp.sec + t.header.stamp.nanosec*1e-9
                now = node.get_clock().now().nanoseconds*1e-9
                age = now-stamp
                values = (t.transform.translation.x, t.transform.translation.y,
                          t.transform.rotation.x, t.transform.rotation.y,
                          t.transform.rotation.z, t.transform.rotation.w)
                if stamp > 0 and -0.1 <= age <= args.max_age and all(map(math.isfinite,values)):
                    print(f"[LOCALIZATION READY] map->odom TF age={age:.3f}s", flush=True)
                    success = True
                    break
            except Exception:
                pass
        if not success:
            print("[LOCALIZATION FAILED] No fresh AMCL map->odom TF. "
                  "RViz 2D Pose Estimate required; CAN not started.", file=sys.stderr, flush=True)
            return 1
        return 0
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
