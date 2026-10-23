#!/usr/bin/env python3
"""Read one ROS 2 message to verify startup readiness, without ros2 CLI/daemon.

Read-only subscriber. Never publishes a motion command or invokes a service.
"""

import argparse
import os
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

MESSAGE_TYPES = {
    'sensor_msgs/msg/Imu': Imu,
    'std_msgs/msg/String': String,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('topic')
    parser.add_argument('message_type', choices=tuple(MESSAGE_TYPES))
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--best-effort', action='store_true')
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')

    node = None
    received = False

    def callback(_msg):
        nonlocal received
        received = True

    try:
        rclpy.init()
        node = rclpy.create_node(f'amr_readiness_probe_{os.getpid()}', enable_rosout=False)
        qos = QoSProfile(
            depth=10,
            reliability=(ReliabilityPolicy.BEST_EFFORT
                         if args.best_effort else ReliabilityPolicy.RELIABLE),
            durability=DurabilityPolicy.VOLATILE,
        )
        # Keep the subscription alive through the spin loop.
        subscription = node.create_subscription(
            MESSAGE_TYPES[args.message_type], args.topic, callback, qos,
        )
        deadline = time.monotonic() + args.timeout
        while not received and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))
        if received:
            print(f'[PROBE READY] {args.topic}: message received', flush=True)
            return 0
        print(f'[PROBE TIMEOUT] No message from {args.topic} within {args.timeout:g}s',
              file=sys.stderr, flush=True)
        return 2
    except Exception as error:
        print(f'[PROBE ERROR] {type(error).__name__}: {error}', file=sys.stderr, flush=True)
        return 3
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
