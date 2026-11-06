#!/usr/bin/env python3
"""BENCH DIAGNOSTIC: publish only zero RPM to isolated MANUAL request topics.

Never publishes /wheel_rpm_left, /wheel_rpm_right, /amr_motion_authorized,
CAN frames, or any nonzero RPM. This does not implement existing web commands.
Used to verify a zero-input handshake in the non-actuating mode mux.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class IsolatedManualZero(Node):
    def __init__(self):
        super().__init__("amr_manual_zero_diagnostic")
        self.left = self.create_publisher(
            Float64, "/amr_manual/wheel_rpm_left_requested", 10)
        self.right = self.create_publisher(
            Float64, "/amr_manual/wheel_rpm_right_requested", 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().warning(
            "ISOLATED ZERO-ONLY DIAGNOSTIC; not a real manual web controller")

    def tick(self):
        self.left.publish(Float64(data=0.0))
        self.right.publish(Float64(data=0.0))


def main():
    rclpy.init()
    node = IsolatedManualZero()
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
