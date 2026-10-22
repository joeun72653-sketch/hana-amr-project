#!/usr/bin/env python3
# test

import json
from datetime import datetime
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ActualRPMViewer(Node):
    def __init__(self):
        super().__init__('actual_rpm_viewer')

        self.subscription = self.create_subscription(
            String,
            '/amr_motor_state',
            self.callback,
            10
        )

        print("[INFO] CANopen 실제 RPM 모니터링 시작", flush=True)

    def callback(self, msg):
        try:
            state = json.loads(msg.data)
            actual = state.get('actual')

            if not isinstance(actual, list) or len(actual) != 2:
                return

            now = datetime.now(ZoneInfo('Asia/Seoul'))

            print(
                f"수신 시간(KST): {now:%Y-%m-%d %H:%M:%S.%f} | "
                f"left_wheel: {actual[0]} RPM | "
                f"right_wheel: {actual[1]} RPM",
                flush=True
            )

        except Exception as e:
            print(f"[ERROR] {e}", flush=True)


def main():
    rclpy.init()
    node = ActualRPMViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
