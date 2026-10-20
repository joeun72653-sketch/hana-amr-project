#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from datetime import datetime
from zoneinfo import ZoneInfo


class EncoderEcho(Node):

    def __init__(self):
        super().__init__('encoder_echo')

        self.create_subscription(
            Float64MultiArray,
            '/sensor_calibration/encoder_wheel_delta',
            self.callback,
            10
        )

    def callback(self, msg):

        if len(msg.data) < 3:
            print("[WARN] Encoder 데이터 부족", flush=True)
            return

        timestamp = msg.data[0]
        encoder1 = msg.data[1]
        encoder2 = msg.data[2]

        # 감속비 15:1 적용
        left_wheel_rpm = encoder1 / 15.0
        right_wheel_rpm = encoder2 / 15.0

        try:
            kst = datetime.fromtimestamp(
                timestamp,
                tz=ZoneInfo("Asia/Seoul")
            )

            print(
                f"측정 시간(KST): {kst:%Y-%m-%d %H:%M:%S.%f} | "
                f"Left Wheel RPM: {left_wheel_rpm:.1f} | "
                f"Right Wheel RPM: {right_wheel_rpm:.1f}",
                flush=True
            )

        except (ValueError, OverflowError, OSError) as e:
            print(f"[ERROR] 시간 변환 실패: {e}", flush=True)


def main():

    rclpy.init()
    node = EncoderEcho()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
