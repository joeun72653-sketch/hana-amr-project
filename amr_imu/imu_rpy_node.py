#!/usr/bin/env python3

import math
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class IMURPYNode(Node):

    def __init__(self):
        super().__init__('amr_imu_rpy')

        self.publisher = self.create_publisher(
            String,
            '/amr/imu/rpy',
            10
        )

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.callback,
            10
        )

        self.get_logger().info(
            'IMU Roll Pitch Yaw node started'
        )

    def callback(self, msg):

        q = msg.orientation

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        # Quaternion -> Euler
        sinr = 2.0 * (w*x + y*z)
        cosr = 1.0 - 2.0 * (x*x + y*y)

        roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (w*y - z*x)
        sinp = max(-1.0, min(1.0, sinp))

        pitch = math.asin(sinp)

        siny = 2.0 * (w*z + x*y)
        cosy = 1.0 - 2.0 * (y*y + z*z)

        yaw = math.atan2(siny, cosy)

        data = {
            'roll': round(math.degrees(roll), 2),
            'pitch': round(math.degrees(pitch), 2),
            'yaw': round(math.degrees(yaw), 2)
        }

        output = String()
        output.data = json.dumps(data)

        self.publisher.publish(output)


def main():
    rclpy.init()

    node = IMURPYNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
