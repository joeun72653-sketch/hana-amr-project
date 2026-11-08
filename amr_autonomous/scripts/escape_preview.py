#!/usr/bin/env python3
"""Read-only depth/LiDAR escape candidate simulation. NEVER commands motors."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

MOTIONS = (("reverse", 2), ("pivot_left", 4), ("pivot_right", 8),
           ("reverse_left", 64), ("reverse_right", 128))

class EscapePreview(Node):
    def __init__(self):
        super().__init__("amr_escape_preview")
        self.depth = None
        self.depth_at = None
        self.mask = 255
        self.mask_at = None
        self.last = None
        self.create_subscription(Bool, "/amr/depth_obstacle", self.on_depth, 10)
        self.create_subscription(Int32, "/lidar_safty_bitmask", self.on_mask, 10)
        self.pub = self.create_publisher(String, "/amr_escape/preview", 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().warning("READ ONLY: candidate selection, NO CAN/RPM/cmd_vel")
    def on_depth(self, msg):
        self.depth = bool(msg.data)
        self.depth_at = time.monotonic()
    def on_mask(self, msg):
        self.mask = int(msg.data) if 0 <= int(msg.data) <= 255 else 255
        self.mask_at = time.monotonic()
    def tick(self):
        now = time.monotonic()
        depth_fresh = self.depth_at is not None and now-self.depth_at < 0.3
        lidar_fresh = self.mask_at is not None and now-self.mask_at < 0.35
        valid = depth_fresh and lidar_fresh and self.mask != 255
        candidates = [name for name, bit in MOTIONS if valid and not self.mask & bit]
        if not valid:
            action = "STOP_SENSOR_INVALID"
        elif not self.depth:
            action = "NORMAL_NAV2"
        elif candidates:
            action = "ESCAPE_CANDIDATE_" + candidates[0].upper()
        else:
            action = "STOP_NO_CLEAR_DIRECTION"
        report = dict(action=action, depth_blocked=self.depth,
                      depth_fresh=depth_fresh, lidar_mask=self.mask,
                      lidar_fresh=lidar_fresh, candidate_directions=candidates,
                      motor_output=False, note="Advisory only; requires physical clearance validation")
        self.pub.publish(String(data=json.dumps(report)))
        if action != self.last:
            self.get_logger().info(json.dumps(report))
            self.last = action

def main():
    rclpy.init()
    node = EscapePreview()
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
