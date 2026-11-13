"""Depth-only mode publishes exactly the required ROS topics (static contract)."""

import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parent.parent
DEPTH = ROOT / 'amr_websocket/amr_depth_ros_publisher.py'
LAUNCHER = ROOT / 'one_command_launch.py'
PROBE = ROOT / 'amr_tools/ros_topic_probe.py'


class DepthOnlyTests(unittest.TestCase):
    def test_depth_publisher_has_no_ai_color_mjpeg_or_network_server(self):
        source = DEPTH.read_text(encoding='utf-8')
        ast.parse(source)
        for forbidden in ('ultralytics', 'import cv2', 'ThreadingHTTPServer',
                          'OBSensorType.COLOR_SENSOR', 'model.predict('):
            self.assertNotIn(forbidden, source)
        self.assertEqual(source.count("self.create_publisher("), 2)
        self.assertIn("'/camera/depth/image_raw'", source)
        self.assertIn("'/camera/registered_color/camera_info'", source)

    def test_depth_contract_uses_sensor_intrinsics_and_capture_timestamp(self):
        source = DEPTH.read_text(encoding='utf-8')
        self.assertIn("frame.get_global_timestamp_us()", source)
        self.assertIn("frame.get_depth_scale()", source)
        self.assertIn("profile.get_intrinsic()", source)
        self.assertIn("msg.encoding = '32FC1'", source)
        self.assertIn("msg.step = width * 4", source)
        self.assertIn("msg.header.frame_id = self.frame_id", source)
        self.assertIn("info.header.frame_id = frame_id", source)
        self.assertIn("msg.data = data.tobytes(order='C')", source)

    def test_launcher_uses_depth_publisher_not_http_when_skipping_ai(self):
        source = LAUNCHER.read_text(encoding='utf-8')
        ast.parse(source)
        self.assertIn("SCRIPTS['camera_depth']", source)
        self.assertIn("'camera_depth' if args.skip_detech else 'camera'", source)
        self.assertIn("'sensor_msgs/msg/Image'", source)
        self.assertIn("'sensor_msgs/msg/CameraInfo'", source)
        self.assertIn("if not args.skip_camera and not args.skip_detech:", source)
        self.assertIn("not args.skip_camera and not args.skip_detech and not CAMERA_MODEL.is_dir()", source)

    def test_probe_can_read_depth_messages(self):
        probe = PROBE.read_text(encoding='utf-8')
        self.assertIn("'sensor_msgs/msg/Image': Image", probe)
        self.assertIn("'sensor_msgs/msg/CameraInfo': CameraInfo", probe)


if __name__ == '__main__':
    unittest.main()
