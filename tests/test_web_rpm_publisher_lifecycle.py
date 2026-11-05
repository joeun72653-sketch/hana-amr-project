"""Web RPM DDS publishers must disappear when the remote client DISARMs.

These tests use ROS stubs so they also run on machines without ROS installed.
"""

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


class FakeMessage:
    def __init__(self, data=None):
        self.data = data


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class FakeNode:
    def __init__(self, _name):
        self.publishers = []
        self.destroyed = []

    def create_publisher(self, _type, topic, _depth):
        publisher = FakePublisher(topic)
        self.publishers.append(publisher)
        return publisher

    def destroy_publisher(self, publisher):
        self.publishers.remove(publisher)
        self.destroyed.append(publisher)
        return True

    def create_subscription(self, *_args):
        return None

    def create_timer(self, *_args):
        return None

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *_args: None)


def load_bridge_module():
    rclpy = types.ModuleType('rclpy')
    node = types.ModuleType('rclpy.node')
    node.Node = FakeNode
    executors = types.ModuleType('rclpy.executors')
    executors.SingleThreadedExecutor = object
    std_msgs = types.ModuleType('std_msgs')
    messages = types.ModuleType('std_msgs.msg')
    messages.Bool = FakeMessage
    messages.Float64 = FakeMessage
    messages.String = FakeMessage
    modules = {
        'rclpy': rclpy,
        'rclpy.node': node,
        'rclpy.executors': executors,
        'std_msgs': std_msgs,
        'std_msgs.msg': messages,
        'websockets': types.ModuleType('websockets'),
    }
    path = (Path(__file__).resolve().parents[1] /
            'amr_remote_client/amr_ws_to_ros.py')
    spec = importlib.util.spec_from_file_location('test_amr_ws_bridge', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class WebRpmPublisherLifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_bridge_module()

    def setUp(self):
        flag_patch = patch.object(
            self.module, 'CAL_FLAG',
            types.SimpleNamespace(exists=lambda: False))
        flag_patch.start()
        self.addCleanup(flag_patch.stop)
        self.bridge = self.module.AMRBridge(bench_motion=True)

    def rpm_topics(self):
        return {pub.topic for pub in self.bridge.publishers
                if pub.topic.startswith('/wheel_rpm_')}

    def test_disarmed_start_has_no_rpm_publishers(self):
        self.assertEqual(self.rpm_topics(), set())
        self.assertEqual([pub.topic for pub in self.bridge.publishers],
                         ['/amr_motion_authorized'])

    def test_arm_disarm_rearm_lifecycle(self):
        self.bridge.arm('a' * 32)
        self.assertEqual(self.rpm_topics(),
                         {'/wheel_rpm_left', '/wheel_rpm_right'})
        left = self.bridge.left_pub
        right = self.bridge.right_pub
        self.bridge.publish(100, 120)
        self.assertEqual(left.messages, [100.0])
        self.assertEqual(right.messages, [120.0])

        self.bridge.stop(disarm=True)
        self.assertEqual(self.rpm_topics(), set())
        self.assertIsNone(self.bridge.left_pub)
        self.assertIsNone(self.bridge.right_pub)
        self.assertIn(left, self.bridge.destroyed)
        self.assertIn(right, self.bridge.destroyed)
        self.bridge.publish(200, 200)
        self.assertEqual(left.messages, [100.0])
        self.assertEqual(right.messages, [120.0])
        self.assertEqual(self.bridge.auth_pub.messages[-1], False)

        self.bridge.arm('b' * 32)
        self.assertEqual(self.rpm_topics(),
                         {'/wheel_rpm_left', '/wheel_rpm_right'})
        self.assertIsNot(left, self.bridge.left_pub)
        self.assertIsNot(right, self.bridge.right_pub)

    def test_stop_retains_publishers_until_disarm(self):
        self.bridge.arm('a' * 32)
        self.bridge.stop(disarm=False)
        self.assertEqual(self.rpm_topics(),
                         {'/wheel_rpm_left', '/wheel_rpm_right'})
        self.assertEqual(self.bridge.left_pub.messages[-1], 0.0)
        self.assertEqual(self.bridge.right_pub.messages[-1], 0.0)
        self.bridge.stop(disarm=True)
        self.assertEqual(self.rpm_topics(), set())

    def test_calibration_flag_releases_publishers(self):
        self.bridge.arm('a' * 32)
        flag = types.SimpleNamespace(exists=lambda: True)
        with patch.object(self.module, 'CAL_FLAG', flag):
            self.bridge.control_tick()
            self.bridge.arm('b' * 32)
        self.assertEqual(self.rpm_topics(), set())
        self.assertEqual(self.bridge.phase, 'DISARMED')
        self.assertEqual(self.bridge.auth_pub.messages[-1], False)


if __name__ == '__main__':
    unittest.main()
