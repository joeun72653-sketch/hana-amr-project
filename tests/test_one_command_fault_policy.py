"""Regression tests for One Command child-fault classification (no ROS/hardware)."""

import unittest

from one_command_launch import console_log_line, is_fatal_child_line


class FaultPolicyTests(unittest.TestCase):
    BRIDGE_MARKERS = ('[ROS ERROR]', '[FAULT]')
    CAN_MARKERS = ('[CRITICAL CAN]', '[CRITICAL] CAN worker')

    def test_web_command_timeout_is_recoverable(self):
        self.assertFalse(is_fatal_child_line(
            'bridge', '[FAULT] command_timeout', self.BRIDGE_MARKERS))

    def test_web_timeout_still_prints_in_quiet_mode(self):
        visible, _ = console_log_line('[FAULT] command_timeout', False, False)
        self.assertTrue(visible)

    def test_other_web_faults_remain_fatal(self):
        for fault in ('motor_not_ready_or_stale', 'can_fault',
                      'handshake_timeout', 'stop_timeout:waiting_for_zero'):
            with self.subTest(fault=fault):
                self.assertTrue(is_fatal_child_line(
                    'bridge', f'[FAULT] {fault}', self.BRIDGE_MARKERS))

    def test_bridge_ros_crash_remains_fatal(self):
        self.assertTrue(is_fatal_child_line(
            'bridge', '[ROS ERROR] executor failed', self.BRIDGE_MARKERS))

    def test_can_crash_remains_fatal(self):
        self.assertTrue(is_fatal_child_line(
            'can', '[CRITICAL CAN] RuntimeError', self.CAN_MARKERS))

    def test_other_children_do_not_inherit_bridge_exception(self):
        self.assertTrue(is_fatal_child_line(
            'camera', '[FAULT] command_timeout', ('[FAULT]',)))

    def test_disconnection_is_not_fatal_under_existing_policy(self):
        self.assertFalse(is_fatal_child_line(
            'bridge', '[DISCONNECTED] ping timeout', self.BRIDGE_MARKERS))


if __name__ == '__main__':
    unittest.main()
