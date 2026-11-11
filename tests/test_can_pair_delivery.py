"""CAN target-pair regression tests (no ROS, serial adapter or motor needed)."""

import importlib.util
import unittest
from collections import deque
from pathlib import Path


MODULE_PATH = (Path(__file__).resolve().parents[1] / 'amr_remote_client'
               / 'amr_ros_wheel_rpm_can.py')
spec = importlib.util.spec_from_file_location('can_wheel_pair', MODULE_PATH)
can_driver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(can_driver)


class FakeSerial:
    def reset_input_buffer(self):
        pass


class FakeCan:
    def __init__(self):
        self.ser = FakeSerial()
        self.buffer = bytearray()
        self.targets = {1: 0, 2: 0}
        self.actual = {1: 0, 2: 0}
        self.responses = deque()
        self.events = []

    def send(self, node, payload):
        rpm = int.from_bytes(payload[4:], 'little', signed=True)
        self.targets[node] = rpm
        self.events.append(('send', node, rpm))
        self.responses.append((0x580 + node, b'\x60\xff\x60\x00\x00\x00\x00\x00'))

    def receive(self, timeout):
        if not self.responses:
            raise AssertionError('Unexpected wait with no CAN response')
        can_id, frame = self.responses.popleft()
        self.events.append(('ack', can_id - 0x580))
        return can_id, frame

    def read_sdo(self, node, index, signed=False):
        if index == 0x606C:
            return self.actual[node]
        if index == 0x60FF:
            return self.targets[node]
        raise AssertionError(f'Unexpected SDO index 0x{index:04X}')


class PairedCanTests(unittest.TestCase):
    def setUp(self):
        self.state = can_driver.DualWheelState()
        self.worker = can_driver.CanWorker(self.state)
        self.can = FakeCan()

    @staticmethod
    def snapshot(desired, stop_verified=True, epoch=0):
        return {'desired': tuple(desired), 'stop_verified': stop_verified,
                'epoch': epoch}

    def test_both_targets_sent_before_waiting_for_ack(self):
        self.worker.write_pair(self.can, (350, -370))
        self.assertEqual(self.can.events[:4], [
            ('send', 1, 350), ('send', 2, -370),
            ('ack', 1), ('ack', 2),
        ])
        self.assertEqual(self.worker.applied, (350, -370))

    def test_short_zero_does_not_hold_next_nonzero_command(self):
        self.worker.write_pair(self.can, (350, -350))
        self.can.actual.update({1: 300, 2: -300})
        self.worker.process_motion(self.can, self.snapshot((0, 0)))
        self.assertEqual(self.worker.applied, (0, 0))
        self.assertIsNotNone(self.worker.stop_monitor)
        self.worker.process_motion(self.can, self.snapshot((370, -320)))
        self.assertEqual(self.worker.applied, (370, -320))
        self.assertIsNone(self.worker.stop_monitor)
        self.assertEqual(self.can.targets, {1: 370, 2: -320})

    def test_stop_feedback_is_monitored_after_commands_are_sent(self):
        self.worker.write_pair(self.can, (330, -330))
        self.can.actual.update({1: 330, 2: -330})
        self.worker.process_motion(self.can, self.snapshot((0, 0)))
        self.worker.poll_motor_feedback(self.can)
        self.assertEqual(self.worker.stop_verify_count, 0)
        self.can.actual.update({1: 0, 2: 0})
        for _ in range(3):
            self.worker.poll_motor_feedback(self.can)
        self.assertIsNone(self.worker.stop_monitor)
        self.assertEqual(self.worker.stop_verify_count, 1)

    def test_latched_stop_requires_actual_zero_before_unlock(self):
        self.state.stop_epoch = 3
        self.state.stop_verified = False
        self.worker.process_motion(self.can,
                                   self.snapshot((0, 0), False, 3))
        self.assertFalse(self.state.stop_verified)
        self.assertEqual(self.worker.stop_requested_epoch, 3)
        for _ in range(3):
            self.worker.poll_motor_feedback(self.can)
        self.assertTrue(self.state.stop_verified)

    def test_reversing_waits_for_actual_stop(self):
        self.worker.write_pair(self.can, (250, -250))
        self.can.actual.update({1: 250, 2: -250})
        self.worker.process_motion(self.can, self.snapshot((-250, 250)))
        self.assertEqual(self.worker.applied, (0, 0))
        self.assertTrue(self.worker.reversal_pending)
        self.worker.process_motion(self.can, self.snapshot((-250, 250)))
        self.assertEqual(self.worker.applied, (0, 0))
        self.can.actual.update({1: 0, 2: 0})
        for _ in range(3):
            self.worker.poll_motor_feedback(self.can)
        self.worker.process_motion(self.can, self.snapshot((-250, 250)))
        self.worker.process_motion(self.can, self.snapshot((-250, 250)))
        self.assertEqual(self.worker.applied, (-250, 250))

    def test_stale_single_side_latches_both_wheels(self):
        self.state.armed = True
        self.state.stop_verified = True
        self.state.armed_at = 10.0
        self.state.received_at = {'left': 10.30, 'right': 10.0}
        self.state.command = {'left': 300, 'right': 300}
        snapshot = self.state.snapshot(now=10.36)
        self.assertFalse(snapshot['armed'])
        self.assertTrue(snapshot['timeout_latched'])
        self.assertEqual(snapshot['desired'], (0, 0))


if __name__ == '__main__':
    unittest.main()
