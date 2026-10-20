#!/usr/bin/env python3
"""Real AMR: two ROS 2 wheel-RPM topics -> SystemBase uCAN -> CANopen.

No 15:1 gearbox conversion. ROS topics carry requested signed RPM values
for the drives (units must match 0x60FF drive configuration). To preserve
legacy forward polarity, left +RPM becomes node-1 negative and right +RPM
becomes node-2 positive. Confirm motor IDs/polarities on lifted wheels.

Safety: independent hardware E-stop AND drive-side comms-loss stop required.
The Python process cannot guarantee a physical stop after a CAN/link failure.
"""

import json
import math
import queue
import sys
import threading
import time
from pathlib import Path

CAN_PORT = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG04BIQZ-if00-port0'
CAN_BAUD = 115200
# CANopen 0x60FF carries a signed 32-bit integer. This is a wire-format bound,
# not a software speed limit or a verified safe mechanical speed.
CANOPEN_INT32_MAX = (1 << 31) - 1
COMMAND_TIMEOUT = 0.35
AUTH_TIMEOUT = 0.5  # Web bridge must refresh explicit permission
FEEDBACK_INTERVAL = 0.5
ENCODER_INTERVAL = 0.1
STOP_TIMEOUT = 5.0
NODE_FOR = {'left': 1, 'right': 2}  # verify physical wiring
CAN_SIGN = {'left': -1, 'right': +1}  # preserves old 'forward': (-rpm, +rpm)


def speed_payload(rpm):
    return b'\x23\xff\x60\x00' + int(rpm).to_bytes(4, 'little', signed=True)


class DualWheelState:
    """Two-sided zero handshake; a stale side disarms BOTH wheels."""

    def __init__(self, timeout=COMMAND_TIMEOUT):
        self.lock = threading.Lock()
        self.timeout = timeout
        self.command = {'left': 0, 'right': 0}
        self.received_at = {'left': None, 'right': None}
        self.zero_seen = {'left': False, 'right': False}
        self.zero_at = {'left': None, 'right': None}
        self.armed_at = None
        self.timeout_latched = False
        self.armed = False
        self.stop_verified = False
        self.stop_epoch = 0
        self.fault = False
        self.status = 'WAIT_BOTH_ZERO'
        self.applied = (0, 0)
        self.actual = (0, 0)
        self.shutdown = False
        self.auth_enabled = False
        self.auth_at = None

    def _latch(self, reason):
        # Called under self.lock. Force a NEW pair of zero messages.
        self.command = {'left': 0, 'right': 0}
        self.received_at = {'left': None, 'right': None}
        self.zero_seen = {'left': False, 'right': False}
        self.zero_at = {'left': None, 'right': None}
        self.armed_at = None
        self.armed = False
        self.stop_verified = False
        self.timeout_latched = self.timeout_latched or reason == 'WATCHDOG_TIMEOUT'
        self.stop_epoch += 1
        self.status = reason

    def latch(self, reason):
        with self.lock:
            if not self.fault:
                self._latch(reason)

    def set_fault(self, reason):
        with self.lock:
            self._latch(reason)
            self.fault = True

    def authorize(self, enabled, now=None):
        """Only an explicit fresh bridge authorization permits zero-handshake arming.

        A false edge cancels motion and forces an independently verified CAN stop.
        Repeated false messages do not create new stop epochs.
        """
        if now is None:
            now = time.monotonic()
        with self.lock:
            if self.fault or self.shutdown:
                return
            if enabled:
                self.auth_enabled = True
                self.auth_at = now
            else:
                if self.auth_enabled or self.armed:
                    self.auth_enabled = False
                    self.auth_at = None
                    self._latch('WEB_DISARM')
                else:
                    self.auth_at = None

    def receive(self, side, value, now=None):
        """Float64 has no timestamp: use LOCAL monotonic arrival time."""
        if now is None:
            now = time.monotonic()
        if side not in NODE_FOR:
            raise ValueError(f'Unexpected wheel side: {side}')
        with self.lock:
            if self.fault or self.shutdown:
                return False
            if (not isinstance(value, (int, float)) or
                    not math.isfinite(value) or abs(value) > CANOPEN_INT32_MAX):
                self._latch('INVALID_RPM')
                return False

            # Check the previous deadlines BEFORE consuming the new sample.
            self._expire_locked(now)
            if not self.auth_enabled:
                # Repeated DISARM zeros must never unlock the CAN drive.
                return False

            if not self.armed:
                # While latched, two fresh zero messages are an UNLOCK KEY,
                # never a motor RPM command. Both must arrive after the latch.
                if value != 0.0:
                    self.zero_seen = {'left': False, 'right': False}
                    self.zero_at = {'left': None, 'right': None}
                    if not self.timeout_latched:
                        self.status = 'WAIT_BOTH_ZERO'
                    return False

                self.zero_seen[side] = True
                self.zero_at[side] = now
                if (all(self.zero_seen.values()) and
                        all(t is not None and 0 <= now-t < self.timeout
                            for t in self.zero_at.values())):
                    # Unlock BOTH wheels atomically. Discard BOTH unlock
                    # messages. Each topic must provide a NEW post-unlock
                    # sample before ANY requested RPM reaches the drives.
                    self.armed = True
                    self.armed_at = now
                    self.command = {'left': 0, 'right': 0}
                    self.received_at = {'left': None, 'right': None}
                    self.zero_seen = {'left': False, 'right': False}
                    self.zero_at = {'left': None, 'right': None}
                    self.timeout_latched = False
                    self.status = 'ARMED_WAIT_NEW_RPM'
                    return True
                return False

            # These samples are AFTER the unlock key, and only these may
            # be used for requested motor commands. A regular 0 here is
            # a legitimate stop command, unlike a handshake zero.
            self.command[side] = int(round(value))
            self.received_at[side] = now
            if self._both_fresh_locked(now):
                self.status = 'ARMED' if self.stop_verified else 'WAIT_STOP_VERIFY'
            return True

    def _both_fresh_locked(self, now):
        return all(t is not None and 0 <= now-t < self.timeout
                   for t in self.received_at.values())

    def _expire_locked(self, now):
        if self.auth_enabled and (self.auth_at is None or
                                  not 0 <= now - self.auth_at < AUTH_TIMEOUT):
            self.auth_enabled = False
            self.auth_at = None
            self._latch('AUTH_TIMEOUT')
        if not self.armed:
            return
        # A newly unlocked side that never supplies its first RPM must
        # still expire after 350 ms.
        for t in self.received_at.values():
            t = self.armed_at if t is None else t
            if t is None or not (0 <= now-t < self.timeout):
                self._latch('WATCHDOG_TIMEOUT')
                break

    def snapshot(self, now=None):
        if now is None:
            now = time.monotonic()
        with self.lock:
            self._expire_locked(now)
            allowed = self.armed and self.stop_verified and not self.fault and self._both_fresh_locked(now)
            desired = (
                CAN_SIGN['left'] * self.command['left'],
                CAN_SIGN['right'] * self.command['right'],
            ) if allowed else (0, 0)
            return {
                'desired': desired,
                'armed': self.armed,
                'web_authorized': self.auth_enabled,
                'timeout_latched': self.timeout_latched,
                'stop_verified': self.stop_verified,
                'epoch': self.stop_epoch,
                'fault': self.fault,
                'status': self.status,
                'applied': self.applied,
                'actual': self.actual,
                'shutdown': self.shutdown,
            }

    def verify_stop_epoch(self, epoch):
        with self.lock:
            if epoch == self.stop_epoch and not self.fault:
                self.stop_verified = True
                if self.armed and all(t is not None for t in self.received_at.values()):
                    self.status = 'ARMED'
                elif not self.auth_enabled:
                    self.status = 'DISARMED'


def timeout_heartbeat(latched, previously_latched):
    """Value for /rpm_topic_timeout: 10 Hz True, one False on reset."""
    if latched:
        return True
    if previously_latched:
        return False
    return None


class CanWorker(threading.Thread):
    def __init__(self, state):
        super().__init__(daemon=True)
        self.state = state
        self.ready = threading.Event()
        self.applied = (0, 0)
        self.last_feedback = 0.0
        self.last_encoder = 0.0
        self.previous_encoder = None
        self.encoder_samples = queue.Queue(maxsize=10)
        self.stop_tracking_lock = threading.Lock()
        self.stop_verify_count = 0
        self.stop_verified_at = None

    def _set_applied(self, value):
        self.applied = tuple(value)
        with self.state.lock:
            self.state.applied = self.applied

    def write_speed(self, can, node, rpm):
        can.ser.reset_input_buffer()
        can.buffer.clear()
        can.send(node, speed_payload(rpm))
        response = can.wait_sdo(node, 0x60FF)
        if not response or response[0] != 0x60:
            raise RuntimeError(f'Node {node} speed write ACK failed')
        print(f'[CAN] Node{node} target={rpm}', flush=True)

    def stop_verify(self, can):
        # Transmit BOTH zeros BEFORE waiting for either acknowledgment.
        pending = {1, 2}
        for node in (1, 2):
            can.send(node, speed_payload(0))
        deadline = time.monotonic() + 2.0
        while pending and time.monotonic() < deadline:
            response = can.receive(max(0, deadline - time.monotonic()))
            if response is None:
                break
            can_id, data = response
            node = can_id - 0x580
            if node not in pending or len(data) < 4 or data[1:4] != b'\xff\x60\x00':
                continue
            if data[0] == 0x80:
                raise RuntimeError(f'Node{node} STOP SDO abort')
            if data[0] == 0x60:
                pending.remove(node)
        if pending:
            raise TimeoutError(f'STOP ACK missing: {sorted(pending)}')
        self._set_applied((0, 0))
        deadline = time.monotonic() + STOP_TIMEOUT
        stable = 0
        while time.monotonic() < deadline:
            actual = (can.read_sdo(1, 0x606C, signed=True),
                      can.read_sdo(2, 0x606C, signed=True))
            with self.state.lock:
                self.state.actual = actual
            stable = stable + 1 if all(abs(x) <= 1 for x in actual) else 0
            if stable >= 3:
                for node in (1, 2):
                    if can.read_sdo(node, 0x60FF, signed=True) != 0:
                        raise RuntimeError(f'Node{node} target not zero')
                with self.stop_tracking_lock:
                    self.stop_verify_count += 1
                    self.stop_verified_at = time.monotonic()
                    count = self.stop_verify_count
                print('[STOP VERIFIED] actual=', actual,
                      'count=', count, flush=True)
                return
            time.sleep(0.1)
        raise TimeoutError('STOP verification timeout')

    def read_sdo_init(self, can, node, index, signed=False):
        """Retry timed-out SDO reads only during startup verification."""
        for attempt in range(1, 4):
            try:
                value = can.read_sdo(node, index, signed=signed)
                time.sleep(0.1)
                return value
            except TimeoutError as exc:
                print(
                    f'[SDO INIT] Node {node} 0x{index:04X} '
                    f'timeout ({attempt}/3): {exc}',
                    flush=True,
                )
                if attempt == 3:
                    raise
                time.sleep(0.2)

    def run(self):
        can = None
        try:
            import serial
            sys.path.insert(0, str(Path.home() / 'amr_motor_control'))
            from amr_keyboard_safe import UCan

            with serial.Serial(CAN_PORT, CAN_BAUD, timeout=0.02) as ser:
                can = UCan(ser)
                # safe_start writes zero targets, then verifies drive state.
                # Its own SDO reads are unchanged; this retry applies only
                # to the additional verification reads below.
                can.safe_start()
                print('[INIT] CANopen state verification', flush=True)
                time.sleep(0.2)
                for node in (1, 2):
                    sw = self.read_sdo_init(can, node, 0x6041)
                    target = self.read_sdo_init(can, node, 0x60FF, signed=True)
                    actual = self.read_sdo_init(can, node, 0x606C, signed=True)
                    print(
                        f'[INIT] Node {node}: SW=0x{sw:04X}, '
                        f'Target={target}, Actual={actual}',
                        flush=True,
                    )
                    if (sw & 0x006F) != 0x0027 or target != 0 or actual != 0:
                        raise RuntimeError(
                            f'Node{node} not enabled/zero: '
                            f'SW=0x{sw:04X}, target={target}, actual={actual}'
                        )
                self.stop_verify(can)
                self.state.verify_stop_epoch(0)
                self.ready.set()
                print('[READY] ROS wheel RPM topics; requires BOTH zero messages to arm', flush=True)
                while True:
                    s = self.state.snapshot()
                    if s['shutdown']:
                        break
                    # Every latch (watchdog, invalid value, etc.) demands a
                    # physically verified dual stop, even if both zeros arrive
                    # before the worker observes that latch.
                    if not s['stop_verified']:
                        self.stop_verify(can)
                        self.state.verify_stop_epoch(s['epoch'])
                        continue
                    desired = s['desired']
                    reversing = any(
                        old != 0 and new != 0 and (old > 0) != (new > 0)
                        for old, new in zip(self.applied, desired)
                    )
                    if reversing:
                        print('[REVERSE] stop both wheels and verify', flush=True)
                        self.stop_verify(can)
                        continue  # Always take a fresh two-topic snapshot.
                    if desired == (0, 0) and self.applied != (0, 0):
                        self.stop_verify(can)
                        continue
                    if desired != (0, 0) and desired != self.applied:
                        for i, rpm in enumerate(desired):
                            fresh = self.state.snapshot()
                            if not fresh['stop_verified'] or fresh['desired'] != desired or fresh['epoch'] != s['epoch']:
                                break
                            if rpm != self.applied[i]:
                                self.write_speed(can, i + 1, rpm)
                                next_applied = list(self.applied)
                                next_applied[i] = rpm
                                self._set_applied(next_applied)
                    now = time.monotonic()
                    # CANopen 0x606C: actual motor-shaft RPM.
                    # Publish forward-positive motor RPM without gear reduction.
                    if now - self.last_encoder >= ENCODER_INTERVAL:
                        raw_left = can.read_sdo(1, 0x606C, signed=True)
                        raw_right = can.read_sdo(2, 0x606C, signed=True)
                        stamp = time.time()

                        with self.state.lock:
                            self.state.actual = (raw_left, raw_right)

                        left_rpm = raw_left * CAN_SIGN['left']
                        right_rpm = raw_right * CAN_SIGN['right']
                        sample = (stamp, float(left_rpm), float(right_rpm))

                        try:
                            self.encoder_samples.put_nowait(sample)
                        except queue.Full:
                            try:
                                self.encoder_samples.get_nowait()
                            except queue.Empty:
                                pass
                            self.encoder_samples.put_nowait(sample)

                        self.last_encoder = time.monotonic()
                        self.last_feedback = self.last_encoder
                    time.sleep(0.02)
                self.stop_verify(can)
        except Exception as exc:
            print('[CRITICAL CAN]', repr(exc), flush=True)
            self.state.set_fault('CAN_FAULT')
            # The CAN link may itself have failed; software cannot guarantee stop.
            print('[ACTION] Use independent hardware E-stop; inspect drive status', flush=True)
        finally:
            self.ready.set()


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64, Bool, Float64MultiArray, String

    state = DualWheelState()
    worker = CanWorker(state)
    worker.start()
    if not worker.ready.wait(timeout=15.0) or state.snapshot()['fault']:
        raise RuntimeError('CAN initialization failed or timed out; ROS node not started')

    rclpy.init()

    class WheelRpmNode(Node):
        def __init__(self):
            super().__init__('amr_wheel_rpm_can')
            self.create_subscription(Float64, 'wheel_rpm_left',
                                     lambda m: self.receive('left', m.data), 1)
            self.create_subscription(Float64, 'wheel_rpm_right',
                                     lambda m: self.receive('right', m.data), 1)
            self.create_subscription(Bool, '/amr_motion_authorized',
                                     lambda m: state.authorize(m.data), 10)
            self.timeout_pub = self.create_publisher(Bool, 'rpm_topic_timeout', 10)
            self.state_pub = self.create_publisher(String, '/amr_motor_state', 10)
            self.encoder_pub = self.create_publisher(
                Float64MultiArray, '/sensor_calibration/encoder_wheel_delta', 10)
            self.timer = self.create_timer(0.1, self.check_status)
            self.last_status = None
            self.was_timeout_latched = False
            self.get_logger().info(
                'Listening wheel_rpm_left/right (std_msgs/Float64); no software RPM limit; '
                f'watchdog {COMMAND_TIMEOUT*1000:.0f} ms PER topic. '
                'Only explicit /amr_motion_authorized=True permits arming. '
                'Both topics must receive 0 AFTER a latch to unlock together. '
                'These zeros are discarded; motion waits for two NEW samples. '
                'Publishes Bool True on /rpm_topic_timeout at 10 Hz while '
                'watchdog-latched; Bool False once after unlocking.')

        def receive(self, side, rpm):
            accepted = state.receive(side, rpm)
            if not accepted and state.snapshot()['status'] == 'INVALID_RPM':
                self.get_logger().error(f'Invalid {side} RPM {rpm!r}; both sides latched')

        def check_status(self):
            # ROS publishing stays in the ROS executor thread.
            while True:
                try:
                    sample = worker.encoder_samples.get_nowait()
                except queue.Empty:
                    break
                self.encoder_pub.publish(Float64MultiArray(data=list(sample)))
            s = state.snapshot()  # Also checks both 350 ms deadlines.
            with worker.stop_tracking_lock:
                stop_count = worker.stop_verify_count
                stop_at = worker.stop_verified_at
            self.state_pub.publish(String(data=json.dumps({
                'status': s['status'],
                'armed': s['armed'],
                'web_authorized': s['web_authorized'],
                'stop_verified': s['stop_verified'],
                'fault': s['fault'],
                'timeout_latched': s['timeout_latched'],
                'applied': list(s['applied']),
                'actual': list(s['actual']),
                'stop_epoch': s['epoch'],
                'stop_verify_count': stop_count,
                'stop_verified_at': stop_at,
            }, ensure_ascii=False)))
            pulse = timeout_heartbeat(s['timeout_latched'], self.was_timeout_latched)
            if pulse is not None:
                self.timeout_pub.publish(Bool(data=pulse))
            self.was_timeout_latched = s['timeout_latched']
            if s['status'] != self.last_status:
                details = (f"State: {s['status']} | armed={s['armed']} | "
                           f"stop_verified={s['stop_verified']}")
                if s['fault']:
                    self.get_logger().error(details + '; use independent hardware E-stop')
                else:
                    self.get_logger().info(details)
                self.last_status = s['status']

    node = WheelRpmNode()
    try:
        rclpy.spin(node)
    finally:
        with state.lock:
            state.shutdown = True
            state.armed = False
            state.command = {'left': 0, 'right': 0}
        # Worker performs verified stop on exit; never claim guaranteed stop.
        worker.join(timeout=STOP_TIMEOUT + 3)
        if worker.is_alive():
            print('[CRITICAL] CAN worker not stopped; use independent E-stop', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()