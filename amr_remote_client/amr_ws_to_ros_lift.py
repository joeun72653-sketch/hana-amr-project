#!/usr/bin/env python3
"""AMR ROS2 Jazzy WebSocket bridge. 30-500 RPM bench request validation.

Default mode publishes no nonzero RPM. --bench-motion is an explicit opt-in.
The CAN worker, driver comms-loss stop, and independent E-stop remain required.
"""
import argparse
import asyncio
import json
import math
import os
import threading
import time
from pathlib import Path

import rclpy
import websockets
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, Float64, String

CAL_FLAG = Path.home() / '.amr_calibration_active'
LIFT_STATE_FILE = Path.home() / '.amr_lift_interlock.json'
MOTOR_STATE_FILE = Path.home() / '.amr_motor_interlock.json'

def _read_interlock(path):
    try:
        data = json.loads(path.read_text())
        return data if isinstance(data, dict) else None
    except (OSError, ValueError):
        return None

def _write_interlock(path, data):
    tmp = path.with_name(path.name + '.' + str(os.getpid()) + '.tmp')
    tmp.write_text(json.dumps(data))
    os.replace(tmp, path)

SERVER = 'ws://192.168.0.31:8765'
MIN_REQUEST_RPM = 30.0
MAX_REQUEST_RPM = 500.0
MAX_APPROVED_RPM = 500.0  # requested bench ceiling; verify under load
COMMAND_TIMEOUT = 0.25
STATE_TIMEOUT = 0.5
PUBLISH_INTERVAL = 0.05
HANDSHAKE_TIMEOUT = 2.0
STOP_TIMEOUT = 5.0
COMMANDS = {'forward', 'reverse', 'left', 'right',
            'forward_left', 'forward_right', 'reverse_left', 'reverse_right'}


def calculate_rpm(command, rpm):
    """Wheel topic RPM; CAN node applies the physical left-wheel sign.

    A/D: pivot in place. Diagonal combinations: 50% inner wheel.
    """
    return {
        'forward': (rpm, rpm),
        'reverse': (-rpm, -rpm),
        'left': (-rpm, rpm),
        'right': (rpm, -rpm),
        'forward_left': (rpm * 0.5, rpm),
        'forward_right': (rpm, rpm * 0.5),
        'reverse_left': (-rpm * 0.5, -rpm),
        'reverse_right': (-rpm, -rpm * 0.5),
    }[command]


class AMRBridge(Node):
    def __init__(self, bench_motion=False):
        super().__init__('amr_ws_to_ros')
        self.bench_motion = bench_motion
        self.calibration_hold = False
        self.left_pub = self.create_publisher(Float64, '/wheel_rpm_left', 10)
        self.right_pub = self.create_publisher(Float64, '/wheel_rpm_right', 10)
        self.auth_pub = self.create_publisher(Bool, '/amr_motion_authorized', 10)
        self.lock = threading.RLock()
        self.motor_state = None
        self.state_at = None
        self.session = None
        self.phase = 'DISARMED'
        self.requested = (0.0, 0.0)
        self.command_at = None
        self.stop_count_before = None
        self.stop_requires_new = False
        self.stop_at = None
        self.stop_status_at = None
        self.fault_reason = None
        self.depth_blocked = True
        self.depth_at = None
        self.must_release = False
        self.create_subscription(Bool, '/amr/depth_obstacle', self.on_depth, 10)
        self.create_subscription(String, '/amr_motor_state', self.on_state, 10)
        self.create_timer(PUBLISH_INTERVAL, self.control_tick)
        self.get_logger().info('AMR bench bridge initialized')

    def on_state(self, msg):
        try:
            state = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(state, dict):
            return
        with self.lock:
            self.motor_state = state
            self.state_at = time.monotonic()

    def on_depth(self, msg):
        with self.lock:
            self.depth_blocked = bool(msg.data)
            self.depth_at = time.monotonic()

    def depth_blocked_now(self):
        with self.lock:
            return (self.depth_at is None or
                    time.monotonic() - self.depth_at > 0.35 or
                    self.depth_blocked)

    def state(self):
        with self.lock:
            if self.state_at is None or time.monotonic() - self.state_at > STATE_TIMEOUT:
                return None
            return dict(self.motor_state)

    @staticmethod
    def is_stopped(s):
        return (s is not None and s.get('fault') is False
                and s.get('stop_verified') is True
                and s.get('applied') == [0, 0]
                and s.get('actual') == [0, 0])

    @staticmethod
    def is_ready(s):
        return (s is not None and s.get('armed') is True
                and s.get('fault') is False
                and s.get('stop_verified') is True
                and s.get('timeout_latched') is False
                and s.get('status') == 'ARMED')

    def publish(self, left, right):
        # Keep RPM publication ordered against DISARM under the same lock.
        with self.lock:
            if CAL_FLAG.exists() or self.session is None or self.phase == 'DISARMED':
                return
            self.left_pub.publish(Float64(data=float(left)))
            self.right_pub.publish(Float64(data=float(right)))

    def zero(self):
        self.publish(0, 0)

    def arm(self, token):
        if not self.lift_safe_for_drive():
            return
        if CAL_FLAG.exists():
            print('[CAL] ARM ignored during calibration', flush=True)
            return
        with self.lock:
            self.session = token
            self.phase = 'ARM_READY'
            self.requested = (0.0, 0.0)
            self.command_at = None
            self.fault_reason = None

    def set_phase(self, phase):
        with self.lock:
            self.phase = phase

    def request(self, left, right):
        if not self.lift_safe_for_drive():
            return False
        with self.lock:
            if self.session is None or self.phase not in ('ARMED_IDLE', 'RUNNING'):
                return False
            self.requested = (float(left), float(right))
            self.command_at = time.monotonic()
            self.phase = 'RUNNING'
            return True

    def stop(self, disarm=False):
        state = self.state()
        with self.lock:
            previous_phase = self.phase
            self.requested = (0.0, 0.0)
            self.command_at = None
            if disarm:
                self.session = None
                self.phase = 'DISARMED'
                self.auth_pub.publish(Bool(data=False))
            else:
                self.phase = 'STOPPING'
                self.stop_at = time.monotonic()
                self.stop_status_at = self.state_at
                self.stop_count_before = (state or {}).get('stop_verify_count')
                self.stop_requires_new = (
                    any(x != 0 for x in (state or {}).get('applied', []))
                    or any(x != 0 for x in (state or {}).get('actual', []))
                    or (previous_phase == 'RUNNING' and not self.is_stopped(state))
                )
        # Deauthorization is a separate ROS signal. The CAN worker performs
        # a verified dual-wheel zero stop on the authorization falling edge.
        # Never re-arm the CAN node by repeatedly publishing DISARM zeros.
        if not disarm:
            self.zero()

    def stop_completed(self):
        s = self.state()
        if not self.is_stopped(s):
            return False, 'waiting_for_zero'
        with self.lock:
            before = self.stop_count_before
            need_new = self.stop_requires_new
            after_stop_report = (self.state_at is not None and
                                 self.stop_at is not None and self.state_at > self.stop_at)
        if not after_stop_report:
            return False, 'waiting_for_new_state'
        if need_new:
            count = s.get('stop_verify_count')
            if type(count) is not int or type(before) is not int or count <= before:
                return False, 'waiting_for_new_physical_verification'
            return True, 'new_physical_stop_verified'
        return True, 'already_stopped_confirmed'

    def lift_safe_for_drive(self):
        # If a lift service has ever been started, a stale state FAILS CLOSED.
        if not LIFT_STATE_FILE.exists():
            return True
        v = _read_interlock(LIFT_STATE_FILE)
        return (v is not None and time.monotonic() - v.get('time', -1e9) < 0.6
                and v.get('state') in ('IDLE', 'TOP', 'BOTTOM')
                and v.get('verified_stopped') is True)

    def publish_motor_interlock(self):
        s = self.state()
        with self.lock:
            phase = self.phase
            session = self.session
        try:
            _write_interlock(MOTOR_STATE_FILE, {
                'time': time.monotonic(), 'phase': phase,
                'session': session is not None,
                'verified_stopped': self.is_stopped(s),
                'calibration': CAL_FLAG.exists(),
            })
        except OSError as e:
            self.get_logger().error('motor interlock write failed: ' + repr(e))
            self.stop(disarm=True)

    def control_tick(self):
        self.publish_motor_interlock()
        if not self.lift_safe_for_drive():
            # Lift owns the stationary period. Keep the ARM lease alive, but
            # never publish nonzero wheel RPM while the lift is moving.
            # If wheel motion was ongoing, fail closed and require a new ARM.
            with self.lock:
                phase = self.phase
                session = self.session
            if phase == 'RUNNING':
                self.stop(disarm=True)
                return
            with self.lock:
                self.requested = (0.0, 0.0)
                self.command_at = None
                self.auth_pub.publish(Bool(data=session is not None and phase != 'DISARMED'))
            if session is not None and phase != 'DISARMED':
                self.zero()
            return
        # Calibration owns motor-command publishing while this flag exists.
        if CAL_FLAG.exists():
            with self.lock:
                if not self.calibration_hold:
                    self.calibration_hold = True
                    self.session = None
                    self.phase = 'DISARMED'
                    self.requested = (0.0, 0.0)
                    self.command_at = None
                    print('[CAL] Calibration active: Web RPM publishing suspended',
                          flush=True)
                # Deauthorize the Web bridge; do not send wheel RPM zeros.
                self.auth_pub.publish(Bool(data=False))
            return

        with self.lock:
            if self.calibration_hold:
                self.calibration_hold = False
                print('[CAL] Calibration ended: Web bridge resumed; ARM required',
                      flush=True)

        now = time.monotonic()
        with self.lock:
            session = self.session
            phase = self.phase
            requested = self.requested
            command_at = self.command_at
            authorized = session is not None and phase != 'DISARMED'
            # Hold lock across publish: a DISARM False cannot be followed by
            # a stale True emitted from an earlier timer snapshot.
            self.auth_pub.publish(Bool(data=authorized))
        if not authorized:
            # No left/right RPM messages at all while DISARMED.
            return
        s = self.state()
        if phase == 'RUNNING':
            if command_at is None or now - command_at >= COMMAND_TIMEOUT:
                with self.lock:
                    self.fault_reason = 'command_timeout'
                self.stop(disarm=True)
                return
            if not self.is_ready(s):
                with self.lock:
                    self.fault_reason = 'motor_not_ready_or_stale'
                self.stop(disarm=True)
                return
            # Forward and forward-diagonal motion only: both wheel RPMs are
            # nonnegative, with at least one wheel requesting motion.
            forward_request = (requested[0] >= 0 and requested[1] >= 0
                               and (requested[0] > 0 or requested[1] > 0))
            if forward_request and self.depth_blocked_now():
                with self.lock:
                    self.must_release = True
                self.stop(disarm=False)
                print('[OBSTACLE] Forward blocked; ARM retained', flush=True)
                return
            if self.bench_motion and session is not None:
                self.publish(*requested)
                return
        self.zero()


async def send(ws, obj):
    try:
        await asyncio.wait_for(ws.send(json.dumps(obj)), timeout=0.2)
    except (OSError, websockets.ConnectionClosed, asyncio.TimeoutError):
        pass


async def run_bridge(bridge):
    print('[MODE]', 'BENCH MOTION up to 500 RPM' if bridge.bench_motion
          else 'DIAGNOSTIC - NONZERO RPM DISABLED', flush=True)
    while True:
        session = None
        last_seq = -1
        handshake_at = None
        last_ready_report = 0.0
        bridge.stop(disarm=True)
        try:
            async with websockets.connect(SERVER, ping_interval=2, ping_timeout=2) as ws:
                await send(ws, {'role': 'robot'})
                hello = json.loads(await asyncio.wait_for(ws.recv(), timeout=3))
                if hello.get('role') != 'robot' or hello.get('status') != 'connected':
                    raise RuntimeError('Registration failed')
                print('[CONNECTED] Windows Server', flush=True)
                while True:
                    now = time.monotonic()
                    with bridge.lock:
                        phase = bridge.phase
                        active = bridge.session
                        fault_reason = bridge.fault_reason
                    state = bridge.state()
                    if session is not None and (active is None or fault_reason):
                        reason = fault_reason or 'bridge_disarmed'
                        session = None
                        await send(ws, {'kind': 'fault', 'reason': reason})
                        print('[FAULT]', reason, flush=True)
                        continue
                    if session is not None and state is None:
                        bridge.stop(disarm=True)
                        session = None
                        await send(ws, {'kind': 'fault', 'reason': 'motor_state_stale'})
                        continue
                    if session is not None and state.get('fault') is True:
                        bridge.stop(disarm=True)
                        session = None
                        await send(ws, {'kind': 'fault', 'reason': 'can_fault'})
                        continue
                    if phase == 'START_PENDING':
                        if bridge.is_ready(state):
                            bridge.set_phase('ARMED_IDLE')
                            handshake_at = None
                            print('[HANDSHAKE] CANopen ready', flush=True)
                        elif handshake_at is not None and now - handshake_at > HANDSHAKE_TIMEOUT:
                            bridge.stop(disarm=True)
                            session = None
                            await send(ws, {'kind': 'fault', 'reason': 'handshake_timeout'})
                            continue
                    if phase == 'STOPPING':
                        ok, reason = bridge.stop_completed()
                        if ok:
                            bridge.set_phase('ARMED_IDLE')
                            print('[STOP VERIFIED]', reason, 'ARM retained', flush=True)
                        else:
                            with bridge.lock:
                                stop_at = bridge.stop_at
                            if stop_at is not None and now - stop_at > STOP_TIMEOUT:
                                bridge.stop(disarm=True)
                                session = None
                                await send(ws, {'kind': 'fault', 'reason': 'stop_timeout:' + reason})
                                continue
                    if session is not None and now - last_ready_report > 0.15:
                        with bridge.lock:
                            phase = bridge.phase
                        ready = bridge.is_ready(bridge.state()) and phase in ('ARMED_IDLE', 'RUNNING')
                        await send(ws, {'kind': 'motor_status', 'session': session, 'ready': ready})
                        last_ready_report = now
                    try:
                        raw = await asyncio.wait_for(ws.recv(), timeout=0.02)
                    except asyncio.TimeoutError:
                        continue
                    try:
                        msg = json.loads(raw)
                    except (ValueError, TypeError):
                        continue
                    if not isinstance(msg, dict):
                        continue
                    kind = msg.get('kind')
                    if kind == 'inhibit':
                        bridge.stop(disarm=True)
                        session = None
                        print('[DISARM]', msg.get('reason'), flush=True)
                        continue
                    if kind == 'arm':
                        if not bridge.lift_safe_for_drive():
                            await send(ws, {'kind': 'fault', 'reason': 'lift_interlock'})
                            continue
                        token = msg.get('session')
                        if not isinstance(token, str) or len(token) != 32:
                            continue
                        bridge.stop(disarm=True)
                        session = token
                        last_seq = -1
                        handshake_at = None
                        bridge.arm(token)
                        print('[ARM_READY] Session registered', flush=True)
                        continue
                    if kind == 'stop':
                        if session is None or msg.get('session') != session:
                            continue
                        # A button/key release is required before motion may restart.
                        with bridge.lock:
                            bridge.must_release = False
                        seq = msg.get('seq')
                        if type(seq) is not int or seq <= last_seq:
                            continue
                        last_seq = seq
                        bridge.stop(disarm=False)
                        print('[STOPPING] Zero requested', flush=True)
                        continue
                    if not bridge.lift_safe_for_drive():
                        # Reject drive commands without destroying the ARM
                        # session; control_tick continuously publishes zero.
                        continue
                    if kind != 'drive_request' or session is None or msg.get('session') != session:
                        continue
                    seq = msg.get('seq')
                    if type(seq) is not int or seq <= last_seq:
                        continue
                    command = msg.get('command')
                    try:
                        rpm = float(msg.get('rpm'))
                    except (TypeError, ValueError):
                        rpm = float('nan')
                    if command not in COMMANDS or not math.isfinite(rpm) or not MIN_REQUEST_RPM <= rpm <= MAX_REQUEST_RPM:
                        bridge.stop(disarm=True)
                        session = None
                        await send(ws, {'kind': 'fault', 'reason': 'invalid_command'})
                        continue
                    if rpm > MAX_APPROVED_RPM:
                        bridge.stop(disarm=True)
                        session = None
                        await send(ws, {'kind': 'fault', 'reason': 'rpm_not_approved'})
                        print(f'[BLOCKED] RPM {rpm} exceeds approved {MAX_APPROVED_RPM}', flush=True)
                        continue
                    last_seq = seq
                    with bridge.lock:
                        phase = bridge.phase
                    if phase == 'ARM_READY':
                        if not bridge.is_stopped(bridge.state()):
                            bridge.stop(disarm=True)
                            session = None
                            await send(ws, {'kind': 'fault', 'reason': 'stop_not_verified'})
                            continue
                        bridge.set_phase('START_PENDING')
                        handshake_at = time.monotonic()
                        print('[START_PENDING] Zero handshake', flush=True)
                        continue
                    if phase in ('START_PENDING', 'STOPPING'):
                        continue
                    if phase not in ('ARMED_IDLE', 'RUNNING'):
                        continue
                    # Depth limits forward commands only. Reverse, pivots and
                    # reverse diagonals remain available once the previous
                    # stop has completed and a new key press is received.
                    forward_command = command in ('forward', 'forward_left', 'forward_right')
                    if forward_command and bridge.depth_blocked_now():
                        with bridge.lock:
                            bridge.must_release = True
                        continue
                    with bridge.lock:
                        if bridge.must_release:
                            continue
                    if msg.get('motion_allowed') is not True or not bridge.bench_motion:
                        if phase == 'RUNNING':
                            bridge.stop(disarm=True)
                            session = None
                            await send(ws, {'kind': 'fault', 'reason': 'motion_permission_lost'})
                        else:
                            print('[BLOCKED] Not authorized for motion', flush=True)
                        continue
                    if not bridge.is_ready(bridge.state()):
                        bridge.stop(disarm=True)
                        session = None
                        await send(ws, {'kind': 'fault', 'reason': 'motor_not_ready'})
                        continue
                    left, right = calculate_rpm(command, rpm)
                    if bridge.request(left, right):
                        print(f'[RPM] {command} Left={left} Right={right}', flush=True)
        except (OSError, websockets.ConnectionClosed, asyncio.TimeoutError, RuntimeError) as exc:
            print('[DISCONNECTED]', repr(exc), flush=True)
        finally:
            bridge.stop(disarm=True)
            print('[DISARM] connection_lost', flush=True)
        await asyncio.sleep(2)


def spin_ros(executor):
    try:
        executor.spin()
    except Exception as exc:
        print('[ROS ERROR]', repr(exc), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bench-motion', action='store_true',
                        help='Explicit bench test; up to 500 RPM bench motor output')
    args = parser.parse_args()
    rclpy.init()
    bridge = AMRBridge(bench_motion=args.bench_motion)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    thread = threading.Thread(target=spin_ros, args=(executor,), daemon=True)
    thread.start()
    try:
        asyncio.run(run_bridge(bridge))
    except KeyboardInterrupt:
        print('[STOP] Keyboard interrupt', flush=True)
    finally:
        bridge.stop(disarm=True)
        executor.shutdown()
        thread.join(timeout=2)
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()