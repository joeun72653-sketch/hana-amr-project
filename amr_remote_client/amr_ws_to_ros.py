#!/usr/bin/env python3
"""AMR ROS2 Jazzy WebSocket bridge. 30-500 RPM bench request validation.

Default mode publishes no nonzero RPM. --bench-motion is an explicit opt-in.
The CAN worker, driver comms-loss stop, and independent E-stop remain required.
"""
import argparse
import asyncio
import json
import math
import threading
import time
from pathlib import Path

import rclpy
import websockets
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, Float64, String

CAL_FLAG = Path.home() / '.amr_calibration_active'
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
        # No Web-controlled ROS publishers exist until an explicit Web ARM.
        self.left_pub = None
        self.right_pub = None
        self.auth_pub = None
        self.lock = threading.RLock()
        self.release_started = None
        self.release_stop_before = None
        self.release_needs_new_stop = False
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

    def _create_web_publishers_locked(self):
        """Only Web ARM creates RPM and Web-authorization publishers."""
        if any(pub is not None for pub in (self.left_pub, self.right_pub, self.auth_pub)):
            raise RuntimeError('Web publisher ownership already active')
        try:
            self.left_pub = self.create_publisher(Float64, '/wheel_rpm_left', 10)
            self.right_pub = self.create_publisher(Float64, '/wheel_rpm_right', 10)
            self.auth_pub = self.create_publisher(Bool, '/amr_motion_authorized', 10)
        except Exception:
            self._destroy_web_publishers_locked()
            raise
        print('[WEB ARM] RPM publishers created', flush=True)

    def _destroy_web_publishers_locked(self):
        """Remove Web control endpoints after DISARM stop verification."""
        had_publisher = any(
            pub is not None for pub in (self.left_pub, self.right_pub, self.auth_pub)
        )
        for name in ('left_pub', 'right_pub', 'auth_pub'):
            pub = getattr(self, name)
            if pub is not None:
                try:
                    if not self.destroy_publisher(pub):
                        print(f'[CRITICAL] Failed to destroy {name}', flush=True)
                except Exception as exc:
                    print(f'[CRITICAL] Failed to destroy {name}: {exc!r}', flush=True)
                finally:
                    setattr(self, name, None)
        if had_publisher:
            print('[WEB DISARM] RPM publishers removed', flush=True)

    def _publish_pair_locked(self, left, right):
        if self.left_pub is None or self.right_pub is None:
            return
        self.left_pub.publish(Float64(data=float(left)))
        self.right_pub.publish(Float64(data=float(right)))

    def publish(self, left, right):
        # Keep RPM publication ordered against DISARM and publisher teardown.
        with self.lock:
            if CAL_FLAG.exists() or self.session is None or self.phase in (
                'DISARMED', 'RELEASING'
            ):
                return
            self._publish_pair_locked(left, right)

    def zero(self):
        self.publish(0, 0)

    def arm(self, token):
        if CAL_FLAG.exists():
            print('[CAL] ARM ignored during calibration', flush=True)
            return False
        with self.lock:
            # An earlier session must release its publishers before a new ARM.
            if self.phase != 'DISARMED':
                print('[BLOCKED] Web ARM waiting for previous stop/disarm', flush=True)
                return False
            self._create_web_publishers_locked()
            self.session = token
            self.phase = 'ARM_READY'
            self.requested = (0.0, 0.0)
            self.command_at = None
            self.fault_reason = None
            return True

    def set_phase(self, phase):
        with self.lock:
            self.phase = phase

    def request(self, left, right):
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
                if self.phase == 'RELEASING':
                    return
                if self.left_pub is None and self.right_pub is None:
                    self.phase = 'DISARMED'
                    # Critically, an unarmed Web bridge never creates or
                    # publishes Web authorization or RPM messages.
                    return
                # Revoke Web authority, then send a verified stop before
                # removing its RPM publishers from the ROS graph.
                self.phase = 'RELEASING'
                self.release_started = time.monotonic()
                self.release_stop_before = (state or {}).get('stop_verify_count')
                self.release_needs_new_stop = (
                    any(x != 0 for x in (state or {}).get('applied', []))
                    or any(x != 0 for x in (state or {}).get('actual', []))
                    or (previous_phase == 'RUNNING' and not self.is_stopped(state))
                )
                if self.auth_pub is not None:
                    self.auth_pub.publish(Bool(data=False))
                self._publish_pair_locked(0, 0)
                print('[WEB DISARM] Stopping before publisher teardown', flush=True)
                return
            if self.phase in ('DISARMED', 'RELEASING') or self.session is None:
                return
            self.phase = 'STOPPING'
            self.stop_at = time.monotonic()
            self.stop_status_at = self.state_at
            self.stop_count_before = (state or {}).get('stop_verify_count')
            self.stop_requires_new = (
                any(x != 0 for x in (state or {}).get('applied', []))
                or any(x != 0 for x in (state or {}).get('actual', []))
                or (previous_phase == 'RUNNING' and not self.is_stopped(state))
            )
            if not CAL_FLAG.exists():
                self._publish_pair_locked(0, 0)

    def _release_tick(self, *, calibration_active=False):
        """Only a prior Web ARM can enter this transient stop state."""
        with self.lock:
            if self.phase != 'RELEASING':
                return
            if not calibration_active:
                self._publish_pair_locked(0, 0)
            state = self.state()
            fresh = (self.state_at is not None and
                     self.release_started is not None and
                     self.state_at > self.release_started)
            before = self.release_stop_before
            after = (state or {}).get('stop_verify_count')
            verified = (
                fresh and self.is_stopped(state) and
                (not self.release_needs_new_stop or
                 (type(before) is int and type(after) is int and after > before))
            )
            timed_out = (self.release_started is not None and
                         time.monotonic() - self.release_started >= STOP_TIMEOUT)
            if verified or timed_out:
                if not verified:
                    print('[CRITICAL] Web DISARM stop unverified; '
                          'check hardware E-stop and CAN drive state', flush=True)
                self._destroy_web_publishers_locked()
                self.phase = 'DISARMED'
                self.release_started = None

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

    def control_tick(self):
        # Calibration owns motor-command publishing while this flag exists.
        if CAL_FLAG.exists():
            with self.lock:
                if not self.calibration_hold:
                    self.calibration_hold = True
                    print('[CAL] Calibration active: Web control releasing',
                          flush=True)
                    self.stop(disarm=True)
            # Calibration owns the command topics. Never keep sending Web
            # zeros while it is in progress, even during stop verification.
            self._release_tick(calibration_active=True)
            return

        with self.lock:
            if self.calibration_hold:
                self.calibration_hold = False
                print('[CAL] Calibration ended: Web control remains DISARMED',
                      flush=True)
            if self.phase == 'RELEASING':
                self._release_tick()
                return

        now = time.monotonic()
        with self.lock:
            session = self.session
            phase = self.phase
            requested = self.requested
            command_at = self.command_at
            authorized = session is not None and phase != 'DISARMED'
            # Hold lock across publish: a DISARM False cannot be followed by
            # a stale True emitted from an earlier timer snapshot.
            if authorized and self.auth_pub is not None:
                self.auth_pub.publish(Bool(data=True))
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
            async with websockets.connect(SERVER, ping_interval=10, ping_timeout=10) as ws:
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
                        token = msg.get('session')
                        if not isinstance(token, str) or len(token) != 32:
                            continue
                        bridge.stop(disarm=True)
                        if not bridge.arm(token):
                            session = None
                            await send(ws, {'kind': 'fault', 'reason': 'web_arm_waiting_for_stop'})
                            continue
                        session = token
                        last_seq = -1
                        handshake_at = None
                        print('[ARM_READY] Session registered', flush=True)
                        continue
                    if kind == 'stop':
                        if session is None or msg.get('session') != session:
                            continue
                        seq = msg.get('seq')
                        if type(seq) is not int or seq <= last_seq:
                            continue
                        last_seq = seq
                        bridge.stop(disarm=False)
                        print('[STOPPING] Zero requested', flush=True)
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
        # Let the active ROS timer finish Web-owned stop verification before
        # executor shutdown, so a disconnect cannot strand the final RPM.
        deadline = time.monotonic() + STOP_TIMEOUT + 0.5
        while time.monotonic() < deadline:
            with bridge.lock:
                if bridge.phase != 'RELEASING':
                    break
            time.sleep(0.05)
        executor.shutdown()
        thread.join(timeout=2)
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
