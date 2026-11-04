#!/usr/bin/env python3
"""Standalone ORGO Modbus RTU lift WebSocket controller.

A short, bounded end-position confirmation handles a controller reporting
STOP before TOP/BOTTOM. It never treats an unconfirmed end position as success.
Application interlocks do not replace a certified E-stop or hard limits.
"""
import argparse
import asyncio
import json
import os
import secrets
import time
from pathlib import Path

import websockets
from pymodbus.client import ModbusSerialClient

DEFAULT_PORT = '/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D20J8ODC-if00-port0'
MOTOR_FILE = Path.home() / '.amr_motor_interlock.json'
LIFT_FILE = Path.home() / '.amr_lift_interlock.json'
STOP = 0xF000
RUN_DOWN = 0xF001
RUN_UP = 0xF002
TOP = 0x0404
BOTTOM = 0x0202
MAX_SYNC_DIFF = 20
MAX_MOTION_SECONDS = 45.0
HEARTBEAT_TIMEOUT = 0.65
END_CONFIRM_SECONDS = 0.8  # Bounded wait for STOP + target position register
TARGET_TOLERANCE = 20  # Pulse: acceptable final target error; does not relax safety faults
TARGET_STOP_TIMEOUT = 2.0


def read_json(path):
    try:
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else None
    except (OSError, ValueError):
        return None


def atomic_json(path, value):
    tmp = path.with_name(path.name + '.' + str(os.getpid()) + '.tmp')
    tmp.write_text(json.dumps(value))
    os.replace(tmp, path)


class Controller:
    def __init__(self, serial_port):
        self.serial = ModbusSerialClient(
            port=serial_port, baudrate=115200, bytesize=8,
            parity='N', stopbits=1, timeout=0.3, retries=0)
        self.clients = set()
        self.owner = None
        self.owner_seen = 0.0
        self.move_started = 0.0
        self.direction = None
        self.start_pos = None
        self.state = 'UNKNOWN'
        self.last = None
        self.fault = None
        self.auth_token = secrets.token_urlsafe(24)
        self.shutting_down = False
        self.confirm_started = None
        self.target_position = None
        self.target_stopping = False
        self.target_stop_at = None
        self.target_stable_samples = 0
        self.target_last_positions = None
        self.target_stop_lead = 0  # No predictive lead: STOP requested at the target crossing

    def motor_safe(self):
        value = read_json(MOTOR_FILE)
        return (value is not None
                and isinstance(value.get('time'), (float, int))
                and 0 <= time.monotonic() - value['time'] < 0.6
                and value.get('phase') == 'DISARMED'
                and value.get('session') is False
                and value.get('verified_stopped') is True
                and value.get('calibration') is False)

    def read(self):
        result = self.serial.read_holding_registers(
            address=1, count=7, device_id=1)
        if result.isError() or len(result.registers) != 7:
            raise RuntimeError('Modbus status read failed')
        return list(result.registers)

    def command(self, value):
        result = self.serial.write_register(
            address=0, value=value, device_id=1)
        if result.isError():
            raise RuntimeError('Modbus command rejected')

    def stop_motion(self, why):
        was_moving = self.direction is not None
        self.direction = None
        self.owner = None
        self.start_pos = None
        self.confirm_started = None
        self.target_position = None
        self.target_stopping = False
        self.target_stop_at = None
        self.target_stable_samples = 0
        self.target_last_positions = None
        if was_moving:
            try:
                self.command(4)
                print('[LIFT STOP]', why, flush=True)
            except Exception as exc:
                self.fault = 'STOP NOT CONFIRMED: ' + str(exc)
                print('[DANGER] Physical E-STOP REQUIRED:', self.fault,
                      flush=True)
        if self.fault:
            self.state = 'FAULT'

    def fault_motion(self, reason, r=None):
        if r is not None:
            print('[LIFT DIAG]',
                  f'direction={self.direction} motor=0x{r[0]:04X} '
                  f'position=0x{r[1]:04X} error=0x{r[2]:04X} '
                  f'pos1={r[5]} pos2={r[6]} reason={reason}', flush=True)
        self.fault = reason
        self.stop_motion(reason)
        self.state = 'FAULT'

    def begin_end_confirmation(self, r, reason):
        """Command STOP once; verify both STOP and target in later polls."""
        if self.confirm_started is not None:
            return
        print('[LIFT] End confirmation started:', reason,
              f'motor=0x{r[0]:04X} pos_state=0x{r[1]:04X} '
              f'pos1={r[5]} pos2={r[6]}', flush=True)
        # The drive must not continue to run while waiting for a status update.
        try:
            self.command(4)
        except Exception as exc:
            self.fault_motion('STOP command failed during end confirmation: '
                              + str(exc), r)
            return
        self.confirm_started = time.monotonic()

    async def emit(self, ws, data):
        try:
            await ws.send(json.dumps(data))
        except websockets.exceptions.ConnectionClosed:
            pass

    def snapshot(self):
        r = self.last
        return {
            'kind': 'lift_status', 'state': self.state,
            'moving': self.direction is not None,
            'pos1': r[5] if r else None,
            'pos2': r[6] if r else None,
            'motor_state': f'0x{r[0]:04X}' if r else None,
            'position_state': f'0x{r[1]:04X}' if r else None,
            'error': r[2] if r else None,
            'drive_interlock_ok': self.motor_safe(),
            'fault': self.fault,
            'target_position': self.target_position,
            'target_control_enabled': True,
        }

    def update(self):
        try:
            r = self.read()
            self.last = r
        except Exception as exc:
            self.fault_motion('Lost Modbus response: ' + str(exc))
            return

        if r[2] or abs(r[5] - r[6]) > MAX_SYNC_DIFF:
            self.fault_motion('Controller fault or position mismatch', r)
            return

        if self.direction:
            if not self.motor_safe():
                self.fault_motion('Drive state is not safely DISARMED', r)
                return
            if (self.owner is None
                    or time.monotonic() - self.owner_seen > HEARTBEAT_TIMEOUT):
                self.fault_motion('Control heartbeat lost', r)
                return
            if time.monotonic() - self.move_started > MAX_MOTION_SECONDS:
                self.fault_motion('Motion time limit reached', r)
                return
            if self.start_pos is not None:
                p1, p2 = self.start_pos
                if self.direction == 'up' and (
                        r[5] < p1 - 5 or r[6] < p2 - 5
                        or max(r[5], r[6]) > 2020):
                    self.fault_motion('Unexpected upward motion position', r)
                    return
                if self.direction == 'down' and (
                        r[5] > p1 + 5 or r[6] > p2 + 5
                        or min(r[5], r[6]) < 0):
                    self.fault_motion('Unexpected downward motion position', r)
                    return

            if self.target_position is not None:
                target_p = self.target_position
                p1, p2 = r[5], r[6]
                lead = self.target_stop_lead
                # Never continue movement after a stop request. Both axes must
                # be physically stationary and within tolerance for success.
                if self.target_stopping:
                    if r[0] == STOP:
                        pair = (p1, p2)
                        if self.target_last_positions is not None and all(
                                abs(x-y) <= 1 for x,y in zip(pair,self.target_last_positions)):
                            self.target_stable_samples += 1
                        else:
                            self.target_stable_samples = 0
                        self.target_last_positions = pair
                        if (time.monotonic() - self.target_stop_at < 0.36
                                or self.target_stable_samples < 2):
                            return
                        within = (abs(p1 - target_p) <= TARGET_TOLERANCE
                                  and abs(p2 - target_p) <= TARGET_TOLERANCE)
                        self.stop_motion('target stop confirmed')
                        if not within:
                            self.fault = (f'Target not reached: target={target_p}, '
                                          f'actual=({p1},{p2}); inspect before restart')
                            self.state = 'FAULT'
                        return
                    if time.monotonic() - self.target_stop_at > TARGET_STOP_TIMEOUT:
                        self.fault_motion('Target STOP not confirmed before timeout', r)
                    return
                # Request STOP upon crossing the target in the direction of travel.
                # WARNING: polling latency and inertia can cause overshoot.
                reached = (max(p1, p2) >= target_p - lead if self.direction == 'up'
                           else min(p1, p2) <= target_p + lead)
                if reached or r[0] == STOP:
                    try:
                        self.command(4)
                        self.target_stopping = True
                        self.target_stop_at = time.monotonic()
                        self.target_stable_samples = 0
                        self.target_last_positions = None
                        print(f'[TARGET STOP] target={target_p} current=({p1},{p2})', flush=True)
                    except Exception as exc:
                        self.fault_motion('Target STOP command failed: ' + str(exc), r)
                    return
                expected = RUN_UP if self.direction == 'up' else RUN_DOWN
                if r[0] != expected:
                    self.fault_motion('Unexpected motor state in target motion', r)
                return

            target = TOP if self.direction == 'up' else BOTTOM
            run_state = RUN_UP if self.direction == 'up' else RUN_DOWN

            # Only the combination STOP + correct END POSITION is a success.
            if r[0] == STOP and r[1] == target:
                self.stop_motion('end position confirmed')
            elif self.confirm_started is not None:
                # STOP was requested already. During the brief confirmation
                # interval the motor must be STOP or still report its previous
                # run state; any other state is a real fault.
                if r[0] not in (STOP, run_state):
                    self.fault_motion('Unexpected motor state during confirmation', r)
                    return
                if time.monotonic() - self.confirm_started > END_CONFIRM_SECONDS:
                    self.fault_motion('End position not confirmed before timeout', r)
                    return
            elif r[1] == target:
                self.begin_end_confirmation(r, 'target position reported')
            elif r[0] == STOP:
                # Possible short reporting delay: do not declare success yet.
                self.begin_end_confirmation(r, 'motor STOP before position state')
            elif r[0] != run_state:
                self.fault_motion('Unexpected motor run status', r)
                return

        if self.direction:
            self.state = ('CONFIRMING_END' if self.confirm_started is not None
                          else 'MOVING_UP' if self.direction == 'up'
                          else 'MOVING_DOWN')
        elif not self.fault:
            self.state = ('TOP' if r[1] == TOP and r[0] == STOP
                          else 'BOTTOM' if r[1] == BOTTOM and r[0] == STOP
                          else 'IDLE' if r[0] == STOP else 'UNKNOWN')

    async def loop(self):
        while not self.shutting_down:
            self.update()
            verified = bool(
                self.last and self.last[0] == STOP
                and self.last[2] == 0 and self.direction is None
                and self.state in ('IDLE', 'TOP', 'BOTTOM'))
            try:
                atomic_json(LIFT_FILE, {
                    'time': time.monotonic(), 'state': self.state,
                    'verified_stopped': verified,
                })
            except OSError as exc:
                self.fault_motion('Lift state interlock write failed: ' + str(exc))
            snapshot = self.snapshot()
            for ws in list(self.clients):
                await self.emit(ws, snapshot)
            await asyncio.sleep(0.12)

    async def handle(self, ws):
        authenticated = False
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except (ValueError, TypeError):
                    continue
                if not isinstance(msg, dict):
                    continue
                if not authenticated:
                    ok = msg.get('kind') == 'auth'
                    await self.emit(ws, {'kind': 'auth', 'ok': ok})
                    if not ok:
                        await ws.close(code=1008,
                                       reason='authentication failed')
                        return
                    authenticated = True
                    self.clients.add(ws)
                    await self.emit(ws, self.snapshot())
                    continue
                kind = msg.get('kind')
                if kind == 'heartbeat':
                    if self.owner is ws:
                        self.owner_seen = time.monotonic()
                    continue
                if kind == 'lift_target_preview':
                    a, b = msg.get('pos1'), msg.get('pos2')
                    valid = (type(a) is int and type(b) is int
                             and 0 <= a <= 2000 and 0 <= b <= 2000
                             and a == b)
                    if valid:
                        message = (f'공통 목표 {a} Pulse 입력 확인. '
                                   '위치 이동 명령은 미지원/미검증이므로 전송하지 않았습니다.')
                    else:
                        message = ('목표값 오류: 두 축에 동일한 0~2000 정수 Pulse를 '
                                   '입력하세요. 실제 이동 명령은 전송하지 않았습니다.')
                    await self.emit(ws, {
                        'kind': 'lift_target_preview_result',
                        'ok': valid,
                        'message': message,
                    })
                    continue
                if kind == 'lift_target_move':
                    a, b = msg.get('pos1'), msg.get('pos2')
                    valid = (type(a) is int and type(b) is int
                             and 0 <= a <= 2000 and a == b)
                    if not valid:
                        await self.emit(ws, {'kind': 'lift_error',
                            'message': 'Lift 1/2 targets must be equal integers 0..2000'})
                        continue
                    if self.direction or self.fault:
                        await self.emit(ws, {'kind': 'lift_error',
                            'message': 'Lift moving or fault latched; STOP/inspect first'})
                        continue
                    self.update()
                    if (self.fault or not self.motor_safe() or not self.last
                            or self.last[0] != STOP or self.last[2] != 0
                            or self.state not in ('TOP', 'BOTTOM', 'IDLE')):
                        await self.emit(ws, {'kind': 'lift_error',
                            'message': 'Target inhibited: lift/motor not safely stopped'})
                        continue
                    current1, current2 = self.last[5:7]
                    if abs(current1-current2) > MAX_SYNC_DIFF:
                        await self.emit(ws, {'kind': 'lift_error',
                            'message': 'Target inhibited: lift axes not synchronized'})
                        continue
                    mean = (current1+current2)/2
                    if abs(mean-a) <= TARGET_TOLERANCE:
                        await self.emit(ws, {'kind': 'lift_result',
                            'message': 'Already near target; no motion sent'})
                        continue
                    direction = 'up' if a > mean else 'down'
                    # The controller uses full-speed UP/DOWN, not a native
                    # servo position command; final-position accuracy is not guaranteed.
                    self.owner = ws
                    self.owner_seen = time.monotonic()
                    self.start_pos = (current1, current2)
                    self.move_started = time.monotonic()
                    self.direction = direction
                    self.target_position = a
                    self.target_stopping = False
                    self.target_stop_at = None
                    self.target_stable_samples = 0
                    self.target_last_positions = None
                    self.confirm_started = None
                    self.state = 'MOVING_UP' if direction == 'up' else 'MOVING_DOWN'
                    try:
                        atomic_json(LIFT_FILE, {'time': time.monotonic(),
                            'state': self.state, 'verified_stopped': False})
                        self.command(3 if direction == 'up' else 2)
                        await self.emit(ws, {'kind': 'lift_result',
                            'message': f'Target {a} P motion started ({direction})'})
                    except Exception as exc:
                        self.fault_motion('Target motion command failed: '+str(exc))
                        await self.emit(ws, {'kind': 'lift_error', 'message': self.fault})
                    continue
                if kind != 'lift_command':
                    continue
                command = msg.get('command')
                if command == 'stop':
                    self.stop_motion('web STOP request')
                    await self.emit(ws, {
                        'kind': 'lift_result',
                        'message': 'STOP requested. Check state confirmation.',
                    })
                    continue
                if command not in ('up', 'down'):
                    continue
                if self.direction:
                    await self.emit(ws, {
                        'kind': 'lift_error',
                        'message': 'Already moving: STOP first',
                    })
                    continue
                if self.fault:
                    await self.emit(ws, {
                        'kind': 'lift_error',
                        'message': self.fault + '; restart after inspection',
                    })
                    continue
                self.update()
                if self.fault:
                    await self.emit(ws, {
                        'kind': 'lift_error', 'message': self.fault,
                    })
                    continue
                if (not self.motor_safe() or not self.last
                        or self.last[0] != STOP or self.last[2] != 0
                        or self.state not in ('TOP', 'BOTTOM', 'IDLE')):
                    motor = read_json(MOTOR_FILE) or {}
                    reason = motor.get(
                        'reason', 'check motor bridge and /amr_motor_state')
                    await self.emit(ws, {
                        'kind': 'lift_error',
                        'message': 'Lift inhibited: motor stop verification '
                                   'missing (' + str(reason)
                                   + '); do not bypass the interlock',
                    })
                    continue
                if ((command == 'up' and self.last[1] == TOP)
                        or (command == 'down' and self.last[1] == BOTTOM)):
                    await self.emit(ws, {
                        'kind': 'lift_result',
                        'message': 'Already at requested end position',
                    })
                    continue
                self.owner = ws
                self.owner_seen = time.monotonic()
                self.start_pos = (self.last[5], self.last[6])
                self.move_started = time.monotonic()
                self.direction = command
                self.confirm_started = None
                self.state = ('MOVING_UP' if command == 'up'
                              else 'MOVING_DOWN')
                try:
                    atomic_json(LIFT_FILE, {
                        'time': time.monotonic(), 'state': self.state,
                        'verified_stopped': False,
                    })
                    self.command(3 if command == 'up' else 2)
                    await self.emit(ws, {
                        'kind': 'lift_result',
                        'message': ('UP command sent' if command == 'up'
                                    else 'DOWN command sent'),
                    })
                except Exception as exc:
                    self.fault_motion('Motion command failed: ' + str(exc))
                    await self.emit(ws, {
                        'kind': 'lift_error', 'message': self.fault,
                    })
        finally:
            self.clients.discard(ws)
            if self.owner is ws:
                self.stop_motion('controller disconnected')


async def main_async(args):
    c = Controller(args.port)
    if not c.serial.connect():
        raise RuntimeError('Cannot open serial port ' + args.port)
    print('\n[LIFT] Listening on 0.0.0.0:8767', flush=True)
    print('[TARGET] enabled=True mode=feedback-crossing (no stop lead)', flush=True)
    print('[LIFT] Motor ROS bridge with interlock must be running and DISARMED.',
          flush=True)
    monitor = asyncio.create_task(c.loop())
    try:
        async with websockets.serve(
                c.handle, '0.0.0.0', 8767, ping_interval=2,
                ping_timeout=3, max_size=4096):
            await asyncio.Future()
    finally:
        c.shutting_down = True
        c.stop_motion('server shutdown')
        monitor.cancel()
        try:
            await monitor
        except asyncio.CancelledError:
            pass
        c.serial.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=DEFAULT_PORT)
    args = parser.parse_args()
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
