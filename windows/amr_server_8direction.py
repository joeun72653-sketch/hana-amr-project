#!/usr/bin/env python3
"""Single-operator AMR WebSocket relay. Use on an isolated trusted LAN only.

--bench-motion enables forwarding authorized requests only when the robot
reports ready. Operator ARM alone never authorizes motor output.
"""
import argparse
import asyncio
import json
import math
import secrets
import time
import websockets

HOST = '0.0.0.0'
PORT = 8765
COMMAND_TIMEOUT = 0.25
MIN_REQUEST_RPM = 30.0
MAX_REQUEST_RPM = 500.0
MAX_APPROVED_RPM = 500.0  # requested bench ceiling; verify under load
COMMANDS = {'forward', 'reverse', 'left', 'right',
            'forward_left', 'forward_right', 'reverse_left', 'reverse_right'}


class Relay:
    def __init__(self, bench_motion=False):
        self.bench_motion = bench_motion
        self.robot = None
        self.operator = None
        self.session = None
        self.operator_armed = False
        self.motor_ready = False
        self.moving = False
        self.last_command_at = None
        self.seq = 0
        self.lock = asyncio.Lock()

    async def send(self, ws, obj):
        if ws is None:
            return False
        try:
            await asyncio.wait_for(ws.send(json.dumps(obj)), timeout=0.2)
            return True
        except (OSError, websockets.ConnectionClosed, asyncio.TimeoutError):
            return False

    async def state(self, reason):
        await self.send(self.operator, {
            'kind': 'state', 'armed': self.operator_armed,
            'motor_ready': self.motor_ready, 'reason': reason,
            'mode': 'bench' if self.bench_motion else 'diagnostic',
        })

    async def disarm(self, reason):
        async with self.lock:
            self.session = None
            self.operator_armed = False
            self.motor_ready = False
            self.moving = False
            self.last_command_at = None
            await self.send(self.robot, {'kind': 'inhibit', 'reason': reason})
            await self.state(reason)
        print('[DISARM]', reason, flush=True)

    async def stop(self):
        async with self.lock:
            if not self.operator_armed:
                return
            self.moving = False
            self.last_command_at = None
            self.motor_ready = False
            self.seq += 1
            ok = await self.send(self.robot, {
                'kind': 'stop', 'session': self.session, 'seq': self.seq,
            })
            await self.state('stopping')
        if not ok:
            await self.disarm('stop_delivery_failed')
        else:
            print('[STOP] ARM retained; waiting for verification', flush=True)

    async def handler(self, ws):
        role = None
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except (ValueError, TypeError):
                    continue
                if not isinstance(msg, dict):
                    continue
                if role is None:
                    requested = msg.get('role')
                    async with self.lock:
                        if requested == 'robot' and self.robot is None:
                            self.robot = ws
                            role = 'robot'
                        elif requested == 'operator' and self.operator is None:
                            self.operator = ws
                            role = 'operator'
                    if role is None:
                        await ws.close(code=1008, reason='Invalid/duplicate role')
                        return
                    await self.send(ws, {'status': 'connected', 'role': role})
                    await self.disarm(role + '_connected')
                    print('[CONNECTED]', role, flush=True)
                    continue

                if role == 'robot':
                    if msg.get('kind') == 'fault':
                        print('[ROBOT FAULT]', msg.get('reason'), flush=True)
                        await self.disarm('robot_fault')
                    elif msg.get('kind') == 'motor_status':
                        if self.session is not None and msg.get('session') == self.session:
                            ready = msg.get('ready') is True
                            if ready != self.motor_ready:
                                self.motor_ready = ready
                                await self.state('motor_ready' if ready else 'motor_not_ready')
                    continue

                kind = msg.get('kind')
                if kind == 'disarm':
                    await self.disarm('operator_disarm')
                    continue
                if kind == 'arm':
                    if self.robot is None:
                        await self.disarm('robot_offline')
                        continue
                    await self.disarm('pre_arm_reset')
                    async with self.lock:
                        self.session = secrets.token_hex(16)
                        self.seq = 0
                        self.operator_armed = True
                        self.motor_ready = False
                        ok = await self.send(self.robot, {
                            'kind': 'arm', 'session': self.session,
                        })
                        if ok:
                            await self.state('operator_arm')
                    if not ok:
                        await self.disarm('arm_delivery_failed')
                    else:
                        print('[ARM] Operator session registered', flush=True)
                    continue
                if kind != 'command':
                    continue
                command = msg.get('command')
                if command == 'stop':
                    await self.stop()
                    continue
                if command == 'release':
                    # Explicit operator release only; never treat a timeout as release.
                    valid = (self.operator_armed and self.motor_ready and
                             self.moving and self.last_command_at is not None and
                             time.monotonic() - self.last_command_at < COMMAND_TIMEOUT)
                    if not valid:
                        await self.stop()  # fail closed
                        continue
                    async with self.lock:
                        self.seq += 1
                        seq = self.seq
                        session = self.session
                        self.moving = False  # watchdog must not DISARM during intended release
                        self.last_command_at = None
                        self.motor_ready = False
                        ok = await self.send(self.robot, {
                            'kind': 'normal_release', 'session': session, 'seq': seq,
                        })
                        await self.state('soft_releasing')
                    if not ok:
                        await self.disarm('release_delivery_failed')
                    else:
                        print('[RELEASE] Requested bounded normal deceleration', flush=True)
                    continue
                if command not in COMMANDS or not self.operator_armed:
                    continue
                try:
                    rpm = float(msg.get('rpm'))
                except (TypeError, ValueError):
                    await self.disarm('invalid_rpm')
                    continue
                if not math.isfinite(rpm) or not MIN_REQUEST_RPM <= rpm <= MAX_REQUEST_RPM:
                    await self.disarm('invalid_rpm')
                    continue
                if rpm > MAX_APPROVED_RPM:
                    print(f'[BLOCKED] requested {rpm} RPM > approved {MAX_APPROVED_RPM} RPM', flush=True)
                    await self.disarm('rpm_not_approved')
                    continue
                async with self.lock:
                    self.seq += 1
                    self.moving = True
                    self.last_command_at = time.monotonic()
                    ok = await self.send(self.robot, {
                        'kind': 'drive_request', 'session': self.session,
                        'seq': self.seq, 'command': command, 'rpm': rpm,
                        'motion_allowed': self.bench_motion and self.motor_ready,
                    })
                if not ok:
                    await self.disarm('robot_send_failed')
        except (OSError, websockets.ConnectionClosed):
            pass
        finally:
            if role == 'robot' and self.robot is ws:
                self.robot = None
                await self.disarm('robot_disconnected')
            elif role == 'operator' and self.operator is ws:
                self.operator = None
                await self.disarm('operator_disconnected')

    async def watchdog(self):
        while True:
            await asyncio.sleep(0.025)
            if self.moving and self.last_command_at is not None:
                if time.monotonic() - self.last_command_at >= COMMAND_TIMEOUT:
                    await self.disarm('command_timeout')


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bench-motion', action='store_true',
                        help='Permit robot-ready-authorized requests; approved limit 500 RPM')
    args = parser.parse_args()
    relay = Relay(args.bench_motion)
    async with websockets.serve(relay.handler, HOST, PORT,
                                ping_interval=2, ping_timeout=2, max_size=4096):
        print(f'[READY] ws://{HOST}:{PORT} mode=' +
              ('BENCH' if args.bench_motion else 'DIAGNOSTIC'), flush=True)
        await relay.watchdog()


if __name__ == '__main__':
    asyncio.run(main())
