#!/usr/bin/env python3
"""Standalone ORGO Modbus RTU lift WebSocket controller.

IMPORTANT: Application-level interlocks/watchdogs do not replace certified
hardware E-stop, hard limits, contactor or a drive-integrated safety function.
"""
import argparse
import asyncio
import json
import signal
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
MID = 0x0101
MAX_SYNC_DIFF = 20
MAX_MOTION_SECONDS = 45.0
HEARTBEAT_TIMEOUT = 0.65


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
        self.serial = ModbusSerialClient(port=serial_port, baudrate=115200,
            bytesize=8, parity='N', stopbits=1, timeout=0.3, retries=0)
        self.clients = set()
        self.owner = None
        self.owner_seen = 0
        self.move_started = 0
        self.direction = None
        self.start_pos = None
        self.state = 'UNKNOWN'
        self.last = None
        self.fault = None
        # DEVELOPMENT MODE: no WebSocket authentication/access allowlist.
        # Use only on an isolated test network; do not expose port 8767 publicly.
        self.shutting_down = False

    def motor_safe(self):
        value = read_json(MOTOR_FILE)
        return (value is not None and isinstance(value.get('time'), (float, int))
            and 0 <= time.monotonic() - value['time'] < 0.6
            and ((value.get('phase') == 'DISARMED' and value.get('session') is False)
                 or (value.get('phase') == 'ARMED_IDLE' and value.get('session') is True))
            and value.get('verified_stopped') is True
            and value.get('calibration') is False)

    def read(self):
        result = self.serial.read_holding_registers(address=1, count=7, device_id=1)
        if result.isError() or len(result.registers) != 7:
            raise RuntimeError('Modbus status read failed')
        return list(result.registers)

    def command(self, value):
        result = self.serial.write_register(address=0, value=value, device_id=1)
        if result.isError():
            raise RuntimeError('Modbus command rejected')

    def stop_motion(self, why):
        was_moving = self.direction is not None
        self.direction = None
        self.owner = None
        self.start_pos = None
        if was_moving:
            try:
                self.command(4)
                print('[LIFT STOP]', why, flush=True)
            except Exception as exc:
                self.fault = 'STOP NOT CONFIRMED: ' + str(exc)
                print('[DANGER] Physical E-STOP REQUIRED:', self.fault, flush=True)
        if self.fault:
            self.state = 'FAULT'

    async def emit(self, ws, data):
        try:
            await ws.send(json.dumps(data))
        except websockets.exceptions.ConnectionClosed:
            pass

    def snapshot(self):
        r = self.last
        return {'kind':'lift_status','state':self.state,
            'moving': self.direction is not None,
            'pos1': r[5] if r else None,'pos2': r[6] if r else None,
            'motor_state': f'0x{r[0]:04X}' if r else None,
            'position_state': f'0x{r[1]:04X}' if r else None,
            'error': r[2] if r else None,
            'drive_interlock_ok': self.motor_safe(),
            'fault': self.fault}

    def update(self):
        try:
            r = self.read()
            self.last = r
        except Exception as exc:
            self.fault = 'Lost Modbus response: ' + str(exc)
            self.stop_motion('read failure')
            self.state = 'FAULT'
            return

        if r[2] or abs(r[5]-r[6]) > MAX_SYNC_DIFF:
            self.fault = 'Controller fault or position mismatch'
            self.stop_motion('fault or synchronization')
            self.state = 'FAULT'
            return

        if self.direction:
            if not self.motor_safe():
                self.fault = 'Drive state is not safely DISARMED'
                self.stop_motion('motor interlock lost')
                return
            if self.owner is None or time.monotonic()-self.owner_seen > HEARTBEAT_TIMEOUT:
                self.fault = 'Control heartbeat lost'
                self.stop_motion('web heartbeat lost')
                return
            if time.monotonic()-self.move_started > MAX_MOTION_SECONDS:
                self.fault = 'Motion time limit reached'
                self.stop_motion('time limit')
                return
            if self.start_pos:
                p1,p2=self.start_pos
                if self.direction == 'up' and (r[5]<p1-5 or r[6]<p2-5 or max(r[5],r[6])>2020):
                    self.fault='Unexpected upward motion position'
                    self.stop_motion('position limit')
                    return
                if self.direction == 'down' and (r[5]>p1+5 or r[6]>p2+5 or min(r[5],r[6])<0):
                    self.fault='Unexpected downward motion position'
                    self.stop_motion('position limit')
                    return
            target = TOP if self.direction == 'up' else BOTTOM
            run_state = RUN_UP if self.direction == 'up' else RUN_DOWN
            if r[1] == target:
                self.stop_motion('end position detected')
            elif r[0] != run_state:
                self.fault = 'Unexpected motor run status'
                self.stop_motion('unexpected run state')
                return
        if self.direction:
            self.state = 'MOVING_UP' if self.direction == 'up' else 'MOVING_DOWN'
        elif not self.fault:
            self.state = 'TOP' if r[1] == TOP and r[0] == STOP else 'BOTTOM' if r[1] == BOTTOM and r[0] == STOP else 'IDLE' if r[0] == STOP else 'UNKNOWN'

    async def loop(self):
        while not self.shutting_down:
            self.update()
            verified = bool(self.last and self.last[0] == STOP and self.last[2] == 0 and self.direction is None and self.state in ('IDLE','TOP','BOTTOM'))
            try:
                atomic_json(LIFT_FILE, {'time':time.monotonic(), 'state':self.state,
                    'verified_stopped':verified})
            except OSError as exc:
                self.fault='Lift state interlock write failed: '+str(exc)
                self.stop_motion('interlock write failed')
            snapshot=self.snapshot()
            for ws in list(self.clients):
                await self.emit(ws,snapshot)
            await asyncio.sleep(0.12)

    async def handle(self, ws):
        self.clients.add(ws)
        await self.emit(ws, self.snapshot())
        try:
            async for raw in ws:
                try: msg=json.loads(raw)
                except (ValueError,TypeError): continue
                if not isinstance(msg,dict): continue
                kind=msg.get('kind')
                if kind=='heartbeat':
                    if self.owner is ws:
                        self.owner_seen=time.monotonic()
                    continue
                if kind!='lift_command': continue
                command=msg.get('command')
                if command=='stop':
                    self.stop_motion('web STOP request')
                    await self.emit(ws,{'kind':'lift_result','message':'STOP requested. Check state confirmation.'})
                    continue
                if command not in ('up','down'):
                    continue
                if self.direction:
                    await self.emit(ws,{'kind':'lift_error','message':'Already moving: STOP first'})
                    continue
                if self.fault:
                    await self.emit(ws,{'kind':'lift_error','message':self.fault+'; restart after inspection'})
                    continue
                self.update()
                if (not self.motor_safe() or not self.last or self.last[0]!=STOP or self.last[2]!=0
                    or self.state not in ('TOP','BOTTOM','IDLE')):
                    await self.emit(ws,{'kind':'lift_error','message':'Motor not verified stopped/disarmed or lift not ready'})
                    continue
                if command=='up' and self.last[1]==TOP or command=='down' and self.last[1]==BOTTOM:
                    await self.emit(ws,{'kind':'lift_result','message':'Already at requested end position'})
                    continue
                # Store intended owner before issuing drive command.
                self.owner=ws
                self.owner_seen=time.monotonic()
                self.start_pos=(self.last[5],self.last[6])
                self.move_started=time.monotonic()
                self.direction=command
                self.state='MOVING_UP' if command=='up' else 'MOVING_DOWN'
                atomic_json(LIFT_FILE,{'time':time.monotonic(),'state':self.state,'verified_stopped':False})
                try:
                    self.command(3 if command=='up' else 2)
                    await self.emit(ws,{'kind':'lift_result','message':'UP command sent' if command=='up' else 'DOWN command sent'})
                except Exception as exc:
                    self.fault='Motion command failed: '+str(exc)
                    self.stop_motion('send failed')
                    await self.emit(ws,{'kind':'lift_error','message':self.fault})
        finally:
            self.clients.discard(ws)
            if self.owner is ws:
                self.stop_motion('controller disconnected')


async def main_async(args):
    c=Controller(args.port)
    if not c.serial.connect():
        raise RuntimeError('Cannot open serial port '+args.port)
    print('\n[LIFT] Listening on 0.0.0.0:8767',flush=True)
    print('[WARNING] DEVELOPMENT MODE: no access authentication. Isolate network.',flush=True)
    print('[LIFT] Motor ROS bridge with interlock must be running and DISARMED.',flush=True)
    monitor=asyncio.create_task(c.loop())
    try:
        async with websockets.serve(c.handle,'0.0.0.0',8767,ping_interval=2,ping_timeout=3,max_size=4096):
            await asyncio.Future()
    finally:
        c.shutting_down=True
        c.stop_motion('server shutdown')
        monitor.cancel()
        try:await monitor
        except asyncio.CancelledError:pass
        c.serial.close()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',default=DEFAULT_PORT)
    args=parser.parse_args()
    try:asyncio.run(main_async(args))
    except KeyboardInterrupt:pass

if __name__=='__main__':main()