#!/usr/bin/env python3
"""ROS2 UInt8 /lift/command -> existing ORGO lift WebSocket controller.

0 = STOP/idle, 1 = TOP ascent, 2 = BOTTOM descent.
The ROS bridge must not open the RS-232 port itself. Run lift_websocket.py
separately. Interlocks remain in lift_websocket.py.
"""
import argparse
import asyncio
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, String, Bool
import websockets


class LiftRosBridge(Node):
    def __init__(self, url, token):
        super().__init__('amr_lift_ros_bridge')
        self.url, self.token = url, token
        self.lock = threading.Lock()
        self.network_loop = None
        self.command_queue = None
        self.last_status = None
        self.last_status_at = 0.0
        self.connected = False
        self.authed = False
        self.running = True
        self.state_pub = self.create_publisher(String, '/lift/state', 10)
        self.moving_pub = self.create_publisher(Bool, '/lift/moving', 10)
        self.create_subscription(UInt8, '/lift/command', self.on_command, 10)
        self.thread = threading.Thread(target=self.network_thread, daemon=True)
        self.thread.start()
        self.get_logger().info('Listening: /lift/command UInt8; 0=STOP, 1=TOP, 2=BOTTOM')

    def on_command(self, msg):
        value = int(msg.data)
        if value not in (0, 1, 2):
            self.get_logger().warning(f'Invalid lift command: {value}')
            return
        with self.lock:
            loop, queue = self.network_loop, self.command_queue
            connected, authed = self.connected, self.authed
            status = dict(self.last_status) if self.last_status else None
            age = time.monotonic() - self.last_status_at
        if not (loop and queue and connected and authed):
            self.get_logger().error('Lift WebSocket unavailable; no command sent')
            return
        if value != 0 and (
            status is None or age > 0.6 or status.get('fault')
            or status.get('state') in ('FAULT', 'UNKNOWN')
            or status.get('moving') is not False
            or status.get('drive_interlock_ok') is not True
        ):
            self.get_logger().error('Lift motion rejected: stale state, fault, moving, or unsafe wheel state')
            return
        try:
            loop.call_soon_threadsafe(queue.put_nowait, value)
        except RuntimeError:
            self.get_logger().error('Lift WebSocket disconnected')

    def network_thread(self):
        asyncio.run(self.network_worker())

    async def network_worker(self):
        while self.running and rclpy.ok():
            try:
                async with websockets.connect(self.url, ping_interval=2, ping_timeout=3) as ws:
                    queue = asyncio.Queue()
                    with self.lock:
                        self.network_loop = asyncio.get_running_loop()
                        self.command_queue = queue
                        self.connected = True
                        self.authed = not bool(self.token)
                        self.last_status = None
                        self.last_status_at = 0.0
                    if self.token:
                        await ws.send(json.dumps({'kind': 'auth', 'token': self.token}))
                    self.get_logger().info(f'Lift WS connected: {self.url}')
                    receiver = asyncio.create_task(self.receiver(ws))
                    sender = asyncio.create_task(self.sender(ws, queue))
                    done, pending = await asyncio.wait((receiver, sender), return_when=asyncio.FIRST_COMPLETED)
                    for task in pending:
                        task.cancel()
                    await asyncio.gather(*pending, return_exceptions=True)
                    for task in done:
                        if task.exception():
                            raise task.exception()
            except Exception as exc:
                self.get_logger().warning(f'Lift WS disconnected: {exc}')
            finally:
                with self.lock:
                    self.network_loop = None
                    self.command_queue = None
                    self.connected = False
                    self.authed = False
                    self.last_status = None
                self.state_pub.publish(String(data='DISCONNECTED'))
                self.moving_pub.publish(Bool(data=False))
            if self.running and rclpy.ok():
                await asyncio.sleep(1)

    async def receiver(self, ws):
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except (TypeError, ValueError):
                continue
            if not isinstance(msg, dict):
                continue
            if msg.get('kind') == 'auth':
                with self.lock:
                    self.authed = msg.get('ok') is True
                if msg.get('ok') is not True:
                    self.get_logger().error('Lift WS auth failed')
            elif msg.get('kind') == 'lift_status':
                with self.lock:
                    self.last_status = msg
                    self.last_status_at = time.monotonic()
                self.state_pub.publish(String(data=str(msg.get('state', 'UNKNOWN'))))
                self.moving_pub.publish(Bool(data=msg.get('moving') is True))
            elif msg.get('kind') == 'lift_error':
                self.get_logger().error('Lift rejected: ' + str(msg.get('message', msg.get('reason'))))
            elif msg.get('kind') == 'lift_result':
                self.get_logger().info('Lift response: ' + str(msg.get('message', '')))

    async def sender(self, ws, queue):
        while True:
            try:
                value = await asyncio.wait_for(queue.get(), timeout=0.15)
            except asyncio.TimeoutError:
                value = None
            if value is not None:
                if value == 0:
                    while not queue.empty():
                        try:
                            queue.get_nowait()
                        except asyncio.QueueEmpty:
                            break
                command = {0: 'stop', 1: 'up', 2: 'down'}[value]
                await ws.send(json.dumps({'kind': 'lift_command', 'command': command}))
                self.get_logger().info(f'/lift/command={value} -> {command}')
            await ws.send(json.dumps({'kind': 'heartbeat'}))

    def destroy_node(self):
        self.running = False
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='ws://127.0.0.1:8767')
    parser.add_argument('--token', default='', help='Only if lift server requires auth')
    args = parser.parse_args()
    rclpy.init()
    node = LiftRosBridge(args.url, args.token)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
