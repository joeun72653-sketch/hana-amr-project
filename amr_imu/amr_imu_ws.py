#!/usr/bin/env python3
"""Read-only ROS2 IMU RPY -> WebSocket telemetry on port 8766.
No motor commands are accepted or forwarded.
"""
import asyncio
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import websockets

HOST = '0.0.0.0'
PORT = 8766
STALE_SECONDS = 1.5

class ImuTelemetry(Node):
    def __init__(self):
        super().__init__('amr_imu_websocket_telemetry')
        self.lock = threading.Lock()
        self.latest = None
        self.received_at = 0.0
        self.create_subscription(String, '/amr/imu/rpy', self.on_rpy, 10)

    def on_rpy(self, msg):
        try:
            obj = json.loads(msg.data)
            angles = {k: float(obj[k]) for k in ('roll', 'pitch', 'yaw')}
            if not all(math.isfinite(v) for v in angles.values()):
                return
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            return
        with self.lock:
            self.latest = angles
            self.received_at = time.monotonic()

    def snapshot(self):
        with self.lock:
            if self.latest is None or time.monotonic() - self.received_at > STALE_SECONDS:
                return {'kind': 'imu', 'status': 'stale'}
            return {'kind': 'imu', 'status': 'ok', **self.latest}

async def handler(ws, node):
    # Read-only connection. The server never parses or executes incoming commands.
    try:
        while True:
            await ws.send(json.dumps(node.snapshot(), ensure_ascii=False))
            await asyncio.sleep(0.1)
    except (websockets.ConnectionClosed, OSError):
        pass

async def serve(node):
    async with websockets.serve(lambda ws: handler(ws, node), HOST, PORT,
                                ping_interval=10, ping_timeout=5, max_size=1024):
        print(f'[IMU] Read-only telemetry on ws://{HOST}:{PORT}', flush=True)
        await asyncio.Future()

def main():
    rclpy.init()
    node = ImuTelemetry()
    worker = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    worker.start()
    try:
        asyncio.run(serve(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        worker.join(timeout=2)

if __name__ == '__main__':
    main()
