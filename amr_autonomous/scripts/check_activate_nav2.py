#!/usr/bin/env python3
"""Passive Nav2 readiness verifier for the LIVE AMR launch.

Nav2's own lifecycle_manager_navigation already owns transitions. Issuing a
second change_state can race its bringup and abort startup. This process ONLY
waits for all required lifecycle nodes to become ACTIVE; never commands wheels.
"""
import argparse
import sys
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.node import Node

NODES = (
    "/controller_server",
    "/planner_server",
    "/behavior_server",
    "/bt_navigator",
    "/local_costmap/local_costmap",
    "/global_costmap/global_costmap",
)
ACTIVE = State.PRIMARY_STATE_ACTIVE
ERROR = State.TRANSITION_STATE_ERRORPROCESSING
FINAL = State.PRIMARY_STATE_FINALIZED


def get_state(node, client):
    if not client.service_is_ready():
        return None
    future = client.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if not future.done():
        client.remove_pending_request(future)
        return None
    result = future.result()
    return result.current_state.id if result is not None else None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--timeout", type=float, default=120)
    p.add_argument("--autostart-grace", type=float, default=8)
    args = p.parse_args()
    if not 10 <= args.timeout <= 240 or not 0 <= args.autostart_grace <= 30:
        p.error("timeout must be 10..240; autostart-grace 0..30")
    rclpy.init()
    node = Node("amr_nav2_lifecycle_verifier")
    clients = {name: node.create_client(GetState, name+"/get_state") for name in NODES}
    deadline = time.monotonic()+args.timeout
    last = {}
    try:
        time.sleep(args.autostart_grace)
        while time.monotonic() < deadline:
            all_active = True
            for name, client in clients.items():
                current = get_state(node, client)
                if last.get(name) != current:
                    print(f"[NAV2] {name}: {current if current is not None else 'WAITING'}", flush=True)
                    last[name] = current
                if current in (ERROR, FINAL):
                    raise RuntimeError(f"{name}: ERROR/FINAL state {current}")
                if current != ACTIVE:
                    all_active = False
            if all_active:
                print("[NAV2 READY] Planner, Controller, Behavior, BT "
                      "and both Costmaps ACTIVE", flush=True)
                return 0
            time.sleep(0.5)
        raise TimeoutError("Nav2 lifecycle manager did not activate all nodes; "
                           "CAN remains off. Check TF /scan_slam and Nav2 logs.")
    except Exception as e:
        print(f"[NAV2 STARTUP FAILED] {e}", file=sys.stderr, flush=True)
        return 1
    finally:
        for client in clients.values():
            node.destroy_client(client)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
