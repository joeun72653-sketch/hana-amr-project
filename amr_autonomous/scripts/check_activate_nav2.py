#!/usr/bin/env python3
"""Verify Nav2 lifecycle startup and activate remaining inactive servers safely.

Preview only. Uses lifecycle services; never sends goals, cmd_vel or CAN RPM.
It performs a bounded one-time startup recovery, not an infinite restart loop.
"""
import argparse
import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

ACTIVE = State.PRIMARY_STATE_ACTIVE
INACTIVE = State.PRIMARY_STATE_INACTIVE
UNCONFIGURED = State.PRIMARY_STATE_UNCONFIGURED

# Avoid launching BT Navigator before its behavior/controller/planner dependencies.
SERVERS = (
    "/controller_server",
    "/planner_server",
    "/behavior_server",
    "/bt_navigator",
)
COSTMAPS = ("/local_costmap/local_costmap", "/global_costmap/global_costmap")


def call(node, client, req, timeout=5.0):
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError("Lifecycle service call timed out")
    result = future.result()
    if result is None:
        raise RuntimeError("Lifecycle service returned no response")
    return result


def state(node, name):
    client = node.create_client(GetState, name + "/get_state")
    try:
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        return call(node, client, GetState.Request()).current_state.id
    finally:
        node.destroy_client(client)


def transition(node, name, transition_id):
    client = node.create_client(ChangeState, name + "/change_state")
    try:
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"{name} change_state unavailable")
        req = ChangeState.Request()
        req.transition.id = transition_id
        if not call(node, client, req).success:
            raise RuntimeError(f"{name} transition {transition_id} rejected")
        print(f"[NAV2] {name}: transition {transition_id} accepted", flush=True)
    finally:
        node.destroy_client(client)


def wait_state(node, name, expected, deadline):
    while time.monotonic() < deadline:
        current = state(node, name)
        if current == expected:
            return
        time.sleep(0.25)
    raise TimeoutError(f"{name} did not reach state {expected}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=70.0)
    parser.add_argument("--autostart-grace", type=float, default=8.0)
    args = parser.parse_args()
    if not 10 <= args.timeout <= 180 or not 0 <= args.autostart_grace <= 30:
        parser.error("timeout must be 10..180, autostart-grace 0..30")
    rclpy.init()
    node = Node("amr_nav2_lifecycle_verifier")
    deadline = time.monotonic() + args.timeout
    try:
        # Let the built-in Nav2 lifecycle manager finish first. Inactive
        # nodes are recovered only after the grace period, not during startup.
        time.sleep(args.autostart_grace)
        for name in SERVERS:
            while True:
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"Nav2 startup expired before {name}")
                current = state(node, name)
                if current is None:
                    time.sleep(0.5)
                    continue
                if current == ACTIVE:
                    print(f"[NAV2] {name}: ACTIVE", flush=True)
                    break
                if current == UNCONFIGURED:
                    transition(node, name, Transition.TRANSITION_CONFIGURE)
                    wait_state(node, name, INACTIVE, deadline)
                elif current == INACTIVE:
                    transition(node, name, Transition.TRANSITION_ACTIVATE)
                    wait_state(node, name, ACTIVE, deadline)
                else:
                    raise RuntimeError(f"{name}: unexpected lifecycle state {current}")

        # Costmaps are lifecycle-managed by their owners (controller/planner).
        # Never independently activate them; verify owner activation succeeded.
        for name in COSTMAPS:
            wait_state(node, name, ACTIVE, deadline)
            print(f"[NAV2] {name}: ACTIVE", flush=True)
        for name in SERVERS:
            if state(node, name) != ACTIVE:
                raise RuntimeError(f"{name} became inactive during verification")
        print("[NAV2 READY] Planner, Controller, Behavior, BT and both Costmaps ACTIVE",
              flush=True)
    except Exception as exc:
        print(f"[NAV2 STARTUP FAILED] {exc}", file=sys.stderr, flush=True)
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
