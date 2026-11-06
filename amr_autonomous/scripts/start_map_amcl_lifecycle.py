#!/usr/bin/env python3
"""Configure and activate existing Map Server + AMCL (not motor control).

Waits for lifecycle services; aborts on unexpected state or timeout.
Requires source /opt/ros/jazzy/setup.bash and ~/real_amr/install/setup.bash.
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


def request(node, client, request_type, value=None, timeout=4.0):
    req = request_type.Request()
    if value is not None:
        req.transition.id = value
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError("Lifecycle service call timeout")
    result = future.result()
    if result is None:
        raise RuntimeError("Lifecycle service call returned no result")
    return result


def activate(node, name, deadline):
    status = node.create_client(GetState, name + "/get_state")
    change = node.create_client(ChangeState, name + "/change_state")
    try:
        while time.monotonic() < deadline:
            if status.wait_for_service(timeout_sec=0.5) and change.wait_for_service(timeout_sec=0.5):
                break
        else:
            raise TimeoutError(name + " lifecycle services unavailable")
        while time.monotonic() < deadline:
            state = request(node, status, GetState).current_state.id
            if state == ACTIVE:
                print("[LIFECYCLE]", name, "already active", flush=True)
                return
            if state == UNCONFIGURED:
                transition = Transition.TRANSITION_CONFIGURE
            elif state == INACTIVE:
                transition = Transition.TRANSITION_ACTIVATE
            else:
                # Transitional/error states should be handled by their owner.
                raise RuntimeError(f"{name} unexpected lifecycle state={state}")
            if not request(node, change, ChangeState, transition).success:
                raise RuntimeError(f"{name} transition {transition} rejected")
        raise TimeoutError(name + " not active before timeout")
    finally:
        node.destroy_client(status)
        node.destroy_client(change)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()
    if not 1 <= args.timeout <= 180:
        parser.error("--timeout must be 1..180 seconds")
    rclpy.init()
    node = Node("amr_map_amcl_lifecycle_starter")
    try:
        deadline = time.monotonic() + args.timeout
        activate(node, "/map_server", deadline)
        activate(node, "/amcl", deadline)
        print("[LIFECYCLE] map_server and amcl ACTIVE", flush=True)
    except Exception as error:
        print(f"[LIFECYCLE ERROR] {error}", file=sys.stderr, flush=True)
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
