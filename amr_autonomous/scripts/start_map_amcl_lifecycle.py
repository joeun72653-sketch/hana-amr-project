#!/usr/bin/env python3
"""Race-tolerant startup of Map Server and AMCL lifecycle nodes (no motor commands).

A Nav2 lifecycle manager may already be configuring the map_server. Handle
intermediate states and rejected duplicate transitions by re-reading state.
Never assume that a successful service call means the target has reached ACTIVE.
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
TRANSIENT = {
    State.TRANSITION_STATE_CONFIGURING,
    State.TRANSITION_STATE_CLEANINGUP,
    State.TRANSITION_STATE_SHUTTINGDOWN,
    State.TRANSITION_STATE_ACTIVATING,
    State.TRANSITION_STATE_DEACTIVATING,
}
ERROR = State.TRANSITION_STATE_ERRORPROCESSING
FINAL = State.PRIMARY_STATE_FINALIZED


def send(node, client, request, deadline, label):
    future = client.call_async(request)
    # Use bounded ROS spinning and never issue an overlapping request.
    limit = min(deadline, time.monotonic() + 8.0)
    while not future.done() and time.monotonic() < limit:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        client.remove_pending_request(future)
        raise TimeoutError(f"{label}: lifecycle service timeout")
    result = future.result()
    if result is None:
        raise RuntimeError(f"{label}: lifecycle service returned no result")
    return result


def activate(node, name, deadline):
    state_client = node.create_client(GetState, name + "/get_state")
    change_client = node.create_client(ChangeState, name + "/change_state")
    last = None
    try:
        while time.monotonic() < deadline:
            if not (state_client.wait_for_service(timeout_sec=0.5)
                    and change_client.wait_for_service(timeout_sec=0.5)):
                continue
            try:
                current = send(node, state_client, GetState.Request(), deadline, name).current_state.id
            except TimeoutError:
                time.sleep(0.25)
                continue
            if current == ACTIVE:
                print(f"[LIFECYCLE] {name}: ACTIVE", flush=True)
                return
            if current in (ERROR, FINAL):
                raise RuntimeError(f"{name}: entered error/final state {current}")
            if current in TRANSIENT:
                # The node is already transitioning; NEVER issue a competing change_state.
                if current != last:
                    print(f"[LIFECYCLE] {name}: transitioning ({current}), waiting", flush=True)
                last = current
                time.sleep(0.25)
                continue
            if current not in (UNCONFIGURED, INACTIVE):
                raise RuntimeError(f"{name}: unexpected lifecycle state {current}")
            action = (Transition.TRANSITION_CONFIGURE if current == UNCONFIGURED
                      else Transition.TRANSITION_ACTIVATE)
            req = ChangeState.Request()
            req.transition.id = action
            try:
                response = send(node, change_client, req, deadline, name)
            except TimeoutError:
                # Recheck after a service delay; do not assume failure or success.
                time.sleep(0.25)
                continue
            if not response.success:
                # Another lifecycle manager may have won the race. Re-read state.
                print(f"[LIFECYCLE] {name}: transition {action} rejected; "
                      "rechecking state instead of aborting", flush=True)
                time.sleep(0.5)
            else:
                print(f"[LIFECYCLE] {name}: transition {action} accepted", flush=True)
                time.sleep(0.2)
        raise TimeoutError(f"{name}: not ACTIVE within startup deadline")
    finally:
        node.destroy_client(state_client)
        node.destroy_client(change_client)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()
    if not 10 <= args.timeout <= 180:
        parser.error("--timeout must be 10..180 seconds")
    rclpy.init()
    node = Node("amr_map_amcl_lifecycle_starter")
    try:
        deadline = time.monotonic() + args.timeout
        activate(node, "/map_server", deadline)
        activate(node, "/amcl", deadline)
        print("[LIFECYCLE READY] map_server and amcl ACTIVE", flush=True)
    except Exception as error:
        print(f"[LIFECYCLE ERROR] {error}", file=sys.stderr, flush=True)
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
