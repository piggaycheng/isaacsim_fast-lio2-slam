#!/usr/bin/env python3
"""Fail explicitly if the Nav2 navigation lifecycle stack does not activate."""

import time

import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("navigation_readiness")
    names = ("planner_server", "controller_server", "bt_navigator")
    clients = {
        name: node.create_client(GetState, f"/{name}/get_state")
        for name in names
    }
    deadline = time.monotonic() + 35
    states = {}
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if not all(client.service_is_ready() for client in clients.values()):
                rclpy.spin_once(node, timeout_sec=0.2)
                continue
            states = {}
            for name, client in clients.items():
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2)
                if not future.done() or future.exception() is not None:
                    raise RuntimeError(f"Cannot query lifecycle state of {name}")
                if future.result() is None:
                    raise RuntimeError(f"Empty lifecycle state response from {name}")
                states[name] = future.result().current_state.id
            if all(state == 3 for state in states.values()):
                node.get_logger().info("Nav2 planner, controller and navigator active")
                return
            rclpy.spin_once(node, timeout_sec=0.5)
        raise TimeoutError(f"Nav2 activation timed out; lifecycle states: {states}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
