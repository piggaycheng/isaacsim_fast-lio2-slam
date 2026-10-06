#!/usr/bin/env python3
"""The robot's single MQTT connection for an external fleet adapter.

Publishes (all under <prefix>/<robot>/):
  state       pose from odometry/global (JSON, QoS 0, rate limited)
  online      retained; true while connected, false through the last will
  task_state  py_trees task state (QoS 1, retained) on change and once a second while running
Subscribes (QoS 1):
  command     task JSON, see task_bt.py
  cancel      {"goal_id": "..."} (goal_id optional)
The task runner is off when the task_bt parameter is false.
"""

import json
import os
import queue
import time

import paho.mqtt.client as mqtt
import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from fleet_pose import pose_payload
from task_bt import NavigateToPose as NavigateStep, TaskRunner


class FleetBridge(Node):
    def __init__(self):
        super().__init__("fleet_bridge")
        host = self.declare_parameter("mqtt_host", "localhost").value
        port = self.declare_parameter("mqtt_port", 1883).value
        prefix = self.declare_parameter("mqtt_topic_prefix", "fleet").value.strip("/")
        username = self.declare_parameter("mqtt_username", "").value
        password = self.declare_parameter("mqtt_password", "").value
        rate = self.declare_parameter("publish_rate", 2.0).value
        tick_rate = self.declare_parameter("tick_rate", 10.0).value
        self.tasks_enabled = self.declare_parameter("task_bt", True).value
        self.robot = self.get_namespace().strip("/")
        if not self.robot:
            raise ValueError("fleet_bridge must run in a robot namespace")
        base = f"{prefix}/{self.robot}"
        self.state_topic, self.online_topic = f"{base}/state", f"{base}/online"
        self.command_topic, self.cancel_topic = f"{base}/command", f"{base}/cancel"
        self.task_state_topic = f"{base}/task_state"
        self.min_period = 1.0 / rate if rate > 0 else 0.0
        self.last_publish = 0.0
        self.inbox = queue.Queue()

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"{self.robot}-fleet-bridge-{os.getpid()}",
        )
        if username:
            self.client.username_pw_set(username, password)
        self.client.will_set(self.online_topic, "false", qos=1, retain=True)
        self.client.on_connect = self.on_connect
        self.client.on_message = lambda c, u, m: self.inbox.put((m.topic, m.payload))
        self.client.reconnect_delay_set(1, 10)
        self.client.connect_async(host, port)
        self.client.loop_start()

        self.create_subscription(Odometry, "odometry/global", self.on_odometry, 10)
        if self.tasks_enabled:
            navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
            self.runner = TaskRunner(
                lambda step: NavigateStep(self, navigate_client, step), self.publish_task_state)
            self.create_timer(1.0 / tick_rate, self.on_tick)
            self.create_timer(1.0, self.on_heartbeat)
        self.get_logger().info(
            f"Fleet bridge for {self.robot} on mqtt://{host}:{port}/{base}/ "
            f"(tasks {'on' if self.tasks_enabled else 'off'})")

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code != 0:
            self.get_logger().warning(f"MQTT connection refused: {reason_code}")
            return
        client.publish(self.online_topic, "true", qos=1, retain=True)
        if self.tasks_enabled:
            client.subscribe([(self.command_topic, 1), (self.cancel_topic, 1)])

    def on_odometry(self, message):
        now = time.monotonic()
        if now - self.last_publish < self.min_period:
            return
        self.last_publish = now
        self.client.publish(self.state_topic, pose_payload(self.robot, message), qos=0)

    def publish_task_state(self, state, log=True):
        if log:
            self.get_logger().info(f"task state: {state}")
        self.client.publish(
            self.task_state_topic, json.dumps({"robot": self.robot, **state}), qos=1, retain=True)

    def on_heartbeat(self):
        if self.runner.active:
            self.publish_task_state(self.runner.state, log=False)

    def on_tick(self):
        while True:
            try:
                topic, payload = self.inbox.get_nowait()
            except queue.Empty:
                break
            if topic == self.command_topic:
                self.runner.submit(payload)
            else:
                try:
                    goal_id = json.loads(payload).get("goal_id")
                except (ValueError, AttributeError):
                    goal_id = None
                self.runner.cancel(goal_id)
        self.runner.tick()

    def close(self):
        if self.tasks_enabled and self.runner.active:
            self.runner.cancel()
        info = self.client.publish(self.online_topic, "false", qos=1, retain=True)
        info.wait_for_publish(timeout=2.0)
        self.client.disconnect()
        self.client.loop_stop()


def main():
    rclpy.init()
    node = FleetBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
