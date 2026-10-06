#!/usr/bin/env python3
"""Publish this robot's map-frame pose to MQTT for an external fleet adapter.

Runs in the robot's namespace and subscribes to odometry/global. State goes to
<prefix>/<robot>/state (JSON, QoS 0, rate limited); <prefix>/<robot>/online is
retained, true while connected and false through the last will.
"""

import json
import math
import os
import time

import paho.mqtt.client as mqtt
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def yaw_from_quaternion(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )


def pose_payload(robot, message):
    pose = message.pose.pose
    stamp = message.header.stamp
    return json.dumps({
        "robot": robot,
        "frame_id": message.header.frame_id,
        "stamp": stamp.sec + stamp.nanosec * 1e-9,
        "x": pose.position.x,
        "y": pose.position.y,
        "yaw": yaw_from_quaternion(pose.orientation),
    })


class MqttPoseBridge(Node):
    def __init__(self):
        super().__init__("mqtt_pose_bridge")
        host = self.declare_parameter("mqtt_host", "localhost").value
        port = self.declare_parameter("mqtt_port", 1883).value
        prefix = self.declare_parameter("mqtt_topic_prefix", "fleet").value.strip("/")
        username = self.declare_parameter("mqtt_username", "").value
        password = self.declare_parameter("mqtt_password", "").value
        rate = self.declare_parameter("publish_rate", 2.0).value
        self.robot = self.get_namespace().strip("/")
        if not self.robot:
            raise ValueError("mqtt_pose_bridge must run in a robot namespace")
        self.min_period = 1.0 / rate if rate > 0 else 0.0
        self.last_publish = 0.0
        self.state_topic = f"{prefix}/{self.robot}/state"
        self.online_topic = f"{prefix}/{self.robot}/online"

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"{self.robot}-pose-bridge-{os.getpid()}",
        )
        if username:
            self.client.username_pw_set(username, password)
        self.client.will_set(self.online_topic, "false", qos=1, retain=True)
        self.client.on_connect = self.on_connect
        self.client.reconnect_delay_set(1, 10)
        self.client.connect_async(host, port)
        self.client.loop_start()

        self.create_subscription(Odometry, "odometry/global", self.on_odometry, 10)
        self.get_logger().info(
            f"Publishing {self.robot} pose to mqtt://{host}:{port}/{self.state_topic}")

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            client.publish(self.online_topic, "true", qos=1, retain=True)
        else:
            self.get_logger().warning(f"MQTT connection refused: {reason_code}")

    def on_odometry(self, message):
        now = time.monotonic()
        if now - self.last_publish < self.min_period:
            return
        self.last_publish = now
        self.client.publish(self.state_topic, pose_payload(self.robot, message), qos=0)

    def close(self):
        info = self.client.publish(self.online_topic, "false", qos=1, retain=True)
        info.wait_for_publish(timeout=2.0)
        self.client.loop_stop()
        self.client.disconnect()


def main():
    rclpy.init()
    node = MqttPoseBridge()
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
