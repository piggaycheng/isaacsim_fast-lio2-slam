#!/usr/bin/env python3
"""MQTT change_floor for a single robot without Open-RMF (e.g. scripts/run_nav.sh).

Listens on rmf/<fleet_name>/robot/<robot_name>/command for the same change_floor payload
as fleet_bridge_node and answers on .../command_result. Other actions are rejected; no
register or heartbeat is sent. Omitting initial_pose shows the floor's map and waits for
an RViz 2D Pose Estimate. The RViz Floor panel publishes navigation/floor_request, which is
sent to the broker as such a command, and follows navigation/floor_command_status.
"""

from collections import OrderedDict
import math
import json
import queue
import time

import paho.mqtt.client as mqtt
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from slam_localization_3d.srv import ChangeFloor
from std_msgs.msg import String

import rmf_protocol as rmf


class FloorCommandBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("floor_command_bridge", **kwargs)
        param = lambda name, default: self.declare_parameter(name, default).value
        host, port = param("mqtt_host", "localhost"), param("mqtt_port", 1883)
        username, password = param("mqtt_username", ""), param("mqtt_password", "")
        self.robot = param("robot_name", "carter")
        fleet = param("fleet_name", "manual")
        self.floor_base_z = float(param("floor_base_z", -0.526))
        self.timeout = float(param("floor_switch_timeout", 90.0))
        self.manual_pose_timeout = float(param("manual_pose_timeout", 300.0))
        if not all(math.isfinite(value) and value > 0
                   for value in (self.timeout, self.manual_pose_timeout)):
            raise ValueError("floor_switch_timeout and manual_pose_timeout must be finite and positive")
        if not math.isfinite(self.floor_base_z):
            raise ValueError("floor_base_z must be finite")
        self.topic = rmf.topics(fleet, self.robot)
        self.level = ""
        self.future = None
        self.cmd_id = None
        self.started = None
        self.deadline = None
        self.overdue = False
        self.results = OrderedDict()
        self.inbox = queue.Queue()
        self.next_cmd_id = int(time.time() * 1000)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "navigation/floor_command_status", latched)
        self.create_subscription(String, "navigation/floor_request", self.on_floor_request, 1)
        self.floor_client = self.create_client(ChangeFloor, "navigation/change_floor")
        self.create_subscription(
            String, "navigation/current_floor", lambda message: setattr(self, "level", message.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=f"floor_{fleet}_{self.robot}",
            clean_session=True)
        if username:
            self.client.username_pw_set(username, password)
        self.client.on_connect = self.on_connect
        self.client.on_message = lambda c, u, m: self.inbox.put(m.payload)
        self.client.reconnect_delay_set(1, 10)
        self.client.connect_async(host, port, keepalive=20)
        self.client.loop_start()
        self.create_timer(0.1, self.on_tick)
        self.get_logger().info(
            f"MQTT change_floor on mqtt://{host}:{port}/{self.topic['command']}")

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code != 0:
            self.get_logger().warning(f"MQTT connection refused: {reason_code}")
            return
        client.subscribe(self.topic["command"], 1)

    def status(self, text):
        self.status_pub.publish(String(data=text))

    def on_floor_request(self, message):
        floor = message.data.strip()
        if not self.client.is_connected():
            self.status(f"{floor}: MQTT broker is not connected")
            return
        self.next_cmd_id = max(self.next_cmd_id + 1, int(time.time() * 1000))
        self.client.publish(self.topic["command"], json.dumps({
            "robot_id": self.robot, "cmd_id": self.next_cmd_id, "action": "change_floor",
            "floor_name": floor,
        }), qos=1)
        self.status(f"Sent change_floor {floor} (cmd {self.next_cmd_id}) via MQTT")

    def result(self, cmd_id, status, message, cache=True):
        payload = rmf.command_result_payload(self.robot, cmd_id, status, message, level=self.level)
        if cache:
            self.results[cmd_id] = payload
            while len(self.results) > 64:
                self.results.popitem(last=False)
        self.client.publish(self.topic["command_result"], payload, qos=1)
        self.get_logger().info(f"change_floor {cmd_id} {status}: {message}")
        self.status(f"cmd {cmd_id} {status}: {message}")

    def on_command(self, payload):
        try:
            parsed = rmf.parse_command(payload, self.robot)
        except rmf.CommandError as error:
            cmd_id = rmf.cmd_id_of(payload)
            self.get_logger().warning(f"command rejected: {error}")
            if cmd_id is not None:
                self.result(cmd_id, "failed", str(error), cache=False)
            return
        if parsed is None:
            return
        cmd_id, action, data = parsed
        if action != "change_floor":
            self.result(cmd_id, "failed", "Only change_floor is supported without Open-RMF",
                        cache=False)
            return
        if cmd_id in self.results:
            self.client.publish(self.topic["command_result"], self.results[cmd_id], qos=1)
            return
        if self.future is not None:
            if cmd_id != self.cmd_id:
                self.result(cmd_id, "failed", "Another floor switch is in progress")
            return
        if not self.floor_client.service_is_ready():
            self.result(cmd_id, "failed", "Robot floor manager is unavailable", cache=False)
            return
        request = rmf.fill_change_floor_request(
            ChangeFloor.Request(), data, self.floor_base_z, self.get_clock().now().to_msg())
        self.cmd_id = cmd_id
        self.started = time.monotonic()
        self.deadline = self.timeout + (
            self.manual_pose_timeout if request.manual_initial_pose else 0.0)
        self.overdue = False
        self.future = self.floor_client.call_async(request)
        self.status(
            f"Switching to {request.floor_name} (cmd {cmd_id}); when the map appears, "
            "click 2D Pose Estimate" if request.manual_initial_pose
            else f"Switching to {request.floor_name} (cmd {cmd_id})")
        self.get_logger().info(
            f"MQTT floor switch {cmd_id}: {request.floor_name}"
            + (" (set the pose with RViz 2D Pose Estimate)" if request.manual_initial_pose else ""))

    def poll(self):
        future = self.future
        if future is None:
            return
        if not future.done():
            if time.monotonic() - self.started >= self.deadline and not self.overdue:
                self.overdue = True
                self.result(self.cmd_id, "failed",
                            "Floor switch response timed out; outcome unknown")
            return
        error = future.exception()
        response = None if error is not None else future.result()
        if response is not None:
            self.level = response.current_floor
        if not self.overdue:
            self.result(
                self.cmd_id,
                "completed" if response is not None and response.success else "failed",
                str(error) if error is not None else (
                    "Floor manager returned no response" if response is None else response.message))
        self.future = None
        self.cmd_id = None

    def on_tick(self):
        self.poll()
        while True:
            try:
                payload = self.inbox.get_nowait()
            except queue.Empty:
                break
            self.on_command(payload)

    def close(self):
        self.client.loop_stop()
        self.client.disconnect()


def main():
    rclpy.init()
    node = FloorCommandBridge()
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
