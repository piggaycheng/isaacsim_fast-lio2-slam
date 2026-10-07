#!/usr/bin/env python3
"""The robot's single MQTT connection for an Open-RMF fleet adapter (amr_mqtt_interface_spec.md).

Topics under rmf/<fleet>/robot/<robot>/ :
  register (out)        sent on every (re)connect until register_ack succeeds
  register_ack (in)     handshake; heartbeat and commands start only after success
  heartbeat (out)       pose from odometry/global, battery, status, current_cmd_id
  command (in)          navigate / dock run as one Nav2 goal (py_trees task); stop cancels;
                        task (extension) runs a list of task_bt steps
  task_state (out)      extension: task progress (retained) on every change
  command_result (out)  completed / failed / canceled for a cmd_id
  deregister (out)      on shutdown
  status (LWT)          offline message published by the broker on unexpected disconnect
"""

import json
import math
import queue

import paho.mqtt.client as mqtt
import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

import rmf_protocol as rmf
from fleet_pose import yaw_from_quaternion
from task_bt import NavigateToPose as NavigateStep, TaskRunner

REGISTER_RETRY_S = 5.0


class FleetBridge(Node):
    def __init__(self):
        super().__init__("fleet_bridge")
        param = lambda name, default: self.declare_parameter(name, default).value
        host, port = param("mqtt_host", "localhost"), param("mqtt_port", 1883)
        username, password = param("mqtt_username", ""), param("mqtt_password", "")
        self.fleet = param("fleet_name", "")
        self.robot = self.get_namespace().strip("/")
        if not self.robot or not self.fleet:
            raise ValueError("fleet_bridge needs a robot namespace and the fleet_name parameter")
        self.level = param("level_name", "L1")
        self.waypoint = param("waypoint_name", "")
        self.charger, self.parking = param("default_charger", ""), param("default_parking", "")
        self.specs = {
            "footprint_radius": param("footprint_radius", 0.35),
            "max_linear_velocity": param("max_linear_velocity", 1.2),
            "max_angular_velocity": param("max_angular_velocity", 1.0),
        }
        self.battery = float(param("battery", 100.0))
        heartbeat_rate = param("heartbeat_rate", 2.0)
        tick_rate = param("tick_rate", 10.0)
        self.topic = rmf.topics(self.fleet, self.robot)
        self.pose = None
        self.registered = False
        self.last_register = -REGISTER_RETRY_S
        self.cmd_ids = {}
        self.inbox = queue.Queue()

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=rmf.client_id(self.fleet, self.robot),
            clean_session=True)
        if username:
            self.client.username_pw_set(username, password)
        self.client.will_set(self.topic["status"], rmf.offline_payload(self.robot), qos=1)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = lambda *args: setattr(self, "registered", False)
        self.client.on_message = lambda c, u, m: self.inbox.put((m.topic, m.payload))
        self.client.reconnect_delay_set(1, 10)
        self.client.connect_async(host, port, keepalive=20)
        self.client.loop_start()

        self.create_subscription(Odometry, "odometry/global", self.on_odometry, 10)
        self.create_subscription(BatteryState, "battery_state", self.on_battery, 10)
        navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.runner = TaskRunner(
            lambda step: NavigateStep(self, navigate_client, step), self.on_task_state)
        self.create_timer(1.0 / tick_rate, self.on_tick)
        self.create_timer(1.0 / heartbeat_rate, self.on_heartbeat)
        self.get_logger().info(
            f"Fleet bridge for {self.robot} (fleet {self.fleet}) on mqtt://{host}:{port}/"
            f"rmf/{self.fleet}/robot/{self.robot}/")

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code != 0:
            self.get_logger().warning(f"MQTT connection refused: {reason_code}")
            return
        self.registered = False
        self.last_register = -REGISTER_RETRY_S
        client.subscribe([(self.topic["register_ack"], 1), (self.topic["command"], 1)])

    def on_odometry(self, message):
        pose = message.pose.pose
        self.pose = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))

    def on_battery(self, message):
        if math.isfinite(message.percentage):
            self.battery = message.percentage * 100.0

    def publish(self, name, payload, qos=1):
        self.client.publish(self.topic[name], payload, qos=qos)

    def maybe_register(self):
        if self.registered or self.pose is None or not self.client.is_connected():
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if abs(now - self.last_register) < REGISTER_RETRY_S:
            return
        self.last_register = now
        self.publish("register", rmf.register_payload(
            self.fleet, self.robot, self.pose, self.level, self.waypoint, self.specs,
            self.charger, self.parking))

    def on_task_state(self, state):
        self.get_logger().info(f"task state: {state}")
        cmd_id = self.cmd_ids.get(state.get("goal_id"))
        self.client.publish(self.topic["task_state"], json.dumps(
            {"robot_id": self.robot, "cmd_id": cmd_id, **state}), qos=1, retain=True)
        status = rmf.RESULT_STATUS.get(state["status"])
        if status is None or cmd_id is None:
            return
        self.publish("command_result", rmf.command_result_payload(
            self.robot, cmd_id, status, state.get("message", ""), self.pose))

    def handle_command(self, payload):
        try:
            parsed = rmf.parse_command(payload, self.robot)
        except rmf.CommandError as error:
            self.get_logger().warning(f"command rejected: {error}")
            cmd_id = rmf.cmd_id_of(payload)
            if cmd_id is not None:
                self.publish("command_result", rmf.command_result_payload(
                    self.robot, cmd_id, "failed", str(error)))
            return
        if parsed is None:
            return
        cmd_id, action, steps = parsed
        if action == "stop":
            self.runner.cancel()
            self.publish("command_result", rmf.command_result_payload(
                self.robot, cmd_id, "completed", "Stopped", self.pose))
            return
        goal_id = str(cmd_id)
        self.cmd_ids[goal_id] = cmd_id
        self.runner.submit(json.dumps({"goal_id": goal_id, "steps": steps}))

    def on_heartbeat(self):
        self.maybe_register()
        if not self.registered or self.pose is None:
            return
        active = self.runner.active
        cmd_id = self.cmd_ids.get(self.runner.task.goal_id) if active else None
        self.publish("heartbeat", rmf.heartbeat_payload(
            self.robot, self.pose, self.battery, "moving" if active else "idle", cmd_id), qos=0)

    def on_tick(self):
        while True:
            try:
                topic, payload = self.inbox.get_nowait()
            except queue.Empty:
                break
            if topic == self.topic["register_ack"]:
                ack = rmf.parse_ack(payload, self.robot)
                if ack is None:
                    continue
                self.registered = ack[0]
                if ack[0]:
                    self.get_logger().info(f"registered with Open-RMF: {ack[1]}")
                else:
                    self.get_logger().warning(f"registration rejected: {ack[1]}; retrying")
            elif self.registered:
                self.handle_command(payload)
            else:
                self.get_logger().warning("ignoring command before register_ack")
        self.runner.tick()

    def close(self):
        if self.runner.active:
            self.runner.cancel()
        if self.client.is_connected():
            info = self.client.publish(self.topic["deregister"], rmf.deregister_payload(
                self.fleet, self.robot, "Shutdown"), qos=1)
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
