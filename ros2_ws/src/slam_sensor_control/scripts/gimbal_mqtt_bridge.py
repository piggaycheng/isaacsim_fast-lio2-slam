#!/usr/bin/env python3
"""MQTT front end for the gimbal action server, so external systems can rotate the gimbal.

Run it in the robot's namespace next to gimbal_action_server.py. MQTT commands on
<prefix>/<robot>/command become `gimbal/move` / `gimbal/rotate` goals; the outcome is
published on <prefix>/<robot>/result and the measured pose on <prefix>/<robot>/state
(see gimbal_protocol.py for the payloads).
"""

import math
import queue
import time

import paho.mqtt.client as mqtt
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

import gimbal_protocol as protocol
from slam_sensor_control.action import GimbalMove, GimbalRotate

PAN_JOINT = "gimbal_pan_joint"
TILT_JOINT = "gimbal_tilt_joint"
RESULT_STATUS = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_ABORTED: "aborted",
    GoalStatus.STATUS_CANCELED: "canceled",
}


class GimbalMqttBridge(Node):
    def __init__(self):
        super().__init__("gimbal_mqtt_bridge")
        param = lambda name, default: self.declare_parameter(name, default).value
        host, port = param("mqtt_host", "localhost"), param("mqtt_port", 1883)
        username, password = param("mqtt_username", ""), param("mqtt_password", "")
        self.robot = self.get_namespace().strip("/")
        self.topic = protocol.topics(param("mqtt_topic", "slam/gimbal"), self.robot)
        state_rate = param("state_rate", 2.0)
        self.pose = None
        self.pose_time = 0.0
        self.state_timeout = param("state_timeout", 2.0)
        self.active = None  # (cmd_id, goal handle or None while the goal is being accepted)
        self.inbox = queue.Queue()

        self.action_clients = {
            "move": ActionClient(self, GimbalMove, "gimbal/move"),
            "rotate": ActionClient(self, GimbalRotate, "gimbal/rotate"),
        }
        self.create_subscription(JointState, "gimbal/joint_states", self.on_joint_states, 10)

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"gimbal_{self.robot.replace('/', '_') or 'robot'}_{int(time.time())}",
            clean_session=True)
        if username:
            self.client.username_pw_set(username, password)
        self.client.will_set(self.topic["status"], protocol.offline_payload(self.robot),
                             qos=1, retain=True)
        self.client.on_connect = self.on_connect
        self.client.on_message = lambda c, u, m: self.inbox.put(m.payload)
        self.client.reconnect_delay_set(1, 10)
        # connect_async never blocks the node if the broker is down.
        self.client.connect_async(host, port, keepalive=20)
        self.client.loop_start()

        self.create_timer(0.05, self.on_tick)
        self.create_timer(1.0 / state_rate, self.on_state)
        self.get_logger().info(f"Gimbal MQTT bridge on mqtt://{host}:{port}/{self.topic['command']}")

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code != 0:
            self.get_logger().warning(f"MQTT connection refused: {reason_code}")
            return
        client.subscribe(self.topic["command"], qos=1)
        client.publish(self.topic["status"], protocol.online_payload(self.robot), qos=1, retain=True)

    def on_joint_states(self, message):
        names = list(message.name)
        if PAN_JOINT in names and TILT_JOINT in names and len(message.position) == len(names):
            self.pose = (message.position[names.index(PAN_JOINT)],
                         message.position[names.index(TILT_JOINT)])
            self.pose_time = time.monotonic()

    def fresh_pose(self):
        if self.pose is None or time.monotonic() - self.pose_time > self.state_timeout:
            return None
        return self.pose

    def publish_result(self, cmd_id, status, message, pose=None):
        self.get_logger().info(f"cmd {cmd_id}: {status} ({message})")
        self.client.publish(self.topic["result"], protocol.result_payload(
            self.robot, cmd_id, status, message, pose or self.fresh_pose()), qos=1)

    def on_state(self):
        pose = self.fresh_pose()
        if pose is None or not self.client.is_connected():
            return
        cmd_id = self.active[0] if self.active else None
        self.client.publish(self.topic["state"], protocol.state_payload(
            self.robot, pose, self.active is not None, cmd_id), qos=0)

    def handle_command(self, payload):
        try:
            cmd_id, action, pan, tilt = protocol.parse_command(payload)
        except protocol.CommandError as error:
            self.get_logger().warning(f"command rejected: {error}")
            cmd_id = protocol.cmd_id_of(payload)
            if cmd_id is not None:
                self.publish_result(cmd_id, "rejected", str(error))
            return
        if action == "cancel":
            self.cancel(cmd_id)
            return
        client = self.action_clients[action]
        if not client.server_is_ready():
            self.publish_result(cmd_id, "rejected", "Gimbal action server is not available")
            return
        if action == "move":
            goal = GimbalMove.Goal(pan=pan, tilt=tilt)
        else:
            goal = GimbalRotate.Goal(delta_pan=pan, delta_tilt=tilt)
        self.active = (cmd_id, None)
        client.send_goal_async(goal, feedback_callback=None).add_done_callback(
            lambda future, cmd_id=cmd_id: self.on_goal_response(cmd_id, future))

    def cancel(self, cmd_id):
        if self.active is None:
            self.publish_result(cmd_id, "succeeded", "Nothing to cancel")
            return
        active_id, handle = self.active
        if handle is not None:
            handle.cancel_goal_async()
        self.publish_result(cmd_id, "succeeded", f"Cancel requested for cmd {active_id}")

    def on_goal_response(self, cmd_id, future):
        handle = future.result()
        if not handle.accepted:
            if self.active and self.active[0] == cmd_id:
                self.active = None
            self.publish_result(cmd_id, "rejected", "Goal rejected (angle out of limits?)")
            return
        if self.active and self.active[0] == cmd_id:
            self.active = (cmd_id, handle)
        handle.get_result_async().add_done_callback(
            lambda result_future: self.on_result(cmd_id, result_future))

    def on_result(self, cmd_id, future):
        wrapped = future.result()
        if self.active and self.active[0] == cmd_id:
            self.active = None
        status = RESULT_STATUS.get(wrapped.status, "aborted")
        result = wrapped.result
        pose = (result.pan, result.tilt) if math.isfinite(result.pan + result.tilt) else None
        self.publish_result(cmd_id, status, result.message, pose)

    def on_tick(self):
        while True:
            try:
                payload = self.inbox.get_nowait()
            except queue.Empty:
                return
            self.handle_command(payload)

    def close(self):
        if self.client.is_connected():
            info = self.client.publish(self.topic["status"], protocol.offline_payload(self.robot),
                                       qos=1, retain=True)
            info.wait_for_publish(timeout=2.0)
        self.client.disconnect()
        self.client.loop_stop()


def main():
    rclpy.init()
    node = GimbalMqttBridge()
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
