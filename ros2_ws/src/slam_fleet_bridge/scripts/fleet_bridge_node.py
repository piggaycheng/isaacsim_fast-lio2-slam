#!/usr/bin/env python3
"""The robot's single MQTT connection for an Open-RMF fleet adapter (amr_mqtt_interface_spec.md).

Topics under rmf/<fleet>/robot/<robot>/ :
  register (out)        retried every 3 s until register_ack succeeds, and at
                        once on register_ack require_register (adapter started later/restarted)
  register_ack (in)     handshake; commands are accepted only after success
  heartbeat (out)       pose from odometry/global, battery, status, current_cmd_id; sent even
                        before registration (status "error") so a late adapter can ask for it
  command (in)          navigate / dock run as one Nav2 goal (py_trees task); stop cancels;
                        task (extension) runs a list of task_bt steps
                        (navigate, gimbal_move, gimbal_rotate); gimbal_move / gimbal_rotate
                        (extension) run one gimbal step
                        change_floor (extension) coordinates target-floor PGM/PCD
                        and localization through the robot's internal service
  task_state (out)      extension: task progress (retained) on every change
  command_result (out)  completed / failed / canceled for a cmd_id
  deregister (out)      on shutdown
  status (LWT)          offline message published by the broker on unexpected disconnect
"""

import json
import math
import queue
import time
import uuid
from collections import OrderedDict

import paho.mqtt.client as mqtt
import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from rosgraph_msgs.msg import Clock
from slam_sensor_control.action import GimbalMove, GimbalRotate

import rmf_protocol as rmf
from fleet_pose import yaw_from_quaternion
from task_bt import STEP_BEHAVIOURS, TaskRunner

REGISTER_RETRY_S = 3.0


class FleetBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("fleet_bridge", **kwargs)
        param = lambda name, default: self.declare_parameter(name, default).value
        host, port = param("mqtt_host", "localhost"), param("mqtt_port", 1883)
        username, password = param("mqtt_username", ""), param("mqtt_password", "")
        self.fleet = param("fleet_name", "")
        self.robot = self.get_namespace().strip("/")
        if not self.robot or not self.fleet:
            raise ValueError("fleet_bridge needs a robot namespace and the fleet_name parameter")
        self.level = param("level_name", "1F")
        self.waypoint = param("waypoint_name", "")
        self.charger, self.parking = param("default_charger", ""), param("default_parking", "")
        # robot.launch.py passes the global costmap robot_radius and cmd_vel_safety's speed caps;
        # no generic default fits every robot.
        self.specs = {
            name: float(param(name, 0.0))
            for name in ("footprint_radius", "max_linear_velocity", "max_angular_velocity")
        }
        for name, value in self.specs.items():
            if not value > 0.0:
                raise ValueError(f"fleet_bridge needs a positive {name} parameter")
        # base_link z in every floor map when change_floor omits initial_pose.z (Nova Carter default).
        self.floor_base_z = float(param("floor_base_z", -0.526))
        self.battery = float(param("battery", 100.0))
        heartbeat_rate = param("heartbeat_rate", 2.0)
        tick_rate = param("tick_rate", 10.0)
        self.topic = rmf.topics(self.fleet, self.robot)
        self.pose = None
        self.pose_stamp = None
        self.pose_received = None
        self.correction_stamp = None
        self.correction_received = None
        self.last_clock = None
        self.session_id = str(uuid.uuid4())
        self.session_reset_pending = True
        self.session_reset_error = ""
        self.cancel_future = None
        self.cancel_started = None
        self.cancel_goals = None
        self.nav_status = {}
        self.task_sessions = {}
        self.pose_timeout = float(param("pose_timeout", 1.0))
        self.correction_timeout = float(param("correction_timeout", 5.0))
        self.cancel_timeout = float(param("cancel_timeout", 10.0))
        if not all(math.isfinite(value) and value > 0 for value in (
                self.pose_timeout, self.correction_timeout, self.cancel_timeout)):
            raise ValueError(
                "pose_timeout, correction_timeout and cancel_timeout must be positive and finite")
        self.registered = False
        self.last_register = -REGISTER_RETRY_S
        self.cmd_ids = {}
        self.inbox = queue.Queue()
        self.floor_future = None
        self.floor_cmd_id = None
        self.floor_results = OrderedDict()
        self.floor_error = False
        self.floor_prepared = False
        self.floor_action = None
        self.floor_started = None
        self.floor_response_overdue = False
        self.floor_timeout = float(param("floor_switch_timeout", 90.0))
        # Extra wait for an operator's RViz 2D Pose Estimate (change_floor without initial_pose).
        self.manual_pose_timeout = float(param("manual_pose_timeout", 300.0))
        if not all(math.isfinite(value) and value > 0
                   for value in (self.floor_timeout, self.manual_pose_timeout)):
            raise ValueError("floor_switch_timeout and manual_pose_timeout must be finite and positive")
        self.floor_deadline = self.floor_timeout
        from slam_localization_3d.srv import ChangeFloor, PrepareFloorChange
        from std_msgs.msg import Header, String
        from std_srvs.srv import Trigger
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        self.floor_client = self.create_client(ChangeFloor, "navigation/change_floor")
        self.prepare_client = self.create_client(
            PrepareFloorChange, "navigation/prepare_floor_change")
        self.create_subscription(
            String, "navigation/current_floor", self.on_floor,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            String, "navigation/floor_transition_state", self.on_floor_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.cancel_client = self.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal")
        self.create_subscription(
            GoalStatusArray, "navigate_to_pose/_action/status", self.on_nav_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(Header, "localization_3d/accepted_correction", self.on_correction, 10)
        self.create_subscription(
            Clock, "/clock", self.on_clock,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_service(Trigger, "fleet_bridge/reset_session", self.on_reset_session)

        self.connect_mqtt(host, port, username, password)

        self.create_subscription(Odometry, "odometry/global", self.on_odometry, 10)
        self.create_subscription(BatteryState, "battery_state", self.on_battery, 10)
        action_types = {"navigate": NavigateToPose, "gimbal_move": GimbalMove,
                        "gimbal_rotate": GimbalRotate}
        clients = {kind: ActionClient(self, action_types[kind], name)
                   for kind, (_, name) in STEP_BEHAVIOURS.items()}
        self.runner = TaskRunner(
            lambda step: STEP_BEHAVIOURS[step["type"]][0](self, clients[step["type"]], step),
            self.on_task_state, cancel_timeout=self.cancel_timeout)
        self.create_timer(1.0 / tick_rate, self.on_tick)
        self.create_timer(1.0 / heartbeat_rate, self.on_heartbeat)
        self.get_logger().info(
            f"Fleet bridge for {self.robot} (fleet {self.fleet}, session {self.session_id}) "
            f"on mqtt://{host}:{port}/rmf/{self.fleet}/robot/{self.robot}/")

    def connect_mqtt(self, host, port, username, password):
        self.mqtt_connection = (host, port, username, password)
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=rmf.client_id(self.fleet, self.robot),
            clean_session=True)
        if username:
            self.client.username_pw_set(username, password)
        self.client.will_set(
            self.topic["status"], rmf.offline_payload(self.robot, self.session_id), qos=1)
        self.client.on_connect = self.on_connect
        self.client.on_message = lambda c, u, m: self.inbox.put((m.topic, m.payload))
        self.client.reconnect_delay_set(1, 10)
        self.client.connect_async(host, port, keepalive=20)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code != 0:
            self.get_logger().warning(f"MQTT connection refused: {reason_code}")
            return
        # Transport reconnects and adapter-requested registrations keep the boot ID.
        self.last_register = -REGISTER_RETRY_S
        client.subscribe([(self.topic["register_ack"], 1), (self.topic["command"], 1)])

    def on_odometry(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.pose_stamp is not None and stamp < self.pose_stamp:
            return
        pose = message.pose.pose
        values = (pose.position.x, pose.position.y, pose.orientation.x,
                  pose.orientation.y, pose.orientation.z, pose.orientation.w)
        if (
            message.header.frame_id != "map" or message.child_frame_id != "base_link"
            or not all(math.isfinite(value) for value in values)
            or not 0.98 <= sum(getattr(pose.orientation, axis) ** 2 for axis in "xyzw") <= 1.02
            or stamp <= 0 or not -0.5 <= now - stamp <= self.pose_timeout
        ):
            self.get_logger().warning("Ignoring invalid or stale global odometry")
            return
        self.pose = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        self.pose_stamp = stamp
        self.pose_received = time.monotonic()

    def on_correction(self, message):
        stamp = message.stamp.sec + message.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if message.frame_id == "map" and stamp > 0 and 0 <= now - stamp <= self.correction_timeout:
            self.correction_stamp = stamp
            self.correction_received = time.monotonic()

    def localization_ready(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        wall = time.monotonic()
        return (
            not self.session_reset_pending and not self.session_reset_error
            and self.pose is not None and self.pose_stamp is not None
            and self.pose_received is not None and self.correction_received is not None
            and self.correction_stamp is not None
            and 0 <= wall - self.pose_received <= self.pose_timeout
            and 0 <= wall - self.correction_received <= self.correction_timeout
            and 0 <= now - self.pose_stamp <= self.pose_timeout
            and 0 <= now - self.correction_stamp <= self.correction_timeout
            and self.pose_stamp >= self.correction_stamp
            and self.floor_future is None and not self.floor_error and not self.floor_prepared
        )

    def on_clock(self, message):
        current = message.clock.sec + message.clock.nanosec * 1e-9
        if self.last_clock is not None and current < self.last_clock - 0.1:
            self.reset_session("Simulation clock reset")
        self.last_clock = current

    def on_reset_session(self, request, response):
        self.reset_session("Explicit vehicle state reset")
        response.success = True
        response.message = f"New session {self.session_id}; waiting for cancellation and fresh localization"
        return response

    def reset_session(self, reason):
        self.registered = False
        self.session_id = str(uuid.uuid4())
        self.session_reset_pending = True
        self.session_reset_error = ""
        self.pose = self.pose_stamp = self.pose_received = None
        self.correction_stamp = self.correction_received = None
        if self.cancel_future is None or self.cancel_future.done():
            self.cancel_future = self.cancel_started = None
        self.cancel_goals = None
        self.nav_status.clear()
        self.runner.cancel()
        self.floor_results.clear()
        if self.floor_future is None:
            self.floor_error = False
        self.last_register = -REGISTER_RETRY_S
        # MQTT stores the will at CONNECT, so changing its session requires a new connection.
        self.client.disconnect()
        self.client.loop_stop()
        self.connect_mqtt(*self.mqtt_connection)
        self.get_logger().warning(f"{reason}; started session {self.session_id}")

    def on_nav_status(self, message):
        self.nav_status = {
            bytes(status.goal_info.goal_id.uuid): status.status for status in message.status_list
        }

    def poll_session_reset(self):
        if not self.session_reset_pending or self.session_reset_error:
            return
        if self.cancel_future is None:
            if not self.cancel_client.service_is_ready():
                return
            self.cancel_started = time.monotonic()
            # Zero UUID and stamp cancel all goals, including a previous bridge process's goals.
            self.cancel_future = self.cancel_client.call_async(CancelGoal.Request())
        future = self.cancel_future
        if future.done() and self.cancel_goals is None:
            error = future.exception()
            result = None if error is not None else future.result()
            if result is None or result.return_code != 0:
                self.session_reset_error = f"Cannot cancel old Nav2 goals: {error or result}"
            else:
                self.cancel_goals = {bytes(goal.goal_id.uuid) for goal in result.goals_canceling}
        if (
            self.cancel_goals is not None
            and all(self.nav_status.get(goal) in (4, 5, 6) for goal in self.cancel_goals)
            and not self.runner.active and self.floor_future is None
        ):
            self.session_reset_pending = False
            self.cmd_ids.clear()
            self.task_sessions.clear()
            self.get_logger().info(f"Old vehicle work stopped for session {self.session_id}")
        elif time.monotonic() - self.cancel_started >= self.cancel_timeout:
            self.session_reset_error = "Timed out confirming old vehicle work stopped"
        if self.session_reset_error:
            self.get_logger().error(self.session_reset_error + "; reset_session required to retry")

    def on_floor(self, message):
        self.level = message.data

    def on_floor_state(self, message):
        self.floor_prepared = message.data != "idle"

    def floor_result(self, cmd_id, status, message):
        payload = rmf.command_result_payload(
            self.robot, cmd_id, status, message, self.pose, level=self.level,
            session_id=self.floor_session_id)
        if self.floor_session_id == self.session_id:
            self.floor_results[cmd_id] = payload
            while len(self.floor_results) > 64:
                self.floor_results.popitem(last=False)
        self.publish("command_result", payload)

    def begin_floor_switch(self, cmd_id, command, action="change_floor"):
        self.floor_session_id = self.session_id
        if cmd_id in self.floor_results:
            self.publish("command_result", self.floor_results[cmd_id])
            return
        if self.floor_future is not None:
            if cmd_id != self.floor_cmd_id:
                self.floor_result(cmd_id, "failed", "Another floor switch is in progress")
            return
        if self.runner.active:
            self.floor_result(cmd_id, "failed", "Finish or stop the current task before changing floor")
            return
        client = self.prepare_client if action == "prepare_floor_change" else self.floor_client
        if not client.service_is_ready():
            self.floor_result(cmd_id, "failed", "Robot floor manager is unavailable")
            return
        from slam_localization_3d.srv import ChangeFloor, PrepareFloorChange
        manual = action == "change_floor" and command["initial_pose"] is None
        request = (
            PrepareFloorChange.Request() if action == "prepare_floor_change" else
            rmf.fill_change_floor_request(
                ChangeFloor.Request(), command, self.floor_base_z, self.get_clock().now().to_msg()))
        self.floor_deadline = self.floor_timeout + (
            self.manual_pose_timeout if manual else 0.0)
        self.floor_action = action
        if action == "prepare_floor_change":
            self.floor_prepared = True
        self.floor_cmd_id = cmd_id
        self.floor_started = time.monotonic()
        self.floor_response_overdue = False
        self.floor_future = client.call_async(request)
        self.floor_error = False
        self.get_logger().info(
            f"MQTT {action} {cmd_id}"
            + (" (waiting for RViz 2D Pose Estimate)" if manual else ""))

    def poll_floor_switch(self):
        future = self.floor_future
        if future is None:
            return
        if not future.done():
            if time.monotonic() - self.floor_started >= self.floor_deadline and not self.floor_response_overdue:
                self.floor_error = True
                self.floor_response_overdue = True
                # Keep tracking the request; a local timeout does not cancel
                # the robot's coordinated switch or prove that it stopped.
                self.get_logger().error("Floor switch response overdue; blocking new commands")
                self.floor_result(self.floor_cmd_id, "failed",
                                  "Floor switch response timed out; outcome unknown, commands blocked")
            return
        error = future.exception()
        result = None if error is not None else future.result()
        if result is not None and self.floor_session_id == self.session_id:
            self.level = result.current_floor
            if result.success and self.floor_action == "change_floor":
                pose = result.final_pose.pose.pose
                self.pose = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
                stamp = result.final_pose.header.stamp
                self.pose_stamp = stamp.sec + stamp.nanosec * 1e-9
        self.floor_error = self.floor_response_overdue or result is None or not result.success
        if not self.floor_response_overdue:
            self.floor_result(
                self.floor_cmd_id, "failed" if self.floor_error else "completed",
                str(error) if error is not None else (
                    "Floor manager returned no response" if result is None else result.message),
            )
        else:
            self.get_logger().warning("Late floor switch response received; retry with a new cmd_id")
        self.floor_future = None
        self.floor_cmd_id = None

    def on_battery(self, message):
        if math.isfinite(message.percentage):
            self.battery = message.percentage * 100.0

    def publish(self, name, payload, qos=1):
        self.client.publish(self.topic[name], payload, qos=qos)

    def maybe_register(self):
        if (self.registered or self.pose is None or not self.level
                or self.session_reset_pending or self.session_reset_error
                or self.floor_future is not None or self.floor_error
                or self.floor_prepared
                or not self.client.is_connected()):
            return
        now = time.monotonic()
        if now - self.last_register < REGISTER_RETRY_S:
            return
        self.last_register = now
        self.publish("register", rmf.register_payload(
            self.fleet, self.robot, self.pose, self.level, self.waypoint, self.specs,
            self.charger, self.parking, session_id=self.session_id))

    def on_task_state(self, state):
        self.get_logger().info(f"task state: {state}")
        if state["status"] == "canceling" and state.get("message"):
            self.get_logger().error(state["message"])
        cmd_id = self.cmd_ids.get(state.get("goal_id"))
        session_id = self.task_sessions.get(state.get("goal_id"), self.session_id)
        self.client.publish(self.topic["task_state"], json.dumps(
            {"robot_id": self.robot, "cmd_id": cmd_id, "session_id": session_id, **state}),
            qos=1, retain=True)
        status = rmf.RESULT_STATUS.get(state["status"])
        if status is None or cmd_id is None:
            return
        self.publish("command_result", rmf.command_result_payload(
            self.robot, cmd_id, status, state.get("message", ""), self.pose,
            session_id=session_id))

    def handle_command(self, payload):
        try:
            parsed = rmf.parse_command(payload, self.robot, self.session_id)
        except rmf.CommandError as error:
            self.get_logger().warning(f"command rejected: {error}")
            cmd_id = rmf.cmd_id_of(payload)
            if cmd_id is not None:
                self.publish("command_result", rmf.command_result_payload(
                    self.robot, cmd_id, "failed", str(error), session_id=self.session_id))
            return
        if parsed is None:
            return
        cmd_id, action, steps = parsed
        if self.session_reset_pending or self.session_reset_error:
            self.get_logger().warning("ignoring command while old vehicle work is stopping")
            return
        if not self.registered and action not in rmf.FLOOR_ACTIONS:
            self.get_logger().warning("ignoring command before register_ack")
            return
        if action in rmf.FLOOR_ACTIONS:
            self.begin_floor_switch(cmd_id, steps, action)
            return
        if self.floor_future is not None or self.floor_error or self.floor_prepared:
            self.publish("command_result", rmf.command_result_payload(
                self.robot, cmd_id, "failed",
                "Floor switch pending or failed; retry change_floor before sending other commands",
                session_id=self.session_id))
            return
        if action == "stop":
            session_id = self.session_id
            self.runner.cancel(on_done=lambda success, message: self.publish(
                "command_result", rmf.command_result_payload(
                    self.robot, cmd_id, "completed" if success else "failed", message, self.pose,
                    session_id=session_id)))
            return
        if action == "proceed":
            ok, message = self.runner.proceed(str(cmd_id))
            if not ok:
                self.get_logger().warning(f"proceed {cmd_id} ignored: {message}")
            return
        goal_id = str(cmd_id)
        self.cmd_ids[goal_id] = cmd_id
        self.task_sessions[goal_id] = self.session_id
        self.runner.submit(json.dumps({"goal_id": goal_id, "steps": steps}))

    def on_heartbeat(self):
        self.maybe_register()
        if self.pose is None:
            return
        active = self.runner.active
        cmd_id = self.cmd_ids.get(self.runner.task.goal_id) if active else None
        # Not idle (nor accepting commands) until registered.
        status = ("moving" if active else "idle") if self.registered else "error"
        if self.floor_future is not None:
            status, cmd_id = "moving", self.floor_cmd_id
        elif self.floor_prepared:
            status = "moving"
        if self.floor_error:
            status = "error"
        ready = self.localization_ready()
        if not ready and self.floor_future is None and not self.floor_prepared:
            status = "error"
        self.publish("heartbeat", rmf.heartbeat_payload(
            self.robot, self.pose, self.battery, status, cmd_id,
            level=self.level, session_id=self.session_id, localization_ready=ready), qos=0)

    def on_tick(self):
        self.poll_floor_switch()
        while True:
            try:
                topic, payload = self.inbox.get_nowait()
            except queue.Empty:
                break
            if topic == self.topic["register_ack"]:
                ack = rmf.parse_ack(payload, self.robot, self.session_id)
                if ack is None:
                    continue
                status, message = ack
                self.registered = status == "success"
                if self.registered:
                    self.get_logger().info(f"registered with Open-RMF: {message}")
                elif status == "require_register":
                    self.get_logger().info(f"adapter asked to re-register: {message}")
                    self.last_register = -REGISTER_RETRY_S
                    self.maybe_register()
                else:
                    self.get_logger().warning(f"registration rejected: {message}; retrying")
            else:
                self.handle_command(payload)
        self.runner.tick()
        self.poll_session_reset()

    def close(self):
        if self.runner.active:
            self.runner.cancel()
        if self.client.is_connected():
            info = self.client.publish(self.topic["deregister"], rmf.deregister_payload(
                self.fleet, self.robot, "Shutdown", self.session_id), qos=1)
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
