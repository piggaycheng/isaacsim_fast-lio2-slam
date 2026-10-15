#!/usr/bin/env python3
"""Coordinate floor changes without exposing mixed maps to a moving robot."""

import copy
import math
from pathlib import Path
import time

import rclpy
import yaml
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ClearEntireCostmap, LoadMap, ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from rclpy.time import Time
from robot_localization.srv import SetPose
from slam_localization_3d.srv import ChangeFloor, LoadLocalizationMap
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from floor_maps import floor_maps, validate_initial_pose
from global_tf_gate import yaw_of
from localization_3d_pose import seconds, valid_pose
from wait_for_navigation import NAVIGATION_NODES


class FloorSwitchError(RuntimeError):
    pass


class FloorManager(Node):
    def __init__(self, **kwargs):
        super().__init__("floor_manager", **kwargs)
        self.maps_root = self.declare_parameter("maps_root", "").value
        if not Path(self.maps_root).is_dir():
            raise ValueError("maps_root must be an existing directory")
        self.current_floor = self.declare_parameter("initial_floor", "1F").value
        self.timeout = float(self.declare_parameter("switch_timeout", 60.0).value)
        self.service_timeout = float(self.declare_parameter("service_timeout", 10.0).value)
        if not all(math.isfinite(value) and value > 0
                   for value in (self.timeout, self.service_timeout)):
            raise ValueError("switch_timeout and service_timeout must be finite and positive")
        self.busy = False
        self.filter_editor_active = self.declare_parameter("filter_editor_active", False).value
        self.pose = None
        self.global_odom = None
        self.local_odom = None
        self.navigation_seen = False
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.floor_publisher = self.create_publisher(
            String, "navigation/current_floor",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.publish_floor()
        self.create_subscription(
            PoseWithCovarianceStamped, "localization_3d/global_pose", self.on_pose, 10
        )
        self.create_subscription(Odometry, "odometry/global", self.on_global_odom, 10)
        self.create_subscription(Odometry, "odometry/local", self.on_local_odom, 10)
        self.service_clients = {
            "safety": self.create_client(SetBool, "navigation/set_map_switch_hold"),
            "adapter": self.create_client(SetBool, "localization_3d/set_map_switch_hold"),
            "tf": self.create_client(SetBool, "global_tf_gate/set_map_switch_hold"),
            "nav": self.create_client(
                ManageLifecycleNodes, "lifecycle_manager_navigation/manage_nodes"
            ),
            "map": self.create_client(LoadMap, "map_server/load_map"),
            "pcd": self.create_client(LoadLocalizationMap, "localization_3d/load_map"),
            "ekf": self.create_client(SetPose, "global_ekf/set_pose"),
            "nav_state": self.create_client(GetState, "bt_navigator/get_state"),
            **{f"state_{name}": self.create_client(GetState, f"{name}/get_state")
               for name in NAVIGATION_NODES},
            **{name: self.create_client(
                CancelGoal, f"{name}/_action/cancel_goal"
            ) for name in ("navigate_to_pose", "navigate_through_poses")},
            **{name: self.create_client(
                ClearEntireCostmap, f"{name}/clear_entirely_{name}"
            ) for name in ("global_costmap", "local_costmap")},
        }
        # The coroutine awaits default-group clients/timers; keep its service
        # in another group so a single-threaded executor can progress them.
        self.create_service(
            ChangeFloor, "navigation/change_floor", self.change_floor,
            callback_group=ReentrantCallbackGroup(),
        )
        self.create_timer(0.2, self.observe_navigation, clock=self.steady_clock)

    def observe_navigation(self):
        self.navigation_seen |= self.service_clients["nav"].service_is_ready()

    def publish_floor(self):
        self.floor_publisher.publish(String(data=self.current_floor))

    def on_pose(self, message):
        if message.header.frame_id == "map" and valid_pose(message.pose.pose):
            self.pose = message

    def on_global_odom(self, message):
        self.global_odom = message

    def on_local_odom(self, message):
        self.local_odom = message

    async def wait_until(self, predicate, deadline, description):
        future = Future()

        def check():
            if future.done():
                return
            if predicate():
                future.set_result(True)
            elif time.monotonic() >= deadline:
                future.set_exception(FloorSwitchError(f"Timed out: {description}"))

        timer = self.create_timer(0.05, check, clock=self.steady_clock)
        try:
            check()
            await future
        finally:
            self.destroy_timer(timer)

    async def call(self, name, request, deadline):
        client = self.service_clients[name]
        operation_deadline = min(deadline, time.monotonic() + self.service_timeout)
        await self.wait_until(client.service_is_ready, operation_deadline, f"{name} service")
        future = client.call_async(request)
        try:
            await self.wait_until(future.done, operation_deadline, f"{name} response")
        finally:
            if not future.done():
                client.remove_pending_request(future)
        result = future.result()
        if result is None:
            raise FloorSwitchError(f"{name} returned no response")
        if hasattr(result, "success") and not result.success:
            raise FloorSwitchError(f"{name} failed: {getattr(result, 'message', '')}")
        return result

    async def hold(self, name, enabled, deadline):
        await self.call(name, SetBool.Request(data=enabled), deadline)

    async def navigation(self, command, deadline):
        await self.call("nav", ManageLifecycleNodes.Request(command=command), deadline)

    async def pause_navigation(self, deadline):
        state = await self.call("nav_state", GetState.Request(), deadline)
        if state.current_state.id == State.PRIMARY_STATE_ACTIVE:
            await self.navigation(ManageLifecycleNodes.Request.PAUSE, deadline)
        elif state.current_state.id != State.PRIMARY_STATE_INACTIVE:
            raise FloorSwitchError("Nav2 is neither active nor paused")

    def stopped(self, after):
        message = self.local_odom
        if (
            message is None or message.header.frame_id != "odom"
            or message.child_frame_id != "base_link"
            or seconds(message.header.stamp) <= after
        ):
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        twist = message.twist.twist
        return (
            0 <= now - seconds(message.header.stamp) < 1.0
            and all(math.isfinite(value) and abs(value) < 0.03 for value in (
                twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z,
            ))
        )

    def localized(self, after, target):
        if self.pose is None:
            return False
        stamp = seconds(self.pose.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        if stamp <= after or not 0 <= now - stamp < 1.0:
            return False
        first, second = self.pose.pose.pose, target.pose.pose
        turn = yaw_of(first.orientation) - yaw_of(second.orientation)
        return (
            math.hypot(first.position.x - second.position.x,
                       first.position.y - second.position.y) < 1.0
            and abs(math.atan2(math.sin(turn), math.cos(turn))) < 0.5
        )

    def tf_ready(self, after):
        message = self.global_odom
        now = self.get_clock().now().nanoseconds * 1e-9
        if (
            message is None or message.header.frame_id != "map"
            or message.child_frame_id != "base_link"
            or seconds(message.header.stamp) <= after
            or not 0 <= now - seconds(message.header.stamp) < 1.0
            or not valid_pose(message.pose.pose)
            or self.pose is None
            or math.hypot(
                message.pose.pose.position.x - self.pose.pose.pose.position.x,
                message.pose.pose.position.y - self.pose.pose.pose.position.y,
            ) >= 0.5
            or abs(math.remainder(
                yaw_of(message.pose.pose.orientation) - yaw_of(self.pose.pose.pose.orientation),
                math.tau,
            )) >= 0.2
        ):
            return False
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            return False
        return seconds(transform.header.stamp) > after and (
            0 <= now - seconds(transform.header.stamp) < 1.0
            and math.hypot(
                transform.transform.translation.x - message.pose.pose.position.x,
                transform.transform.translation.y - message.pose.pose.position.y,
            ) < 0.2
            and abs(math.remainder(
                yaw_of(transform.transform.rotation) - yaw_of(message.pose.pose.orientation),
                math.tau,
            )) < 0.2
        )

    async def change_floor(self, request, response):
        response.current_floor = self.current_floor
        if self.busy:
            response.message = "A floor switch is already in progress"
            self.get_logger().warning(response.message)
            return response
        if self.filter_editor_active:
            response.message = "Floor switching requires restarting without the costmap filter editor"
            self.get_logger().error(response.message)
            return response
        try:
            validate_initial_pose(request.initial_pose)
            map_yaml, pcd = floor_maps(self.maps_root, request.floor_name)
        except (ValueError, OSError, yaml.YAMLError) as error:
            response.message = str(error)
            self.get_logger().error(f"Floor switch rejected: {error}")
            return response
        self.busy = True
        deadline = time.monotonic() + self.timeout
        try:
            await self.hold("safety", True, deadline)
            stopped_after = self.get_clock().now().nanoseconds * 1e-9
            navigation_ready = self.service_clients["nav"].service_is_ready()
            if not navigation_ready and self.navigation_seen:
                raise FloorSwitchError("Previously available navigation manager disappeared")
            if navigation_ready:
                state = await self.call("nav_state", GetState.Request(), deadline)
                if state.current_state.id == State.PRIMARY_STATE_UNCONFIGURED:
                    navigation_ready = False
                else:
                    if state.current_state.id == State.PRIMARY_STATE_ACTIVE:
                        for name in ("navigate_to_pose", "navigate_through_poses"):
                            result = await self.call(name, CancelGoal.Request(), deadline)
                            if result.return_code != CancelGoal.Response.ERROR_NONE:
                                raise FloorSwitchError(
                                    f"{name} cancellation failed: {result.return_code}")
                    await self.pause_navigation(deadline)
            await self.wait_until(
                lambda: self.stopped(stopped_after), deadline, "fresh stationary odometry"
            )
            await self.hold("adapter", True, deadline)
            await self.hold("tf", True, deadline)
            # From here a failure may leave a partially changed map pair.
            self.current_floor = ""
            self.publish_floor()
            result = await self.call("map", LoadMap.Request(map_url=map_yaml), deadline)
            if result.result != LoadMap.Response.RESULT_SUCCESS:
                raise FloorSwitchError(f"Nav2 map load failed: {result.result}")
            await self.call("pcd", LoadLocalizationMap.Request(
                pcd_path=pcd, initial_pose=request.initial_pose,
            ), deadline)
            self.pose = None
            await self.hold("adapter", False, deadline)
            localization_after = self.get_clock().now().nanoseconds * 1e-9
            await self.wait_until(
                lambda: self.localized(localization_after, request.initial_pose),
                deadline, "accepted target-floor ICP pose near initial_pose",
            )
            reset = copy.deepcopy(self.pose)
            reset.header.stamp = self.get_clock().now().to_msg()
            await self.call("ekf", SetPose.Request(pose=reset), deadline)
            self.tf_buffer.clear()
            tf_after = self.get_clock().now().nanoseconds * 1e-9
            await self.hold("tf", False, deadline)
            await self.wait_until(
                lambda: self.tf_ready(tf_after), deadline, "fresh target-floor global TF"
            )
            if navigation_ready:
                await self.navigation(ManageLifecycleNodes.Request.RESUME, deadline)
            else:
                # The first localization lets the launch readiness gate start
                # Nav2. This also permits MQTT initialization before register.
                while True:
                    states = [
                        (await self.call(f"state_{name}", GetState.Request(), deadline)).current_state.id
                        for name in NAVIGATION_NODES
                    ]
                    if all(state == State.PRIMARY_STATE_ACTIVE for state in states):
                        break
                    next_check = time.monotonic() + 0.1
                    await self.wait_until(
                        lambda: time.monotonic() >= next_check, deadline, "Nav2 startup"
                    )
            for name in ("global_costmap", "local_costmap"):
                await self.call(name, ClearEntireCostmap.Request(), deadline)
            await self.wait_until(
                lambda: self.tf_ready(tf_after) and self.localized(
                    localization_after, request.initial_pose),
                deadline, "fresh localization after costmap restart",
            )
            # No pre-switch goal survives PAUSE; only new commands may now pass.
            await self.hold("safety", False, deadline)
            self.current_floor = request.floor_name
            self.publish_floor()
            response.success = True
            response.final_pose.header = self.global_odom.header
            response.final_pose.pose = self.global_odom.pose
            response.message = f"Floor {self.current_floor} maps, ICP and navigation ready"
            self.get_logger().info(response.message)
        except (FloorSwitchError, RuntimeError) as error:
            response.message = f"Floor switch failed: {error}"
            self.get_logger().error(response.message)
            try:
                recovery_deadline = time.monotonic() + self.service_timeout * 2
                await self.hold("safety", True, recovery_deadline)
                await self.pause_navigation(recovery_deadline)
                response.message += "; robot held stopped, Nav2 paused"
            except (FloorSwitchError, RuntimeError) as recovery_error:
                response.message += f"; could not confirm stop/pause: {recovery_error}"
                self.get_logger().error(response.message)
        finally:
            response.current_floor = self.current_floor
            self.busy = False
        return response


def main():
    rclpy.init()
    node = FloorManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
