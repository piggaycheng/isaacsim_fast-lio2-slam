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
from slam_localization_3d.srv import ChangeFloor, LoadLocalizationMap, PrepareFloorChange
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
        self.manual_pose_timeout = float(
            self.declare_parameter("manual_pose_timeout", 300.0).value)
        if not all(math.isfinite(value) and value > 0
                   for value in (self.timeout, self.service_timeout, self.manual_pose_timeout)):
            raise ValueError(
                "switch_timeout, service_timeout and manual_pose_timeout must be finite and positive")
        # base_link height in every floor map; RViz 2D Pose Estimate always sends z = 0.
        self.floor_base_z = float(self.declare_parameter("floor_base_z", -0.526).value)
        if not math.isfinite(self.floor_base_z):
            raise ValueError("floor_base_z must be finite")
        self.manual_pose = None
        self.busy = False
        self.transition_state = "idle"
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
        self.state_publisher = self.create_publisher(
            String, "navigation/floor_transition_state",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.set_transition_state("idle")
        self.create_subscription(
            PoseWithCovarianceStamped, "localization_3d/global_pose", self.on_pose, 10
        )
        self.create_subscription(Odometry, "odometry/global", self.on_global_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.on_initial_pose, 10)
        self.create_subscription(Odometry, "odometry/local", self.on_local_odom, 10)
        self.service_clients = {
            "safety": self.create_client(SetBool, "navigation/set_map_switch_hold"),
            "adapter": self.create_client(SetBool, "localization_3d/set_map_switch_hold"),
            "tf": self.create_client(SetBool, "global_tf_gate/set_map_switch_hold"),
            "icp": self.create_client(SetBool, "localization_3d/set_registration_hold"),
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
        self.create_service(
            PrepareFloorChange, "navigation/prepare_floor_change", self.prepare_floor_change,
            callback_group=ReentrantCallbackGroup(),
        )
        self.create_timer(0.2, self.observe_navigation, clock=self.steady_clock)

    def observe_navigation(self):
        self.navigation_seen |= self.service_clients["nav"].service_is_ready()

    def publish_floor(self):
        self.floor_publisher.publish(String(data=self.current_floor))

    def set_transition_state(self, state):
        self.transition_state = state
        self.state_publisher.publish(String(data=state))

    def on_pose(self, message):
        if message.header.frame_id == "map" and valid_pose(message.pose.pose):
            self.pose = message

    def on_initial_pose(self, message):
        # (arrival time, pose); only poses sent after the target PGM is shown are used.
        self.manual_pose = (time.monotonic(), message)

    def manual_initial_pose(self, after):
        if self.manual_pose is None or self.manual_pose[0] <= after:
            return None
        pose = copy.deepcopy(self.manual_pose[1])
        self.manual_pose = None
        pose.header.stamp = self.get_clock().now().to_msg()
        position = pose.pose.pose.position
        if position.z == 0.0:
            position.z = self.floor_base_z
        try:
            validate_initial_pose(pose)
        except ValueError as error:
            self.get_logger().warning(f"Ignoring invalid RViz initial pose: {error}")
            return None
        return pose

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

    async def stop_for_floor_change(self, deadline):
        await self.hold("safety", True, deadline)
        await self.hold("adapter", True, deadline)
        await self.hold("tf", True, deadline)
        await self.hold("icp", True, deadline)
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
                            raise FloorSwitchError(f"{name} cancellation failed: {result.return_code}")
                await self.pause_navigation(deadline)
        await self.wait_until(
            lambda: self.stopped(stopped_after), deadline, "fresh stationary odometry")
        return navigation_ready

    async def hold_after_failure(self, response):
        self.set_transition_state("failed")
        # An adapter may already have been released for target-map confirmation.
        stopped = False
        for name in ("safety", "adapter", "tf", "icp"):
            try:
                await self.hold(name, True, time.monotonic() + self.service_timeout)
                if name == "safety":
                    stopped = True
            except (FloorSwitchError, RuntimeError) as error:
                response.message += f"; could not hold {name}: {error}"
                self.get_logger().error(response.message)
        try:
            await self.pause_navigation(time.monotonic() + self.service_timeout * 2)
            response.message += "; robot held stopped, Nav2 paused" if stopped else "; Nav2 paused"
        except (FloorSwitchError, RuntimeError) as error:
            response.message += f"; could not confirm Nav2 pause: {error}"
            self.get_logger().error(response.message)

    async def prepare_floor_change(self, request, response):
        response.current_floor = self.current_floor
        if self.busy or self.filter_editor_active:
            response.message = (
                "A floor switch is already in progress" if self.busy else
                "Floor switching requires restarting without the costmap filter editor")
            self.get_logger().warning(response.message)
            return response
        self.busy = True
        self.set_transition_state("preparing")
        try:
            await self.stop_for_floor_change(time.monotonic() + self.timeout)
            self.current_floor = ""
            self.publish_floor()
            self.set_transition_state("prepared")
            response.success = True
            response.message = (
                "Prepared for elevator travel: robot stopped, Nav2 and global ICP held; "
                "use change_floor after arrival to relocalize and resume")
            self.get_logger().info(response.message)
        except (FloorSwitchError, RuntimeError) as error:
            response.message = f"Floor preparation failed: {error}"
            self.get_logger().error(response.message)
            await self.hold_after_failure(response)
        finally:
            response.current_floor = self.current_floor
            self.busy = False
        return response

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

    def localization_details(self, after, target):
        expected = target.pose.pose
        details = (
            f"target x={expected.position.x:.3f}, y={expected.position.y:.3f}, "
            f"yaw={yaw_of(expected.orientation):.3f} rad"
        )
        if self.pose is None:
            return details + "; no accepted global pose received since PCD load"
        actual = self.pose.pose.pose
        distance = math.hypot(
            actual.position.x - expected.position.x, actual.position.y - expected.position.y)
        turn = abs(math.remainder(
            yaw_of(actual.orientation) - yaw_of(expected.orientation), math.tau))
        stamp = seconds(self.pose.header.stamp)
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        return (
            details + f"; latest pose x={actual.position.x:.3f}, y={actual.position.y:.3f}, "
            f"yaw={yaw_of(actual.orientation):.3f} rad; "
            f"distance={distance:.3f} m (required <1.0), "
            f"yaw_error={turn:.3f} rad (required <0.5); "
            f"age={age:.3f} s (required >=0 and <1.0), newer_than_load={stamp > after}"
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
            if not request.manual_initial_pose:
                validate_initial_pose(request.initial_pose)
            map_yaml, pcd = floor_maps(self.maps_root, request.floor_name)
        except (ValueError, OSError, yaml.YAMLError) as error:
            response.message = str(error)
            self.get_logger().error(f"Floor switch rejected: {error}")
            return response
        self.busy = True
        self.set_transition_state("switching")
        deadline = time.monotonic() + self.timeout
        try:
            navigation_ready = await self.stop_for_floor_change(deadline)
            # From here a failure may leave a partially changed map pair.
            self.current_floor = ""
            self.publish_floor()
            # RViz poses clicked once the target map starts loading belong to that floor.
            map_requested = time.monotonic()
            result = await self.call("map", LoadMap.Request(map_url=map_yaml), deadline)
            if result.result != LoadMap.Response.RESULT_SUCCESS:
                raise FloorSwitchError(f"Nav2 map load failed: {result.result}")
            initial_pose = request.initial_pose
            if request.manual_initial_pose:
                shown = time.monotonic()
                self.get_logger().info(
                    f"Floor {request.floor_name} map shown; waiting up to "
                    f"{self.manual_pose_timeout:g} s for an RViz 2D Pose Estimate")
                manual = []

                def received():
                    pose = self.manual_initial_pose(map_requested)
                    if pose is not None:
                        manual.append(pose)
                    return bool(manual)

                await self.wait_until(
                    received, shown + self.manual_pose_timeout, "RViz 2D Pose Estimate")
                initial_pose = manual[0]
                deadline = time.monotonic() + self.timeout
            target_pose = initial_pose.pose.pose
            self.get_logger().info(
                f"Floor {request.floor_name} localization target: "
                f"x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, "
                f"z={target_pose.position.z:.3f}, yaw={yaw_of(target_pose.orientation):.3f} rad")
            await self.call("pcd", LoadLocalizationMap.Request(
                pcd_path=pcd, initial_pose=initial_pose,
            ), deadline)
            self.pose = None
            await self.hold("adapter", False, deadline)
            localization_after = self.get_clock().now().nanoseconds * 1e-9
            await self.hold("icp", False, deadline)
            try:
                await self.wait_until(
                    lambda: self.localized(localization_after, initial_pose),
                    deadline, "accepted target-floor ICP pose near initial_pose",
                )
            except FloorSwitchError as error:
                raise FloorSwitchError(
                    f"{error}; {self.localization_details(localization_after, initial_pose)}"
                ) from error
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
                    localization_after, initial_pose),
                deadline, "fresh localization after costmap restart",
            )
            # No pre-switch goal survives PAUSE; only new commands may now pass.
            await self.hold("safety", False, deadline)
            self.current_floor = request.floor_name
            self.publish_floor()
            self.set_transition_state("idle")
            response.success = True
            response.final_pose.header = self.global_odom.header
            response.final_pose.pose = self.global_odom.pose
            response.message = f"Floor {self.current_floor} maps, ICP and navigation ready"
            self.get_logger().info(response.message)
        except (FloorSwitchError, RuntimeError) as error:
            response.message = f"Floor switch failed: {error}"
            self.get_logger().error(response.message)
            await self.hold_after_failure(response)
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
