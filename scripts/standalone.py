#!/home/yu/isaacsim-6.1.0/python.sh

import argparse
import math
import sys
from pathlib import Path

from cmd_vel_control import receiver_for
from isaacsim import SimulationApp

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_localization_3d/launch"),
)
from robot_fleet import (  # noqa: E402
    DEFAULT_ROBOT_TYPE, RobotSpec, load_robot_profile, namespaced_topic, parse_robot_specs,
)


OFFICE_ASSET_PATH = "/Isaac/Environments/Office/office.usd"
SURROUNDING_BUILDINGS_PRIM_PATH = "/Root/SM_Buildings"
# Prim of the single robot when no --robot is given (root ROS namespace).
CARTER_PRIM_PATH = "/World/Carter"
GROUND_TRUTH_TOPIC = "/isaac/ground_truth/odom"
BOX_PRIM_ROOT = "/World/TestBoxes"
DEFAULT_BOX_SIZE = [0.6, 0.6, 1.0]
LINEAR_JOG_SPEED = 1.0
ANGULAR_JOG_SPEED = 0.75
FOLLOW_CAMERA_DISTANCE = 2.5
FOLLOW_CAMERA_HEIGHT = 1.5
FOLLOW_CAMERA_LOOK_AHEAD = 0.6
FOLLOW_CAMERA_TARGET_HEIGHT = 0.5
# Multi-robot overview: fixed top-down view over all spawn points.
OVERVIEW_CAMERA_MARGIN = 5.0
OVERVIEW_CAMERA_MIN_HEIGHT = 12.0
# The Office ceiling is at about 3 m; clip everything above this height.
OVERVIEW_CAMERA_CUT_HEIGHT = 2.6
KIT_EXTRA_ARGS = [
    "--/rtx/post/dlss/execMode=0",
    "--/app/renderer/skipGpuRenderProducts=false",
    "--/rtx/rendermode=RealTime",
]

parser = argparse.ArgumentParser(description="Launch Isaac Sim with the Office environment.")
parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim GUI.")
parser.add_argument("--auto-jog", action="store_true", help="Drive forward automatically for headless SLAM tests.")
parser.add_argument(
    "--validation-control-dir",
    help="Opt-in file-controlled physical obstacles for braking/navigation validation.",
)
parser.add_argument("--test", action="store_true", help="Load the stage and exit after ten frames.")
parser.add_argument(
    "--ros-cmd-vel", action="store_true",
    help="Subscribe to ROS 2 cmd_vel instead of keyboard or auto-jog.",
)
parser.add_argument(
    "--robot", action="append", default=[], metavar="NAME[:TYPE]@X,Y[,YAW]",
    help="Spawn a robot of TYPE (config/robots/TYPE.yaml, default "
    f"{DEFAULT_ROBOT_TYPE}) at world X,Y (m) and YAW (rad) whose ROS topics and TF live "
    "under /NAME. Repeat for more robots. Without --robot one robot uses root topics.",
)
parser.add_argument(
    "--lidar-motion-compensation",
    choices=("noncompensated", "compensated"),
    default="noncompensated",
    help="Select RTX LiDAR motion compensation; navigation uses compensated clouds.",
)
parser.add_argument(
    "--box", action="append", default=[], metavar="X,Y[,SX,SY,SZ]",
    help="Add a static collision box on the floor at world X,Y (m, same as the Office "
    "map frame). Default size 0.6,0.6,1.0 m. Repeat for more boxes.",
)
args, _ = parser.parse_known_args()
if args.ros_cmd_vel and (args.auto_jog or args.test):
    parser.error("--ros-cmd-vel cannot be combined with --auto-jog or --test")
if args.validation_control_dir and not args.ros_cmd_vel:
    parser.error("--validation-control-dir requires --ros-cmd-vel")
if args.validation_control_dir and args.robot:
    parser.error("--validation-control-dir supports only the single default robot")


class SimRobot:
    """One simulated robot: prim paths and ROS names from its robot-type profile."""

    def __init__(self, spec: RobotSpec):
        self.spec = spec
        self.name = spec.name
        self.profile = load_robot_profile(spec.robot_type)
        simulation = self.profile["simulation"]
        self.simulation = simulation
        self.prim_path = f"/World/{spec.name}" if spec.name else CARTER_PRIM_PATH
        self.articulation_path = f"{self.prim_path}/{simulation['articulation']}"
        self.lidar_path = f"{self.prim_path}/{simulation['lidar']}"
        self.imu_path = f"{self.prim_path}/{simulation['imu']}"
        self.forward_sign = float(simulation["forward_sign"])
        self.label = spec.name or "robot"
        self.robot = None
        self.controller = None
        self.receiver = None

    def topic(self, name: str) -> str:
        return namespaced_topic(self.name, name)

    def graph_path(self, base: str) -> str:
        return f"{base}_{self.name}" if self.name else base

    def drive(self, linear: float, angular: float) -> None:
        self.robot.apply_wheel_actions(
            self.controller.forward(command=[linear * self.forward_sign, angular])
        )


try:
    robots = [
        SimRobot(spec) for spec in (
            parse_robot_specs(args.robot) if args.robot
            else [RobotSpec("", DEFAULT_ROBOT_TYPE, 0.0, 0.0, 0.0)]
        )
    ]
except ValueError as error:
    parser.error(str(error))


def parse_box(text: str) -> list[float]:
    try:
        values = [float(value) for value in text.split(",")]
    except ValueError:
        values = []
    if len(values) == 2:
        values += DEFAULT_BOX_SIZE
    if len(values) != 5 or min(values[2:]) <= 0:
        parser.error(f"--box expects X,Y or X,Y,SX,SY,SZ with positive sizes, got {text!r}")
    return values


boxes = [parse_box(text) for text in args.box]
lidar_motion_compensation_state = args.lidar_motion_compensation.upper()

simulation_app = SimulationApp(
    {
        "headless": args.headless,
        "enable_motion_bvh": True,
        "extra_args": KIT_EXTRA_ARGS,
    }
)

import carb
import isaacsim.core.experimental.utils.app as app_utils
import omni
import omni.appwindow
import omni.graph.core as og
import usdrt
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.experimental.utils.stage import is_stage_loading
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
from isaacsim.sensors.experimental.physics import IMU
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
from isaacsim.storage.native import get_assets_root_path, is_file
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, Usd, UsdGeom, UsdPhysics

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()


pressed_keys = set()
input_interface = None
keyboard = None
keyboard_subscription = None
if args.ros_cmd_vel:
    for sim_robot in robots:
        sim_robot.receiver = receiver_for(sim_robot.name)


def on_keyboard_event(event, *_) -> bool:
    key = event.input.name
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        pressed_keys.add(key)
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        pressed_keys.discard(key)
    return True


def get_jog_command() -> list[float]:
    """Keyboard (linear, angular) in the robot's forward convention."""
    if "SPACE" in pressed_keys:
        return [0.0, 0.0]

    forward = int(bool(pressed_keys & {"W", "UP"}))
    backward = int(bool(pressed_keys & {"S", "DOWN"}))
    left = int(bool(pressed_keys & {"A", "LEFT"}))
    right = int(bool(pressed_keys & {"D", "RIGHT"}))
    return [
        (forward - backward) * LINEAR_JOG_SPEED,
        (left - right) * ANGULAR_JOG_SPEED,
    ]


def rotate_vector_by_quaternion(vector, quaternion) -> list[float]:
    vx, vy, vz = (float(value) for value in vector)
    qw, qx, qy, qz = (float(value) for value in quaternion)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return [
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    ]


def update_follow_camera(sim_robot: SimRobot, camera_path: str) -> None:
    positions, orientations = sim_robot.robot.get_world_poses()
    position = positions.numpy()[0]
    forward = rotate_vector_by_quaternion(
        [sim_robot.forward_sign, 0.0, 0.0],
        orientations.numpy()[0],
    )
    eye = [
        float(position[0] - forward[0] * FOLLOW_CAMERA_DISTANCE),
        float(position[1] - forward[1] * FOLLOW_CAMERA_DISTANCE),
        float(position[2] + FOLLOW_CAMERA_HEIGHT),
    ]
    target = [
        float(position[0] + forward[0] * FOLLOW_CAMERA_LOOK_AHEAD),
        float(position[1] + forward[1] * FOLLOW_CAMERA_LOOK_AHEAD),
        float(position[2] + FOLLOW_CAMERA_TARGET_HEIGHT),
    ]
    ViewportManager.set_camera_view(camera_path, eye=eye, target=target)


class OverviewCamera:
    """Fixed top-down view fitting all spawn points plus a margin.

    The near plane cuts away the Office ceiling. It is reset to the default once
    the user moves the camera, so zooming in does not clip the scene.
    """

    def __init__(self, robots: list[SimRobot], viewport) -> None:
        stage = omni.usd.get_context().get_stage()
        self.camera = UsdGeom.Camera(stage.GetPrimAtPath(str(viewport.camera_path)))
        self.session_layer = stage.GetSessionLayer()
        self.stage = stage
        self.default_range = self.camera.GetClippingRangeAttr().Get()
        self.eye = set_overview_camera(robots, viewport)

    def update(self) -> None:
        if self.eye is None:
            return
        position = self.camera.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        if (position - Gf.Vec3d(*self.eye)).GetLength() > 0.05:
            with Usd.EditContext(self.stage, self.session_layer):
                self.camera.GetClippingRangeAttr().Set(self.default_range)
            self.eye = None


def set_overview_camera(robots: list[SimRobot], viewport) -> list[float]:
    """Fixed top-down view fitting all spawn points plus a margin; returns the eye."""
    camera_path = str(viewport.camera_path)
    stage = omni.usd.get_context().get_stage()
    camera = UsdGeom.Camera(stage.GetPrimAtPath(camera_path))
    width, height_px = viewport.resolution
    tan_x = camera.GetHorizontalApertureAttr().Get() / (2.0 * camera.GetFocalLengthAttr().Get())
    tan_y = tan_x * height_px / width
    xs = [sim_robot.spec.x for sim_robot in robots]
    ys = [sim_robot.spec.y for sim_robot in robots]
    cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    height = max(
        OVERVIEW_CAMERA_MIN_HEIGHT,
        ((max(xs) - min(xs)) / 2.0 + OVERVIEW_CAMERA_MARGIN) / tan_x,
        ((max(ys) - min(ys)) / 2.0 + OVERVIEW_CAMERA_MARGIN) / tan_y,
    )
    # A tiny -Y offset avoids the degenerate straight-down look-at and keeps
    # world +Y up and +X right on screen, like a top-down RViz view.
    eye = [cx, cy - 1e-3 * height, height]
    ViewportManager.set_camera_view(camera_path, eye=eye, target=[cx, cy, 0.0])
    # Kit's viewport cameras live in the session layer, which overrides the root layer.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        camera.GetClippingRangeAttr().Set(
            Gf.Vec2f(height - OVERVIEW_CAMERA_CUT_HEIGHT, height + 100.0))
    return eye


def create_ros2_publishers(sim_robot: SimRobot, publish_clock: bool) -> None:
    graph_path = sim_robot.graph_path("/World/FASTLIO_ROS2")
    keys = og.Controller.Keys
    clock_nodes = [("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock")]
    clock_connections = [
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
    ]
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                *(clock_nodes if publish_clock else []),
                ("ComputeGroundTruth", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishGroundTruth", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                ("ReadIMU.outputs:sensorTime", "PublishIMU.inputs:timeStamp"),
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                *(clock_connections if publish_clock else []),
                ("OnPlaybackTick.outputs:tick", "ComputeGroundTruth.inputs:execIn"),
                ("ComputeGroundTruth.outputs:execOut", "PublishGroundTruth.inputs:execIn"),
                ("ComputeGroundTruth.outputs:position", "PublishGroundTruth.inputs:position"),
                ("ComputeGroundTruth.outputs:orientation", "PublishGroundTruth.inputs:orientation"),
                (
                    "ComputeGroundTruth.outputs:linearVelocity",
                    "PublishGroundTruth.inputs:linearVelocity",
                ),
                (
                    "ComputeGroundTruth.outputs:angularVelocity",
                    "PublishGroundTruth.inputs:angularVelocity",
                ),
                ("ReadSimTime.outputs:simulationTime", "PublishGroundTruth.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("PublishIMU.inputs:topicName", sim_robot.topic("/isaac/imu")),
                ("PublishIMU.inputs:frameId", "imu_link"),
                ("PublishJointState.inputs:topicName", sim_robot.topic("/isaac/joint_states")),
                (
                    "PublishJointState.inputs:targetPrim",
                    [usdrt.Sdf.Path(sim_robot.articulation_path)],
                ),
                *([("PublishClock.inputs:topicName", "/clock")] if publish_clock else []),
                # Simulator truth for covariance validation only; never fused.
                ("PublishGroundTruth.inputs:topicName", sim_robot.topic(GROUND_TRUTH_TOPIC)),
                ("PublishGroundTruth.inputs:odomFrameId", "isaac_world"),
                (
                    "PublishGroundTruth.inputs:chassisFrameId",
                    sim_robot.simulation["articulation"].rsplit("/", 1)[-1],
                ),
                ("PublishGroundTruth.inputs:robotFront", [sim_robot.forward_sign, 0.0, 0.0]),
            ],
        },
    )
    og.Controller.set(
        og.Controller.attribute(f"{graph_path}/ComputeGroundTruth.inputs:chassisPrim"),
        [usdrt.Sdf.Path(sim_robot.articulation_path)],
    )
    og.Controller.set(
        og.Controller.attribute(f"{graph_path}/ReadIMU.inputs:imuPrim"),
        [usdrt.Sdf.Path(sim_robot.imu_path)],
    )


def create_ros2_drive_subscriber(sim_robot: SimRobot) -> None:
    graph_path = sim_robot.graph_path("/World/CarterROS2Drive")
    keys = og.Controller.Keys
    _, nodes, _, _ = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("RecordCommand", "omni.graph.scriptnode.ScriptNode"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("SubscribeTwist.outputs:execOut", "RecordCommand.inputs:execIn"),
            ],
            keys.SET_VALUES: [
                ("SubscribeTwist.inputs:topicName", sim_robot.topic("/cmd_vel")),
                ("SubscribeTwist.inputs:queueSize", 1),
            ],
        },
    )
    script_node = nodes[2]
    for name in ("linearVelocity", "angularVelocity"):
        og.Controller.create_attribute(
            script_node, f"inputs:{name}",
            og.Type(og.BaseDataType.DOUBLE, 3, 0, og.AttributeRole.VECTOR),
            og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT,
        )
        og.Controller.connect(
            og.Controller.attribute(f"{graph_path}/SubscribeTwist.outputs:{name}"),
            script_node.get_attribute(f"inputs:{name}"),
        )
    script_node.get_attribute("inputs:script").set(
        "def compute(db):\n"
        "    from cmd_vel_control import receiver_for\n"
        "    try:\n"
        f"        receiver_for({sim_robot.name!r}).accept_twist(\n"
        "            db.inputs.linearVelocity, db.inputs.angularVelocity)\n"
        "    except ValueError as error:\n"
        "        db.log_error(str(error))\n"
    )


def spawn_robot(sim_robot: SimRobot, stage, assets_root_path: str) -> None:
    simulation = sim_robot.simulation
    usd_path = assets_root_path + simulation["asset"]
    if not is_file(usd_path):
        raise FileNotFoundError(f"{sim_robot.spec.robot_type} asset was not found: {usd_path}")
    yaw = sim_robot.spec.yaw
    sim_robot.robot = WheeledRobot(
        paths=sim_robot.prim_path,
        wheel_dof_names=list(simulation["wheel_joints"]),
        usd_path=usd_path,
        positions=[sim_robot.spec.x, sim_robot.spec.y, float(simulation["spawn_height"])],
        orientations=[math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)],
    )
    sim_robot.controller = DifferentialController(
        wheel_radius=float(simulation["wheel_radius"]),
        wheel_base=float(simulation["wheel_base"]),
    )

    lidar_prim = stage.GetPrimAtPath(sim_robot.lidar_path)
    if not lidar_prim.IsValid():
        raise RuntimeError(f"{sim_robot.label} LiDAR prim was not found: {sim_robot.lidar_path}")
    lidar = Lidar(
        sim_robot.lidar_path,
        accumulate_outputs=None,
        aux_output_level="BASIC",
        attributes={
            "omni:sensor:Core:outputMotionCompensationState":
                lidar_motion_compensation_state,
        },
    )
    motion_compensation_state = lidar.prims[0].GetAttribute(
        "omni:sensor:Core:outputMotionCompensationState"
    ).Get()
    if motion_compensation_state != lidar_motion_compensation_state:
        raise RuntimeError(
            "RTX LiDAR motion compensation state mismatch: "
            f"expected {lidar_motion_compensation_state}, got {motion_compensation_state}"
        )
    lidar_sensor = LidarSensor(lidar, annotators=[])
    lidar_sensor.attach_writer(
        "RtxLidarROS2PublishPointCloud",
        topicName=sim_robot.topic("/isaac/lidar_points"),
        frameId="lidar_link",
        outputIntensity=True,
        outputTimestamp=True,
        outputEmitterId=True,
        outputChannelId=True,
    )
    sim_robot.lidar_sensor = lidar_sensor

    IMU.create(
        sim_robot.imu_path,
        translations=[[0.0, 0.0, 0.0]],
        linear_acceleration_filter_size=3,
        angular_velocity_filter_size=3,
        orientation_filter_size=3,
    )
    print(
        f"Added {sim_robot.spec.robot_type} {sim_robot.prim_path} at "
        f"({sim_robot.spec.x:.2f}, {sim_robot.spec.y:.2f}, yaw {yaw:.2f}); "
        f"ROS topics {sim_robot.topic('/isaac/*')}, cmd_vel {sim_robot.topic('/cmd_vel')}"
    )


def verify_spawn_poses(sim_robots) -> None:
    """The ROS side derives each initial pose from the spec, so the spawn must match it."""
    for sim_robot in sim_robots:
        positions, orientations = sim_robot.robot.get_world_poses()
        x, y = (float(value) for value in positions.numpy()[0][:2])
        qw, qx, qy, qz = (float(value) for value in orientations.numpy()[0])
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        spec = sim_robot.spec
        yaw_error = math.atan2(math.sin(yaw - spec.yaw), math.cos(yaw - spec.yaw))
        print(f"{sim_robot.label} world pose after start: ({x:.3f}, {y:.3f}, yaw {yaw:.3f})")
        if math.hypot(x - spec.x, y - spec.y) > 0.15 or abs(yaw_error) > 0.1:
            raise RuntimeError(
                f"{sim_robot.label} spawned at ({x:.3f}, {y:.3f}, yaw {yaw:.3f}) instead of "
                f"({spec.x}, {spec.y}, yaw {spec.yaw})"
            )


lead = robots[0]
try:
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        raise RuntimeError(
            "Could not find the Isaac Sim assets root. Check the asset server or local asset configuration."
        )

    office_usd_path = assets_root_path + OFFICE_ASSET_PATH
    if not is_file(office_usd_path):
        raise FileNotFoundError(f"Office environment asset was not found: {office_usd_path}")

    omni.usd.get_context().open_stage(office_usd_path)

    simulation_app.update()
    simulation_app.update()
    while is_stage_loading():
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    buildings_prim = stage.GetPrimAtPath(SURROUNDING_BUILDINGS_PRIM_PATH)
    if not buildings_prim.IsValid():
        raise RuntimeError(
            f"Surrounding buildings prim was not found: {SURROUNDING_BUILDINGS_PRIM_PATH}"
        )
    UsdGeom.Imageable(buildings_prim).MakeInvisible()

    # Static (collision-only, no rigid body) boxes that Carter's LiDAR sees but cannot push.
    for index, (x, y, size_x, size_y, size_z) in enumerate(boxes):
        box_path = f"{BOX_PRIM_ROOT}/Box_{index}"
        Cube(
            box_path, sizes=1.0, colors="orange",
            positions=[x, y, size_z / 2.0], scales=[size_x, size_y, size_z],
        )
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(box_path))
        print(f"Spawned static box at ({x:.2f}, {y:.2f}) size {size_x}x{size_y}x{size_z} m")

    for index, sim_robot in enumerate(robots):
        spawn_robot(sim_robot, stage, assets_root_path)
        create_ros2_publishers(sim_robot, publish_clock=index == 0)
        if sim_robot.receiver is not None:
            create_ros2_drive_subscriber(sim_robot)
    command_receiver = lead.receiver

    validation_scene = None
    if args.validation_control_dir:
        from safety_validation_scene import SafetyValidationScene

        validation_scene = SafetyValidationScene(args.validation_control_dir, stage, lead.robot)

    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    physics_scenes = SimulationManager.get_physics_scenes()
    if not physics_scenes:
        raise RuntimeError("Isaac Sim did not create a physics scene")
    physics_scenes[0].set_enabled_gpu_dynamics(False)

    if not args.headless and command_receiver is None:
        app_window = omni.appwindow.get_default_app_window()
        input_interface = carb.input.acquire_input_interface()
        keyboard = app_window.get_keyboard()
        keyboard_subscription = input_interface.subscribe_to_keyboard_events(
            keyboard,
            on_keyboard_event,
        )

    print(f"Loaded Office environment: {office_usd_path}")
    print(f"Hidden surrounding buildings: {SURROUNDING_BUILDINGS_PRIM_PATH}")
    print(f"RTX LiDAR output motion compensation: {lidar_motion_compensation_state}")
    print(f"ROS 2 LiDAR: {lead.topic('/isaac/lidar_points')} [sensor_msgs/msg/PointCloud2]")
    print(f"ROS 2 IMU: {lead.topic('/isaac/imu')} [sensor_msgs/msg/Imu]")
    print(f"ROS 2 joint states: {lead.topic('/isaac/joint_states')} [sensor_msgs/msg/JointState]")
    print("ROS 2 simulation clock: /clock [rosgraph_msgs/msg/Clock]")
    print(f"ROS 2 simulator ground truth: {lead.topic(GROUND_TRUTH_TOPIC)} [nav_msgs/msg/Odometry]")
    if command_receiver is not None:
        print("Command source: native ROS 2 cmd_vel per robot (0.5 s watchdog; keyboard disabled)")
    if not args.headless and command_receiver is None:
        print("Jog controls: W/S or Up/Down = forward/backward, A/D or Left/Right = turn, Space = stop")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(10):
        simulation_app.update()
    verify_spawn_poses(robots)

    follow_camera_path = None
    overview_camera = None
    if not args.headless:
        active_viewport = get_active_viewport()
        if active_viewport is None:
            raise RuntimeError("Could not find the active viewport for the follow camera")
        if len(robots) > 1:
            overview_camera = OverviewCamera(robots, active_viewport)
            print("Main viewport: fixed top-down overview of all robots")
        else:
            follow_camera_path = str(active_viewport.camera_path)
            update_follow_camera(lead, follow_camera_path)
            print(f"Main viewport follows {lead.prim_path} from behind")

    if args.test:
        start_position = lead.robot.get_world_poses()[0].numpy()[0]
        for _ in range(60):
            lead.drive(0.2, 0.0)
            if follow_camera_path is not None:
                update_follow_camera(lead, follow_camera_path)
            if overview_camera is not None:
                overview_camera.update()
            simulation_app.update()
        lead.drive(0.0, 0.0)
        end_position = lead.robot.get_world_poses()[0].numpy()[0]
        displacement = end_position - start_position
        distance_moved = float((displacement**2).sum() ** 0.5)
        if distance_moved < 0.01:
            raise RuntimeError(f"Nova Carter jog test failed; moved only {distance_moved:.4f} m")
        forward = rotate_vector_by_quaternion(
            [lead.forward_sign, 0.0, 0.0], lead.robot.get_world_poses()[1].numpy()[0],
        )
        if float(displacement[0] * forward[0] + displacement[1] * forward[1]) <= 0.01:
            raise RuntimeError(
                f"Nova Carter forward jog moved in the wrong direction: displacement={displacement.tolist()}"
            )
        if not 0.15 <= distance_moved <= 0.25:
            raise RuntimeError(
                f"Nova Carter jog speed differs from the commanded 0.2 m/s: "
                f"moved {distance_moved:.3f} m in one simulated second"
            )
        print(
            f"Nova Carter jog test passed: displacement={displacement.tolist()}, "
            f"distance={distance_moved:.3f} m"
        )
    else:
        while simulation_app.is_running():
            if validation_scene is not None:
                validation_scene.update(SimulationManager.get_simulation_time())
            for sim_robot in robots:
                if sim_robot.receiver is not None:
                    try:
                        command = sim_robot.receiver.command()
                    except ValueError as error:
                        carb.log_error(f"Rejected {sim_robot.label} drive command: {error}")
                        command = (0.0, 0.0)
                elif args.auto_jog:
                    command = (0.2, 0.15)
                else:
                    # Keyboard drives the followed (first) robot only.
                    command = get_jog_command() if sim_robot is lead else (0.0, 0.0)
                sim_robot.drive(*command)
            if follow_camera_path is not None:
                update_follow_camera(lead, follow_camera_path)
            if overview_camera is not None:
                overview_camera.update()
            simulation_app.update()
finally:
    for sim_robot in robots:
        if sim_robot.receiver is not None and sim_robot.robot is not None:
            sim_robot.drive(0.0, 0.0)
    if input_interface is not None and keyboard_subscription is not None:
        input_interface.unsubscribe_to_keyboard_events(keyboard, keyboard_subscription)
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    simulation_app.close()