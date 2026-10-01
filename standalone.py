#!/home/yu/isaacsim-6.1.0/python.sh

import argparse

from cmd_vel_control import receiver
from isaacsim import SimulationApp


OFFICE_ASSET_PATH = "/Isaac/Environments/Office/office.usd"
CARTER_ASSET_PATH = "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
SURROUNDING_BUILDINGS_PRIM_PATH = "/Root/SM_Buildings"
CARTER_PRIM_PATH = "/World/Carter"
CARTER_ARTICULATION_PATH = f"{CARTER_PRIM_PATH}/chassis_link"
CARTER_LIDAR_PRIM_PATH = f"{CARTER_PRIM_PATH}/chassis_link/sensors/XT_32/PandarXT_32_10hz"
CARTER_IMU_PRIM_PATH = f"{CARTER_LIDAR_PRIM_PATH}/fastlio_imu"
CARTER_SPAWN_POSITION = [0.0, 0.0, 0.05]
GROUND_TRUTH_TOPIC = "/isaac/ground_truth/odom"
BOX_PRIM_ROOT = "/World/TestBoxes"
DEFAULT_BOX_SIZE = [0.6, 0.6, 1.0]
LINEAR_JOG_SPEED = 0.75
ANGULAR_JOG_SPEED = 1.2
CARTER_FORWARD_SIGN = -1.0
FOLLOW_CAMERA_DISTANCE = 2.5
FOLLOW_CAMERA_HEIGHT = 1.5
FOLLOW_CAMERA_LOOK_AHEAD = 0.6
FOLLOW_CAMERA_TARGET_HEIGHT = 0.5
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
    help="Opt-in file-controlled physical obstacles for tests/validate_braking.py.",
)
parser.add_argument("--test", action="store_true", help="Load the stage and exit after ten frames.")
parser.add_argument(
    "--ros-cmd-vel", action="store_true",
    help="Subscribe to ROS 2 /cmd_vel instead of keyboard or auto-jog.",
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
from pxr import UsdGeom, UsdPhysics

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()


pressed_keys = set()
input_interface = None
keyboard = None
keyboard_subscription = None
command_receiver = receiver if args.ros_cmd_vel else None


def on_keyboard_event(event, *_) -> bool:
    key = event.input.name
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        pressed_keys.add(key)
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        pressed_keys.discard(key)
    return True


def get_jog_command() -> list[float]:
    if "SPACE" in pressed_keys:
        return [0.0, 0.0]

    forward = int(bool(pressed_keys & {"W", "UP"}))
    backward = int(bool(pressed_keys & {"S", "DOWN"}))
    left = int(bool(pressed_keys & {"A", "LEFT"}))
    right = int(bool(pressed_keys & {"D", "RIGHT"}))
    return [
        (forward - backward) * LINEAR_JOG_SPEED * CARTER_FORWARD_SIGN,
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


def update_follow_camera(carter, camera_path: str) -> None:
    positions, orientations = carter.get_world_poses()
    position = positions.numpy()[0]
    forward = rotate_vector_by_quaternion(
        [CARTER_FORWARD_SIGN, 0.0, 0.0],
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


def create_ros2_publishers() -> None:
    graph_path = "/World/FASTLIO_ROS2"
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
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
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
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
                ("PublishIMU.inputs:topicName", "/isaac/imu"),
                ("PublishIMU.inputs:frameId", "imu_link"),
                ("PublishJointState.inputs:topicName", "/isaac/joint_states"),
                (
                    "PublishJointState.inputs:targetPrim",
                    [usdrt.Sdf.Path(CARTER_ARTICULATION_PATH)],
                ),
                ("PublishClock.inputs:topicName", "/clock"),
                # Simulator truth for covariance validation only; never fused.
                ("PublishGroundTruth.inputs:topicName", GROUND_TRUTH_TOPIC),
                ("PublishGroundTruth.inputs:odomFrameId", "isaac_world"),
                ("PublishGroundTruth.inputs:chassisFrameId", "chassis_link"),
                ("PublishGroundTruth.inputs:robotFront", [CARTER_FORWARD_SIGN, 0.0, 0.0]),
            ],
        },
    )
    og.Controller.set(
        og.Controller.attribute(f"{graph_path}/ComputeGroundTruth.inputs:chassisPrim"),
        [usdrt.Sdf.Path(CARTER_ARTICULATION_PATH)],
    )
    og.Controller.set(
        og.Controller.attribute(f"{graph_path}/ReadIMU.inputs:imuPrim"),
        [usdrt.Sdf.Path(CARTER_IMU_PRIM_PATH)],
    )


def create_ros2_drive_subscriber() -> None:
    graph_path = "/World/CarterROS2Drive"
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
                ("SubscribeTwist.inputs:topicName", "/cmd_vel"),
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
        "    from cmd_vel_control import receiver\n"
        "    try:\n"
        "        receiver.accept_twist(db.inputs.linearVelocity, db.inputs.angularVelocity)\n"
        "    except ValueError as error:\n"
        "        db.log_error(str(error))\n"
    )


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

    carter_usd_path = assets_root_path + CARTER_ASSET_PATH
    if not is_file(carter_usd_path):
        raise FileNotFoundError(f"Nova Carter asset was not found: {carter_usd_path}")

    carter = WheeledRobot(
        paths=CARTER_PRIM_PATH,
        wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
        usd_path=carter_usd_path,
        positions=CARTER_SPAWN_POSITION,
    )
    controller = DifferentialController(wheel_radius=0.14, wheel_base=0.4132)

    # Static (collision-only, no rigid body) boxes that Carter's LiDAR sees but cannot push.
    for index, (x, y, size_x, size_y, size_z) in enumerate(boxes):
        box_path = f"{BOX_PRIM_ROOT}/Box_{index}"
        Cube(
            box_path, sizes=1.0, colors="orange",
            positions=[x, y, size_z / 2.0], scales=[size_x, size_y, size_z],
        )
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(box_path))
        print(f"Spawned static box at ({x:.2f}, {y:.2f}) size {size_x}x{size_y}x{size_z} m")

    lidar_prim = stage.GetPrimAtPath(CARTER_LIDAR_PRIM_PATH)
    if not lidar_prim.IsValid():
        raise RuntimeError(f"Nova Carter LiDAR prim was not found: {CARTER_LIDAR_PRIM_PATH}")
    lidar = Lidar(
        CARTER_LIDAR_PRIM_PATH,
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
        topicName="/isaac/lidar_points",
        frameId="lidar_link",
        outputIntensity=True,
        outputTimestamp=True,
        outputEmitterId=True,
        outputChannelId=True,
    )

    IMU.create(
        CARTER_IMU_PRIM_PATH,
        translations=[[0.0, 0.0, 0.0]],
        linear_acceleration_filter_size=3,
        angular_velocity_filter_size=3,
        orientation_filter_size=3,
    )
    create_ros2_publishers()
    if command_receiver is not None:
        create_ros2_drive_subscriber()

    validation_scene = None
    if args.validation_control_dir:
        from safety_validation_scene import SafetyValidationScene

        validation_scene = SafetyValidationScene(args.validation_control_dir, stage, carter)

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
    print(f"Added Nova Carter: {CARTER_PRIM_PATH}")
    print(f"RTX LiDAR output motion compensation: {motion_compensation_state}")
    print("ROS 2 LiDAR: /isaac/lidar_points [sensor_msgs/msg/PointCloud2]")
    print("ROS 2 IMU: /isaac/imu [sensor_msgs/msg/Imu]")
    print("ROS 2 joint states: /isaac/joint_states [sensor_msgs/msg/JointState]")
    print("ROS 2 simulation clock: /clock [rosgraph_msgs/msg/Clock]")
    print(f"ROS 2 simulator ground truth: {GROUND_TRUTH_TOPIC} [nav_msgs/msg/Odometry]")
    if command_receiver is not None:
        print("Carter command source: native ROS 2 /cmd_vel (0.5 s watchdog; keyboard disabled)")
    if not args.headless and command_receiver is None:
        print("Jog controls: W/S or Up/Down = forward/backward, A/D or Left/Right = turn, Space = stop")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(10):
        simulation_app.update()

    follow_camera_path = None
    if not args.headless:
        active_viewport = get_active_viewport()
        if active_viewport is None:
            raise RuntimeError("Could not find the active viewport for the Carter follow camera")
        follow_camera_path = str(active_viewport.camera_path)
        update_follow_camera(carter, follow_camera_path)
        print("Main viewport follows Nova Carter from behind")

    if args.test:
        start_position = carter.get_world_poses()[0].numpy()[0]
        for _ in range(60):
            carter.apply_wheel_actions(
                controller.forward(command=[0.2 * CARTER_FORWARD_SIGN, 0.0])
            )
            if follow_camera_path is not None:
                update_follow_camera(carter, follow_camera_path)
            simulation_app.update()
        carter.apply_wheel_actions(controller.forward(command=[0.0, 0.0]))
        end_position = carter.get_world_poses()[0].numpy()[0]
        displacement = end_position - start_position
        distance_moved = float((displacement**2).sum() ** 0.5)
        if distance_moved < 0.01:
            raise RuntimeError(f"Nova Carter jog test failed; moved only {distance_moved:.4f} m")
        if displacement[0] >= -0.01:
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
                validation_scene.update()
            if command_receiver is not None:
                try:
                    linear, angular = command_receiver.command()
                except ValueError as error:
                    carb.log_error(f"Rejected simulator drive command: {error}")
                    linear, angular = 0.0, 0.0
                command = [linear * CARTER_FORWARD_SIGN, angular]
            else:
                command = (
                    [0.2 * CARTER_FORWARD_SIGN, 0.15]
                    if args.auto_jog
                    else get_jog_command()
                )
            carter.apply_wheel_actions(controller.forward(command=command))
            if follow_camera_path is not None:
                update_follow_camera(carter, follow_camera_path)
            simulation_app.update()
finally:
    if command_receiver is not None:
        carter.apply_wheel_actions(controller.forward(command=[0.0, 0.0]))
    if input_interface is not None and keyboard_subscription is not None:
        input_interface.unsubscribe_to_keyboard_events(keyboard, keyboard_subscription)
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    simulation_app.close()