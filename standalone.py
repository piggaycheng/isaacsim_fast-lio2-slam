#!/home/yu/isaacsim-6.1.0/python.sh

import argparse

from isaacsim import SimulationApp


OFFICE_ASSET_PATH = "/Isaac/Environments/Office/office.usd"
CARTER_ASSET_PATH = "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
SURROUNDING_BUILDINGS_PRIM_PATH = "/Root/SM_Buildings"
CARTER_PRIM_PATH = "/World/Carter"
CARTER_LIDAR_PRIM_PATH = f"{CARTER_PRIM_PATH}/chassis_link/sensors/XT_32/PandarXT_32_10hz"
CARTER_IMU_PRIM_PATH = f"{CARTER_LIDAR_PRIM_PATH}/fastlio_imu"
LIDAR_MOTION_COMPENSATION_STATE = "NONCOMPENSATED"
CARTER_SPAWN_POSITION = [0.0, 0.0, 0.05]
LINEAR_JOG_SPEED = 0.5
ANGULAR_JOG_SPEED = 1.2
CARTER_FORWARD_SIGN = -1.0
KIT_EXTRA_ARGS = [
    "--/rtx/post/dlss/execMode=0",
    "--/app/renderer/skipGpuRenderProducts=false",
    "--/rtx/rendermode=RealTime",
]

parser = argparse.ArgumentParser(description="Launch Isaac Sim with the Office environment.")
parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim GUI.")
parser.add_argument("--auto-jog", action="store_true", help="Drive forward automatically for headless SLAM tests.")
parser.add_argument("--test", action="store_true", help="Load the stage and exit after ten frames.")
args, _ = parser.parse_known_args()

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
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.experimental.utils.stage import is_stage_loading
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
from isaacsim.sensors.experimental.physics import IMU
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
from isaacsim.storage.native import get_assets_root_path, is_file
from pxr import UsdGeom

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()


pressed_keys = set()
input_interface = None
keyboard = None
keyboard_subscription = None


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
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                ("ReadIMU.outputs:sensorTime", "PublishIMU.inputs:timeStamp"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("PublishIMU.inputs:topicName", "/isaac/imu"),
                ("PublishIMU.inputs:frameId", "imu_link"),
                ("PublishClock.inputs:topicName", "/clock"),
            ],
        },
    )
    og.Controller.set(
        og.Controller.attribute(f"{graph_path}/ReadIMU.inputs:imuPrim"),
        [usdrt.Sdf.Path(CARTER_IMU_PRIM_PATH)],
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
    controller = DifferentialController(wheel_radius=0.04295, wheel_base=0.4132)

    lidar_prim = stage.GetPrimAtPath(CARTER_LIDAR_PRIM_PATH)
    if not lidar_prim.IsValid():
        raise RuntimeError(f"Nova Carter LiDAR prim was not found: {CARTER_LIDAR_PRIM_PATH}")
    lidar = Lidar(
        CARTER_LIDAR_PRIM_PATH,
        accumulate_outputs=None,
        aux_output_level="BASIC",
        attributes={
            "omni:sensor:Core:outputMotionCompensationState":
                LIDAR_MOTION_COMPENSATION_STATE,
        },
    )
    motion_compensation_state = lidar.prims[0].GetAttribute(
        "omni:sensor:Core:outputMotionCompensationState"
    ).Get()
    if motion_compensation_state != LIDAR_MOTION_COMPENSATION_STATE:
        raise RuntimeError(
            "RTX LiDAR motion compensation state mismatch: "
            f"expected {LIDAR_MOTION_COMPENSATION_STATE}, got {motion_compensation_state}"
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

    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    physics_scenes = SimulationManager.get_physics_scenes()
    if not physics_scenes:
        raise RuntimeError("Isaac Sim did not create a physics scene")
    physics_scenes[0].set_enabled_gpu_dynamics(False)

    if not args.headless:
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
    print("ROS 2 simulation clock: /clock [rosgraph_msgs/msg/Clock]")
    if not args.headless:
        print("Jog controls: W/S or Up/Down = forward/backward, A/D or Left/Right = turn, Space = stop")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(10):
        simulation_app.update()

    if args.test:
        start_position = carter.get_world_poses()[0].numpy()[0]
        for _ in range(60):
            carter.apply_wheel_actions(
                controller.forward(command=[0.2 * CARTER_FORWARD_SIGN, 0.0])
            )
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
        print(
            f"Nova Carter jog test passed: displacement={displacement.tolist()}, "
            f"distance={distance_moved:.3f} m"
        )
    else:
        while simulation_app.is_running():
            command = (
                [0.2 * CARTER_FORWARD_SIGN, 0.15]
                if args.auto_jog
                else get_jog_command()
            )
            carter.apply_wheel_actions(controller.forward(command=command))
            simulation_app.update()
finally:
    if input_interface is not None and keyboard_subscription is not None:
        input_interface.unsubscribe_to_keyboard_events(keyboard, keyboard_subscription)
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    simulation_app.close()