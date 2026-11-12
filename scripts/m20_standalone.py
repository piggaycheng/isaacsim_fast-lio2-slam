"""Isaac Sim 6.2 standalone: Deep Robotics M20 driven by a policy trained in locomotion_training.

Modeled on standalone_examples/api/isaacsim.robot.policy.examples/go2_standalone.py, but the M20
(wheel-legged) is not a bundled policy spec, so observations and actions are built here to match
my_locomotion_training "Velocity-Flat-M20":
  obs (56) = base lin vel(3) | base ang vel(3) | projected gravity(3) | command(3) | leg pos rel(12)
             | leg vel rel *0.05 (12) | last action(16) | wheel vel *0.05 (4)
  act (16) = 12 leg position offsets (*0.25 rad, added to default pose)
             | 4 wheel velocities (*10 rad/s, clipped +-60)
The policy is the TorchScript file exported by `isaaclab play` (logs/rsl_rl/m20_flat/<run>/exported/policy.pt).
Without --policy the robot only holds its default pose (zero actions), which is useful as a smoke test.

With --external the policy runs in the ROS 2 node `m20_policy_node` instead: each control step this script
publishes /clock (simulation time), /m20/imu, /m20/base_velocity, /cmd_vel and /m20/joint_states at every physics step (like
real sensor streams) and applies the newest /m20/joint_commands once per control period (DECIMATION physics steps), waiting up to
--external-timeout for a new command if none arrived. See ros2_ws/src/m20_policy_node.
The rough-terrain policy (Velocity-Rough-M20, obs 243 incl. a 187-point height scan) also needs the terrain map: with
--external an RTX LiDAR + IMU are mounted on base_link and published like standalone.py (/isaac/lidar_points, /isaac/imu,
/tf_static body -> base_link -> lidar_link/imu_link) for FAST-LIO2 + elevation_mapping_cupy; the node samples the scan from
that map. Pass --no-lidar for the flat policy.
"""

import argparse
import math
import re

from isaacsim import SimulationApp

M20_ASSET_PATH = "/Isaac/Robots/DeepRobotics/M20/M20.usd"
M20_PRIM_PATH = "/World/M20"
BASE_PATH = f"{M20_PRIM_PATH}/base_link"
LIDAR_ROOT_PATH = f"{BASE_PATH}/lidar"
LIDAR_CONFIG = "XT32_SD10"
OFFICE_ASSET_PATH = "/Isaac/Environments/Office/office.usd"
SURROUNDING_BUILDINGS_PRIM_PATH = "/Root/SM_Buildings"
WALKABLE_NAME_KEYWORDS = ("stair", "floor")
GROUND_COLLISION_PLANE_PATH = "/Root/GroundPlane/CollisionPlane"
KIT_EXTRA_ARGS = [
    "--/rtx/post/dlss/execMode=0",
    "--/app/renderer/skipGpuRenderProducts=false",
    "--/rtx/rendermode=RealTime",
]
VARIANTS = {"Physics": "PhysX", "Sensor": "None", "Robot": "Robot"}
PHYSICS_DT = 0.005
DECIMATION = 4
STANDING_HEIGHT = 0.49
CAMERA_BACK_M = 2.5
CAMERA_UP_M = 1.5
CAMERA_TARGET_UP_M = 0.3
LEG_JOINTS = re.compile(r".*_(hipx|hipy|knee)_joint")
WHEEL_JOINTS = re.compile(r".*_wheel_joint")
DEFAULT_POS = [
    (re.compile(r".*_hipx_joint"), 0.0),
    (re.compile(r"f[lr]_hipy_joint"), -0.65),
    (re.compile(r"h[lr]_hipy_joint"), 0.65),
    (re.compile(r"f[lr]_knee_joint"), 1.3),
    (re.compile(r"h[lr]_knee_joint"), -1.3),
]
LEG_ACTION_SCALE = 0.25
WHEEL_ACTION_SCALE = 10.0
WHEEL_TARGET_LIMIT = 60.0
LEG_STIFFNESS, LEG_DAMPING, LEG_MAX_EFFORT, LEG_MAX_VELOCITY = 80.0, 2.0, 76.4, 22.4
WHEEL_STIFFNESS, WHEEL_DAMPING, WHEEL_MAX_EFFORT, WHEEL_MAX_VELOCITY = 0.0, 2.0, 21.6, 79.3

SENSOR_WARMUP_S = 3.0


# scripted velocity commands: (seconds, [vx, vy, yaw_rate]); training ranges are vx +-1.5, vy +-0.3, yaw +-1.0
COMMAND_PHASES = [
    (3.0, [0.0, 0.0, 0.0]),
    (5.0, [0.8, 0.0, 0.0]),
    (4.0, [0.5, 0.0, 0.6]),
    (3.0, [0.0, 0.0, 0.0]),
    (4.0, [-0.5, 0.0, 0.0]),
]

parser = argparse.ArgumentParser(description="Run the M20 with an exported locomotion_training policy.")
parser.add_argument("--policy", type=str, default=None, help="TorchScript policy.pt exported by isaaclab play.")
parser.add_argument("--external", action="store_true", help="Run inference in the external m20_policy_node via ROS 2.")
parser.add_argument("--no-cmd-vel", action="store_true", help="With --external: do not publish /cmd_vel (an external publisher such as m20_jog provides it).")
parser.add_argument("--external-timeout", type=float, default=0.2, help="Seconds to wait for each external action.")
parser.add_argument("--environment", choices=["office", "grid"], default="office", help="Scene: Isaac Office (structure for LiDAR odometry) or flat grid.")
parser.add_argument("--office-ground", choices=["mesh", "plane"], default="mesh", help="Office ground collider: floor/stair meshes (can descend stairs) or the invisible collision plane.")
parser.add_argument("--x", type=float, default=0.0, help="Spawn X (m).")
parser.add_argument("--y", type=float, default=0.0, help="Spawn Y (m).")
parser.add_argument("--yaw", type=float, default=0.0, help="Spawn yaw (rad).")
parser.add_argument("--flat-scan", action="store_true", help="Local --policy mode with the rough policy: observe a flat 187-point height scan.")
parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim GUI.")
parser.add_argument("--test", action="store_true", help="Run the command script once and exit.")
parser.add_argument("--demo", action="store_true", help="Loop the scripted commands instead of a zero command.")
parser.add_argument("--device", choices=["cpu", "cuda"], default="cuda", help="Simulation device.")
parser.add_argument("--command", type=float, nargs=3, metavar=("VX", "VY", "WZ"), help="Constant command instead of zero/script (interactive control: m20_jog publishes /cmd_vel).")
parser.add_argument("--no-lidar", action="store_true", help="With --external: do not mount the LiDAR/IMU (flat policy, no height map).")
parser.add_argument("--lidar-offset", type=float, nargs=3, default=[0.2, 0.0, 0.15], metavar=("X", "Y", "Z"), help="LiDAR position in base_link (m).")
parser.add_argument("--lidar-near-range", type=float, default=1.0, help="LiDAR drops returns closer than this (m); removes hits on the robot body.")
parser.add_argument("--lidar-pitch", type=float, default=0.0, help="LiDAR pitch about its Y axis in degrees (positive = nose down).")
parser.add_argument("--lidar-topic", default="/isaac/lidar_points")
parser.add_argument("--imu-topic", default="/isaac/imu")
parser.add_argument("--lidar-frame", default="lidar_link")
parser.add_argument("--imu-frame", default="imu_link")
parser.add_argument("--base-frame", default="base_link", help="Robot frame used by elevation_mapping_cupy.")
parser.add_argument("--body-frame", default="body", help="FAST-LIO body_frame; must equal config/isaac_lio.yaml body_frame.")
parser.add_argument(
    "--no-tf",
    action="store_true",
    help="Do not publish /tf_static from OmniGraph (use when sensor_transforms and local_ekf already publish base_link/lidar_link/imu_link).",
)
parser.add_argument(
    "--box",
    action="append",
    default=[],
    metavar="X,Y[,SX,SY,SZ]",
    help="Add a static collision box on the floor at world X,Y (m, same as the Office map frame). Default size 0.6,0.6,1.0 m. Repeat for more boxes.",
)
args, _ = parser.parse_known_args()


def parse_box(text: str) -> list[float]:
    try:
        values = [float(value) for value in text.split(",")]
    except ValueError:
        values = []
    if len(values) == 2:
        values += [0.6, 0.6, 1.0]
    if len(values) != 5 or min(values[2:]) <= 0:
        parser.error(f"--box expects X,Y or X,Y,SX,SY,SZ with positive sizes, got {text!r}")
    return values


boxes = [parse_box(text) for text in args.box]
use_lidar = args.external and not args.no_lidar

simulation_app = SimulationApp(
    {"headless": args.headless, "enable_motion_bvh": True, "extra_args": KIT_EXTRA_ARGS}
    if use_lidar
    else {"headless": args.headless}
)

import time

import carb
import numpy as np
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils.stage import define_prim, is_stage_loading, set_stage_units, set_stage_up_axis
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.storage.native import get_assets_root_path, is_file
from pxr import Usd, UsdGeom, UsdPhysics

if args.external and args.policy:
    parser.error("--policy and --external are mutually exclusive")
policy = None
if args.policy:
    import torch

    policy = torch.jit.load(args.policy, map_location="cpu").eval()
print(f"Policy: {'external ROS 2 node' if args.external else args.policy or 'none (zero actions)'}")


def quat_rotate_inverse(quat_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Express a world-frame vector in the frame described by ``quat_wxyz``."""
    w, x, y, z = quat_wxyz
    q_vec = np.array([-x, -y, -z])
    t = 2.0 * np.cross(q_vec, vec)
    return vec + w * t + np.cross(q_vec, t)


robot = None
leg_idx = wheel_idx = None
default_leg_pos = None
last_action = np.zeros(16, dtype=np.float32)
step_count = 0
needs_init = True
reset_needed = False
failed = False
base_command = np.zeros(3, dtype=np.float32)


def build_observation() -> np.ndarray:
    positions, orientations = robot.get_world_poses()
    quat = orientations.numpy().reshape(-1)
    lin_vel, ang_vel = robot.get_velocities()
    dof_pos = robot.get_dof_positions().numpy().reshape(-1)
    dof_vel = robot.get_dof_velocities().numpy().reshape(-1)
    return np.concatenate(
        [
            quat_rotate_inverse(quat, lin_vel.numpy().reshape(-1)),
            quat_rotate_inverse(quat, ang_vel.numpy().reshape(-1)),
            quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0])),
            base_command,
            dof_pos[leg_idx] - default_leg_pos,
            dof_vel[leg_idx] * 0.05,
            last_action,
            *([np.full(187, STANDING_HEIGHT - 0.5)] if args.flat_scan else []),
            dof_vel[wheel_idx] * 0.05,
        ]
    ).astype(np.float32)


def apply_action(action: np.ndarray) -> None:
    leg_targets = default_leg_pos + LEG_ACTION_SCALE * action[:12]
    wheel_targets = np.clip(WHEEL_ACTION_SCALE * action[12:], -WHEEL_TARGET_LIMIT, WHEEL_TARGET_LIMIT)
    apply_targets(leg_targets, wheel_targets)


def apply_targets(leg_targets: np.ndarray, wheel_targets: np.ndarray) -> None:
    robot.set_dof_position_targets(leg_targets.reshape(1, -1), dof_indices=leg_idx)
    robot.set_dof_velocity_targets(wheel_targets.reshape(1, -1), dof_indices=wheel_idx)


class ExternalPolicyBridge:
    """Publishes the robot state over ROS 2 and applies the actions returned by m20_policy_node."""

    def __init__(self, timeout: float) -> None:
        import isaacsim.core.experimental.utils.app as app_utils

        app_utils.enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()
        import rclpy
        from geometry_msgs.msg import Twist, TwistStamped
        from rosgraph_msgs.msg import Clock
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Imu, JointState

        self._rclpy, self._Twist, self._TwistStamped = rclpy, Twist, TwistStamped
        self._Imu, self._JointState, self._Clock = Imu, JointState, Clock
        rclpy.init()
        self._node = rclpy.create_node("m20_isaac_sim")
        self._clock_pub = self._node.create_publisher(Clock, "/clock", 1)
        # best effort like the real robot's sensor topics; matches the policy node's default best_effort_qos
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._imu_pub = self._node.create_publisher(Imu, "/m20/imu", qos)
        self._vel_pub = self._node.create_publisher(TwistStamped, "/m20/base_velocity", qos)
        self._cmd_pub = None if args.no_cmd_vel else self._node.create_publisher(Twist, "/cmd_vel", 1)
        self._joint_pub = self._node.create_publisher(JointState, "/m20/joint_states", qos)
        self._node.create_subscription(JointState, "/m20/joint_commands", self._on_command, qos)
        self._timeout = timeout
        self._received = None
        self._fresh = False  # a command arrived that has not been applied yet
        self.timeouts = 0

    def _on_command(self, msg) -> None:
        self._received = msg
        self._fresh = True

    def close(self) -> None:
        self._node.destroy_node()
        self._rclpy.shutdown()

    def step(self) -> None:
        stamp_ns = int(round(step_count * PHYSICS_DT * 1e9))
        # simulation time since the last (re)start, like Isaac Sim's own clock; also published on /clock
        clock = self._Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(stamp_ns, 1_000_000_000)
        stamp = clock.clock
        pos, quat = robot.get_world_poses()
        quat = quat.numpy().reshape(-1)
        lin_vel, ang_vel = robot.get_velocities()
        lin_b = quat_rotate_inverse(quat, lin_vel.numpy().reshape(-1))
        ang_b = quat_rotate_inverse(quat, ang_vel.numpy().reshape(-1))

        imu = self._Imu()
        imu.header.stamp, imu.header.frame_id = stamp, "base_link"
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = (float(v) for v in quat)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (float(v) for v in ang_b)
        vel = self._TwistStamped()
        vel.header.stamp, vel.header.frame_id = stamp, "base_link"
        vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z = (float(v) for v in lin_b)
        cmd = self._Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = (float(v) for v in base_command)
        joints = self._JointState()
        joints.header.stamp = stamp
        joints.name = list(dof_names)
        joints.position = [float(v) for v in robot.get_dof_positions().numpy().reshape(-1)]
        joints.velocity = [float(v) for v in robot.get_dof_velocities().numpy().reshape(-1)]

        if step_count == 0:  # (re)start: drop commands from the previous run
            self._fresh = False
        self._clock_pub.publish(clock)
        self._imu_pub.publish(imu)
        self._vel_pub.publish(vel)
        if not args.no_cmd_vel:
            self._cmd_pub.publish(cmd)
        self._joint_pub.publish(joints)

        # the state is published every physics step like a real sensor stream; commands are only collected here and the
        # newest one is applied once per control period
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        if step_count % DECIMATION != 0:
            return
        # do not outrun the node: wait for a command that arrived since the last control period
        deadline = time.monotonic() + self._timeout
        while not self._fresh and time.monotonic() < deadline:
            self._rclpy.spin_once(self._node, timeout_sec=0.005)
        if self._fresh:
            self._fresh = False
            self._apply(self._received)
            return
        self.timeouts += 1
        if self.timeouts in (1, 10) or self.timeouts % 100 == 0:
            carb.log_warn(f"m20_standalone: no action from m20_policy_node ({self.timeouts} timeouts); holding last targets")

    def _apply(self, msg) -> None:
        index = {name: i for i, name in enumerate(msg.name)}
        leg = np.array([msg.position[index[dof_names[i]]] for i in leg_ids], dtype=np.float32)
        wheel = np.array([msg.velocity[index[dof_names[i]]] for i in wheel_ids], dtype=np.float32)
        if not (np.all(np.isfinite(leg)) and np.all(np.isfinite(wheel))):
            carb.log_warn("m20_standalone: ignoring non-finite action from m20_policy_node")
            return
        apply_targets(leg, wheel)


bridge = None


def restart() -> None:
    global last_action, step_count
    robot.reset_to_default_state()
    robot.set_dof_position_targets(default_leg_pos.reshape(1, -1), dof_indices=leg_idx)
    robot.set_dof_velocity_targets(np.zeros((1, len(wheel_idx))), dof_indices=wheel_idx)
    last_action = np.zeros(16, dtype=np.float32)
    step_count = 0


def on_physics_step(step_size: float, context: object) -> None:
    global needs_init, reset_needed, failed, last_action, step_count
    if failed:
        return
    try:
        if needs_init:
            restart()
            needs_init = False
        elif reset_needed:
            reset_needed = False
            needs_init = True
        else:
            if bridge is not None:
                bridge.step()
            elif step_count % DECIMATION == 0 and policy is not None:
                obs = torch.from_numpy(build_observation()).unsqueeze(0)
                with torch.inference_mode():
                    last_action = policy(obs).squeeze(0).numpy().astype(np.float32)
                apply_action(last_action)
            step_count += 1
    except Exception as error:  # noqa: BLE001 - a physics callback must not raise
        failed = True
        carb.log_error(f"m20_standalone: policy deployment failed, stopping: {error}")


def mount_height_map_sensors():
    """Mount an RTX LiDAR + IMU on base_link and publish them (and the static TFs) for FAST-LIO2."""
    import isaacsim.core.experimental.utils.app as app_utils
    import omni.graph.core as og
    import usdrt
    from isaacsim.sensors.experimental.physics import IMU
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
    from pxr import Gf, Sdf, Usd, UsdGeom

    app_utils.enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()

    lidar = Lidar.create(LIDAR_ROOT_PATH, config=LIDAR_CONFIG, accumulate_outputs=None, aux_output_level="BASIC")
    root = UsdGeom.Xformable(stage.GetPrimAtPath(LIDAR_ROOT_PATH))
    mount_local = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d.YAxis(), args.lidar_pitch)) * Gf.Matrix4d().SetTranslate(
        Gf.Vec3d(*args.lidar_offset)
    )
    root.ClearXformOpOrder()
    root.AddTransformOp().Set(mount_local)

    lidar_path = str(lidar.paths[0])
    lidar_prim = stage.GetPrimAtPath(lidar_path)
    lidar_prim.CreateAttribute("omni:sensor:Core:nearRangeM", Sdf.ValueTypeNames.Float).Set(float(args.lidar_near_range))
    for prim in Usd.PrimRange(stage.GetPrimAtPath(LIDAR_ROOT_PATH)):
        if prim.IsA(UsdGeom.Mesh):
            UsdGeom.Imageable(prim).MakeInvisible()
    lidar_sensor = LidarSensor(lidar, annotators=[])
    lidar_sensor.attach_writer(
        "RtxLidarROS2PublishPointCloud",
        topicName=args.lidar_topic,
        frameId=args.lidar_frame,
        outputIntensity=True,
        outputTimestamp=True,
        outputEmitterId=False,
        outputChannelId=True,
    )
    imu_path = f"{lidar_path}/fastlio_imu"
    IMU.create(
        imu_path,
        translations=[[0.0, 0.0, 0.0]],
        linear_acceleration_filter_size=3,
        angular_velocity_filter_size=3,
        orientation_filter_size=3,
    )

    time_code = Usd.TimeCode.Default()
    mount = UsdGeom.Xformable(stage.GetPrimAtPath(lidar_path)).ComputeLocalToWorldTransform(
        time_code
    ) * UsdGeom.Xformable(stage.GetPrimAtPath(BASE_PATH)).ComputeLocalToWorldTransform(time_code).GetInverse()
    # FAST-LIO publishes map -> body; body -> base_link is the inverse of the sensor mount (LiDAR and IMU are co-located).
    transforms = (
        []
        if args.no_tf
        else [
            ("BodyToBase", args.body_frame, args.base_frame, mount.GetInverse()),
            ("BaseToLidar", args.base_frame, args.lidar_frame, mount),
            ("BaseToImu", args.base_frame, args.imu_frame, mount),
        ]
    )
    keys = og.Controller.Keys
    tf_nodes, tf_connections, tf_values = [], [], []
    for name, parent, child, matrix in transforms:
        quat = Gf.Quatd(matrix.RemoveScaleShear().ExtractRotationQuat())
        imag = quat.GetImaginary()
        tf_nodes.append((name, "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
        tf_connections += [
            ("OnPlaybackTick.outputs:tick", f"{name}.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", f"{name}.inputs:timeStamp"),
        ]
        tf_values += [
            (f"{name}.inputs:parentFrameId", parent),
            (f"{name}.inputs:childFrameId", child),
            (f"{name}.inputs:topicName", "tf_static"),
            (f"{name}.inputs:staticPublisher", True),
            (f"{name}.inputs:translation", list(matrix.ExtractTranslation())),
            (f"{name}.inputs:rotation", [imag[0], imag[1], imag[2], quat.GetReal()]),
        ]
        print(f"Static TF {parent} -> {child}: t={list(matrix.ExtractTranslation())}")

    graph_path = "/World/FASTLIO_ROS2"
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
                *tf_nodes,
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                ("ReadIMU.outputs:sensorTime", "PublishIMU.inputs:timeStamp"),
                *tf_connections,
            ],
            keys.SET_VALUES: [
                ("PublishIMU.inputs:topicName", args.imu_topic),
                ("PublishIMU.inputs:frameId", args.imu_frame),
                *tf_values,
            ],
        },
    )
    og.Controller.set(og.Controller.attribute(f"{graph_path}/ReadIMU.inputs:imuPrim"), [usdrt.Sdf.Path(imu_path)])
    print(f"LiDAR {lidar_path} -> {args.lidar_topic}, IMU {imu_path} -> {args.imu_topic}")
    # The robot jolts for the first seconds after the reset; FAST-LIO would initialise gravity/bias on that
    # motion and diverge, so the IMU/TF graph only starts publishing after SENSOR_WARMUP_S of simulation time.
    sensor_graph = og.Controller.graph(graph_path)
    sensor_graph.set_disabled(True)
    return lidar_sensor, sensor_graph


def enable_stair_collision(stage) -> None:
    """Office stairs/floors get exact mesh colliders; with --office-ground plane the office collision plane stays instead."""
    if args.office_ground == "mesh":
        plane = stage.GetPrimAtPath(GROUND_COLLISION_PLANE_PATH)
        if plane.IsValid():
            UsdPhysics.CollisionAPI(plane).GetCollisionEnabledAttr().Set(False)
    keywords = WALKABLE_NAME_KEYWORDS if args.office_ground == "mesh" else ("stair",)
    roots = [
        p for p in stage.GetPseudoRoot().GetChild("Root").GetChildren()
        if any(k in p.GetName().lower() for k in keywords)
    ]
    count = 0
    for root in roots:
        # Instance proxies cannot author collision APIs, so make the subtree local first.
        for prim in Usd.PrimRange(root):
            if prim.IsInstance():
                prim.SetInstanceable(False)
        for prim in Usd.PrimRange(root):
            if prim.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
                count += 1
    print(f"Enabled mesh colliders on {count} meshes under {len(roots)} walkable prims")


assets_root_path = get_assets_root_path()
if assets_root_path is None:
    raise RuntimeError("Could not find Isaac Sim assets folder")

if args.environment == "office":
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
    if buildings_prim.IsValid():
        UsdGeom.Imageable(buildings_prim).MakeInvisible()
    enable_stair_collision(stage)
    set_stage_up_axis("Z")
    set_stage_units(meters_per_unit=1.0)
else:
    set_stage_up_axis("Z")
    set_stage_units(meters_per_unit=1.0)
    ground = define_prim("/World/Ground", "Xform")
    ground.GetReferences().AddReference(assets_root_path + "/Isaac/Environments/Grid/default_environment.usd")
if boxes:
    from isaacsim.core.experimental.objects import Cube

    stage = omni.usd.get_context().get_stage()
    for index, (x, y, size_x, size_y, size_z) in enumerate(boxes):
        box_path = f"/World/Obstacles/Box_{index}"
        Cube(
            box_path,
            sizes=1.0,
            colors="orange",
            positions=[x, y, size_z / 2.0],
            scales=[size_x, size_y, size_z],
        )
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(box_path))
        print(f"Spawned static box at ({x:.2f}, {y:.2f}) size {size_x}x{size_y}x{size_z} m")
if not omni.usd.get_context().get_stage().GetPrimAtPath("/World/PhysicsScene").IsValid() and not any(
    prim.IsA(UsdPhysics.Scene) for prim in omni.usd.get_context().get_stage().Traverse()
):
    define_prim("/World/PhysicsScene", "PhysicsScene")
SimulationManager.set_physics_sim_device(args.device)

m20_prim = define_prim(M20_PRIM_PATH, "Xform")
m20_prim.GetReferences().AddReference(assets_root_path + M20_ASSET_PATH)
for variant_set, selection in VARIANTS.items():
    m20_prim.GetVariantSets().GetVariantSet(variant_set).SetVariantSelection(selection)

robot = Articulation(M20_PRIM_PATH, positions=[[args.x, args.y, STANDING_HEIGHT]], reset_xform_op_properties=True)

dof_names = list(robot.dof_names)
leg_ids = [i for i, n in enumerate(dof_names) if LEG_JOINTS.fullmatch(n)]
wheel_ids = [i for i, n in enumerate(dof_names) if WHEEL_JOINTS.fullmatch(n)]
if len(leg_ids) != 12 or len(wheel_ids) != 4:
    raise RuntimeError(f"Unexpected M20 joints: {dof_names}")
leg_idx, wheel_idx = np.array(leg_ids), np.array(wheel_ids)
default_leg_pos = np.array(
    [next(v for p, v in DEFAULT_POS if p.fullmatch(dof_names[i])) for i in leg_ids], dtype=np.float32
)
print(f"Leg joints: {[dof_names[i] for i in leg_ids]}")
print(f"Wheel joints: {[dof_names[i] for i in wheel_ids]}")

# match the actuator model used in training (implicit PD, see my_locomotion_training/assets/m20.py)
robot.set_dof_gains(LEG_STIFFNESS, LEG_DAMPING, dof_indices=leg_idx)
robot.set_dof_gains(WHEEL_STIFFNESS, WHEEL_DAMPING, dof_indices=wheel_idx)
robot.set_dof_max_efforts(LEG_MAX_EFFORT, dof_indices=leg_idx)
robot.set_dof_max_efforts(WHEEL_MAX_EFFORT, dof_indices=wheel_idx)
robot.set_dof_max_velocities(LEG_MAX_VELOCITY, dof_indices=leg_idx)
robot.set_dof_max_velocities(WHEEL_MAX_VELOCITY, dof_indices=wheel_idx)
default_positions = np.zeros((1, len(dof_names)), dtype=np.float32)
default_positions[0, leg_idx] = default_leg_pos
robot.set_default_state(
    positions=[[args.x, args.y, STANDING_HEIGHT]],
    orientations=[[math.cos(args.yaw / 2.0), 0.0, 0.0, math.sin(args.yaw / 2.0)]],
    dof_positions=default_positions,
    dof_velocities=np.zeros_like(default_positions),
)

SimulationManager.set_physics_dt(PHYSICS_DT)
RenderingManager.set_dt(PHYSICS_DT * DECIMATION)

frame_dt = PHYSICS_DT * DECIMATION
if args.command is not None:
    frame_commands = np.tile(np.asarray(args.command, dtype=np.float32), (int(10.0 / frame_dt), 1))
elif not (args.demo or args.test):
    frame_commands = np.zeros((int(10.0 / frame_dt), 3), dtype=np.float32)
else:
    frame_commands = np.concatenate(
        [np.tile(np.asarray(c, dtype=np.float32), (int(round(s / frame_dt)), 1)) for s, c in COMMAND_PHASES]
    )

# The sensor must stay referenced: its destructor detaches the writer and stops the point cloud.
lidar_sensor, sensor_graph = mount_height_map_sensors() if use_lidar else (None, None)
if args.external:
    bridge = ExternalPolicyBridge(args.external_timeout)

callback_id = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)

def follow_robot_camera() -> None:
    """Keep the viewport camera behind and above the base, rotating with its heading."""
    if args.headless:
        return
    from omni.kit.viewport.utility import get_active_viewport_camera_path
    from omni.kit.viewport.utility.camera_state import ViewportCameraState
    from pxr import Gf

    pos, quat = robot.get_world_poses()
    pos = pos.numpy().reshape(-1)[:3]
    w, x, y, z = quat.numpy().reshape(-1)[:4]
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    forward = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    eye = pos - CAMERA_BACK_M * forward + np.array([0.0, 0.0, CAMERA_UP_M])
    target = pos + np.array([0.0, 0.0, CAMERA_TARGET_UP_M])
    camera_state = ViewportCameraState(str(get_active_viewport_camera_path()))
    camera_state.set_position_world(Gf.Vec3d(*map(float, eye)), True)
    camera_state.set_target_world(Gf.Vec3d(*map(float, target)), True)


timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

i = 0
start_xy = None
while simulation_app.is_running():
    simulation_app.update()
    if failed:
        break
    if SimulationManager.is_simulating():
        if start_xy is None:
            start_xy = robot.get_world_poses()[0].numpy().reshape(-1)[:2].copy()
        if i == len(frame_commands):
            i = 0
            if args.test:
                end_xy = robot.get_world_poses()[0].numpy().reshape(-1)[:2]
                print(f"Test finished: base moved {np.linalg.norm(end_xy - start_xy):.3f} m from spawn")
                break
        if sensor_graph is not None and sensor_graph.is_disabled() and step_count * PHYSICS_DT >= SENSOR_WARMUP_S:
            sensor_graph.set_disabled(False)
        follow_robot_camera()
        base_command = frame_commands[i]
        i += 1
    else:
        reset_needed = True

timeline.stop()
SimulationManager.deregister_callback(callback_id)
if bridge is not None:
    bridge.close()
simulation_app.close(exit_code=1 if failed else 0)
