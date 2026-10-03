"""Robot specs, robot-type profiles and per-robot ROS namespacing.

Pure Python (only PyYAML), shared by the ROS launch files and Isaac Sim's
standalone.py so both sides read the same robot-type profile.

Each robot runs its stack under /<namespace> with its own TF tree
(/<namespace>/tf), so frame names (map, odom, base_link, camera_init, body)
stay unchanged. An empty namespace leaves every name and file untouched.

Nav2 Humble resolves costmap layer topics relative to the nested costmap node
(/<ns>/local_costmap), and parameter files are keyed by node name, so YAML
configs are rewritten: absolute topic values get the namespace prefix and the
top-level node keys are nested under the namespace.
"""

import copy
import math
import re
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_ROBOT_TYPE = "nova_carter"
PROFILE_DIRECTORY = Path(__file__).resolve().parent.parent / "config" / "robots"
NAMESPACE_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
TOPIC_KEY_PATTERN = re.compile(r"^(topic|.*_topic|(odom|imu|pose|twist)\d+)$")
GLOBAL_TOPICS = frozenset(("/clock", "/parameter_events", "/rosout"))
SPEC_PATTERN = re.compile(r"^(?P<name>[^:@\s]+)(:(?P<type>[^:@\s]+))?@(?P<pose>[^@\s]+)$")
# FAST_LIO_LOCALIZATION2 hardcodes these absolute names; remap them per node.
UPSTREAM_TOPICS = (
    "/cloud_registered", "/cloud_registered_body", "/cloud_effected", "/Laser_map",
    "/Odometry", "/path", "/initialpose", "/map_to_odom", "/submap", "/cur_scan_in_map",
)
SIMULATION_KEYS = (
    "asset", "articulation", "lidar", "imu", "wheel_joints", "wheel_radius",
    "wheel_base", "forward_sign", "spawn_height",
)
FRAME_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")
SENSOR_FRAMES = ("lidar_link", "imu_link")
# local_ekf (odom -> base_link) inputs a profile may select (slam_nav/config/local_ekf_inputs.yaml).
LOCAL_ODOMETRY_INPUTS = ("wheel", "imu", "lio")
DEFAULT_LOCAL_ODOMETRY_INPUTS = ("wheel", "imu")
ROBOT_LOCALIZATION_INPUT_PATTERN = re.compile(r"^(odom|imu|pose|twist)\d+(_.*)?$")
# Without wheel odometry (so LIO is a local input) the global EKF propagates
# between PCD corrections with the scan-to-scan LIO velocity (lio_odometry's
# lio/twist) instead. Its covariance has its own parameters (global_fusion.yaml
# lio_odometry twist_*_variance), so calibrating the local LIO pose variance
# never changes the global EKF.
GLOBAL_EKF_LIO_TWIST = {
    "twist0": "/lio/twist",
    "twist0_config": [False, False, False, False, False, False,
                      True, True, False, False, False, True,
                      False, False, False],
    "twist0_queue_size": 10,
    "twist0_nodelay": True,
}
# Vehicle-specific parameters needed only when a local_ekf input is selected.
INPUT_OVERRIDES = {
    "lio": {
        "lio_odometry": ("position_variance", "yaw_variance"),
    },
    "wheel": {
        "wheel_encoder_odometry": (
            "left_joint", "right_joint", "wheel_radius", "wheel_base",
            "encoder_ticks_per_revolution", "left_distance_scale", "right_distance_scale",
            "left_direction", "right_direction", "distance_noise_ratio",
            "distance_variance_per_meter", "position_variance_per_radian",
            "yaw_variance_per_meter", "yaw_variance_per_radian",
        ),
    },
}
# Vehicle-specific parameters that the base configs omit: every profile must set them.
REQUIRED_OVERRIDES = {
    "nav_imu_adapter": (
        "orientation_variance", "angular_velocity_variance", "linear_acceleration_variance",
    ),
    "global_pose_adapter": (
        "registration_covariance_scale", "min_covariance_xy", "min_covariance_yaw",
    ),
    "ground_obstacle_filter": ("ground_z",),
}
MIN_SPAWN_SEPARATION = 1.5




@dataclass(frozen=True)
class RobotSpec:
    name: str
    robot_type: str
    x: float
    y: float
    yaw: float


# Robot and spawn that recorded maps/office (its body frame is the map frame).
MAP_REFERENCE = RobotSpec("", DEFAULT_ROBOT_TYPE, 0.0, 0.0, 0.0)


def normalize_namespace(namespace):
    """Return a bare namespace token ("" for the root namespace)."""
    value = (namespace or "").strip().strip("/")
    if value and not NAMESPACE_PATTERN.match(value):
        raise ValueError(
            f"Invalid robot namespace {namespace!r}: use a letter followed by letters, digits or _"
        )
    return value


def parse_robot_spec(text):
    """Parse NAME[:TYPE]@X,Y[,YAW] (map/world metres and radians)."""
    match = SPEC_PATTERN.match(text.strip())
    if not match:
        raise ValueError(f"Robot spec must be NAME[:TYPE]@X,Y[,YAW], got {text!r}")
    name = normalize_namespace(match["name"])
    robot_type = match["type"] or DEFAULT_ROBOT_TYPE
    if not NAMESPACE_PATTERN.match(robot_type):
        raise ValueError(f"Invalid robot type {robot_type!r}")
    try:
        values = [float(value) for value in match["pose"].split(",")]
    except ValueError:
        values = []
    if len(values) not in (2, 3) or not all(math.isfinite(value) for value in values):
        raise ValueError(f"Robot pose must be X,Y[,YAW], got {match['pose']!r}")
    x, y, yaw = (values + [0.0])[:3]
    return RobotSpec(name, robot_type, x, y, yaw)


def parse_robot_specs(texts):
    """Parse specs from an iterable or a ';'-separated string and validate the fleet."""
    if isinstance(texts, str):
        texts = [text for text in texts.split(";") if text.strip()]
    specs = [parse_robot_spec(text) for text in texts]
    if not specs:
        raise ValueError("At least one robot is required")
    names = [spec.name for spec in specs]
    # Case-insensitive: each robot also names a (lowercase) Compose project.
    if len({name.lower() for name in names}) != len(names):
        raise ValueError(f"Robot names must be unique (ignoring case): {names}")
    for index, first in enumerate(specs):
        for second in specs[index + 1:]:
            if math.hypot(first.x - second.x, first.y - second.y) < MIN_SPAWN_SEPARATION:
                raise ValueError(
                    f"Robots {first.name} and {second.name} spawn closer than "
                    f"{MIN_SPAWN_SEPARATION} m"
                )
    return specs


def format_robot_specs(specs):
    return ";".join(f"{s.name}:{s.robot_type}@{s.x:g},{s.y:g},{s.yaw:g}" for s in specs)


def load_robot_profile(robot_type, directory=PROFILE_DIRECTORY):
    """Load and validate config/robots/<robot_type>.yaml."""
    if not NAMESPACE_PATTERN.match(robot_type or ""):
        raise ValueError(f"Invalid robot type {robot_type!r}")
    path = Path(directory) / f"{robot_type}.yaml"
    if not path.is_file():
        available = sorted(item.stem for item in Path(directory).glob("*.yaml"))
        raise ValueError(f"Unknown robot type {robot_type!r}; available: {available}")
    profile = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    simulation = profile.get("simulation")
    if not isinstance(simulation, dict) or any(key not in simulation for key in SIMULATION_KEYS):
        raise ValueError(f"{path}: simulation needs {', '.join(SIMULATION_KEYS)}")
    if len(simulation["wheel_joints"]) != 2:
        raise ValueError(f"{path}: wheel_joints must list the left and right joints")
    if "lidar_translation" in simulation:
        translation = simulation["lidar_translation"]
        if (not isinstance(translation, list) or len(translation) != 3
                or not all(isinstance(value, (int, float)) and math.isfinite(value)
                           for value in translation)):
            raise ValueError(f"{path}: lidar_translation needs three finite values")
    frames = profile.get("sensor_frames")
    if not isinstance(frames, dict) or set(frames) != set(SENSOR_FRAMES):
        raise ValueError(f"{path}: sensor_frames must define {', '.join(SENSOR_FRAMES)}")
    for frame, values in frames.items():
        if not isinstance(values, dict) or set(values) != set(FRAME_KEYS) or not all(
            isinstance(values[key], (int, float)) and math.isfinite(values[key])
            for key in FRAME_KEYS
        ):
            raise ValueError(f"{path}: sensor_frames.{frame} needs finite {', '.join(FRAME_KEYS)}")
    overrides = profile.setdefault("parameter_overrides", {}) or {}
    if not isinstance(overrides, dict):
        raise ValueError(f"{path}: parameter_overrides must be a mapping")
    local = profile.get("local_odometry") or {}
    if not isinstance(local, dict):
        raise ValueError(f"{path}: local_odometry must be a mapping")
    try:
        inputs = parse_local_odometry_inputs(local.get("inputs", DEFAULT_LOCAL_ODOMETRY_INPUTS))
    except ValueError as error:
        raise ValueError(f"{path}: local_odometry.inputs: {error}") from None
    profile["local_odometry"] = dict(local, inputs=inputs)
    profile["parameter_overrides"] = overrides
    profile["robot_type"] = robot_type
    try:
        require_overrides(profile, REQUIRED_OVERRIDES)
        require_input_overrides(profile, inputs)
    except ValueError as error:
        raise ValueError(f"{path}: {error}") from None
    return profile


def require_overrides(profile, required):
    overrides = profile["parameter_overrides"]
    for node, keys in required.items():
        parameters = ((overrides.get(node) or {}).get("ros__parameters") or {})
        missing = [key for key in keys if key not in parameters]
        if missing:
            raise ValueError(
                f"parameter_overrides.{node}.ros__parameters needs {', '.join(missing)}"
            )


def require_input_overrides(profile, inputs):
    """Fail if the profile lacks the vehicle parameters of a selected local_ekf input."""
    for name in inputs:
        try:
            require_overrides(profile, INPUT_OVERRIDES.get(name, {}))
        except ValueError as error:
            raise ValueError(f"local_odometry input {name!r}: {error}") from None


def parse_local_odometry_inputs(value):
    """Validate local_ekf inputs from a list or a comma-separated string, in canonical order."""
    items = value.split(",") if isinstance(value, str) else value
    if not isinstance(items, (list, tuple)):
        raise ValueError(f"expected a list of {', '.join(LOCAL_ODOMETRY_INPUTS)}")
    names = [str(item).strip().lower() for item in items if str(item).strip()]
    unknown = sorted(set(names) - set(LOCAL_ODOMETRY_INPUTS))
    if unknown:
        raise ValueError(f"unknown inputs {unknown}; choose from {', '.join(LOCAL_ODOMETRY_INPUTS)}")
    if len(set(names)) != len(names):
        raise ValueError(f"duplicate inputs in {names}")
    if not names:
        raise ValueError("select at least one input")
    if names == ["imu"]:
        raise ValueError("imu alone gives no translation; add wheel or lio")
    return tuple(name for name in LOCAL_ODOMETRY_INPUTS if name in names)


def local_odometry_inputs(profile, override=""):
    """The launch override (comma-separated) if set, else the profile's inputs."""
    if not (override or "").strip():
        return profile["local_odometry"]["inputs"]
    inputs = parse_local_odometry_inputs(override)
    require_input_overrides(profile, inputs)
    return inputs


def strip_robot_localization_inputs(parameters):
    return {key: value for key, value in parameters.items()
            if not ROBOT_LOCALIZATION_INPUT_PATTERN.match(key)}


def local_ekf_inputs(templates, inputs):
    """robot_localization parameters for the selected input templates."""
    parameters, counts, filter_parameters = {}, {}, {}
    for name in inputs:
        template = templates[name]
        kind = template["kind"]
        prefix = f"{kind}{counts.get(kind, 0)}"
        counts[kind] = counts.get(kind, 0) + 1
        parameters[prefix] = template["topic"]
        template_parameters = dict(template.get("parameters") or {})
        template_filter = dict(template.get("filter") or {})
        for other, override in (template.get("when") or {}).items():
            if other in inputs:
                template_parameters.update(override.get("parameters") or {})
                template_filter.update(override.get("filter") or {})
        for key, value in template_parameters.items():
            parameters[f"{prefix}_{key}"] = copy.deepcopy(value)
        filter_parameters.update(copy.deepcopy(template_filter))
    return {**filter_parameters, **parameters}


def with_local_ekf_inputs(config, templates, inputs, node="local_ekf"):
    """Replace the inputs of the local_ekf parameter block with the selected templates."""
    result = copy.deepcopy(config)
    parameters = strip_robot_localization_inputs(result[node]["ros__parameters"])
    parameters.update(local_ekf_inputs(templates, inputs))
    result[node]["ros__parameters"] = parameters
    return result


def with_global_ekf_inputs(config, inputs, node="global_ekf"):
    """Without wheel odometry, propagate the global EKF with the LIO velocity."""
    if "wheel" in inputs:
        return config
    result = copy.deepcopy(config)
    parameters = {key: value for key, value in result[node]["ros__parameters"].items()
                  if not re.match(r"^(odom|twist)\d+(_.*)?$", key)}
    parameters.update(copy.deepcopy(GLOBAL_EKF_LIO_TWIST))
    result[node]["ros__parameters"] = parameters
    return result


def imu_mount(profile):
    """base_link -> imu_link (FAST-LIO body) as the imu_mount node parameter."""
    mount = profile["sensor_frames"]["imu_link"]
    return [float(mount[key]) for key in FRAME_KEYS]


def body_pose(spec, profile):
    """World (x, y, z, yaw) of the FAST-LIO body (imu_link) when spawned at spec.

    spec x, y, yaw is the Isaac world pose of the robot prim; base_link faces the
    asset front (prim -x when forward_sign < 0) and base_link -> imu_link comes
    from sensor_frames.
    """
    simulation = profile["simulation"]
    base_yaw = spec.yaw + (math.pi if float(simulation["forward_sign"]) < 0 else 0.0)
    mount = profile["sensor_frames"]["imu_link"]
    cos, sin = math.cos(base_yaw), math.sin(base_yaw)
    return (
        spec.x + cos * mount["x"] - sin * mount["y"],
        spec.y + sin * mount["x"] + cos * mount["y"],
        float(simulation["spawn_height"]) + mount["z"],
        base_yaw + mount["yaw"],
    )


def initial_pose(spec, profile, reference_profile=None):
    """Map -> camera_init (x, y, z, yaw) for a robot spawned at spec.

    FAST-LIO's camera_init is the body frame at startup and the Office maps were
    recorded from MAP_REFERENCE, so the map frame is that robot's body frame at
    its spawn. The reference spawn therefore yields the identity.
    """
    if reference_profile is None:
        reference_profile = load_robot_profile(MAP_REFERENCE.robot_type)
    mx, my, mz, myaw = body_pose(MAP_REFERENCE, reference_profile)
    bx, by, bz, byaw = body_pose(spec, profile)
    cos, sin = math.cos(myaw), math.sin(myaw)
    dx, dy = bx - mx, by - my
    yaw = math.atan2(math.sin(byaw - myaw), math.cos(byaw - myaw))
    values = (cos * dx + sin * dy, -sin * dx + cos * dy, bz - mz, yaw)
    return tuple(round(value, 6) + 0.0 for value in values)


def merge_overrides(config, overrides):
    """Deep-merge overrides for the node keys that exist in this parameter file."""
    result = copy.deepcopy(config)

    def merge(target, source):
        for key, value in source.items():
            if isinstance(value, dict) and isinstance(target.get(key), dict):
                merge(target[key], value)
            else:
                target[key] = copy.deepcopy(value)

    for node, value in (overrides or {}).items():
        if node in result and isinstance(value, dict):
            merge(result[node], value)
    return result


def namespaced_topic(namespace, name):
    """Prefix an absolute topic with the robot namespace; leave others unchanged."""
    namespace = normalize_namespace(namespace)
    if not namespace or not name.startswith("/") or name in GLOBAL_TOPICS:
        return name
    return f"/{namespace}{name}"


def namespace_config(config, namespace):
    """Return a parameter-file dict for the robot namespace."""
    namespace = normalize_namespace(namespace)
    if not namespace:
        return config

    def rewrite(value, key=None):
        if isinstance(value, dict):
            return {name: rewrite(item, name) for name, item in value.items()}
        if isinstance(value, list):
            return [rewrite(item, key) for item in value]
        if isinstance(value, str) and isinstance(key, str) and TOPIC_KEY_PATTERN.match(key):
            return namespaced_topic(namespace, value)
        return value

    rewritten = rewrite(copy.deepcopy(config))
    wildcards = {key: value for key, value in rewritten.items() if key.startswith("/**")}
    nodes = {key: value for key, value in rewritten.items() if not key.startswith("/**")}
    if any(key.startswith("/") for key in nodes):
        raise ValueError("Namespaced parameter files must use relative node names")
    result = dict(wildcards)
    if nodes:
        result[namespace] = nodes
    return result


def namespace_rviz(config, namespace):
    """Point every RViz display and tool topic at the robot namespace."""
    namespace = normalize_namespace(namespace)
    if not namespace:
        return config

    def rewrite(value):
        if isinstance(value, dict):
            result = {}
            for key, item in value.items():
                if key == "Topic" and isinstance(item, dict) and isinstance(item.get("Value"), str):
                    item = dict(item, Value=namespaced_topic(namespace, item["Value"]))
                elif key == "Interactive Markers Namespace" and isinstance(item, str):
                    item = namespaced_topic(namespace, item)
                result[key] = rewrite(item)
            return result
        if isinstance(value, list):
            return [rewrite(item) for item in value]
        return value

    return rewrite(copy.deepcopy(config))


FLEET_COLORS = (
    "0; 200; 255", "255; 140; 0", "120; 220; 60", "230; 80; 230", "255; 220; 0", "160; 120; 255",
)


def _topic(value, reliability="Reliable", durability="Volatile"):
    return {"Depth": 5, "Durability Policy": durability, "History Policy": "Keep Last",
            "Reliability Policy": reliability, "Value": value}


def fleet_rviz(names):
    """One RViz config showing every robot; TF comes merged from fleet_relay.py.

    Map-frame topics are read straight from /<ns>/...; robot-frame topics from
    the /fleet/<ns>/... relay. A panel selects the robot for shared pose tools.
    """
    names = [normalize_namespace(name) for name in names]
    if not names or not all(names):
        raise ValueError("fleet RViz needs robot names")
    displays = [
        {"Class": "rviz_default_plugins/Grid", "Enabled": True, "Name": "Grid",
         "Plane": "XY", "Reference Frame": "<Fixed Frame>"},
        {"Alpha": 0.7, "Class": "rviz_default_plugins/Map", "Color Scheme": "map",
         "Enabled": True, "Name": "Map",
         "Topic": _topic(f"/{names[0]}/map", durability="Transient Local")},
        {"Class": "rviz_default_plugins/TF", "Enabled": False, "Frame Timeout": 5, "Name": "TF",
         "Show Arrows": False, "Show Axes": True, "Show Names": True},
    ]
    tools = [{"Class": f"rviz_default_plugins/{name}"}
             for name in ("Interact", "MoveCamera", "Select")]
    for index, name in enumerate(names):
        color = FLEET_COLORS[index % len(FLEET_COLORS)]
        displays.append({
            "Class": "rviz_common/Group", "Enabled": True, "Name": name, "Displays": [
                {"Alpha": 1, "Class": "rviz_default_plugins/Polygon", "Color": color,
                 "Enabled": True, "Name": "Footprint",
                 "Topic": _topic(f"/{name}/global_costmap/published_footprint")},
                {"Class": "rviz_default_plugins/Odometry", "Enabled": True, "Name": "Pose",
                 "Keep": 1, "Position Tolerance": 0.1, "Angle Tolerance": 0.1,
                 "Shape": {"Value": "Arrow", "Color": color, "Alpha": 1,
                           "Shaft Length": 0.6, "Shaft Radius": 0.06,
                           "Head Length": 0.25, "Head Radius": 0.15},
                 "Covariance": {"Value": False},
                 "Topic": _topic(f"/{name}/odometry/global")},
                {"Alpha": 1, "Buffer Length": 1, "Class": "rviz_default_plugins/Path",
                 "Color": color, "Enabled": True, "Line Style": "Lines", "Line Width": 0.04,
                 "Name": "Plan", "Pose Style": "None", "Topic": _topic(f"/{name}/plan")},
                {"Class": "rviz_default_plugins/LaserScan", "Color": color,
                 "Color Transformer": "FlatColor", "Decay Time": 0, "Enabled": True,
                 "Name": "Obstacle scan", "Size (Pixels)": 3, "Style": "Points",
                 "Topic": _topic(f"/fleet/{name}/scan", reliability="Best Effort")},
                {"Alpha": 1, "Class": "rviz_default_plugins/Polygon", "Color": "255; 0; 0",
                 "Enabled": True, "Name": "Collision stop zone",
                 "Topic": _topic(f"/fleet/{name}/collision_monitor/polygon_stop")},
                {"Alpha": 1, "Class": "rviz_default_plugins/Polygon", "Color": "255; 0; 150",
                 "Enabled": True, "Name": "Collision surround stop zone",
                 "Topic": _topic(f"/fleet/{name}/collision_monitor/polygon_surround")},
                {"Alpha": 1, "Class": "rviz_default_plugins/Polygon", "Color": "255; 170; 0",
                 "Enabled": False, "Name": "Collision slowdown zone",
                 "Topic": _topic(f"/fleet/{name}/collision_monitor/polygon_slowdown")},
                {"Class": "rviz_default_plugins/PointCloud2", "Color": color,
                 "Color Transformer": "FlatColor", "Enabled": False,
                 "Name": "3D obstacles", "Size (Pixels)": 2, "Style": "Points",
                 "Topic": _topic(f"/fleet/{name}/perception/obstacles", reliability="Best Effort")},
                {"Alpha": 0.4, "Class": "rviz_default_plugins/Map", "Color Scheme": "costmap",
                 "Enabled": False, "Name": "Global costmap",
                 "Topic": _topic(f"/{name}/global_costmap/costmap",
                                 durability="Transient Local")},
            ],
        })
    tools += [
        {"Class": "rviz_default_plugins/SetInitialPose",
         "Topic": _topic(f"/{names[0]}/initialpose")},
        {"Class": "rviz_default_plugins/SetGoal", "Topic": _topic(f"/{names[0]}/goal_pose")},
    ]
    return {
        "Panels": [{"Class": "rviz_common/Displays", "Name": "Displays"},
                   {"Class": "rviz_common/Tool Properties", "Name": "Tool Properties"},
                   {"Class": "slam_localization_3d/FleetPanel", "Name": "Fleet Control",
                    "Robots": names, "Selected Robot": names[0]}],
        "Visualization Manager": {
            "Class": "", "Displays": displays, "Enabled": True,
            "Global Options": {"Background Color": "48; 48; 48", "Fixed Frame": "map",
                               "Frame Rate": 30},
            "Name": "root", "Tools": tools,
            "Views": {"Current": {
                "Class": "rviz_default_plugins/Orbit", "Distance": 23,
                "Focal Point": {"X": 0, "Y": 0, "Z": 0}, "Name": "Current View",
                "Pitch": 1.3, "Target Frame": "map", "Yaw": 0, "Value": "Orbit (rviz)"}},
        },
        "Window Geometry": {"Height": 900, "Width": 1440},
    }
