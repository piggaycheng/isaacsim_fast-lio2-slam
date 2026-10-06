"""Pan/tilt gimbal command logic for Isaac Sim's native ROS 2 JointState subscription."""

import math

PAN_JOINT = "gimbal_pan_joint"
TILT_JOINT = "gimbal_tilt_joint"


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GimbalController:
    """Rate-limited position targets: pan takes the shortest way round, tilt stays in limits."""

    def __init__(self, max_speed=math.radians(90.0), tilt_limits=(-math.pi / 2, math.pi / 2)):
        self.max_speed = max_speed
        self.tilt_limits = tilt_limits
        self.pan_goal = None
        self.tilt_goal = None
        self.pan_target = 0.0
        self.tilt_target = 0.0

    def accept_joint_state(self, names, positions):
        """Store the pan/tilt goals named in a sensor_msgs/JointState; ignore other joints."""
        names, positions = list(names), list(positions)
        if len(names) != len(positions):
            raise ValueError("Gimbal command needs one position per joint name")
        goals = {}
        for name, position in zip(names, positions):
            if name in (PAN_JOINT, TILT_JOINT):
                if not math.isfinite(position):
                    raise ValueError(f"Non-finite {name} command")
                goals[name] = float(position)
        if PAN_JOINT in goals:
            self.pan_goal = goals[PAN_JOINT]
        if TILT_JOINT in goals:
            low, high = self.tilt_limits
            self.tilt_goal = min(max(goals[TILT_JOINT], low), high)

    def reset(self, pan, tilt):
        """Start from the measured joint angles with no pending goal."""
        self.pan_target, self.tilt_target = pan, tilt
        self.pan_goal = self.tilt_goal = None

    def step(self, dt):
        """Advance the targets by dt seconds and return (pan, tilt) position targets (rad)."""
        limit = self.max_speed * max(dt, 0.0)
        if self.pan_goal is not None:
            error = wrap_to_pi(self.pan_goal - self.pan_target)
            self.pan_target += min(max(error, -limit), limit)
            if abs(error) <= limit:
                self.pan_goal = None
        if self.tilt_goal is not None:
            error = self.tilt_goal - self.tilt_target
            self.tilt_target += min(max(error, -limit), limit)
            if abs(error) <= limit:
                self.tilt_goal = None
        return self.pan_target, self.tilt_target


_controllers = {}


def controller_for(name=""):
    """Per-robot controller shared with the OmniGraph script node."""
    if name not in _controllers:
        _controllers[name] = GimbalController()
    return _controllers[name]
