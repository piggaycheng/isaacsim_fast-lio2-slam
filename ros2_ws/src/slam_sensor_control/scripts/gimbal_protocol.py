"""MQTT protocol for the gimbal bridge: topics and payloads (angles in degrees).

Pure functions without ROS or MQTT dependencies so they can be unit tested.

Topics under <prefix>/<robot>/ (default prefix `slam/gimbal`):
  command (in)   {"cmd_id": 1, "action": "move",   "pan_deg": 90, "tilt_deg": 10}   absolute pose
                 {"cmd_id": 2, "action": "rotate", "pan_deg": 15, "tilt_deg": 0}    relative to now
                 {"cmd_id": 3, "action": "cancel"}                                  stop the active move
  result (out)   {"robot_id", "cmd_id", "status", "message", "pan_deg", "tilt_deg", "timestamp"}
                 status: succeeded | aborted | canceled | rejected
  state (out)    pan_deg/tilt_deg measured, busy, current_cmd_id, published at a low rate
  status (out)   retained {"online": true/false}; false is also the LWT on unexpected loss
"""

import json
import math
import time

ACTIONS = ("move", "rotate", "cancel")


class CommandError(ValueError):
    pass


def topics(prefix, robot):
    base = f"{prefix.rstrip('/')}/{robot}" if robot else prefix.rstrip("/")
    return {name: f"{base}/{name}" for name in ("command", "result", "state", "status")}


def _angle(command, key, required):
    value = command.get(key, None if required else 0.0)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise CommandError(f"'{key}' must be a finite number (degrees)")
    return math.radians(float(value))


def cmd_id_of(payload):
    """Best-effort cmd_id of a payload, to report a rejection."""
    try:
        value = json.loads(payload).get("cmd_id")
    except (ValueError, AttributeError, TypeError):
        return None
    return value if isinstance(value, (int, str)) and not isinstance(value, bool) else None


def parse_command(payload):
    """Return (cmd_id, action, pan_rad, tilt_rad); the angles are None for cancel.

    `move` needs pan_deg and tilt_deg; for `rotate` both default to 0 and are deltas.
    """
    try:
        command = json.loads(payload)
    except (ValueError, TypeError) as error:
        raise CommandError(f"invalid JSON: {error}") from error
    if not isinstance(command, dict):
        raise CommandError("command must be a JSON object")
    cmd_id = command.get("cmd_id")
    if isinstance(cmd_id, bool) or not isinstance(cmd_id, (int, str)):
        raise CommandError("'cmd_id' must be an integer or string")
    action = command.get("action")
    if action not in ACTIONS:
        raise CommandError(f"unknown action {action!r}")
    if action == "cancel":
        return cmd_id, action, None, None
    required = action == "move"
    return cmd_id, action, _angle(command, "pan_deg", required), _angle(command, "tilt_deg", required)


def result_payload(robot, cmd_id, status, message="", pose=None):
    payload = {"robot_id": robot, "cmd_id": cmd_id, "status": status, "message": message,
               "timestamp": time.time()}
    if pose is not None and all(math.isfinite(value) for value in pose):
        payload["pan_deg"] = math.degrees(pose[0])
        payload["tilt_deg"] = math.degrees(pose[1])
    return json.dumps(payload)


def state_payload(robot, pose, busy, cmd_id):
    return json.dumps({
        "robot_id": robot, "pan_deg": math.degrees(pose[0]), "tilt_deg": math.degrees(pose[1]),
        "busy": busy, "current_cmd_id": cmd_id, "timestamp": time.time()})


def online_payload(robot):
    return json.dumps({"robot_id": robot, "online": True, "timestamp": time.time()})


def offline_payload(robot):
    return json.dumps({"robot_id": robot, "online": False, "timestamp": time.time()})
