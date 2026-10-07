"""Open-RMF fleet adapter MQTT protocol (amr_mqtt_interface_spec.md): topics and payloads.

Pure functions without ROS or MQTT dependencies so they can be unit tested.
"""

import json
import math
import time

COMMAND_ACTIONS = ("navigate", "dock", "stop", "task")
RESULT_STATUS = {"succeeded": "completed", "aborted": "failed", "canceled": "canceled",
                 "rejected": "failed"}


class CommandError(ValueError):
    pass


def topics(fleet, robot):
    base = f"rmf/{fleet}/robot/{robot}"
    return {name: f"{base}/{name}" for name in (
        "register", "register_ack", "heartbeat", "command", "command_result", "deregister",
        "status", "task_state")}


def client_id(fleet, robot):
    return f"amr_{fleet}_{robot}"


def now():
    return time.time()


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def register_payload(fleet, robot, pose, level, waypoint="", specs=None, charger="", parking="",
                     timestamp=None):
    location = {"x": pose[0], "y": pose[1], "yaw": wrap_angle(pose[2]), "level_name": level}
    if waypoint:
        location["waypoint_name"] = waypoint
    payload = {
        "robot_id": robot, "fleet_name": fleet,
        "timestamp": now() if timestamp is None else timestamp,
        "initial_location": location,
        "specs": {key: value for key, value in (specs or {}).items() if value is not None},
    }
    if charger:
        payload["default_charger"] = charger
    if parking:
        payload["default_parking"] = parking
    return json.dumps(payload)


def heartbeat_payload(robot, pose, battery, status, cmd_id, level=None):
    payload = {
        "robot_id": robot, "x": pose[0], "y": pose[1], "yaw": wrap_angle(pose[2]),
        "battery": min(100.0, max(0.0, float(battery))), "status": status,
        "current_cmd_id": cmd_id,
    }
    if level:
        payload["level_name"] = level
    return json.dumps(payload)


def deregister_payload(fleet, robot, reason):
    return json.dumps({"robot_id": robot, "fleet_name": fleet, "reason": reason,
                       "timestamp": now()})


def offline_payload(robot):
    return json.dumps({"robot_id": robot, "status": "offline",
                       "reason": "Unexpected connection loss (LWT triggered)",
                       "timestamp": now()})


def command_result_payload(robot, cmd_id, status, message="", location=None):
    payload = {"robot_id": robot, "cmd_id": cmd_id, "status": status, "message": message,
               "timestamp": now()}
    if location is not None:
        payload["final_location"] = {
            "x": location[0], "y": location[1], "yaw": wrap_angle(location[2])}
    return json.dumps(payload)


def parse_ack(payload, robot):
    """Return (accepted, message) for a register_ack addressed to `robot`, else None."""
    try:
        ack = json.loads(payload)
    except (ValueError, TypeError):
        return None
    if not isinstance(ack, dict) or ack.get("robot_id") != robot:
        return None
    message = ack.get("message") or ack.get("error_code") or ""
    return ack.get("status") == "success", str(message)


def _number(source, key, default=None):
    value = source.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise CommandError(f"'{key}' must be a finite number")
    return float(value)


def cmd_id_of(payload):
    """Best-effort cmd_id of a command payload, for reporting a rejection."""
    try:
        value = json.loads(payload).get("cmd_id")
    except (ValueError, AttributeError, TypeError):
        return None
    return value if isinstance(value, int) and not isinstance(value, bool) else None


def parse_command(payload, robot):
    """Return (cmd_id, action, steps) with steps a list of task_bt steps, None for stop.

    `task` is an extension to the spec: {"action": "task", "steps": [...]} runs several
    task_bt steps in order (validated by task_bt). Returns None for a command addressed
    to another robot.
    """
    try:
        command = json.loads(payload)
    except (ValueError, TypeError) as error:
        raise CommandError(f"invalid JSON: {error}") from error
    if not isinstance(command, dict):
        raise CommandError("command must be a JSON object")
    if command.get("robot_id", robot) != robot:
        return None
    cmd_id = command.get("cmd_id")
    if isinstance(cmd_id, bool) or not isinstance(cmd_id, int):
        raise CommandError("'cmd_id' must be an integer")
    action = command.get("action")
    if action not in COMMAND_ACTIONS:
        raise CommandError(f"unknown action {action!r}")
    if action == "stop":
        return cmd_id, action, None
    if action == "task":
        steps = command.get("steps")
        if not isinstance(steps, list) or not steps:
            raise CommandError("'steps' must be a non-empty list")
        return cmd_id, action, steps
    target = command.get("target")
    if not isinstance(target, dict):
        raise CommandError("'target' must be an object")
    step = {"type": "navigate", "x": _number(target, "x"), "y": _number(target, "y"),
            "yaw": _number(target, "yaw", 0.0)}
    return cmd_id, action, [step]
