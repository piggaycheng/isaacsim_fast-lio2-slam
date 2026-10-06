"""py_trees task runner for one robot: turns a command into a behaviour tree.

A command is JSON:
  {"goal_id": "42", "steps": [{"type": "navigate", "x": 1.0, "y": 2.0, "yaw": 0.0}]}
Steps run in order; the task succeeds when all of them succeed and aborts at the
first failure. Step types are listed in STEP_TYPES; types in PLANNED_STEP_TYPES
are accepted by the protocol but rejected until they are implemented.
"""

import json
import math
from dataclasses import dataclass

import py_trees
from py_trees.common import Status

RUNNING, SUCCEEDED, ABORTED, CANCELED, REJECTED = (
    "running", "succeeded", "aborted", "canceled", "rejected")
PLANNED_STEP_TYPES = ("rotate", "take_photo")


class TaskError(ValueError):
    pass


@dataclass
class Task:
    goal_id: str
    steps: list


def _number(step, key, index, default=None):
    value = step.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise TaskError(f"step {index}: '{key}' must be a finite number")
    return float(value)


def _navigate(step, index):
    return {"type": "navigate", "x": _number(step, "x", index), "y": _number(step, "y", index),
            "yaw": _number(step, "yaw", index, 0.0)}


STEP_TYPES = {"navigate": _navigate}


def goal_id_of(payload):
    """Best-effort goal_id of a command, for reporting a rejection."""
    try:
        goal_id = json.loads(payload).get("goal_id")
    except (ValueError, AttributeError):
        return None
    return None if goal_id in (None, "") else str(goal_id)


def parse_command(payload):
    try:
        command = json.loads(payload)
    except (ValueError, TypeError) as error:
        raise TaskError(f"invalid JSON: {error}") from error
    if not isinstance(command, dict):
        raise TaskError("command must be a JSON object")
    goal_id = command.get("goal_id")
    if isinstance(goal_id, bool) or not isinstance(goal_id, (str, int)) or goal_id == "":
        raise TaskError("'goal_id' must be a non-empty string")
    steps = command.get("steps")
    if not isinstance(steps, list) or not steps:
        raise TaskError("'steps' must be a non-empty list")
    parsed = []
    for index, step in enumerate(steps):
        kind = step.get("type") if isinstance(step, dict) else None
        if kind in STEP_TYPES:
            parsed.append(STEP_TYPES[kind](step, index))
        elif kind in PLANNED_STEP_TYPES:
            raise TaskError(f"step {index}: '{kind}' is not implemented yet")
        else:
            raise TaskError(f"step {index}: unknown type {kind!r}")
    return Task(str(goal_id), parsed)


class NavigateToPose(py_trees.behaviour.Behaviour):
    """Sends one Nav2 goal; RUNNING until its result arrives."""

    def __init__(self, node, client, step):
        super().__init__(f"navigate({step['x']:.2f}, {step['y']:.2f})")
        self.node, self.client, self.step = node, client, step
        self.send_future = self.result_future = self.goal_handle = None

    def initialise(self):
        self.send_future = self.result_future = self.goal_handle = None
        self.feedback_message = ""
        if not self.client.server_is_ready():
            return
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose as Action
        goal = Action.Goal()
        pose = goal.pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = self.step["x"], self.step["y"]
        pose.pose.orientation.z = math.sin(self.step["yaw"] / 2)
        pose.pose.orientation.w = math.cos(self.step["yaw"] / 2)
        self.send_future = self.client.send_goal_async(goal)

    def update(self):
        if self.send_future is None:
            self.feedback_message = "navigate_to_pose action server is not available"
            return Status.FAILURE
        if self.result_future is None:
            if not self.send_future.done():
                return Status.RUNNING
            self.goal_handle = self.send_future.result()
            if not self.goal_handle.accepted:
                self.feedback_message = "navigation goal rejected by Nav2"
                return Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done():
            return Status.RUNNING
        from action_msgs.msg import GoalStatus
        if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        self.feedback_message = "navigation did not succeed"
        return Status.FAILURE

    def terminate(self, new_status):
        # INVALID means the tree was interrupted while this goal may still run.
        if new_status == Status.INVALID and self.goal_handle is not None and (
                self.result_future is None or not self.result_future.done()):
            self.goal_handle.cancel_goal_async()


class TaskRunner:
    """Owns the current task's tree; reports changes through on_state(dict)."""

    def __init__(self, step_factory, on_state):
        self.step_factory, self.on_state = step_factory, on_state
        self.tree = None
        self.state = None

    @property
    def active(self):
        return self.state is not None and self.state["status"] == RUNNING

    def _set(self, status, step, message=""):
        state = {"goal_id": self.task.goal_id, "status": status, "step": step,
                 "steps": len(self.task.steps), "message": message}
        changed = state != self.state
        self.state = state
        if changed:
            self.on_state(state)

    def submit(self, payload):
        try:
            task = parse_command(payload)
        except TaskError as error:
            self.on_state({"goal_id": goal_id_of(payload), "status": REJECTED, "message": str(error)})
            return
        if self.active and task.goal_id == self.task.goal_id:
            return
        self.cancel()
        self.task = task
        root = py_trees.composites.Sequence(
            f"task {task.goal_id}", memory=True,
            children=[self.step_factory(step) for step in task.steps])
        self.tree = py_trees.trees.BehaviourTree(root)
        self._set(RUNNING, 0)

    def cancel(self, goal_id=None):
        if not self.active or (goal_id is not None and str(goal_id) != self.task.goal_id):
            return
        self.tree.root.stop(Status.INVALID)
        self._set(CANCELED, self.state["step"], "canceled by request")

    def tick(self):
        if not self.active:
            return
        self.tree.tick()
        root = self.tree.root
        if root.status == Status.SUCCESS:
            self._set(SUCCEEDED, len(self.task.steps) - 1)
        elif root.status == Status.FAILURE:
            failed = next((child for child in root.children if child.status == Status.FAILURE), root)
            self._set(ABORTED, root.children.index(failed) if failed is not root else 0,
                      failed.feedback_message or f"{failed.name} failed")
        else:
            current = root.current_child
            self._set(RUNNING, root.children.index(current) if current in root.children else 0)
