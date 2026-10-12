"""py_trees task runner for one robot: turns a command into a behaviour tree.

A command is JSON:
  {"goal_id": "42", "steps": [{"type": "navigate", "x": 1.0, "y": 2.0, "yaw": 0.0}]}
Steps run in order; the task succeeds when all of them succeed and aborts at the
first failure. After each step except the last the task reports "waiting" and
does not start the next step until TaskRunner.proceed() is called. Step types are listed in STEP_TYPES (navigate; gimbal_move and
gimbal_rotate take pan_deg/tilt_deg in degrees); types in PLANNED_STEP_TYPES
are accepted by the protocol but rejected until they are implemented.
"""

import json
import math
import time
from dataclasses import dataclass

import py_trees
from py_trees.common import Status

RUNNING, WAITING, CANCELING, SUCCEEDED, ABORTED, CANCELED, REJECTED = (
    "running", "waiting", "canceling", "succeeded", "aborted", "canceled", "rejected")
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


def _gimbal_move(step, index):
    return {"type": "gimbal_move", "pan_deg": _number(step, "pan_deg", index),
            "tilt_deg": _number(step, "tilt_deg", index)}


def _gimbal_rotate(step, index):
    pan = _number(step, "pan_deg", index, 0.0)
    if abs(pan) > 180.0:
        raise TaskError(f"step {index}: |'pan_deg'| must be <= 180 for gimbal_rotate")
    return {"type": "gimbal_rotate", "pan_deg": pan, "tilt_deg": _number(step, "tilt_deg", index, 0.0)}


STEP_TYPES.update({"gimbal_move": _gimbal_move, "gimbal_rotate": _gimbal_rotate})


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


class ActionStep(py_trees.behaviour.Behaviour):
    """Sends one ROS action goal; RUNNING until its result arrives.

    Subclasses build the goal and may refine what counts as success.
    """

    unavailable = "action server is not available"
    rejected = "goal rejected by the action server"
    request_failed = "goal request failed"
    not_succeeded = "action did not succeed"

    def __init__(self, name, node, client, step):
        super().__init__(name)
        self.node, self.client, self.step = node, client, step
        self.send_future = self.result_future = self.goal_handle = None
        self.cancel_requested = False
        self.cancel_future = None
        self.nav_status = None
        self.cancel_error = ""

    def initialise(self):
        self.send_future = self.result_future = self.goal_handle = None
        self.cancel_requested = False
        self.cancel_future = None
        self.nav_status = None
        self.cancel_error = ""
        self.feedback_message = ""
        if not self.client.server_is_ready():
            return
        self.send_future = self.client.send_goal_async(self.build_goal())

    def build_goal(self):
        raise NotImplementedError

    def succeeded(self, result):
        """Whether a result of a goal that ended SUCCEEDED really achieved the step."""
        return True

    def failure_message(self, result):
        return self.not_succeeded

    def update(self):
        if self.send_future is None:
            self.feedback_message = self.unavailable
            return Status.FAILURE
        if self.result_future is None:
            if not self.send_future.done():
                return Status.RUNNING
            if self.send_future.exception() is not None:
                self.feedback_message = f"{self.request_failed}: {self.send_future.exception()}"
                self.cancel_error = self.feedback_message
                return Status.RUNNING if self.cancel_requested else Status.FAILURE
            self.goal_handle = self.send_future.result()
            if not self.goal_handle.accepted:
                self.feedback_message = self.rejected
                return Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
        if self.cancel_requested and self.cancel_future is None and not self.result_future.done():
            self.cancel_future = self.goal_handle.cancel_goal_async()
        if self.cancel_future is not None and self.cancel_future.done():
            error = self.cancel_future.exception()
            if error is not None:
                self.cancel_error = f"cancellation failed: {error}"
            elif not self.cancel_future.result().goals_canceling:
                self.cancel_error = "the action server did not accept the cancellation request"
        if not self.result_future.done():
            return Status.RUNNING
        if self.result_future.exception() is not None:
            self.cancel_error = f"result unavailable: {self.result_future.exception()}"
            self.feedback_message = self.cancel_error
            # A failed result request does not prove the remote goal has ended.
            return Status.RUNNING if self.cancel_requested else Status.FAILURE
        from action_msgs.msg import GoalStatus
        response = self.result_future.result()
        self.nav_status = response.status
        if self.nav_status == GoalStatus.STATUS_SUCCEEDED and self.succeeded(response.result):
            return Status.SUCCESS
        self.feedback_message = self.failure_message(response.result)
        return Status.FAILURE

    def terminate(self, new_status):
        if new_status == Status.INVALID:
            self.request_cancel()

    def request_cancel(self):
        self.cancel_requested = True
        if self.goal_handle is not None and self.goal_handle.accepted and self.cancel_future is None and (
                self.result_future is None or not self.result_future.done()):
            self.cancel_future = self.goal_handle.cancel_goal_async()

    def poll_cancel(self):
        outcome = self.update()
        if outcome == Status.RUNNING:
            return None
        from action_msgs.msg import GoalStatus
        if self.nav_status == GoalStatus.STATUS_CANCELED:
            return CANCELED
        return SUCCEEDED if outcome == Status.SUCCESS else ABORTED


class NavigateToPose(ActionStep):
    """Sends one Nav2 goal."""

    unavailable = "navigate_to_pose action server is not available"
    rejected = "navigation goal rejected by Nav2"
    request_failed = "navigation goal request failed"
    not_succeeded = "navigation did not succeed"

    def __init__(self, node, client, step):
        super().__init__(f"navigate({step['x']:.2f}, {step['y']:.2f})", node, client, step)

    def build_goal(self):
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose as Action
        goal = Action.Goal()
        pose = goal.pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = self.step["x"], self.step["y"]
        pose.pose.orientation.z = math.sin(self.step["yaw"] / 2)
        pose.pose.orientation.w = math.cos(self.step["yaw"] / 2)
        return goal


class GimbalStep(ActionStep):
    """Turns the robot's gimbal; done only when the gimbal reports it reached the pose."""

    action_name = ""
    action_type = ""

    def __init__(self, node, client, step):
        super().__init__(
            f"{step['type']}(pan={step['pan_deg']:.1f}, tilt={step['tilt_deg']:.1f})",
            node, client, step)
        self.unavailable = f"{self.action_name} action server is not available (no gimbal?)"
        self.rejected = f"{self.action_name} goal rejected by the gimbal"
        self.request_failed = f"{self.action_name} goal request failed"
        self.not_succeeded = f"{self.action_name} did not succeed"

    def build_goal(self):
        from slam_sensor_control import action
        return getattr(action, self.action_type).Goal(
            **dict(zip(self.goal_fields, (math.radians(self.step["pan_deg"]),
                                          math.radians(self.step["tilt_deg"])))))

    def succeeded(self, result):
        return bool(result.success)

    def failure_message(self, result):
        return result.message or self.not_succeeded


class GimbalMove(GimbalStep):
    action_name, action_type, goal_fields = "gimbal/move", "GimbalMove", ("pan", "tilt")


class GimbalRotate(GimbalStep):
    action_name, action_type = "gimbal/rotate", "GimbalRotate"
    goal_fields = ("delta_pan", "delta_tilt")


# step type -> (behaviour class, ROS action name relative to the robot namespace)
class ReleaseGate(py_trees.behaviour.Behaviour):
    """Holds the task between two steps until the adapter releases it."""

    def __init__(self, name):
        super().__init__(name)
        self.released = False

    def update(self):
        return Status.SUCCESS if self.released else Status.RUNNING

    def request_cancel(self):
        pass

    def poll_cancel(self):
        return CANCELED

    cancel_error = ""


STEP_BEHAVIOURS = {
    "navigate": (NavigateToPose, "navigate_to_pose"),
    "gimbal_move": (GimbalMove, GimbalMove.action_name),
    "gimbal_rotate": (GimbalRotate, GimbalRotate.action_name),
}


class TaskRunner:
    """Owns the current task's tree; reports changes through on_state(dict)."""

    def __init__(self, step_factory, on_state, cancel_timeout=10.0, clock=time.monotonic):
        if not math.isfinite(cancel_timeout) or cancel_timeout <= 0:
            raise ValueError("cancel_timeout must be a positive finite number")
        self.step_factory, self.on_state = step_factory, on_state
        self.tree = None
        self.state = None
        self.pending = None
        self.cancel_callbacks = []
        self.cancel_timeout, self.clock = cancel_timeout, clock
        self.cancel_failed = False

    @property
    def active(self):
        return self.state is not None and self.state["status"] in (RUNNING, WAITING, CANCELING)

    def proceed(self, goal_id):
        """Release a task waiting after a step. Returns (ok, message)."""
        if not self.active or str(goal_id) != self.task.goal_id:
            return False, "No such active task"
        if self.state["status"] != WAITING:
            return False, "Task is not waiting for release"
        self.tree.root.current_child.released = True
        self._set(RUNNING, self.state["step"] + 1)
        return True, "Released"

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
        if self.pending is not None and task.goal_id == self.pending.goal_id:
            return
        if self.active:
            self._discard_pending()
            if self.cancel_failed:
                self.on_state({"goal_id": task.goal_id, "status": REJECTED,
                               "message": "previous navigation has not ended"})
                return
            self.pending = task
            self._request_cancel()
            return
        self._start(task)

    def _start(self, task):
        self.cancel_failed = False
        self.task = task
        children = []
        for index, step in enumerate(task.steps):
            if index:
                children.append(ReleaseGate(f"release before step {index}"))
            children.append(self.step_factory(step))
        root = py_trees.composites.Sequence(f"task {task.goal_id}", memory=True, children=children)
        self.tree = py_trees.trees.BehaviourTree(root)
        self._set(RUNNING, 0)

    def _discard_pending(self, message="superseded before starting"):
        if self.pending is not None:
            self.on_state({"goal_id": self.pending.goal_id, "status": CANCELED,
                           "step": 0, "steps": len(self.pending.steps), "message": message})
            self.pending = None

    def cancel(self, goal_id=None, on_done=None):
        if not self.active or (goal_id is not None and str(goal_id) != self.task.goal_id):
            if on_done is not None:
                on_done(True, "No active navigation")
            return
        self._discard_pending("canceled before starting")
        if on_done is not None:
            if self.cancel_failed:
                on_done(False, self.state["message"])
            else:
                self.cancel_callbacks.append(on_done)
        self._request_cancel()

    def _request_cancel(self):
        if self.state["status"] == CANCELING:
            return
        self.cancel_failed = False
        self.cancel_started = self.clock()
        self.cancel_child = self.tree.root.current_child
        self._set(CANCELING, self.state["step"])
        if self.cancel_child is not None:
            self.cancel_child.request_cancel()

    def _cancel_tick(self):
        outcome = self.cancel_child.poll_cancel() if self.cancel_child is not None else CANCELED
        if outcome is not None:
            step = self.state["step"]
            # Completing an intermediate goal does not complete the whole task.
            status = CANCELED if outcome == SUCCEEDED and step < len(self.task.steps) - 1 else outcome
            self.tree.root.stop(Status.INVALID)
            self._set(status, step, self.cancel_child.feedback_message if status == ABORTED else "")
            callbacks, self.cancel_callbacks = self.cancel_callbacks, []
            for callback in callbacks:
                callback(True, "Navigation ended")
            pending, self.pending = self.pending, None
            if pending is not None:
                self._start(pending)
            return
        error = self.cancel_child.cancel_error
        if not error and self.clock() - self.cancel_started >= self.cancel_timeout:
            error = "timed out waiting for navigation to end"
        if error and not self.cancel_failed:
            self.cancel_failed = True
            self._set(CANCELING, self.state["step"], error)
            if self.pending is not None:
                self.on_state({"goal_id": self.pending.goal_id, "status": REJECTED,
                               "message": error})
                self.pending = None
            callbacks, self.cancel_callbacks = self.cancel_callbacks, []
            for callback in callbacks:
                callback(False, error)

    def tick(self):
        if not self.active:
            return
        if self.state["status"] == CANCELING:
            self._cancel_tick()
            return
        self.tree.tick()
        root = self.tree.root
        if root.status == Status.SUCCESS:
            self._set(SUCCEEDED, len(self.task.steps) - 1)
        elif root.status == Status.FAILURE:
            failed = next((child for child in root.children if child.status == Status.FAILURE), root)
            self._set(ABORTED, root.children.index(failed) // 2 if failed is not root else 0,
                      failed.feedback_message or f"{failed.name} failed")
        else:
            current = root.current_child
            index = root.children.index(current) if current in root.children else 0
            if isinstance(current, ReleaseGate):
                self._set(WAITING, index // 2)  # step index // 2 just finished
            else:
                self._set(RUNNING, index // 2)
