import json
import sys
import unittest
from pathlib import Path

import py_trees
from py_trees.common import Status

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_fleet_bridge/scripts"
))
from task_bt import (  # noqa: E402
    ABORTED, CANCELED, REJECTED, RUNNING, SUCCEEDED, TaskError, TaskRunner, parse_command,
)


class FakeStep(py_trees.behaviour.Behaviour):
    """Takes step x ticks (default 1); step y < 0 makes it fail. Records interruptions."""

    def __init__(self, step, log):
        super().__init__(step["type"])
        self.ticks = max(1, int(step["x"]))
        self.outcome = Status.FAILURE if step["y"] < 0 else Status.SUCCESS
        self.log = log
        self.count = 0

    def initialise(self):
        self.count = 0

    def update(self):
        self.count += 1
        if self.count < self.ticks:
            return Status.RUNNING
        if self.outcome == Status.FAILURE:
            self.feedback_message = "boom"
        return self.outcome

    def terminate(self, new_status):
        if new_status == Status.INVALID:
            self.log.append(f"interrupted {self.name}")


def command(goal_id="1", ticks=1):
    return json.dumps({"goal_id": goal_id, "steps": [{"type": "navigate", "x": ticks, "y": 2}]})


class ParseTest(unittest.TestCase):
    def test_navigate_defaults_yaw(self):
        task = parse_command(command(goal_id=7))
        self.assertEqual((task.goal_id, task.steps), ("7", [
            {"type": "navigate", "x": 1.0, "y": 2.0, "yaw": 0.0}]))

    def test_invalid_commands_are_rejected(self):
        bad = [
            "not json", "[]",
            json.dumps({"steps": [{"type": "navigate", "x": 0, "y": 0}]}),
            json.dumps({"goal_id": "1", "steps": []}),
            json.dumps({"goal_id": "1", "steps": [{"type": "navigate", "x": "a", "y": 0}]}),
            json.dumps({"goal_id": "1", "steps": [{"type": "navigate", "x": 0, "y": 0, "yaw": float("nan")}]}),
            json.dumps({"goal_id": "1", "steps": [{"type": "fly"}]}),
            json.dumps({"goal_id": "1", "steps": [{"type": "rotate", "yaw": 1}]}),
            json.dumps({"goal_id": "1", "steps": [{"type": "take_photo"}]}),
        ]
        for payload in bad:
            with self.assertRaises(TaskError, msg=payload):
                parse_command(payload)


class RunnerTest(unittest.TestCase):
    def setUp(self):
        self.states, self.log = [], []
        self.runner = TaskRunner(lambda step: FakeStep(step, self.log), self.states.append)

    def statuses(self):
        return [(s["goal_id"], s["status"]) for s in self.states]

    def run_ticks(self, count=10):
        for _ in range(count):
            self.runner.tick()

    def test_steps_run_in_order_then_succeed(self):
        self.runner.submit(json.dumps({"goal_id": "1", "steps": [
            {"type": "navigate", "x": 2, "y": 0}, {"type": "navigate", "x": 1, "y": 1}]}))
        self.run_ticks()
        self.assertEqual(self.statuses(), [("1", RUNNING), ("1", SUCCEEDED)])
        self.assertEqual(self.states[-1]["steps"], 2)

    def test_failure_aborts_with_message_and_step(self):
        self.runner.submit(json.dumps({"goal_id": "1", "steps": [
            {"type": "navigate", "x": 0, "y": 0},
            {"type": "navigate", "x": 1, "y": -1}]}))
        self.run_ticks()
        self.assertEqual(self.states[-1]["status"], ABORTED)
        self.assertEqual((self.states[-1]["step"], self.states[-1]["message"]), (1, "boom"))

    def test_new_goal_cancels_running_one(self):
        self.runner.submit(command("1", ticks=100))
        self.run_ticks(3)
        self.runner.submit(command("2"))
        self.run_ticks()
        self.assertEqual(self.statuses(), [
            ("1", RUNNING), ("1", CANCELED), ("2", RUNNING), ("2", SUCCEEDED)])
        self.assertEqual(self.log, ["interrupted navigate"])

    def test_duplicate_goal_id_is_ignored(self):
        self.runner.submit(command("1", ticks=100))
        self.runner.submit(command("1", ticks=100))
        self.assertEqual(self.statuses(), [("1", RUNNING)])

    def test_cancel_matches_goal_id(self):
        self.runner.submit(command("1", ticks=100))
        self.runner.cancel("other")
        self.assertTrue(self.runner.active)
        self.runner.cancel("1")
        self.assertEqual(self.states[-1]["status"], CANCELED)
        self.run_ticks()
        self.assertEqual(self.states[-1]["status"], CANCELED)

    def test_rejected_command_keeps_running_task(self):
        self.runner.submit(command("1", ticks=100))
        self.runner.submit(json.dumps({"goal_id": "2", "steps": [{"type": "take_photo"}]}))
        self.assertEqual(self.states[-1]["status"], REJECTED)
        self.assertEqual(self.states[-1]["goal_id"], "2")
        self.assertTrue(self.runner.active)
        self.assertEqual(self.runner.state["goal_id"], "1")


if __name__ == "__main__":
    unittest.main()
