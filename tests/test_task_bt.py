import json
import sys
import unittest
import types
from concurrent.futures import Future
from pathlib import Path
from unittest.mock import patch

import py_trees
from py_trees.common import Status

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_fleet_bridge/scripts"
))
from task_bt import (  # noqa: E402
    ABORTED, CANCELED, CANCELING, REJECTED, RUNNING, SUCCEEDED, NavigateToPose,
    TaskError, TaskRunner, parse_command,
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

    def request_cancel(self):
        pass

    def poll_cancel(self):
        return CANCELED

    cancel_error = ""


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
            ("1", RUNNING), ("1", CANCELING), ("1", CANCELED),
            ("2", RUNNING), ("2", SUCCEEDED)])
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
        self.assertEqual(self.states[-1]["status"], CANCELING)
        self.run_ticks()
        self.assertEqual(self.states[-1]["status"], CANCELED)

    def test_rejected_command_keeps_running_task(self):
        self.runner.submit(command("1", ticks=100))
        self.runner.submit(json.dumps({"goal_id": "2", "steps": [{"type": "take_photo"}]}))
        self.assertEqual(self.states[-1]["status"], REJECTED)
        self.assertEqual(self.states[-1]["goal_id"], "2")
        self.assertTrue(self.runner.active)
        self.assertEqual(self.runner.state["goal_id"], "1")


class CancellationTest(unittest.TestCase):
    def setUp(self):
        self.states = []
        self.time = 0.0
        self.client = types.SimpleNamespace(
            server_is_ready=lambda: True, send_goal_async=lambda goal: self.send)
        self.send, self.result, self.cancel_response = Future(), Future(), Future()
        self.cancel_calls = 0
        self.handle = types.SimpleNamespace(
            accepted=True, get_result_async=lambda: self.result,
            cancel_goal_async=self.cancel_goal)
        pose = lambda: types.SimpleNamespace(
            header=types.SimpleNamespace(), pose=types.SimpleNamespace(
                position=types.SimpleNamespace(), orientation=types.SimpleNamespace()))
        modules = {
            "geometry_msgs.msg": types.SimpleNamespace(PoseStamped=pose),
            "nav2_msgs.action": types.SimpleNamespace(
                NavigateToPose=types.SimpleNamespace(Goal=types.SimpleNamespace)),
            "action_msgs.msg": types.SimpleNamespace(GoalStatus=types.SimpleNamespace(
                STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6)),
        }
        self.modules = patch.dict(sys.modules, modules)
        self.modules.start()
        self.addCleanup(self.modules.stop)
        node = types.SimpleNamespace(get_clock=lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: None)))
        self.runner = TaskRunner(
            lambda step: NavigateToPose(node, self.client, step), self.states.append,
            clock=lambda: self.time)
        self.runner.submit(command("1", 100))
        self.runner.tick()

    def cancel_goal(self):
        self.cancel_calls += 1
        return self.cancel_response

    def accept(self):
        self.send.set_result(self.handle)
        self.runner.tick()

    def finish(self, status):
        self.result.set_result(types.SimpleNamespace(status=status))
        self.runner.tick()

    def test_cancel_before_goal_accepted_waits_for_final_result(self):
        self.runner.submit(command("2"))
        self.assertEqual(self.runner.state["status"], CANCELING)
        self.assertEqual(self.runner.task.goal_id, "1")
        self.accept()
        self.assertEqual(self.cancel_calls, 1)
        self.cancel_response.set_result(types.SimpleNamespace(goals_canceling=[1]))
        self.runner.tick()
        self.assertEqual(self.runner.task.goal_id, "1")
        self.finish(5)
        self.assertEqual([(s["goal_id"], s["status"]) for s in self.states], [
            ("1", RUNNING), ("1", CANCELING), ("1", CANCELED), ("2", RUNNING)])

    def test_stop_callback_waits_for_navigation_end(self):
        self.accept()
        done = []
        self.runner.cancel(on_done=lambda *args: done.append(args))
        self.runner.tick()
        self.assertEqual(done, [])
        self.finish(5)
        self.assertEqual(done, [(True, "Navigation ended")])

    def test_success_wins_cancel_race(self):
        self.accept()
        self.runner.cancel()
        self.finish(4)
        self.assertEqual(self.runner.state["status"], SUCCEEDED)

    def test_abort_is_not_reported_as_canceled(self):
        self.accept()
        self.runner.cancel()
        self.finish(6)
        self.assertEqual(self.runner.state["status"], ABORTED)

    def test_intermediate_success_does_not_start_next_step(self):
        self.accept()
        self.finish(5)
        self.send, self.result, self.cancel_response = Future(), Future(), Future()
        self.runner.submit(json.dumps({"goal_id": "2", "steps": [
            {"type": "navigate", "x": 1, "y": 0},
            {"type": "navigate", "x": 2, "y": 0}]}))
        self.runner.tick()
        self.accept()
        self.runner.cancel()
        self.finish(4)
        self.assertEqual(self.runner.state["status"], CANCELED)
        self.assertEqual(self.runner.state["step"], 0)

    def test_cancel_rejected_blocks_replacement_until_goal_ends(self):
        self.accept()
        self.runner.submit(command("2"))
        self.cancel_response.set_result(types.SimpleNamespace(goals_canceling=[]))
        self.runner.tick()
        self.assertEqual(self.states[-1]["status"], REJECTED)
        self.assertEqual(self.runner.state["status"], CANCELING)
        self.runner.submit(command("3"))
        self.assertEqual(self.states[-1]["goal_id"], "3")
        self.assertEqual(self.states[-1]["status"], REJECTED)
        self.finish(4)
        self.assertEqual(self.runner.task.goal_id, "1")
        self.assertEqual(self.runner.state["status"], SUCCEEDED)

    def test_timeout_fails_stop_but_keeps_tracking_navigation(self):
        done = []
        self.runner.cancel(on_done=lambda *args: done.append(args))
        self.time = 10.0
        self.runner.tick()
        self.assertFalse(done[0][0])
        self.assertIn("timed out", done[0][1])
        self.assertTrue(self.runner.active)
        self.accept()
        self.finish(5)
        self.assertEqual(len(done), 1)
        self.assertEqual(self.runner.state["status"], CANCELED)

    def test_stop_discards_waiting_replacement(self):
        self.accept()
        self.runner.submit(command("2"))
        self.runner.cancel()
        self.finish(5)
        self.assertEqual(self.runner.task.goal_id, "1")
        self.assertIsNone(self.runner.pending)
        self.assertIn(("2", CANCELED),
                      [(s["goal_id"], s["status"]) for s in self.states])

    def test_latest_replacement_wins_and_duplicate_does_not_restart(self):
        self.runner.submit(command("2"))
        self.runner.submit(command("2"))
        self.runner.submit(command("3"))
        self.assertEqual(self.runner.pending.goal_id, "3")
        self.assertEqual(sum(s["goal_id"] == "2" for s in self.states), 1)
        self.accept()
        self.finish(5)
        self.assertEqual(self.runner.task.goal_id, "3")

    def test_result_exception_does_not_claim_navigation_ended(self):
        self.accept()
        self.runner.cancel()
        self.result.set_exception(RuntimeError("connection lost"))
        self.runner.tick()
        self.assertEqual(self.runner.state["status"], CANCELING)
        self.assertIn("connection lost", self.runner.state["message"])

    def test_cancel_exception_is_reported(self):
        self.accept()
        done = []
        self.runner.cancel(on_done=lambda *args: done.append(args))
        self.cancel_response.set_exception(RuntimeError("cancel service failed"))
        self.runner.tick()
        self.assertFalse(done[0][0])
        self.assertIn("cancel service failed", done[0][1])

    def test_rejected_navigation_goal_ends_without_cancel_request(self):
        self.runner.submit(command("2"))
        self.handle.accepted = False
        self.accept()
        self.assertEqual(self.cancel_calls, 0)
        self.assertEqual(self.states[-2]["status"], ABORTED)
        self.assertEqual(self.runner.task.goal_id, "2")

    def test_cancel_before_first_tick_does_not_send_goal(self):
        self.accept()
        self.finish(4)
        self.runner.submit(command("2"))
        self.runner.cancel()
        self.runner.tick()
        self.assertEqual(self.runner.state["status"], CANCELED)
        self.assertIsNone(self.runner.tree.root.children[0].send_future)

    def test_invalid_cancel_timeout(self):
        for timeout in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                TaskRunner(lambda step: None, self.states.append, cancel_timeout=timeout)

    def test_goal_request_exception_does_not_release_replacement(self):
        self.runner.submit(command("2"))
        self.send.set_exception(RuntimeError("response lost"))
        self.runner.tick()
        self.assertEqual(self.runner.task.goal_id, "1")
        self.assertEqual(self.runner.state["status"], CANCELING)
        self.assertIn("response lost", self.runner.state["message"])
        self.assertEqual(self.states[-1]["status"], REJECTED)

    def test_new_task_can_be_canceled_after_previous_timeout_resolves(self):
        self.runner.cancel()
        self.time = 10.0
        self.runner.tick()
        self.accept()
        self.finish(5)
        self.runner.submit(command("2"))
        done = []
        self.runner.cancel(on_done=lambda *args: done.append(args))
        self.assertEqual(done, [])
        self.runner.tick()
        self.assertEqual(done, [(True, "Navigation ended")])


if __name__ == "__main__":
    unittest.main()
