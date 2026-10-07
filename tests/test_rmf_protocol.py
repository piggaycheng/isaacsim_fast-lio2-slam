import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_fleet_bridge/scripts"
))
import rmf_protocol as rmf  # noqa: E402


def command(**fields):
    return json.dumps({"robot_id": "r1", "cmd_id": 7, **fields})


class RmfProtocolTest(unittest.TestCase):
    def test_topics_and_client_id(self):
        self.assertEqual(rmf.topics("f", "r1")["heartbeat"], "rmf/f/robot/r1/heartbeat")
        self.assertEqual(rmf.client_id("f", "r1"), "amr_f_r1")

    def test_heartbeat_has_standard_fields_and_wrapped_yaw(self):
        payload = json.loads(rmf.heartbeat_payload("r1", (1.0, 2.0, 7.0), 130, "idle", None))
        self.assertEqual(set(payload), {"robot_id", "x", "y", "yaw", "battery", "status",
                                        "current_cmd_id"})
        self.assertLessEqual(abs(payload["yaw"]), 3.1416)
        self.assertEqual(payload["battery"], 100.0)
        self.assertIsNone(payload["current_cmd_id"])

    def test_register_omits_unset_optionals(self):
        payload = json.loads(rmf.register_payload(
            "f", "r1", (0, 0, 0), "L1", specs={"footprint_radius": 0.3, "max_linear_velocity": None}))
        self.assertEqual(payload["specs"], {"footprint_radius": 0.3})
        self.assertNotIn("default_charger", payload)
        self.assertNotIn("waypoint_name", payload["initial_location"])

    def test_navigate_and_dock_become_one_navigate_step(self):
        target = {"x": 1, "y": 2, "yaw": 0.5}
        for action in ("navigate", "dock"):
            cmd_id, parsed, steps = rmf.parse_command(command(action=action, target=target), "r1")
            self.assertEqual((cmd_id, parsed), (7, action))
            self.assertEqual(steps, [{"type": "navigate", "x": 1.0, "y": 2.0, "yaw": 0.5}])

    def test_stop_and_task(self):
        self.assertEqual(rmf.parse_command(command(action="stop"), "r1"), (7, "stop", None))
        steps = [{"type": "navigate", "x": 0, "y": 0}]
        self.assertEqual(rmf.parse_command(command(action="task", steps=steps), "r1"),
                         (7, "task", steps))

    def test_other_robot_is_ignored(self):
        payload = json.dumps({"robot_id": "r2", "cmd_id": 1, "action": "stop"})
        self.assertIsNone(rmf.parse_command(payload, "r1"))

    def test_invalid_commands_raise(self):
        for payload in ("nope", "[]", command(action="fly"), command(action="navigate"),
                        command(action="task", steps=[]),
                        json.dumps({"robot_id": "r1", "cmd_id": "7", "action": "stop"}),
                        command(action="navigate", target={"x": "a", "y": 0})):
            with self.assertRaises(rmf.CommandError, msg=payload):
                rmf.parse_command(payload, "r1")

    def test_cmd_id_of_for_rejections(self):
        self.assertEqual(rmf.cmd_id_of(command(action="fly")), 7)
        self.assertIsNone(rmf.cmd_id_of("nope"))

    def test_ack(self):
        self.assertEqual(rmf.parse_ack('{"robot_id":"r1","status":"success","message":"ok"}', "r1"),
                         (True, "ok"))
        self.assertEqual(rmf.parse_ack('{"robot_id":"r1","status":"error","error_code":"X"}', "r1"),
                         (False, "X"))
        self.assertIsNone(rmf.parse_ack('{"robot_id":"r2","status":"success"}', "r1"))


if __name__ == "__main__":
    unittest.main()
