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

    def test_floor_switch_requires_explicit_target_floor_base_pose(self):
        data = {"floor_name": "B1", "initial_pose": {"x": 1, "y": 2, "z": 0, "yaw": 1.57}}
        parsed = rmf.parse_command(command(action="change_floor", **data), "r1")
        self.assertEqual(parsed, (7, "change_floor", data))
        planar = {"floor_name": "B1", "initial_pose": {"x": 1, "y": 2, "yaw": 0.5}}
        parsed = rmf.parse_command(command(action="change_floor", **planar), "r1")
        self.assertIsNone(parsed[2]["initial_pose"]["z"])
        for invalid in (
            {"floor_name": "../B1", "initial_pose": data["initial_pose"]},
            {"floor_name": "B1", "initial_pose": "elevator"},
            {"floor_name": "B1", "initial_pose": {"x": 0, "yaw": 0}},
            {"floor_name": "B1", "initial_pose": {"x": 0, "y": 0, "z": "low", "yaw": 0}},
            {"floor_name": "B1", "initial_pose": {"x": True, "y": 0, "z": 0, "yaw": 0}},
            {"floor_name": "B1", "initial_pose": {"x": 0, "y": 0, "z": 0, "yaw": float("nan")}},
        ):
            with self.subTest(invalid=invalid), self.assertRaises(rmf.CommandError):
                rmf.parse_command(command(action="change_floor", **invalid), "r1")

    def test_floor_switch_without_initial_pose_requests_manual_relocalization(self):
        for data in ({"floor_name": "B1"}, {"floor_name": "B1", "initial_pose": None}):
            parsed = rmf.parse_command(command(action="change_floor", **data), "r1")
            self.assertEqual(parsed, (7, "change_floor", {"floor_name": "B1", "initial_pose": None}))

    def test_prepare_floor_change_has_no_target_pose_or_resume_flag(self):
        self.assertEqual(
            rmf.parse_command(command(action="prepare_floor_change"), "r1"),
            (7, "prepare_floor_change", None))

    def test_floor_result_and_heartbeat_report_level(self):
        result = json.loads(rmf.command_result_payload(
            "r1", 7, "completed", location=(1, 2, 0), level="B1"))
        self.assertEqual(result["level_name"], "B1")
        heartbeat = json.loads(rmf.heartbeat_payload("r1", (1, 2, 0), 100, "idle", None, "B1"))
        self.assertEqual(heartbeat["level_name"], "B1")

    def test_gimbal_actions_become_one_gimbal_step(self):
        for action in ("gimbal_move", "gimbal_rotate"):
            cmd_id, parsed, steps = rmf.parse_command(
                command(action=action, pan_deg=30, tilt_deg=-10), "r1")
            self.assertEqual((cmd_id, parsed), (7, action))
            self.assertEqual(steps, [{"type": action, "pan_deg": 30, "tilt_deg": -10}])
        self.assertEqual(rmf.parse_command(command(action="gimbal_rotate", pan_deg=5), "r1")[2],
                         [{"type": "gimbal_rotate", "pan_deg": 5}])

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
                         ("success", "ok"))
        self.assertEqual(rmf.parse_ack('{"robot_id":"r1","status":"error","error_code":"X"}', "r1"),
                         ("error", "X"))
        self.assertEqual(
            rmf.parse_ack('{"robot_id":"r1","status":"require_register","message":"m"}', "r1"),
            ("require_register", "m"))
        self.assertIsNone(rmf.parse_ack('{"robot_id":"r2","status":"success"}', "r1"))

    def test_session_identity_is_in_all_uplink_payloads(self):
        payloads = [
            rmf.register_payload("f", "r1", (0, 0, 0), "1F", session_id="boot-a"),
            rmf.heartbeat_payload("r1", (0, 0, 0), 100, "idle", None,
                                  session_id="boot-a", localization_ready=True),
            rmf.command_result_payload("r1", 1, "completed", session_id="boot-a"),
            rmf.deregister_payload("f", "r1", "Shutdown", session_id="boot-a"),
            rmf.offline_payload("r1", session_id="boot-a"),
        ]
        self.assertTrue(all(json.loads(payload)["session_id"] == "boot-a" for payload in payloads))
        self.assertTrue(json.loads(payloads[1])["localization_ready"])
        self.assertIsNone(json.loads(payloads[1])["current_cmd_id"])

    def test_session_aware_parsers_reject_missing_and_foreign_ids(self):
        for session_id in (None, "boot-old"):
            ack = json.dumps({"robot_id": "r1", "session_id": session_id, "status": "success"})
            cmd = command(session_id=session_id, action="stop")
            self.assertIsNone(rmf.parse_ack(ack, "r1", "boot-a"))
            self.assertIsNone(rmf.parse_command(cmd, "r1", "boot-a"))
        self.assertEqual(rmf.parse_command(
            command(session_id="boot-a", action="stop"), "r1", "boot-a"), (7, "stop", None))
        self.assertEqual(rmf.parse_ack(json.dumps({
            "robot_id": "r1", "session_id": "boot-a", "status": "success"}),
            "r1", "boot-a"), ("success", ""))


if __name__ == "__main__":
    unittest.main()
