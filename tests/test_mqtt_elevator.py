import json
import sys
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from elevator_doors import ElevatorDoors, LOWER_LEVEL  # noqa: E402
from mqtt_elevator import LiftController, MqttLift, parse_floors  # noqa: E402


def doors():
    value = ElevatorDoors.__new__(ElevatorDoors)
    value.cycle = False
    value.fraction = 0.0
    value.target_open = False
    value.started = None
    value.last_time = None
    value.lift_height = value.lift_target = value.lift_start = value.lift_elapsed = 0.0
    value.lift_pending = False
    value._apply = Mock()
    return value


class LiftControllerTest(unittest.TestCase):
    def setUp(self):
        self.doors = doors()
        self.lift = LiftController(self.doors, "lift_1", parse_floors("1F:0,B1:-3.3"))
        self.time = 0.0
        self.step(0.0)

    def step(self, seconds):
        self.time += seconds
        self.lift.update()
        self.doors.update(self.time)

    def run_for(self, seconds, dt=0.5):
        for _ in range(int(seconds / dt)):
            self.step(dt)

    def request(self, session="carter1", floor="B1", door="open", kind="agv"):
        self.lift.submit({"session_id": session, "request_type": kind,
                          "destination_floor": floor, "door_state": door})

    def test_parse_floors(self):
        self.assertEqual(parse_floors("1F:0, B1:-3.3"), {"1F": 0.0, "B1": -3.3})
        for text in ("1F", "1F:0", "1F:0,1F:1", "1F:0,B1:x", ":0,B1:1"):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_floors(text)

    def test_invalid_requests_are_rejected_before_the_sim_loop(self):
        for request in (
            [], {"request_type": "taxi"}, {"request_type": "agv", "destination_floor": "2F"},
            {"request_type": "agv", "destination_floor": "B1", "door_state": "ajar"},
            {"request_type": "agv", "destination_floor": "B1", "session_id": 3},
        ):
            with self.subTest(request=request), self.assertRaises(ValueError):
                self.lift.submit(request)
        self.assertEqual(self.lift.pending, [])

    def test_agv_trip_closes_moves_then_opens_at_destination(self):
        self.request(floor="1F")
        self.run_for(3)
        state = self.lift.state()
        self.assertEqual((state["current_floor"], state["door_state"], state["current_mode"],
                          state["session_id"]), ("1F", "open", "agv", "carter1"))
        self.request(floor="B1", door="closed")
        self.step(0.5)
        self.assertEqual(self.lift.state()["door_state"], "moving")
        self.assertEqual(self.lift.state()["motion_state"], "down")
        self.run_for(2)
        self.assertEqual(self.lift.state()["door_state"], "closed")
        self.request(floor="B1", door="open")
        self.run_for(10)
        state = self.lift.state()
        self.assertEqual(self.doors.lift_height, LOWER_LEVEL)
        self.assertEqual((state["current_floor"], state["destination_floor"], state["motion_state"],
                          state["door_state"]), ("B1", "B1", "stopped", "open"))

    def test_other_sessions_wait_for_end_session(self):
        self.request(floor="1F")
        self.request(session="carter2", floor="B1")
        self.run_for(3)
        self.assertEqual(self.lift.state()["session_id"], "carter1")
        self.assertEqual(self.doors.lift_target, 0.0)
        self.lift.submit({"session_id": "carter2", "request_type": "end_session"})
        self.step(0.5)
        self.assertEqual(self.lift.state()["session_id"], "carter1")
        self.lift.submit({"session_id": "carter1", "request_type": "end_session"})
        self.request(session="carter2", floor="B1")
        self.run_for(12)
        self.assertEqual(self.lift.state()["session_id"], "carter2")
        self.assertEqual(self.lift.state()["current_floor"], "B1")

    def test_new_destination_during_travel_waits_for_landing(self):
        self.request(floor="B1")
        self.run_for(4)
        self.assertTrue(self.doors.moving)
        self.request(floor="1F")
        self.run_for(20)
        self.assertEqual(self.doors.lift_height, 0.0)
        self.assertEqual(self.lift.state()["door_state"], "open")


class MqttLiftTest(unittest.TestCase):
    def test_publishes_state_on_change_and_periodically(self):
        client = Mock()
        paho = ModuleType("paho.mqtt.client")
        paho.Client = Mock(return_value=client)
        paho.CallbackAPIVersion = SimpleNamespace(VERSION2=2)
        package = ModuleType("paho.mqtt")
        package.client = paho
        now = [0.0]
        lift = LiftController(doors(), "lift_1", {"1F": 0.0, "B1": -3.3})
        with patch.dict(sys.modules, {"paho": ModuleType("paho"), "paho.mqtt": package,
                                      "paho.mqtt.client": paho}):
            transport = MqttLift(lift, "broker", 1883, "rmf/lift/", clock=lambda: now[0])
        self.assertEqual(json.loads(client.will_set.call_args.args[1])["current_mode"], "offline")
        transport._on_connect(client, None, None, 0, None)
        client.subscribe.assert_called_once_with("rmf/lift/lift_1/request", qos=1)
        transport.update()
        transport.update()
        self.assertEqual(client.publish.call_count, 1)
        topic, payload = client.publish.call_args.args
        self.assertEqual(topic, "rmf/lift/lift_1/state")
        self.assertEqual(json.loads(payload)["available_floors"], ["1F", "B1"])
        now[0] = 0.6
        transport.update()
        self.assertEqual(client.publish.call_count, 2)
        transport._on_message(client, None, SimpleNamespace(payload=b"{bad"))
        transport._on_message(client, None, SimpleNamespace(payload=json.dumps({
            "session_id": "r1", "request_type": "agv", "destination_floor": "B1"}).encode()))
        transport.update()
        self.assertEqual(json.loads(client.publish.call_args.args[1])["session_id"], "r1")
        self.assertEqual(client.publish.call_count, 3)


if __name__ == "__main__":
    unittest.main()
