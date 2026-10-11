import json
import math
import sys
import threading
import time
import types
import unittest
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/slam_sensor_control/scripts"))
sys.path.insert(0, str(ROOT / "tests"))
import gimbal_protocol as protocol  # noqa: E402
from gimbal_action_server import PAN_JOINT, TILT_JOINT, GimbalActionServer  # noqa: E402
from test_gimbal_action_server import FakeGimbal  # noqa: E402


class FakeMqttClient:
    def __init__(self, *args, **kwargs):
        self.published = []

    def __getattr__(self, name):
        return lambda *args, **kwargs: None

    def is_connected(self):
        return True

    def publish(self, topic, payload, qos=0, retain=False):
        self.published.append((topic, json.loads(payload)))
        return types.SimpleNamespace(wait_for_publish=lambda timeout=None: None)


# The bridge needs paho only for its client; tests drive it through `inbox` instead.
paho = types.ModuleType("paho")
paho_mqtt = types.ModuleType("paho.mqtt")
paho_client = types.ModuleType("paho.mqtt.client")
paho_client.Client = FakeMqttClient
paho_client.CallbackAPIVersion = types.SimpleNamespace(VERSION2=2)
paho.mqtt, paho_mqtt.client = paho_mqtt, paho_client
sys.modules.update({"paho": paho, "paho.mqtt": paho_mqtt, "paho.mqtt.client": paho_client})
from gimbal_mqtt_bridge import GimbalMqttBridge  # noqa: E402


def command(**fields):
    return json.dumps({"cmd_id": 1, **fields})


class GimbalProtocolTest(unittest.TestCase):
    def test_topics(self):
        self.assertEqual(protocol.topics("slam/gimbal", "carter1")["command"],
                         "slam/gimbal/carter1/command")
        self.assertEqual(protocol.topics("slam/gimbal/", "")["result"], "slam/gimbal/result")

    def test_move_converts_degrees_to_radians(self):
        cmd_id, action, pan, tilt = protocol.parse_command(
            command(action="move", pan_deg=90, tilt_deg=-30))
        self.assertEqual((cmd_id, action), (1, "move"))
        self.assertAlmostEqual(pan, math.pi / 2)
        self.assertAlmostEqual(tilt, -math.pi / 6)

    def test_move_requires_both_angles_but_rotate_defaults_to_zero(self):
        with self.assertRaises(protocol.CommandError):
            protocol.parse_command(command(action="move", pan_deg=10))
        _, _, pan, tilt = protocol.parse_command(command(action="rotate", pan_deg=10))
        self.assertAlmostEqual(pan, math.radians(10))
        self.assertEqual(tilt, 0.0)

    def test_cancel_has_no_angles(self):
        self.assertEqual(protocol.parse_command(command(action="cancel")), (1, "cancel", None, None))

    def test_rejects_bad_commands(self):
        for payload in ("not json", "[]", command(action="spin"), json.dumps({"action": "cancel"}),
                        command(action="move", pan_deg="1", tilt_deg=0),
                        command(action="move", pan_deg=True, tilt_deg=0),
                        command(action="move", pan_deg=float("nan"), tilt_deg=0)):
            with self.assertRaises(protocol.CommandError, msg=payload):
                protocol.parse_command(payload)

    def test_cmd_id_of_for_reporting_rejection(self):
        self.assertEqual(protocol.cmd_id_of(command(action="spin")), 1)
        self.assertIsNone(protocol.cmd_id_of("not json"))

    def test_result_payload_reports_pose_in_degrees(self):
        payload = json.loads(protocol.result_payload("r", 3, "succeeded", "ok", (math.pi, 0.0)))
        self.assertAlmostEqual(payload["pan_deg"], 180.0)
        self.assertNotIn("pan_deg", json.loads(protocol.result_payload("r", 3, "failed", pose=(math.nan, 0))))


class GimbalMqttBridgeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.server = GimbalActionServer()
        cls.server.timeout = 1.5
        cls.fake = FakeGimbal()
        cls.bridge = GimbalMqttBridge()
        cls.executor = MultiThreadedExecutor()
        for node in (cls.server, cls.fake, cls.bridge):
            cls.executor.add_node(node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()
        assert cls.wait_for(lambda: cls.bridge.action_clients["move"].server_is_ready()
                            and cls.bridge.action_clients["rotate"].server_is_ready()
                            and cls.bridge.fresh_pose() is not None)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        for node in (cls.server, cls.fake, cls.bridge):
            node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def wait_for(condition, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            time.sleep(0.02)
        return condition()

    def setUp(self):
        self.fake.frozen = False
        self.fake.pan = self.fake.tilt = 0.0
        self.fake.goal = None
        self.bridge.client.published.clear()

    def results(self, cmd_id):
        return [p for t, p in self.bridge.client.published
                if t.endswith("/result") and p["cmd_id"] == cmd_id]

    def request(self, cmd_id, **fields):
        self.bridge.inbox.put(command(**{"action": "move", **fields}).replace('"cmd_id": 1', f'"cmd_id": {cmd_id}'))
        self.assertTrue(self.wait_for(lambda: self.results(cmd_id), timeout=8.0))
        return self.results(cmd_id)[-1]

    def test_move_succeeds_with_degree_pose(self):
        result = self.request(10, pan_deg=45, tilt_deg=20)
        self.assertEqual(result["status"], "succeeded")
        self.assertAlmostEqual(result["pan_deg"], 45.0, delta=1.0)
        self.assertAlmostEqual(result["tilt_deg"], 20.0, delta=1.0)

    def test_rotate_is_relative(self):
        self.request(11, pan_deg=30, tilt_deg=0)
        result = self.request(12, action="rotate", pan_deg=15, tilt_deg=5)
        self.assertEqual(result["status"], "succeeded")
        self.assertAlmostEqual(result["pan_deg"], 45.0, delta=1.0)
        self.assertAlmostEqual(result["tilt_deg"], 5.0, delta=1.0)

    def test_tilt_out_of_limits_is_rejected(self):
        self.assertEqual(self.request(13, pan_deg=0, tilt_deg=120)["status"], "rejected")

    def test_invalid_command_is_rejected_with_cmd_id(self):
        self.assertEqual(self.request(14, pan_deg="x", tilt_deg=0)["status"], "rejected")

    def test_cancel_stops_the_active_move(self):
        self.fake.frozen = True
        self.bridge.inbox.put(command(action="move", pan_deg=90, tilt_deg=0).replace(
            '"cmd_id": 1', '"cmd_id": 15'))
        self.assertTrue(self.wait_for(lambda: self.bridge.active and self.bridge.active[1]))
        self.assertEqual(self.request(16, action="cancel")["status"], "succeeded")
        self.assertEqual(self.wait_for(lambda: self.results(15)) and self.results(15)[-1]["status"],
                         "canceled")

    def test_state_is_published_in_degrees(self):
        self.assertTrue(self.wait_for(lambda: any(
            t.endswith("/state") for t, _ in self.bridge.client.published)))
        state = next(p for t, p in self.bridge.client.published if t.endswith("/state"))
        self.assertIn("pan_deg", state)
        self.assertFalse(state["busy"])


if __name__ == "__main__":
    unittest.main()
