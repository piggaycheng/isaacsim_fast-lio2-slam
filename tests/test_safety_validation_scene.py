import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

from safety_validation_scene import SafetyValidationScene


class SafetyValidationSceneTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.scene = SafetyValidationScene.__new__(SafetyValidationScene)
        self.scene.directory = Path(self.directory.name)
        self.scene.sequence = None
        self.scene.scenario_boxes = []
        self.scene.obstacles = []
        self.scene.motion_started = None
        self.scene.record_telemetry = False
        self.scene.last_telemetry = None
        self.scene.box = Mock()
        self.scene.robot = Mock()
        self.positions = Mock()
        self.positions.numpy.return_value = np.array([[2.0, 3.0, 0.0]])
        self.orientations = Mock()
        self.orientations.numpy.return_value = np.array([[1.0, 0.0, 0.0, 0.0]])
        self.scene.robot.get_world_poses.return_value = (self.positions, self.orientations)

    def request(self, action, **kwargs):
        (self.scene.directory / "request.json").write_text(json.dumps({
            "sequence": 1, "action": action, **kwargs,
        }))
        self.scene.update()
        return json.loads((self.scene.directory / "response.json").read_text())

    def test_ros_front_and_left_are_negative_chassis_axes(self):
        response = self.request("place", offset=[1.0, 0.5], sizes=[0.2, 0.6, 1.2])
        np.testing.assert_allclose(response["center"], [1.0, 2.5, 0.6])
        self.assertAlmostEqual(response["robot_yaw"], math.pi)
        self.scene.box.set_local_scales.assert_called_once_with([[0.2, 0.6, 1.2]])
        self.scene.update()
        self.scene.box.set_local_scales.assert_called_once()

    def test_rotated_chassis_preserves_relative_obstacle_placement(self):
        self.orientations.numpy.return_value = np.array([
            [math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)],
        ])
        response = self.request("place", offset=[1.0, 0.5], sizes=[0.2, 0.6, 1.2])
        np.testing.assert_allclose(response["center"], [2.5, 2.0, 0.6])

    def test_hide_and_invalid_requests(self):
        self.request("hide")
        self.scene.box.set_world_poses.assert_called_once_with(
            positions=[[100.0, 100.0, 0.6]],
        )
        self.scene.sequence = None
        with self.assertRaises(ValueError):
            self.request("place", offset=[1.0, 0.0], sizes=[0.0, 0.6, 1.2])
        with self.assertRaises(ValueError):
            self.request("unsupported")

    def test_motion_uses_simulation_time_and_stops_at_endpoint(self):
        box = Mock()
        self.scene.scenario_boxes = [box]
        self.scene.obstacles = [{
            "center": [2.0, 3.0, 0.6], "velocity": [0.0, -0.5], "duration": 2.0,
        }]
        self.scene.move_obstacles(10.0)
        box.set_world_poses.assert_not_called()
        self.request("start_motion")
        self.scene.update(1.0)
        box.set_world_poses.assert_called_with(positions=[[2.0, 2.5, 0.6]])
        self.scene.update(5.0)
        box.set_world_poses.assert_called_with(positions=[[2.0, 2.0, 0.6]])
        self.scene.sequence = None
        self.request("hide")
        self.assertIsNone(self.scene.motion_started)
        self.assertEqual(self.scene.obstacles, [])
        box.set_world_poses.assert_called_with(positions=[[100.0, 100.0, 0.6]])

    def test_motion_requires_configured_obstacles(self):
        with self.assertRaisesRegex(ValueError, "Configure"):
            self.request("start_motion")

    def test_actual_pose_telemetry_is_throttled_in_simulation_time(self):
        box = Mock()
        positions = Mock()
        positions.numpy.return_value = np.array([[2.0, 3.0, 0.6]])
        box.get_world_poses.return_value = (positions, Mock())
        self.scene.scenario_boxes = [box]
        self.scene.obstacles = [{"center": [2.0, 3.0, 0.6]}]
        self.scene.record_telemetry = True
        self.scene.sequence = 7
        self.scene.update(4.0)
        path = self.scene.directory / "telemetry.json"
        telemetry = json.loads(path.read_text())
        self.assertEqual(telemetry["centers"], [[2.0, 3.0, 0.6]])
        self.assertEqual(telemetry["sequence"], 7)
        self.scene.update(4.01)
        self.assertEqual(json.loads(path.read_text())["sim_time"], 4.0)
        self.scene.update(4.2)
        self.assertEqual(json.loads(path.read_text())["sim_time"], 4.2)

    def test_configure_rotates_positions_and_velocity_into_world_frame(self):
        self.scene.stage = Mock()
        self.scene.scenario_boxes = [Mock()]
        with patch.dict("sys.modules", {
            "isaacsim.core.experimental.objects": Mock(Cube=Mock()),
            "pxr": Mock(UsdPhysics=Mock()),
        }):
            response = self.request("configure", obstacles=[{
                "offset": [2.0, 1.0], "sizes": [0.4, 0.4, 1.2],
                "velocity": [0.0, -0.5], "duration": 4.0,
            }])
        np.testing.assert_allclose(
            response["obstacles"][0]["center"], [0.0, 2.0, 0.6], atol=1e-9,
        )
        np.testing.assert_allclose(response["obstacles"][0]["velocity"], [0.0, 0.5], atol=1e-9)
        self.assertIsNone(self.scene.motion_started)
        self.scene.sequence = None
        with patch.dict("sys.modules", {
            "isaacsim.core.experimental.objects": Mock(Cube=Mock()),
            "pxr": Mock(UsdPhysics=Mock()),
        }), self.assertRaises(ValueError):
            self.request("configure", obstacles=[{
                "offset": [2.0, 1.0], "sizes": [0.4, 0.4, 1.2],
                "velocity": [0.0, float("nan")], "duration": 4.0,
            }])
