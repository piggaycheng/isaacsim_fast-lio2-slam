import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np

from safety_validation_scene import SafetyValidationScene


class SafetyValidationSceneTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.scene = SafetyValidationScene.__new__(SafetyValidationScene)
        self.scene.directory = Path(self.directory.name)
        self.scene.sequence = None
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
