import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from gimbal_control import PAN_JOINT, TILT_JOINT, GimbalController, wrap_to_pi


def run(controller, seconds=5.0, dt=0.01):
    for _ in range(int(seconds / dt)):
        pan, tilt = controller.step(dt)
    return pan, tilt


class TestGimbalController(unittest.TestCase):
    def test_wrap(self):
        self.assertAlmostEqual(wrap_to_pi(3 * math.pi / 2), -math.pi / 2)
        self.assertAlmostEqual(wrap_to_pi(-2 * math.pi), 0.0)

    def test_pan_takes_shortest_path(self):
        controller = GimbalController()
        controller.reset(math.radians(170.0), 0.0)
        controller.accept_joint_state([PAN_JOINT], [math.radians(-170.0)])
        pan, _ = controller.step(0.1)
        self.assertGreater(pan, math.radians(170.0))
        pan, _ = run(controller)
        self.assertAlmostEqual(wrap_to_pi(pan - math.radians(-170.0)), 0.0)
        self.assertAlmostEqual(pan, math.radians(190.0))

    def test_pan_is_continuous_over_many_turns(self):
        controller = GimbalController()
        controller.reset(0.0, 0.0)
        for goal in (120.0, 240.0, 0.0):
            controller.accept_joint_state([PAN_JOINT], [math.radians(goal)])
            pan, _ = run(controller)
            self.assertAlmostEqual(wrap_to_pi(pan - math.radians(goal)), 0.0)

    def test_speed_limit(self):
        controller = GimbalController(max_speed=1.0)
        controller.reset(0.0, 0.0)
        controller.accept_joint_state([PAN_JOINT, TILT_JOINT], [3.0, 1.0])
        pan, tilt = controller.step(0.1)
        self.assertAlmostEqual(pan, 0.1)
        self.assertAlmostEqual(tilt, 0.1)

    def test_tilt_clamped_to_limits(self):
        controller = GimbalController(tilt_limits=(-0.5, 0.5))
        controller.reset(0.0, 0.0)
        controller.accept_joint_state([TILT_JOINT], [2.0])
        self.assertAlmostEqual(run(controller)[1], 0.5)
        controller.accept_joint_state([TILT_JOINT], [-2.0])
        self.assertAlmostEqual(run(controller)[1], -0.5)

    def test_ignores_unknown_joints_and_keeps_other_axis(self):
        controller = GimbalController()
        controller.reset(0.3, 0.2)
        controller.accept_joint_state(["other", PAN_JOINT], [9.0, 1.0])
        pan, tilt = run(controller)
        self.assertAlmostEqual(pan, 1.0)
        self.assertAlmostEqual(tilt, 0.2)

    def test_rejects_invalid_commands(self):
        controller = GimbalController()
        with self.assertRaises(ValueError):
            controller.accept_joint_state([PAN_JOINT], [float("nan")])
        with self.assertRaises(ValueError):
            controller.accept_joint_state([PAN_JOINT, TILT_JOINT], [0.0])


if __name__ == "__main__":
    unittest.main()
