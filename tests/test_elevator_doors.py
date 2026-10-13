import ast
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from elevator_doors import ElevatorDoors


class ElevatorDoorsTest(unittest.TestCase):
    def setUp(self):
        self.doors = ElevatorDoors.__new__(ElevatorDoors)
        self.doors.cycle = True
        self.doors.fraction = 0.0
        self.doors.target_open = False
        self.doors.started = None
        self.doors.last_time = None
        self.doors._apply = Mock()

    def test_missing_office_doors_fail_explicitly(self):
        stage = Mock()
        stage.Traverse.return_value = []
        with patch.dict("sys.modules", {"pxr": Mock()}):
            with self.assertRaisesRegex(RuntimeError, "No Office skeletal elevator doors"):
                ElevatorDoors(stage)

    def test_cycle_endpoints_and_interpolation(self):
        self.doors.update(100.0)
        for elapsed, fraction in (
            (4.0, 0.0), (5.0, 0.0), (6.0, 0.5), (7.0, 1.0),
            (11.0, 1.0), (12.0, 1.0), (13.0, 0.5), (14.0, 0.0),
            (20.0, 0.5),
        ):
            with self.subTest(elapsed=elapsed):
                self.doors.update(100.0 + elapsed)
                self.assertAlmostEqual(self.doors.fraction, fraction)

    def test_pause_and_manual_reversal_are_continuous(self):
        self.doors.cycle = False
        self.doors.update(0.0)
        self.doors.toggle()
        self.doors.update(1.0)
        self.assertEqual(self.doors.fraction, 0.5)
        self.doors.update(1.0)
        self.assertEqual(self.doors.fraction, 0.5)
        self.doors.toggle()
        self.doors.update(1.5)
        self.assertEqual(self.doors.fraction, 0.25)
        self.doors.update(10.0)
        self.assertEqual(self.doors.fraction, 0.0)
        self.doors.toggle()
        self.doors.update(20.0)
        self.assertEqual(self.doors.fraction, 1.0)

    def test_toggle_stops_cycle_without_jumping(self):
        self.doors.update(0.0)
        self.doors.update(6.0)
        self.doors.toggle()
        self.assertFalse(self.doors.cycle)
        self.assertEqual(self.doors.fraction, 0.5)
        self.doors.update(6.5)
        self.assertEqual(self.doors.fraction, 0.25)

    def test_toggle_before_first_frame_is_not_lost(self):
        self.doors.toggle()
        self.doors.update(10.0)
        self.doors.update(11.0)
        self.assertEqual(self.doors.fraction, 0.5)

    def test_clock_reset_restores_closed_geometry(self):
        self.doors.update(0.0)
        self.doors.update(7.0)
        self.doors._apply.reset_mock()
        self.doors.update(0.0)
        self.assertEqual(self.doors.fraction, 0.0)
        self.doors._apply.assert_called_once()

    def test_invalid_time_is_rejected(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.doors.update(value)

    def test_keyboard_toggles_once_per_press_and_preserves_drive_keys(self):
        source = Path(__file__).resolve().parents[1] / "scripts/standalone.py"
        callback = next(
            node for node in ast.parse(source.read_text()).body
            if isinstance(node, ast.FunctionDef) and node.name == "on_keyboard_event"
        )
        events = SimpleNamespace(KEY_PRESS=1, KEY_RELEASE=2)
        controller = Mock()
        namespace = {
            "carb": SimpleNamespace(input=SimpleNamespace(KeyboardEventType=events)),
            "pressed_keys": set(), "elevator_doors": controller,
        }
        exec(compile(ast.Module(body=[callback], type_ignores=[]), str(source), "exec"), namespace)
        event = SimpleNamespace(input=SimpleNamespace(name="E"), type=events.KEY_PRESS)
        handler = namespace["on_keyboard_event"]
        handler(event)
        handler(event)
        controller.toggle.assert_called_once()
        event.type = events.KEY_RELEASE
        handler(event)
        event.type = events.KEY_PRESS
        handler(event)
        self.assertEqual(controller.toggle.call_count, 2)
        event.input.name = "W"
        handler(event)
        self.assertIn("W", namespace["pressed_keys"])
        namespace["elevator_doors"] = None
        event.input.name = "E"
        event.type = events.KEY_RELEASE
        handler(event)
        event.type = events.KEY_PRESS
        handler(event)
        self.assertEqual(controller.toggle.call_count, 2)


if __name__ == "__main__":
    unittest.main()
