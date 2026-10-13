import ast
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from elevator_doors import ElevatorDoors, SHAFT_HOLES, ground_mesh


class GroundMeshTest(unittest.TestCase):
    def test_area_and_winding_exclude_both_shafts(self):
        points, indices = ground_mesh((-25, -25, 25, 25), SHAFT_HOLES)
        area = 0.0
        for offset in range(0, len(indices), 3):
            a, b, c = [points[i] for i in indices[offset:offset + 3]]
            cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            self.assertGreater(cross, 0)
            area += cross / 2
            x, y = sum(p[0] for p in (a, b, c)) / 3, sum(p[1] for p in (a, b, c)) / 3
            self.assertFalse(any(x0 < x < x1 and y0 < y < y1 for x0, y0, x1, y1 in SHAFT_HOLES))
        self.assertEqual(area, 2500 - 8)
        self.assertTrue(all(p[2] == 0 for p in points))

    def test_no_holes_preserves_full_floor(self):
        points, indices = ground_mesh((-1, -1, 1, 1), ())
        self.assertEqual(len(points), 4)
        self.assertEqual(len(indices), 6)

    def test_invalid_bounds_and_holes_are_rejected(self):
        for bounds, holes in (
            ((1, 0, 0, 1), ()), ((0, 0, 1, float("inf")), ()),
            ((0, 0, 1, 1), ((0, 0, 2, 1),)),
            ((0, 0, 1, 1), ((0.5, 0, 0.5, 1),)),
        ):
            with self.subTest(bounds=bounds, holes=holes), self.assertRaises(ValueError):
                ground_mesh(bounds, holes)


class ElevatorDoorsTest(unittest.TestCase):
    def setUp(self):
        self.doors = ElevatorDoors.__new__(ElevatorDoors)
        self.doors.cycle = True
        self.doors.fraction = 0.0
        self.doors.target_open = False
        self.doors.started = None
        self.doors.last_time = None
        self.doors.lift_height = 0.0
        self.doors.lift_target = 0.0
        self.doors.lift_start = 0.0
        self.doors.lift_elapsed = 0.0
        self.doors.lift_pending = False
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

    def test_lift_closes_doors_and_blocks_repeated_commands(self):
        self.doors.cycle = False
        self.doors.update(0.0)
        self.doors.toggle()
        self.doors.update(2.0)
        self.doors.toggle_lift()
        self.doors.toggle_lift()
        self.assertEqual(self.doors.lift_target, -3.3)
        self.doors.toggle()
        self.assertFalse(self.doors.target_open)
        self.doors.update(3.0)
        self.assertEqual(self.doors.fraction, 0.5)
        self.assertEqual(self.doors.lift_height, 0.0)
        self.doors.update(4.0)
        self.assertFalse(self.doors.lift_pending)
        self.doors.update(7.0)
        self.assertAlmostEqual(self.doors.lift_height, -1.65)
        self.doors.toggle()
        self.assertFalse(self.doors.target_open)
        self.doors.update(10.0)
        self.assertEqual(self.doors.lift_height, -3.3)
        self.doors.toggle()
        self.doors.update(12.0)
        self.assertEqual(self.doors.fraction, 1.0)
        self.doors.toggle_lift()
        self.doors.update(14.0)
        self.doors.update(20.0)
        self.assertEqual(self.doors.lift_height, 0.0)

    def test_paused_lift_and_clock_reset(self):
        self.doors.update(0.0)
        self.doors.toggle_lift()
        self.doors.update(0.0)
        self.doors.update(3.0)
        height = self.doors.lift_height
        self.doors.update(3.0)
        self.assertEqual(self.doors.lift_height, height)
        self.doors.update(0.0)
        self.assertEqual(self.doors.lift_height, 0.0)
        self.assertEqual(self.doors.lift_target, 0.0)
        self.assertFalse(self.doors.lift_pending)

    def test_keyboard_toggles_once_per_press_and_preserves_drive_keys(self):
        source = Path(__file__).resolve().parents[1] / "scripts/standalone.py"
        callback = next(
            node for node in ast.parse(source.read_text()).body
            if isinstance(node, ast.FunctionDef) and node.name == "on_keyboard_event"
        )
        events = SimpleNamespace(KEY_PRESS=1, KEY_RELEASE=2)
        for string_input in (False, True):
            with self.subTest(string_input=string_input):
                controller = Mock()
                namespace = {
                    "carb": SimpleNamespace(input=SimpleNamespace(KeyboardEventType=events)),
                    "pressed_keys": set(), "elevator_doors": controller,
                }
                exec(compile(ast.Module(body=[callback], type_ignores=[]), str(source), "exec"), namespace)
                handler = namespace["on_keyboard_event"]

                def send(key, event_type=events.KEY_PRESS):
                    input_value = key.lower() if string_input else SimpleNamespace(name=key)
                    handler(SimpleNamespace(input=input_value, type=event_type))

                send("E")
                send("E")
                controller.toggle.assert_called_once()
                send("E", events.KEY_RELEASE)
                send("E")
                self.assertEqual(controller.toggle.call_count, 2)
                send("L")
                send("L")
                controller.toggle_lift.assert_called_once()
                for key in ("W", "Space", "Up", "Down", "Left", "Right"):
                    send(key)
                    self.assertIn(key.upper(), namespace["pressed_keys"])
                    send(key, events.KEY_RELEASE)
                    self.assertNotIn(key.upper(), namespace["pressed_keys"])
                namespace["elevator_doors"] = None
                send("E", events.KEY_RELEASE)
                send("E")
                send("L", events.KEY_RELEASE)
                send("L")
                self.assertEqual(controller.toggle.call_count, 2)
                controller.toggle_lift.assert_called_once()


if __name__ == "__main__":
    unittest.main()
