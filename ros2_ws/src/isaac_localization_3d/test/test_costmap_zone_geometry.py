import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from costmap_filter_editor import CostmapFilterEditor
from costmap_zone_geometry import polygon_cells, rasterize_zones, validate_polygon


SQUARE = [[1.0, 1.0], [3.0, 1.0], [3.0, 3.0], [1.0, 3.0]]


class ZoneGeometryTest(unittest.TestCase):
    def test_polygon_validation(self):
        for points in (
            [], [[0, 0], [1, 1], [2, 2]],
            [[0, 0], [2, 2], [0, 2], [2, 0]],
            [[0, 0], [2, 0], [2, 2], [0, 0]],
            [[0, 0], [1, 0], [float("nan"), 1]],
        ):
            with self.subTest(points=points):
                with self.assertRaises(ValueError):
                    validate_polygon(points)
        self.assertEqual(validate_polygon(SQUARE), SQUARE)

    def test_concave_and_reversed_polygon(self):
        polygon = [[0, 0], [3, 0], [3, 1], [1, 1], [1, 3], [0, 3]]
        rows, cols = polygon_cells(polygon, 4, 4, 1.0, (0, 0))
        self.assertEqual(set(zip(rows, cols)), {(0, 0), (0, 1), (0, 2), (1, 0), (2, 0)})
        reverse = polygon_cells(list(reversed(polygon)), 4, 4, 1.0, (0, 0))
        self.assertEqual(set(zip(rows, cols)), set(zip(*reverse)))

    def test_bounds_and_subcell_zones_are_rejected(self):
        for polygon in (
            [[-1, 0], [1, 0], [0, 1]],
            [[0.01, 0.01], [0.02, 0.01], [0.01, 0.02]],
        ):
            with self.assertRaises(ValueError):
                polygon_cells(polygon, 4, 4, 1.0, (0, 0))

    def test_overlap_uses_most_restrictive_speed_and_clear_releases(self):
        zones = [
            {"id": 1, "kind": "keepout", "points": SQUARE},
            {"id": 2, "kind": "speed", "percent": 50, "points": SQUARE},
            {"id": 3, "kind": "speed", "percent": 25, "points": SQUARE},
        ]
        keepout, speed = rasterize_zones(zones, 4, 4, 1.0, (0, 0))
        self.assertEqual(keepout[1, 1], 100)
        self.assertEqual(speed[1, 1], 25)
        self.assertEqual(speed[0, 0], 0)
        cleared = rasterize_zones([], 4, 4, 1.0, (0, 0))
        self.assertTrue(all(np.count_nonzero(grid) == 0 for grid in cleared))

    def test_invalid_speed_or_duplicate_id_is_rejected(self):
        for percent in (0, 101, 50.5, True):
            with self.assertRaises(ValueError):
                rasterize_zones(
                    [{"id": 1, "kind": "speed", "percent": percent, "points": SQUARE}],
                    4, 4, 1.0, (0, 0),
                )
        zone = {"id": 1, "kind": "keepout", "points": SQUARE}
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            rasterize_zones([zone, zone], 4, 4, 1.0, (0, 0))


class EditorPersistenceTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.state = Path(self.directory.name) / "zones.json"
        rclpy.init(args=["--ros-args", "-p", f"state_file:={self.state}"])
        self.nodes = []

    def tearDown(self):
        for node in self.nodes:
            node.destroy_node()
        rclpy.shutdown()
        self.directory.cleanup()

    def editor(self):
        node = CostmapFilterEditor()
        self.nodes.append(node)
        return node

    @staticmethod
    def map():
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.info.width = message.info.height = 4
        message.info.resolution = 1.0
        message.info.origin.orientation.w = 1.0
        message.data = [0] * 16
        return message

    def test_saved_zones_reload_and_map_mismatch_fails(self):
        editor = self.editor()
        editor.on_map(self.map())
        editor.draft = SQUARE.copy()
        editor.apply("speed", 25)
        saved = json.loads(self.state.read_text())
        self.assertEqual(saved["zones"][0]["percent"], 25)
        editor.destroy_node()
        self.nodes.remove(editor)
        restored = self.editor()
        restored.on_map(self.map())
        self.assertEqual(restored.zones, saved["zones"])
        restored.destroy_node()
        self.nodes.remove(restored)
        other = self.editor()
        changed = self.map()
        changed.data[0] = 100
        with self.assertRaisesRegex(ValueError, "another map"):
            other.on_map(changed)

    def test_save_failure_preserves_previous_state_and_zones(self):
        editor = self.editor()
        editor.on_map(self.map())
        before = self.state.read_text()
        editor.draft = SQUARE.copy()
        with patch.object(editor, "save", side_effect=OSError("Storage unavailable")):
            editor.apply("keepout")
        self.assertEqual(editor.zones, [])
        self.assertEqual(self.state.read_text(), before)
        self.assertEqual(editor.draft, SQUARE)

    def test_invalid_persisted_zones_do_not_publish_empty_masks(self):
        editor = self.editor()
        editor.on_map(self.map())
        state = json.loads(self.state.read_text())
        state["zones"] = [{"id": 1, "kind": "speed", "percent": 0, "points": SQUARE}]
        self.state.write_text(json.dumps(state))
        editor.destroy_node()
        self.nodes.remove(editor)
        restored = self.editor()
        with self.assertRaisesRegex(ValueError, "percentage"):
            restored.on_map(self.map())
