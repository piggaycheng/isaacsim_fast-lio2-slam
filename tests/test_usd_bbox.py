import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import usd_bbox


class UsdBboxTest(unittest.TestCase):
    def test_yaw_180_flips_x_and_y(self):
        points = np.array([[1.0, 2.0, 3.0]]) @ usd_bbox.yaw_rotation(180)
        np.testing.assert_allclose(points, [[-1.0, -2.0, 3.0]], atol=1e-12)

    def test_yaw_90_expresses_points_in_rotated_frame(self):
        # A point on the +y axis lies on the +x axis of a frame yawed by +90 deg.
        points = np.array([[0.0, 1.0, 0.0]]) @ usd_bbox.yaw_rotation(90)
        np.testing.assert_allclose(points, [[1.0, 0.0, 0.0]], atol=1e-12)

    def test_transform_points_uses_usd_row_vector_convention(self):
        matrix = np.eye(4)
        matrix[3, :3] = [1.0, -2.0, 0.5]
        result = usd_bbox.transform_points([[0, 0, 0], [1, 1, 1]], matrix)
        np.testing.assert_allclose(result, [[1.0, -2.0, 0.5], [2.0, -1.0, 1.5]])

    def test_rect_footprint_is_padded_bounding_box(self):
        points = usd_bbox.box_corners([-0.14, -0.25, 0.0], [0.59, 0.25, 0.55])
        result = usd_bbox.summarize(points, padding=0.06)
        self.assertEqual(result["min"], [-0.14, -0.25, 0.0])
        self.assertEqual(result["size"], [0.73, 0.5, 0.55])
        self.assertEqual(
            result["footprint"], [[0.65, 0.31], [0.65, -0.31], [-0.2, -0.31], [-0.2, 0.31]]
        )
        self.assertEqual(
            usd_bbox.nav2_footprint(result["footprint"]),
            "[[0.65, 0.31], [0.65, -0.31], [-0.2, -0.31], [-0.2, 0.31]]",
        )

    def test_hull_drops_interior_points_and_pads_outward(self):
        square = [[1, 1, 0], [-1, 1, 0], [-1, -1, 0], [1, -1, 0], [0, 0, 0], [0.5, 0.2, 0]]
        hull = usd_bbox.convex_hull_2d(np.array(square, dtype=float))
        self.assertEqual(sorted(map(tuple, hull)), [(-1, -1), (-1, 1), (1, -1), (1, 1)])
        padded = usd_bbox.summarize(square, padding=0.1, shape="hull")["footprint"]
        self.assertEqual(sorted(map(tuple, padded)),
                         [(-1.1, -1.1), (-1.1, 1.1), (1.1, -1.1), (1.1, 1.1)])

    def test_hull_padding_keeps_distance_on_slanted_edges(self):
        triangle = [[0, 0], [2, 0], [0, 2]]
        padded = np.array(usd_bbox.pad_polygon(usd_bbox.convex_hull_2d(
            np.array([[x, y, 0] for x, y in triangle], dtype=float)), 0.1))
        # Both padded hypotenuse vertices lie on x + y = 2 moved outward by exactly 0.1.
        hypotenuse = [p for p in padded if p[0] + p[1] > 1]
        self.assertEqual(len(hypotenuse), 2)
        for x, y in hypotenuse:
            self.assertAlmostEqual((x + y - 2) / math.sqrt(2), 0.1)

    def test_hull_simplification_still_covers_every_point(self):
        angles = np.linspace(0, 2 * math.pi, 200, endpoint=False)
        circle = np.column_stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)])
        result = usd_bbox.summarize(circle, shape="hull", hull_tolerance=0.02, decimals=6)
        self.assertLess(len(result["footprint"]), 40)
        self.assertLess(usd_bbox.outside_distance(result["footprint"], circle), 1e-6)

    def test_empty_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            usd_bbox.summarize(np.empty((0, 3)))

    def test_rejects_negative_padding(self):
        with self.assertRaises(SystemExit):
            usd_bbox.parse_args(["asset.usd", "--padding", "-0.1"])

    def test_default_purposes_skip_collision_guides(self):
        self.assertEqual(usd_bbox.parse_args(["asset.usd"]).purpose, ["default", "render"])


if __name__ == "__main__":
    unittest.main()
