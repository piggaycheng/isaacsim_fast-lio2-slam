import tempfile
import unittest
from pathlib import Path

import numpy as np

import pcd2pgm


class Pcd2PgmTest(unittest.TestCase):
    def test_loads_binary_pcd_and_writes_nav2_map(self):
        points = np.array(
            [(0.0, 0.0, 0.5, 1.0), (1.0, 1.0, 1.0, 2.0), (2.0, 2.0, 3.0, 3.0)],
            dtype="<f4",
        )
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            pcd_path = directory / "input.pcd"
            header = (
                "# .PCD v0.7\n"
                "VERSION 0.7\n"
                "FIELDS x y z intensity\n"
                "SIZE 4 4 4 4\n"
                "TYPE F F F F\n"
                "COUNT 1 1 1 1\n"
                "WIDTH 3\n"
                "HEIGHT 1\n"
                "POINTS 3\n"
                "DATA binary\n"
            )
            with pcd_path.open("wb") as stream:
                stream.write(header.encode("ascii"))
                stream.write(points.tobytes())

            cloud = pcd2pgm.load_xyz(pcd_path)
            occupied, origin, selected = pcd2pgm.rasterize(
                cloud, 0.5, 0.1, 2.0, 0.0, 1, 0.0
            )
            pgm_path, yaml_path = pcd2pgm.write_map(
                directory / "map", occupied, origin, 0.5, "free"
            )

            self.assertEqual((3, 3), occupied.shape)
            self.assertEqual(2, selected)
            self.assertEqual(2, int(occupied.sum()))
            self.assertTrue(pgm_path.read_bytes().startswith(b"P5\n3 3\n255\n"))
            self.assertIn("origin: [0, 0, 0.0]", yaml_path.read_text())

    def test_inflates_obstacles_circularly(self):
        occupied = np.zeros((5, 5), dtype=bool)
        occupied[2, 2] = True
        inflated = pcd2pgm.inflate_obstacles(occupied, 1)
        self.assertEqual(5, int(inflated.sum()))


if __name__ == "__main__":
    unittest.main()
