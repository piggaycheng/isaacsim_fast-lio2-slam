import sys
import unittest
from pathlib import Path

import numpy as np
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from xyz_cloud import make_point_cloud


class XYZCloudTest(unittest.TestCase):
    def test_xyz_from_strided_float64_data(self):
        header = Header(frame_id="map")
        header.stamp.sec = 42
        source = np.arange(20, dtype=np.float64).reshape(2, 10)[:, 1:4]
        msg = make_point_cloud(header, source)

        self.assertEqual(msg.header, header)
        self.assertEqual([(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
                         [("x", 0, PointField.FLOAT32, 1),
                          ("y", 4, PointField.FLOAT32, 1),
                          ("z", 8, PointField.FLOAT32, 1)])
        self.assertEqual((msg.height, msg.width, msg.point_step, msg.row_step),
                         (1, 2, 12, 24))
        self.assertFalse(msg.is_bigendian)
        self.assertTrue(msg.is_dense)
        np.testing.assert_array_equal(np.frombuffer(msg.data, dtype="<f4").reshape(-1, 3), source)

    def test_intensity_is_preserved_when_present(self):
        points = np.array([[1, 2, 3, 7], [4, 5, 6, 8]], dtype=np.float32)
        msg = make_point_cloud(Header(), points)

        self.assertEqual([f.name for f in msg.fields], ["x", "y", "z", "intensity"])
        self.assertEqual((msg.point_step, msg.row_step, len(msg.data)), (16, 32, 32))
        np.testing.assert_array_equal(np.frombuffer(msg.data, dtype="<f4").reshape(-1, 4), points)

    def test_empty_and_nonfinite_clouds(self):
        empty = make_point_cloud(Header(), np.empty((0, 3)))
        self.assertEqual((empty.width, empty.row_step, len(empty.data)), (0, 0, 0))
        self.assertTrue(empty.is_dense)
        cloud = make_point_cloud(Header(), np.array([[1.0, np.nan, 3.0]]))
        self.assertFalse(cloud.is_dense)

    def test_invalid_shape_is_rejected(self):
        for points in (np.empty((1, 2)), np.empty((1, 5)), np.empty(3)):
            with self.subTest(shape=points.shape), self.assertRaises(ValueError):
                make_point_cloud(Header(), points)


if __name__ == "__main__":
    unittest.main()
