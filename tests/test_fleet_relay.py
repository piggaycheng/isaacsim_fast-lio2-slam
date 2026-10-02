import sys
import unittest
from pathlib import Path

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(
    Path(__file__).resolve().parents[1] / "ros2_ws/src/slam_localization_3d/scripts"
))
from fleet_relay import prefix_frame, prefix_transforms  # noqa: E402


def transform(parent, child):
    message = TransformStamped()
    message.header.frame_id = parent
    message.child_frame_id = child
    return message


class FleetRelayTest(unittest.TestCase):
    def test_robot_frames_are_prefixed_and_map_is_shared(self):
        self.assertEqual(prefix_frame("carter1", "base_link"), "carter1/base_link")
        self.assertEqual(prefix_frame("carter1", "/odom"), "carter1/odom")
        self.assertEqual(prefix_frame("carter1", "map"), "map")
        self.assertEqual(prefix_frame("carter1", "carter1/odom"), "carter1/odom")
        self.assertEqual(prefix_frame("carter1", ""), "")

    def test_two_robot_trees_merge_under_map(self):
        edges = set()
        for namespace in ("carter1", "carter2"):
            message = TFMessage(transforms=[
                transform("map", "odom"), transform("odom", "base_link"),
            ])
            edges |= {(t.header.frame_id, t.child_frame_id)
                      for t in prefix_transforms(namespace, message).transforms}
        self.assertEqual(edges, {
            ("map", "carter1/odom"), ("carter1/odom", "carter1/base_link"),
            ("map", "carter2/odom"), ("carter2/odom", "carter2/base_link"),
        })


if __name__ == "__main__":
    unittest.main()
