"""Run in the sourced ROS container with an isolated ROS_DOMAIN_ID other than 0 or 10."""

import os
import struct
import subprocess
import time
import unittest

import rclpy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField


@unittest.skipUnless(
    os.environ.get("ROS_DOMAIN_ID") not in (None, "", "0", "10"),
    "Run adapter integration tests in an isolated ROS_DOMAIN_ID other than 0 or 10",
)
class PointCloud2ToLivoxRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_compact_and_legacy_clouds_produce_identical_livox_points(self):
        node = rclpy.create_node("pointcloud_adapter_probe")
        self.addCleanup(node.destroy_node)
        output = []
        node.create_subscription(
            CustomMsg, "/adapter_probe/output", output.append, 10,
        )
        publisher = node.create_publisher(PointCloud2, "/adapter_probe/input", 10)
        process = subprocess.Popen([
            "ros2", "run", "isaac_fastlio_adapter", "pointcloud2_to_livox",
            "--ros-args", "-r", "__node:=adapter_probe",
            "-p", "input_topic:=/adapter_probe/input",
            "-p", "output_topic:=/adapter_probe/output",
        ])

        def stop():
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)

        self.addCleanup(stop)

        def wait_until(predicate):
            deadline = time.monotonic() + 10
            while not predicate() and time.monotonic() < deadline:
                self.assertIsNone(process.poll(), "Adapter exited unexpectedly")
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertTrue(predicate(), "Timed out waiting for adapter")

        wait_until(lambda: publisher.get_subscription_count() > 0
                   and node.count_publishers("/adapter_probe/output") > 0)
        for legacy in (False, True):
            with self.subTest(legacy=legacy):
                cloud = PointCloud2()
                cloud.header.frame_id = "lidar_link"
                cloud.height, cloud.width = 1, 2
                cloud.fields = [
                    PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                    for name, offset in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))
                ] + [
                    PointField(name="timestamp", offset=16, datatype=PointField.UINT32, count=2),
                ]
                if legacy:
                    cloud.fields.append(PointField(
                        name="emitter_id", offset=24, datatype=PointField.UINT32, count=1,
                    ))
                cloud.fields.append(PointField(
                    name="channel_id", offset=28 if legacy else 24,
                    datatype=PointField.UINT32, count=1,
                ))
                cloud.point_step = 32 if legacy else 28
                cloud.row_step = cloud.point_step * cloud.width
                cloud.is_dense = True
                records = [
                    (1.0, 2.0, 3.0, 0.5, 10_000_000_000, 7),
                    (4.0, 5.0, 6.0, 1.0, 10_050_000_000, 10),
                ]
                cloud.data = b"".join(
                    struct.pack("<ffffQII", *record[:-1], 99, record[-1])
                    if legacy else struct.pack("<ffffQI", *record)
                    for record in records
                )
                output.clear()
                publisher.publish(cloud)
                wait_until(lambda: bool(output))
                message = output[-1]
                self.assertEqual(message.header.frame_id, "lidar_link")
                self.assertEqual((message.header.stamp.sec, message.header.stamp.nanosec), (10, 0))
                self.assertEqual(message.timebase, 10_000_000_000)
                self.assertEqual(message.point_num, 2)
                self.assertEqual(
                    [(p.x, p.y, p.z, p.reflectivity, p.offset_time, p.line, p.tag)
                     for p in message.points],
                    [(1.0, 2.0, 3.0, 127, 0, 3, 0),
                     (4.0, 5.0, 6.0, 255, 50_000_000, 2, 0)],
                )


if __name__ == "__main__":
    unittest.main()
