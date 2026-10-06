"""Map-frame pose of a robot as the JSON published for fleet adapters."""

import json
import math


def yaw_from_quaternion(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )


def pose_payload(robot, message):
    pose = message.pose.pose
    stamp = message.header.stamp
    return json.dumps({
        "robot": robot,
        "frame_id": message.header.frame_id,
        "stamp": stamp.sec + stamp.nanosec * 1e-9,
        "x": pose.position.x,
        "y": pose.position.y,
        "yaw": yaw_from_quaternion(pose.orientation),
    })
