"""Opt-in physical obstacle placement for headless braking validation."""

import json
import math
from pathlib import Path


class SafetyValidationScene:
    def __init__(self, directory, stage, robot):
        from isaacsim.core.experimental.objects import Cube
        from pxr import UsdPhysics

        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.robot = robot
        self.sequence = None
        self.box = Cube(
            "/World/BrakingValidationBox", sizes=1.0, colors="orange",
            positions=[100.0, 100.0, 0.6], scales=[0.2, 0.6, 1.2],
        )
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath("/World/BrakingValidationBox"))

    def update(self):
        request_path = self.directory / "request.json"
        if not request_path.exists():
            return
        request = json.loads(request_path.read_text())
        sequence = request["sequence"]
        if sequence == self.sequence:
            return
        if request["action"] == "hide":
            self.box.set_world_poses(positions=[[100.0, 100.0, 0.6]])
            response = {"sequence": sequence, "action": "hide"}
        elif request["action"] == "place":
            positions, orientations = self.robot.get_world_poses()
            position = positions.numpy()[0]
            quaternion = orientations.numpy()[0]
            w, x, y, z = map(float, quaternion)
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            c, s = math.cos(yaw), math.sin(yaw)
            forward, left = request["offset"]
            sizes = request["sizes"]
            if not all(math.isfinite(v) for v in (forward, left, *sizes)) or min(sizes) <= 0:
                raise ValueError("Invalid braking validation obstacle geometry")
            center = [
                float(position[0]) - c * forward + s * left,
                float(position[1]) - s * forward - c * left,
                sizes[2] / 2,
            ]
            self.box.set_local_scales([sizes])
            self.box.set_world_poses(
                positions=[center], orientations=[[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]],
            )
            response = {"sequence": sequence, "action": "place", "center": center,
                        "yaw": yaw + math.pi, "sizes": sizes,
                        "robot_position": position.tolist(), "robot_yaw": yaw + math.pi}
        else:
            raise ValueError(f"Unknown validation action: {request['action']}")
        temporary = self.directory / "response.tmp"
        temporary.write_text(json.dumps(response))
        temporary.replace(self.directory / "response.json")
        self.sequence = sequence
        print("BRAKING_SCENE " + json.dumps(response), flush=True)
