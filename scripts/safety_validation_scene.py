"""Opt-in physical obstacles for headless braking and navigation validation."""

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
        self.stage = stage
        self.sequence = None
        self.scenario_boxes = []
        self.obstacles = []
        self.motion_started = None
        self.record_telemetry = False
        self.last_telemetry = None
        self.box = Cube(
            "/World/BrakingValidationBox", sizes=1.0, colors="orange",
            positions=[100.0, 100.0, 0.6], scales=[0.2, 0.6, 1.2],
        )
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath("/World/BrakingValidationBox"))

    def anchor(self):
        positions, orientations = self.robot.get_world_poses()
        position = positions.numpy()[0]
        w, x, y, z = map(float, orientations.numpy()[0])
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return [float(position[0]), float(position[1]), yaw]

    def configure(self, request):
        from isaacsim.core.experimental.objects import Cube
        from pxr import UsdPhysics

        anchor = self.anchor()
        definitions = request["obstacles"]
        if not 1 <= len(definitions) <= 3:
            raise ValueError("Navigation validation needs one to three obstacles")
        obstacles = []
        c, s = math.cos(anchor[2]), math.sin(anchor[2])
        for definition in definitions:
            forward, left = definition["offset"]
            sizes = definition["sizes"]
            vx, vy = definition.get("velocity", [0.0, 0.0])
            duration = definition.get("duration", 0.0)
            if (len(sizes) != 3 or min(sizes) <= 0 or duration < 0
                    or not all(math.isfinite(v) for v in (
                        forward, left, *sizes, vx, vy, duration,
                    ))):
                raise ValueError("Invalid navigation validation obstacle geometry or motion")
            obstacles.append({
                "center": [anchor[0] + c * forward - s * left,
                           anchor[1] + s * forward + c * left, sizes[2] / 2],
                "yaw": anchor[2], "sizes": sizes,
                "velocity": [c * vx - s * vy, s * vx + c * vy],
                "duration": duration,
            })
        while len(self.scenario_boxes) < len(obstacles):
            path = f"/World/NavigationValidationBox_{len(self.scenario_boxes)}"
            self.scenario_boxes.append(Cube(
                path, sizes=1.0, colors="orange", positions=[100.0, 100.0, 0.6],
                scales=[0.2, 0.6, 1.2],
            ))
            UsdPhysics.CollisionAPI.Apply(self.stage.GetPrimAtPath(path))
        for box in self.scenario_boxes:
            box.set_world_poses(positions=[[100.0, 100.0, 0.6]])
        self.box.set_world_poses(positions=[[100.0, 100.0, 0.6]])
        self.obstacles = obstacles
        self.motion_started = None
        self.record_telemetry = request.get("record_telemetry", False)
        self.last_telemetry = None
        for box, obstacle in zip(self.scenario_boxes, obstacles):
            box.set_local_scales([obstacle["sizes"]])
            box.set_world_poses(
                positions=[obstacle["center"]],
                orientations=[[math.cos(anchor[2] / 2), 0, 0, math.sin(anchor[2] / 2)]],
            )
        return {"anchor": anchor, "obstacles": obstacles}

    def move_obstacles(self, sim_time):
        if self.motion_started is None:
            return
        for box, obstacle in zip(self.scenario_boxes, self.obstacles):
            elapsed = min(max(sim_time - self.motion_started, 0.0), obstacle["duration"])
            vx, vy = obstacle["velocity"]
            x, y, z = obstacle["center"]
            box.set_world_poses(positions=[[x + vx * elapsed, y + vy * elapsed, z]])

    def update(self, sim_time=0.0):
        self.move_obstacles(sim_time)
        if (self.record_telemetry and self.obstacles and (
                self.last_telemetry is None or sim_time - self.last_telemetry >= 0.1)):
            centers = [
                box.get_world_poses()[0].numpy()[0].tolist()
                for box in self.scenario_boxes[:len(self.obstacles)]
            ]
            temporary = self.directory / "telemetry.tmp"
            temporary.write_text(json.dumps({
                "sequence": self.sequence, "sim_time": sim_time,
                "motion_started": self.motion_started, "centers": centers,
            }))
            temporary.replace(self.directory / "telemetry.json")
            self.last_telemetry = sim_time
        request_path = self.directory / "request.json"
        if not request_path.exists():
            return
        request = json.loads(request_path.read_text())
        sequence = request["sequence"]
        if sequence == self.sequence:
            return
        if request["action"] == "hide":
            self.box.set_world_poses(positions=[[100.0, 100.0, 0.6]])
            for box in self.scenario_boxes:
                box.set_world_poses(positions=[[100.0, 100.0, 0.6]])
            self.obstacles = []
            self.motion_started = None
            response = {"sequence": sequence, "action": "hide"}
        elif request["action"] == "configure":
            response = {"sequence": sequence, "action": "configure", **self.configure(request)}
        elif request["action"] == "start_motion":
            if not self.obstacles:
                raise ValueError("Configure navigation obstacles before starting motion")
            self.motion_started = sim_time
            response = {"sequence": sequence, "action": "start_motion", "sim_time": sim_time}
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
                float(position[0]) + c * forward - s * left,
                float(position[1]) + s * forward + c * left,
                sizes[2] / 2,
            ]
            self.box.set_local_scales([sizes])
            self.box.set_world_poses(
                positions=[center], orientations=[[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]],
            )
            response = {"sequence": sequence, "action": "place", "center": center,
                        "yaw": yaw, "sizes": sizes,
                        "robot_position": position.tolist(), "robot_yaw": yaw}
        else:
            raise ValueError(f"Unknown validation action: {request['action']}")
        temporary = self.directory / "response.tmp"
        temporary.write_text(json.dumps(response))
        temporary.replace(self.directory / "response.json")
        self.sequence = sequence
        print("BRAKING_SCENE " + json.dumps(response), flush=True)
