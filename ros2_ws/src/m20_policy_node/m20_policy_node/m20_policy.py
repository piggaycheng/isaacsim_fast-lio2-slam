"""ROS-independent M20 policy I/O, matching my_locomotion_training "Velocity-Rough-M20" (and Flat without height scan).

obs (243) = base lin vel(3) | base ang vel(3) | projected gravity(3) | command(3) | leg pos rel(12)
            | leg vel rel * 0.05 (12) | last action(16) | height scan(187) | wheel vel * 0.05 (4)
            The flat policy has the same layout without the height scan (56 values).
height scan = clip(base_z - ground_z - 0.5, -1, 1) on a 1.6 x 1.0 m grid (0.1 m, x fastest) aligned with the base yaw.
action (16) = 12 leg position offsets (* 0.25 rad, added to the default pose)
              | 4 wheel velocities (* 10 rad/s, clipped +-60)
Leg order is hipx, hipy, knee, each fl, fr, hl, hr; wheels are fl, fr, hl, hr.
"""

from typing import Callable

import numpy as np

LEG_NAMES = [f"{leg}_{joint}_joint" for joint in ("hipx", "hipy", "knee") for leg in ("fl", "fr", "hl", "hr")]
WHEEL_NAMES = [f"{leg}_wheel_joint" for leg in ("fl", "fr", "hl", "hr")]
JOINT_NAMES = LEG_NAMES + WHEEL_NAMES
DEFAULT_LEG_POS = np.array([0.0] * 4 + [-0.65, -0.65, 0.65, 0.65] + [1.3, 1.3, -1.3, -1.3], dtype=np.float32)
SCAN_X = np.arange(-0.8, 0.8 + 1e-9, 0.1)
SCAN_Y = np.arange(-0.5, 0.5 + 1e-9, 0.1)
# (187, 2) scan points in the yaw-aligned base frame, x varying fastest like IsaacLab's grid_pattern(ordering="xy")
SCAN_POINTS = np.stack(np.meshgrid(SCAN_X, SCAN_Y), axis=-1).reshape(-1, 2)
SCAN_SIZE = len(SCAN_POINTS)
SCAN_OFFSET = 0.5
SCAN_CLIP = 1.0
STANDING_HEIGHT = 0.49
FLAT_OBS_SIZE = 56
OBS_SIZE = FLAT_OBS_SIZE + SCAN_SIZE
ACTION_SIZE = 16
LEG_ACTION_SCALE = 0.25
WHEEL_ACTION_SCALE = 10.0
WHEEL_TARGET_LIMIT = 60.0
VELOCITY_OBS_SCALE = 0.05


def quat_rotate_inverse(quat_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Express a world-frame vector in the frame described by ``quat_wxyz``."""
    w, x, y, z = quat_wxyz
    q_vec = np.array([-x, -y, -z])
    t = 2.0 * np.cross(q_vec, vec)
    return vec + w * t + np.cross(q_vec, t)


def yaw_from_quat(quat_wxyz: np.ndarray) -> float:
    w, x, y, z = quat_wxyz
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def height_scan_observation(
    ground_z: Callable[[np.ndarray], np.ndarray], base_pos: np.ndarray, yaw: float, drift_free: bool = False
) -> np.ndarray:
    """Height scan observation; ``ground_z(xy)`` returns the terrain height at world (N, 2) points, NaN if unknown.

    Unknown cells are treated as flat ground under a robot standing at its default height.
    With ``drift_free`` the base height is taken as the standing height above the median terrain of the scan, which
    removes the vertical drift of the odometry that built the map (the relative terrain profile is kept).
    """
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    xy = np.asarray(base_pos[:2]) + SCAN_POINTS @ rotation.T
    ground = np.asarray(ground_z(xy), dtype=np.float64)
    base_z = base_pos[2]
    if drift_free and np.isfinite(ground).any():
        base_z = STANDING_HEIGHT + np.median(ground[np.isfinite(ground)])
    height = np.where(np.isfinite(ground), base_z - ground, STANDING_HEIGHT)
    return np.clip(height - SCAN_OFFSET, -SCAN_CLIP, SCAN_CLIP).astype(np.float32)


class ElevationMap:
    """Grid map layer in the grid_map_msgs layout: cell (0, 0) is the +x/+y corner, data are column-major."""

    def __init__(self, z: np.ndarray, center_xy: np.ndarray, length_xy: np.ndarray, resolution: float) -> None:
        self.z = z  # (rows along x, cols along y)
        self.center = np.asarray(center_xy, dtype=np.float64)
        self.length = np.asarray(length_xy, dtype=np.float64)
        self.resolution = resolution

    def sample(self, xy: np.ndarray) -> np.ndarray:
        """Nearest-cell terrain height at world (N, 2) points; NaN outside the map or for unknown cells."""
        index = np.floor((self.center + self.length / 2.0 - xy) / self.resolution).astype(int)
        inside = (index >= 0).all(axis=1) & (index[:, 0] < self.z.shape[0]) & (index[:, 1] < self.z.shape[1])
        out = np.full(len(xy), np.nan)
        out[inside] = self.z[index[inside, 0], index[inside, 1]]
        return out


def build_observation(
    base_lin_vel: np.ndarray,
    base_ang_vel: np.ndarray,
    orientation_wxyz: np.ndarray,
    command: np.ndarray,
    leg_pos: np.ndarray,
    leg_vel: np.ndarray,
    wheel_vel: np.ndarray,
    last_action: np.ndarray,
    height_scan: np.ndarray | None = None,
) -> np.ndarray:
    """Assemble the observation (243 values, or 56 without ``height_scan``); base velocities must be in the base frame."""
    gravity = quat_rotate_inverse(np.asarray(orientation_wxyz, dtype=np.float64), np.array([0.0, 0.0, -1.0]))
    obs = np.concatenate(
        [
            base_lin_vel,
            base_ang_vel,
            gravity,
            command,
            leg_pos - DEFAULT_LEG_POS,
            leg_vel * VELOCITY_OBS_SCALE,
            last_action,
            [] if height_scan is None else height_scan,
            wheel_vel * VELOCITY_OBS_SCALE,
        ]
    ).astype(np.float32)
    expected = FLAT_OBS_SIZE if height_scan is None else OBS_SIZE
    if obs.shape != (expected,):
        raise ValueError(f"Observation has shape {obs.shape}, expected ({expected},)")
    return obs


def action_to_targets(action: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Map a raw policy action to leg position targets (rad) and wheel velocity targets (rad/s)."""
    leg_targets = DEFAULT_LEG_POS + LEG_ACTION_SCALE * action[:12]
    wheel_targets = np.clip(WHEEL_ACTION_SCALE * action[12:], -WHEEL_TARGET_LIMIT, WHEEL_TARGET_LIMIT)
    return leg_targets.astype(np.float32), wheel_targets.astype(np.float32)


class M20Policy:
    """TorchScript policy exported by `isaaclab play` (observation normalization is included)."""

    def __init__(self, path: str, device: str = "cpu") -> None:
        import torch

        self._torch = torch
        self._device = device
        self._model = torch.jit.load(path, map_location=device).eval()
        self.last_action = np.zeros(ACTION_SIZE, dtype=np.float32)

    def reset(self) -> None:
        self.last_action = np.zeros(ACTION_SIZE, dtype=np.float32)

    def infer(self, obs: np.ndarray) -> np.ndarray:
        with self._torch.inference_mode():
            tensor = self._torch.from_numpy(obs).to(self._device).unsqueeze(0)
            action = self._model(tensor).squeeze(0).cpu().numpy().astype(np.float32)
        if action.shape != (ACTION_SIZE,):
            raise ValueError(f"Policy returned shape {action.shape}, expected ({ACTION_SIZE},)")
        self.last_action = action
        return action
