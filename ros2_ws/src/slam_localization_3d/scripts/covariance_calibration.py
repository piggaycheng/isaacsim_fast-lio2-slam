#!/usr/bin/env python3
"""Estimate EKF measurement covariances from a rosbag without ground truth.

Record a bag containing wheel odometry, IMU, FAST-LIO odometry and the PCD
global pose while the robot stands still and then drives varied motions.
The estimates only use quantities available on a real vehicle:

* Standstill: IMU gyro bias/noise and PCD pose repeatability.
* Three-cornered hat over fixed windows: wheel, IMU and FAST-LIO yaw
  increments are compared pairwise. Each squared difference is the sum of two
  unknown variances, so the three can be solved together:
      wheel yaw variance = a * distance + b * turned angle
      IMU yaw variance   = q * duration
      LIO yaw variance   = c
* Wheel translation has only FAST-LIO as a second source, so the along-track
  wheel/LIO disagreement cannot be split between them. All of it is booked to
  the wheel (an upper bound, which is the safe side for the EKF):
      wheel distance variance = k * distance + k_turn * turned angle
* The hat assumes independent errors, but a gyro scale error grows with the
  turned angle just like wheel yaw error. The IMU/LIO yaw disagreement alone
  is therefore also fitted (an upper bound that includes the small LIO
  error), and the larger of the two IMU estimates is used. The IMU yaw scale
  against LIO is reported.
  A differential drive cannot slide sideways, so lateral disagreement during
  low-translation windows measures FAST-LIO error; it is reported for
  information (LIO error is not isotropic, so it is not subtracted).
* LIO body->base offset: the wheels define the turning centre, so a wrong
  offset shows up as translation during turns; it is refitted and reported.
* PCD error per axis: PCD registration error is slowly varying (map and
  scan geometry), so consecutive poses share most of it. Relative motion
  between PCD poses 10-60 s apart is compared with FAST-LIO relative motion;
  half the mean squared difference is the PCD pose variance for x, y and yaw.
  ICP covariance is millimetre level, so the published covariance is mostly
  the adapter floors: the tool recommends min_covariance_xy/yaw and scale 1.
  Error that the LiDAR sources share (heading-dependent scan bias) cannot be
  seen without an external reference; --pcd-unobservable-factor (default 2,
  validated in Isaac Sim) covers it.

If the bag also contains simulator ground truth, --ground-truth validates each
estimate against true errors (normalized error ratio 1.0 is ideal; 0.5-2 is
PASS, 1/3-0.5 is CONSERVATIVE, i.e. overstated but safe).
"""

import argparse
import math
import re
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares, nnls
from scipy.ndimage import median_filter

DEFAULT_ROBOT_TYPE = "nova_carter"
PROFILE_PREFIX = ("parameter_overrides",)
CONSISTENT_RATIO = (0.5, 2.0)
# Overstated variance only slows the filter; understated variance makes it overconfident.
CONSERVATIVE_RATIO = 1.0 / 3.0
LATERAL_MAX_DISTANCE = 0.15


def verdict(ratio, upper_bound=False):
    """upper_bound: the estimate is known to include another sensor's error."""
    if CONSISTENT_RATIO[0] <= ratio <= CONSISTENT_RATIO[1]:
        return "PASS"
    if ratio < CONSISTENT_RATIO[0] and upper_bound:
        return "UPPER BOUND"
    if CONSERVATIVE_RATIO <= ratio < CONSISTENT_RATIO[0]:
        return "CONSERVATIVE"
    return "FAIL"


def wrap(angle):
    return (np.asarray(angle) + np.pi) % (2 * np.pi) - np.pi


def quaternion_yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quaternion_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


@dataclass
class Trajectory:
    """Planar pose samples with unwrapped yaw for interpolation."""

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray

    @classmethod
    def create(cls, t, x, y, yaw):
        order = np.argsort(t)
        t = np.asarray(t, dtype=float)[order]
        keep = np.concatenate(([True], np.diff(t) > 0))
        return cls(
            t[keep], np.asarray(x, dtype=float)[order][keep],
            np.asarray(y, dtype=float)[order][keep],
            np.unwrap(np.asarray(yaw, dtype=float)[order])[keep],
        )

    def at(self, times):
        return (
            np.interp(times, self.t, self.x), np.interp(times, self.t, self.y),
            np.interp(times, self.t, self.yaw),
        )

    def covers(self, start, end, max_gap):
        if len(self.t) < 2 or start < self.t[0] or end > self.t[-1]:
            return False
        first = np.searchsorted(self.t, start, side="right") - 1
        last = np.searchsorted(self.t, end, side="left")
        return bool(np.all(np.diff(self.t[first:last + 1]) <= max_gap))


def relative_motion(start, end):
    """Motion from start to end expressed in the start frame (dx, dy, dyaw)."""
    x0, y0, yaw0 = start
    x1, y1, yaw1 = end
    dx, dy = x1 - x0, y1 - y0
    c, s = np.cos(yaw0), np.sin(yaw0)
    return np.array([c * dx + s * dy, -s * dx + c * dy, yaw1 - yaw0])


def weighted_nnls(matrix, target, iterations=4, outlier_ratio=25.0):
    """Nonnegative variance fit for squared errors.

    Squared Gaussian errors have standard deviation proportional to their
    variance, so rows are reweighted by the predicted variance. Rows above
    outlier_ratio times their prediction are dropped as gross failures.
    """
    matrix = np.asarray(matrix, dtype=float)
    target = np.asarray(target, dtype=float)
    keep = np.ones(len(target), dtype=bool)
    weights = np.ones(len(target))
    solution = np.zeros(matrix.shape[1])
    for _ in range(iterations):
        if keep.sum() < matrix.shape[1]:
            break
        solution, _ = nnls(matrix[keep] * weights[keep, None], target[keep] * weights[keep])
        prediction = np.maximum(matrix @ solution, 1e-15)
        weights = 1.0 / prediction
        keep = target <= outlier_ratio * prediction
    return solution, keep



def read_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    metadata = rosbag2_py.Info().read_metadata(str(path), "")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=metadata.storage_identifier),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = [topic for topic in topics if topic in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {topic: get_message(types[topic]) for topic in wanted}
    messages = {topic: [] for topic in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        messages[topic].append(deserialize_message(data, classes[topic]))
    return messages


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def odometry_trajectory(messages, yaw_offset=0.0):
    rows = []
    for message in messages:
        pose = message.pose.pose
        q = pose.orientation
        rows.append((
            stamp_seconds(message), pose.position.x, pose.position.y,
            quaternion_yaw(q.x, q.y, q.z, q.w) + yaw_offset,
        ))
    if not rows:
        return None
    t, x, y, yaw = np.array(rows).T
    return Trajectory.create(t, x, y, yaw)


@dataclass
class LioSamples:
    """FAST-LIO body poses that can be re-projected with another body->base offset."""

    t: np.ndarray
    position: np.ndarray
    rotation: np.ndarray
    yaw: np.ndarray

    @classmethod
    def from_messages(cls, messages):
        t, position, rotation, yaw = [], [], [], []
        for message in messages:
            pose = message.pose.pose
            q = pose.orientation
            t.append(stamp_seconds(message))
            position.append((pose.position.x, pose.position.y, pose.position.z))
            rotation.append(quaternion_matrix(q.x, q.y, q.z, q.w))
            yaw.append(quaternion_yaw(q.x, q.y, q.z, q.w))
        return cls(np.array(t), np.array(position), np.array(rotation), np.array(yaw))

    def base(self, body_to_base_xyz, body_to_base_yaw):
        base = self.position + self.rotation @ np.asarray(body_to_base_xyz, dtype=float)
        return Trajectory.create(self.t, base[:, 0], base[:, 1], self.yaw + body_to_base_yaw)


def lio_base_trajectory(messages, body_to_base_xyz, body_to_base_yaw):
    if not messages:
        return None
    return LioSamples.from_messages(messages).base(body_to_base_xyz, body_to_base_yaw)


@dataclass
class Data:
    wheel: Trajectory
    wheel_t: np.ndarray
    wheel_vx: np.ndarray
    wheel_wz: np.ndarray
    imu_t: np.ndarray
    imu_wz: np.ndarray
    lio: Trajectory
    lio_samples: LioSamples
    lio_body_to_base: tuple
    pcd_t: np.ndarray
    pcd_pose: np.ndarray
    pcd_covariance: np.ndarray
    truth: Trajectory = None


def unique_stamps(messages):
    """Sort by stamp and drop repeated stamps (e.g. a topic with two publishers)."""
    result = []
    for message in sorted(messages, key=stamp_seconds):
        if not result or stamp_seconds(message) > stamp_seconds(result[-1]):
            result.append(message)
    return result


def load_data(args):
    topics = [args.wheel_topic, args.imu_topic, args.lio_topic, args.pcd_topic]
    if args.ground_truth:
        topics.append(args.truth_topic)
    messages = read_bag(args.bag, topics)
    for topic in topics:
        if not messages[topic]:
            raise SystemExit(f"Bag has no messages on {topic}")
    wheel_messages = unique_stamps(messages[args.wheel_topic])
    imu_messages = unique_stamps(messages[args.imu_topic])
    pcd_messages = sorted(messages[args.pcd_topic], key=stamp_seconds)
    lio_samples = LioSamples.from_messages(messages[args.lio_topic])
    pcd_pose, pcd_covariance = [], []
    for message in pcd_messages:
        pose = message.pose.pose
        q = pose.orientation
        pcd_pose.append((pose.position.x, pose.position.y, quaternion_yaw(q.x, q.y, q.z, q.w)))
        full = np.asarray(message.pose.covariance, dtype=float).reshape(6, 6)
        pcd_covariance.append(full[np.ix_((0, 1, 5), (0, 1, 5))])
    return Data(
        wheel=odometry_trajectory(wheel_messages),
        wheel_t=np.array([stamp_seconds(m) for m in wheel_messages]),
        wheel_vx=np.array([m.twist.twist.linear.x for m in wheel_messages]),
        wheel_wz=np.array([m.twist.twist.angular.z for m in wheel_messages]),
        imu_t=np.array([stamp_seconds(m) for m in imu_messages]),
        imu_wz=np.array([m.angular_velocity.z for m in imu_messages]) * args.imu_z_sign,
        lio=lio_samples.base(args.lio_body_to_base[:3], args.lio_body_to_base[3]),
        lio_samples=lio_samples,
        lio_body_to_base=tuple(args.lio_body_to_base),
        pcd_t=np.array([stamp_seconds(m) for m in pcd_messages]),
        pcd_pose=np.array(pcd_pose),
        pcd_covariance=np.array(pcd_covariance),
        truth=odometry_trajectory(messages[args.truth_topic], args.truth_yaw_offset)
        if args.ground_truth else None,
    )


def static_segments(t, vx, wz, min_duration, trim, linear_eps=0.01, angular_eps=0.01, filter_size=31):
    """Standstill intervals from median-filtered wheel twist (ignores encoder spikes)."""
    size = max(1, min(filter_size, len(t)) | 1)
    speed = median_filter(np.abs(vx), size=size, mode="nearest")
    rate = median_filter(np.abs(wz), size=size, mode="nearest")
    still = (speed < linear_eps) & (rate < angular_eps)
    segments = []
    start = None
    for index, value in enumerate(still):
        if value and start is None:
            start = t[index]
        if (not value or index == len(still) - 1) and start is not None:
            end = t[index]
            if end - start >= min_duration + 2 * trim:
                segments.append((start + trim, end - trim))
            start = None
    return segments


def in_segments(times, segments):
    mask = np.zeros(len(times), dtype=bool)
    for start, end in segments:
        mask |= (times >= start) & (times <= end)
    return mask


def integrate(times, values):
    cumulative = np.concatenate(([0.0], np.cumsum(0.5 * (values[1:] + values[:-1]) * np.diff(times))))
    return lambda at: np.interp(at, times, cumulative)


@dataclass
class Window:
    duration: float
    distance: float
    turn: float
    wheel: np.ndarray
    imu_yaw: float
    lio: np.ndarray
    truth: np.ndarray = None


def build_windows(data, window, imu_bias, max_gap):
    wheel_speed = integrate(data.wheel_t, np.abs(data.wheel_vx))
    wheel_turn = integrate(data.wheel_t, np.abs(data.wheel_wz))
    imu_yaw = integrate(data.imu_t, data.imu_wz - imu_bias)
    imu_trajectory = Trajectory.create(data.imu_t, data.imu_t, data.imu_t, data.imu_t * 0)
    start = max(data.wheel.t[0], data.imu_t[0], data.lio.t[0])
    end = min(data.wheel.t[-1], data.imu_t[-1], data.lio.t[-1])
    if data.truth is not None:
        start, end = max(start, data.truth.t[0]), min(end, data.truth.t[-1])
    windows = []
    for t0 in np.arange(start, end - window, window):
        t1 = t0 + window
        sources = [data.wheel, data.lio, imu_trajectory]
        if data.truth is not None:
            sources.append(data.truth)
        if not all(source.covers(t0, t1, max_gap) for source in sources):
            continue
        truth = None
        if data.truth is not None:
            truth = relative_motion(data.truth.at(t0), data.truth.at(t1))
        windows.append(Window(
            duration=window,
            distance=float(wheel_speed(t1) - wheel_speed(t0)),
            turn=float(wheel_turn(t1) - wheel_turn(t0)),
            wheel=relative_motion(data.wheel.at(t0), data.wheel.at(t1)),
            imu_yaw=float(imu_yaw(t1) - imu_yaw(t0)),
            lio=relative_motion(data.lio.at(t0), data.lio.at(t1)),
            truth=truth,
        ))
    return windows


def fit_lever_arm(data, window, max_gap=0.5):
    """Planar body->base offset that best explains wheel motion with LIO.

    A wrong offset makes the LIO-derived base point sweep a circle while the
    robot turns in place, which would otherwise be booked as sensor noise.
    The wheels define the rotation centre, so no ground truth is needed.
    """
    z, yaw_offset = data.lio_body_to_base[2], data.lio_body_to_base[3]
    starts = [
        t0 for t0 in np.arange(data.lio.t[0], data.lio.t[-1] - window, window)
        if data.wheel.covers(t0, t0 + window, max_gap)
        and data.lio.covers(t0, t0 + window, max_gap)
    ]
    if len(starts) < 10:
        return None
    wheel = np.array([
        relative_motion(data.wheel.at(t0), data.wheel.at(t0 + window))[:2] for t0 in starts
    ])

    def residual(offset):
        lio = data.lio_samples.base((offset[0], offset[1], z), yaw_offset)
        return np.concatenate([
            relative_motion(lio.at(t0), lio.at(t0 + window))[:2] - motion
            for t0, motion in zip(starts, wheel)
        ])

    initial = np.asarray(data.lio_body_to_base[:2], dtype=float)
    result = least_squares(residual, initial, loss="soft_l1", f_scale=0.01)
    before = float(np.sqrt(np.mean(residual(initial) ** 2)))
    after = float(np.sqrt(np.mean(result.fun ** 2)))
    return result.x, before, after


def fit_increment_noise(windows):
    """Three-cornered hat on yaw, and wheel/LIO fit on distance."""
    rows, targets = [], []
    for w in windows:
        wheel, imu, lio = w.wheel[2], w.imu_yaw, w.lio[2]
        rows += [
            [w.distance, w.turn, w.duration, 0.0],
            [w.distance, w.turn, 0.0, 1.0],
            [0.0, 0.0, w.duration, 1.0],
        ]
        targets += [(wheel - imu) ** 2, (wheel - lio) ** 2, (imu - lio) ** 2]
    yaw, yaw_keep = weighted_nnls(rows, targets)
    # The hat assumes independent white errors. A gyro scale error grows with the turned
    # angle like the wheel yaw error, so the two correlate and the hat books the IMU part
    # to the wheel. FAST-LIO yaw is far more accurate, so the IMU/LIO disagreement alone
    # bounds the IMU error from above (it also contains the small LIO error).
    imu_lio = np.array([w.imu_yaw - w.lio[2] for w in windows])
    imu_pair, _ = weighted_nnls([[w.duration] for w in windows], imu_lio ** 2)
    lio_turn = np.array([w.lio[2] for w in windows])
    imu_scale = float(
        np.sum(np.array([w.imu_yaw for w in windows]) * lio_turn) / max(np.sum(lio_turn ** 2), 1e-15)
    )
    # Along-track: wheel error + LIO error; the constant absorbs per-window LIO/timing noise.
    distance, distance_keep = weighted_nnls(
        [[w.distance, w.turn, 1.0] for w in windows],
        [(w.wheel[0] - w.lio[0]) ** 2 for w in windows],
    )
    # Lateral with little translation: the wheels cannot slide, and heading error has
    # no lever to move them sideways, so the disagreement is FAST-LIO error.
    lateral = [w for w in windows if w.distance < LATERAL_MAX_DISTANCE]
    lio_lateral = (
        nnls(np.array([[1.0, w.turn] for w in lateral]),
             np.array([(w.wheel[1] - w.lio[1]) ** 2 for w in lateral]))[0]
        if len(lateral) >= 10 else np.full(2, np.nan)
    )
    return {
        "yaw_variance_per_meter": yaw[0],
        "yaw_variance_per_radian": yaw[1],
        "imu_yaw_variance_per_second": yaw[2],
        "imu_lio_yaw_variance_per_second": imu_pair[0],
        "imu_yaw_scale_vs_lio": imu_scale,
        "lio_yaw_variance_per_window": yaw[3],
        "distance_variance_per_meter": distance[0],
        "position_variance_per_radian": distance[1],
        "lio_distance_variance_per_window": distance[2],
        "lio_lateral_variance_per_window": lio_lateral[0],
        "lio_lateral_variance_per_radian": lio_lateral[1],
        "yaw_rows_used": int(yaw_keep.sum()),
        "yaw_rows": len(targets),
        "distance_rows_used": int(distance_keep.sum()),
    }


def wheel_systematic(windows):
    straight = [w for w in windows if w.distance > 0.2 and w.turn < 0.05]
    yaw_per_meter = (
        float(np.median([(w.lio[2] - w.wheel[2]) / w.distance for w in straight]))
        if straight else float("nan")
    )
    moving = [w for w in windows if abs(w.wheel[0]) > 0.1]
    scale = (
        sum(w.lio[0] * w.wheel[0] for w in moving) / sum(w.wheel[0] ** 2 for w in moving)
        if moving else float("nan")
    )
    return yaw_per_meter, scale




PCD_LAG_BINS = ((0.0, 5.0), (5.0, 15.0), (15.0, 40.0), (40.0, 80.0), (80.0, 160.0))


def pcd_lag_errors(data, min_lag, max_lag, max_pairs=20000):
    """Map-frame PCD residual differences (x, y, yaw) and the lag of every pose pair.

    One SE2 transform maps FAST-LIO onto the PCD poses; differencing the residuals
    of two poses removes what the pair shares. A single fitted rotation avoids
    amplifying per-pose yaw noise by the distance travelled between the poses.
    """
    covered = (data.pcd_t >= data.lio.t[0]) & (data.pcd_t <= data.lio.t[-1])
    times, pcd = data.pcd_t[covered], data.pcd_pose[covered]
    if len(times) < 3:
        return np.zeros((0, 3)), np.zeros(0)
    lx, ly, lyaw = data.lio.at(times)
    angle, rotation, translation = align_se2(np.column_stack((lx, ly)), pcd[:, :2])
    residual = np.column_stack((
        pcd[:, :2] - ((rotation @ np.vstack((lx, ly))).T + translation),
        wrap(pcd[:, 2] - (lyaw + angle)),
    ))
    errors, lags = [], []
    for first in range(len(times)):
        lag = times - times[first]
        later = np.nonzero((lag >= min_lag) & (lag <= max_lag) & (lag > 0))[0]
        difference = residual[later] - residual[first]
        difference[:, 2] = wrap(difference[:, 2])
        errors.append(difference)
        lags.append(lag[later])
    errors, lags = np.concatenate(errors), np.concatenate(lags)
    if len(errors) > max_pairs:
        keep = np.linspace(0, len(errors) - 1, max_pairs).astype(int)
        errors, lags = errors[keep], lags[keep]
    return errors, lags


def pcd_error_variance(data, min_lag, max_lag):
    """Per-axis PCD pose variance from pairs far enough apart to decorrelate.

    Var(e_i - e_j) = 2 var(e) once the PCD errors of i and j are independent, so
    half the mean squared difference is the pose variance. FAST-LIO drift over
    the lag is included, which keeps the estimate slightly conservative.
    """
    errors, lags = pcd_lag_errors(data, 0.0, max(max_lag, PCD_LAG_BINS[-1][1]))
    if len(errors) == 0:
        return None
    curve = []
    for low, high in PCD_LAG_BINS:
        inside = (lags >= low) & (lags < high)
        if inside.sum() >= 10:
            curve.append((low, high, int(inside.sum()), np.mean(errors[inside] ** 2, axis=0) / 2))
    used = (lags >= min_lag) & (lags <= max_lag)
    if used.sum() < 20:
        return None
    return {
        "pairs": int(used.sum()),
        "variance": np.mean(errors[used] ** 2, axis=0) / 2,
        "curve": curve,
    }


def pcd_icp_part(data, scale, floor_xy, floor_yaw):
    """Registration (ICP) share of the published covariance, per pose, without floors."""
    diagonal = np.array([np.diag(c) for c in data.pcd_covariance]) / scale
    return np.maximum(diagonal - np.array((floor_xy, floor_xy, floor_yaw)), 0.0)


def recommend_pcd_floors(variance, icp_part, factor, minimum=(1e-6, 1e-8)):
    """Floors so that ICP covariance + floor matches factor * observed variance."""
    median_icp = np.median(icp_part, axis=0)
    target = factor * np.asarray(variance)
    xy = max(float(max(target[0] - median_icp[0], target[1] - median_icp[1])), minimum[0])
    yaw = max(float(target[2] - median_icp[2]), minimum[1])
    return xy, yaw


def pcd_repeatability(data, segments):
    ratios = []
    for start, end in segments:
        mask = (data.pcd_t >= start) & (data.pcd_t <= end)
        if mask.sum() < 3:
            continue
        poses = data.pcd_pose[mask].copy()
        poses[:, 2] = np.unwrap(poses[:, 2])
        spread = np.var(poses, axis=0, ddof=1)
        published = np.mean([np.diag(c) for c in data.pcd_covariance[mask]], axis=0)
        ratios.append(spread / published)
    return np.mean(ratios, axis=0) if ratios else None


def align_se2(source, target):
    """Rotation and translation mapping source xy onto target xy."""
    source_center, target_center = source.mean(axis=0), target.mean(axis=0)
    a, b = source - source_center, target - target_center
    angle = math.atan2(
        np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]), np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1])
    )
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([[c, -s], [s, c]])
    return angle, rotation, target_center - rotation @ source_center


def ground_truth_validation(data, windows, noise, imu_variance_per_second, pcd_predicted=None):
    report = {}
    distance = np.array([w.distance for w in windows])
    turn = np.array([w.turn for w in windows])
    duration = np.array([w.duration for w in windows])
    wheel_yaw_error = np.array([wrap(w.wheel[2] - w.truth[2]) for w in windows])
    imu_yaw_error = np.array([wrap(w.imu_yaw - w.truth[2]) for w in windows])
    wheel_distance_error = np.array([w.wheel[0] - w.truth[0] for w in windows])
    true_yaw, _ = weighted_nnls(np.column_stack((distance, turn)), wheel_yaw_error ** 2)
    true_distance, _ = weighted_nnls(np.column_stack((distance, turn)), wheel_distance_error ** 2)
    true_imu, _ = weighted_nnls(duration[:, None], imu_yaw_error ** 2)
    predicted_wheel_yaw = (
        noise["yaw_variance_per_meter"] * distance + noise["yaw_variance_per_radian"] * turn
    )
    predicted_distance = (
        noise["distance_variance_per_meter"] * distance
        + noise["position_variance_per_radian"] * turn
    )
    predicted_imu = imu_variance_per_second * duration
    moving = distance + turn > 1e-3

    def ratio(errors, predicted, mask):
        return float(np.sum(errors[mask] ** 2) / max(np.sum(predicted[mask]), 1e-15))

    report["wheel_yaw"] = (
        ratio(wheel_yaw_error, predicted_wheel_yaw, moving), true_yaw[0], true_yaw[1]
    )
    report["wheel_distance"] = (
        ratio(wheel_distance_error, predicted_distance, moving), true_distance[0], true_distance[1]
    )
    report["imu_yaw"] = (
        ratio(imu_yaw_error, predicted_imu, np.ones(len(windows), bool)), true_imu[0]
    )

    times = data.pcd_t[(data.pcd_t >= data.truth.t[0]) & (data.pcd_t <= data.truth.t[-1])]
    mask = np.isin(data.pcd_t, times)
    tx, ty, tyaw = data.truth.at(times)
    angle, rotation, translation = align_se2(np.column_stack((tx, ty)), data.pcd_pose[mask, :2])
    aligned = (rotation @ np.vstack((tx, ty))).T + translation
    errors = np.column_stack((
        data.pcd_pose[mask, :2] - aligned, wrap(data.pcd_pose[mask, 2] - (tyaw + angle)),
    ))
    # A constant yaw offset is the map frame's own rotation (or a LiDAR yaw mount
    # error), not registration noise; report it separately.
    yaw_bias = float(np.mean(errors[:, 2]))
    errors[:, 2] -= yaw_bias
    predicted = data.pcd_covariance[mask] if pcd_predicted is None else pcd_predicted[mask]
    diagonal = np.array([np.diag(c) for c in predicted])
    report["pcd"] = {
        "ratio": np.mean(errors ** 2, axis=0) / np.mean(diagonal, axis=0),
        "rmse_xy": float(np.sqrt(np.mean(errors[:, 0] ** 2 + errors[:, 1] ** 2))),
        "rmse_yaw": float(np.sqrt(np.mean(errors[:, 2] ** 2))),
        "yaw_bias": yaw_bias,
        "count": len(errors),
    }
    return report


def _indent(line):
    return len(line) - len(line.lstrip())


def _find_block(lines, path, create=False):
    """(header, end) of the nested YAML mapping block at path; header -1 is the document."""
    header, end = -1, len(lines)
    for name in path:
        parent = _indent(lines[header]) if header >= 0 else -2
        content = [i for i in range(header + 1, end)
                   if lines[i].strip() and not lines[i].lstrip().startswith("#")]
        child = _indent(lines[content[0]]) if content else parent + 2
        found = next(
            (i for i in content if _indent(lines[i]) == child and re.match(
                rf"^\s*{re.escape(name)}:\s*(\{{\}})?\s*(#.*)?$", lines[i])),
            None,
        )
        if found is None:
            if not create:
                return None
            indent = " " * child
            found = end
            while found > header + 1 and (
                    not lines[found - 1].strip() or lines[found - 1].lstrip().startswith("#")):
                found -= 1
            lines.insert(found, f"{indent}{name}:")
            end += 1
        else:
            lines[found] = re.sub(r":\s*\{\}", ":", lines[found])
        header, indent = found, _indent(lines[found])
        end = next(
            (i for i in range(found + 1, end)
             if lines[i].strip() and not lines[i].lstrip().startswith("#")
             and _indent(lines[i]) <= indent),
            end,
        )
    return header, end


def read_parameter(path, section, key, prefix=()):
    lines = Path(path).read_text().splitlines()
    block = _find_block(lines, (*prefix, section, "ros__parameters"))
    if block is None:
        return None
    for line in lines[block[0] + 1:block[1]]:
        match = re.match(rf"^\s+{re.escape(key)}:\s*([^#\s]+)", line)
        if match:
            return float(match.group(1))
    return None


def yaml_float(value):
    """Six significant digits that YAML/ROS always read as a double (never an int)."""
    mantissa, _, exponent = f"{float(value):.6g}".partition("e")
    if "." not in mantissa and "n" not in mantissa:
        mantissa += ".0"
    return f"{mantissa}e{exponent}" if exponent else mantissa


def write_parameters(path, section, values, prefix=()):
    """Replace or insert scalar parameters under [prefix/]section/ros__parameters.

    The section must exist unless prefix is given (robot profile overrides).
    """
    lines = Path(path).read_text().splitlines()
    block = _find_block(lines, (*prefix, section), create=bool(prefix))
    if block is None:
        raise SystemExit(f"{path} has no {section} section")
    block = _find_block(lines, (*prefix, section, "ros__parameters"), create=bool(prefix))
    if block is None:
        raise SystemExit(f"{path} has no {section}/ros__parameters")
    header, end = block
    child = " " * (_indent(lines[header]) + 2)
    for key, value in values.items():
        text = yaml_float(value)
        for index in range(header + 1, end):
            match = re.match(rf"^(\s+){re.escape(key)}:\s*[^#\s]+(.*)$", lines[index])
            if match:
                lines[index] = f"{match.group(1)}{key}: {text}{match.group(2)}"
                break
        else:
            lines.insert(header + 1, f"{child}{key}: {text}")
            end += 1
    Path(path).write_text("\n".join(lines) + "\n")


def default_config(relative):
    path = Path(__file__).resolve().parents[2] / relative
    return path if path.exists() else None


def robot_profile_path(args):
    if not re.match(r"^[A-Za-z][A-Za-z0-9_]*$", args.robot_type):
        raise SystemExit(f"Invalid robot type {args.robot_type!r}")
    if args.profile_dir is None:
        raise SystemExit("Robot profile directory not found; pass --profile-dir")
    path = Path(args.profile_dir) / f"{args.robot_type}.yaml"
    if not path.is_file():
        raise SystemExit(f"Robot profile {path} not found")
    return path


def profile_body_to_base(path):
    """Invert the profile's base_link -> imu_link (FAST-LIO body) mount into x, y, z, yaw."""
    mount = yaml.safe_load(Path(path).read_text())["sensor_frames"]["imu_link"]
    cr, sr = math.cos(mount["roll"]), math.sin(mount["roll"])
    cp, sp = math.cos(mount["pitch"]), math.sin(mount["pitch"])
    cy, sy = math.cos(mount["yaw"]), math.sin(mount["yaw"])
    rotation = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    inverse = rotation.T
    xyz = -inverse @ np.array([mount["x"], mount["y"], mount["z"]], dtype=float)
    yaw = math.atan2(inverse[1, 0], inverse[0, 0])
    return [*(float(value) + 0.0 for value in xyz), yaw]


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path, help="rosbag2 directory")
    parser.add_argument("--window", type=float, default=2.0,
                        help="Increment window in seconds; match the PCD correction interval")
    parser.add_argument("--min-static", type=float, default=5.0,
                        help="Minimum standstill segment used for bias/noise (s)")
    parser.add_argument("--pcd-min-lag", type=float, default=10.0,
                        help="Shortest PCD pair separation (s); beyond the error correlation time")
    parser.add_argument("--pcd-max-lag", type=float, default=60.0,
                        help="Longest PCD pair separation (s); keeps FAST-LIO drift small")
    parser.add_argument("--pcd-unobservable-factor", type=float, default=2.0,
                        help="Multiplier for PCD error shared with FAST-LIO (same LiDAR); "
                             "2 matched Isaac Sim ground truth")
    parser.add_argument("--wheel-topic", default="/wheel/odom")
    parser.add_argument("--imu-topic", default="/nav/imu")
    parser.add_argument("--imu-z-sign", type=float, default=1.0,
                        help="-1 if the IMU z axis points down relative to base_link")
    parser.add_argument("--lio-topic", default="/Odometry")
    parser.add_argument("--lio-body-to-base", type=float, nargs=4,
                        metavar=("X", "Y", "Z", "YAW"),
                        help="Default: inverse of the --robot-type profile's sensor_frames.imu_link")
    parser.add_argument("--keep-lever", action="store_true",
                        help="Do not refit the LIO body->base offset from wheel motion")
    parser.add_argument("--pcd-topic", default="/localization_3d/global_pose")
    parser.add_argument("--ground-truth", action="store_true",
                        help="Validate against simulator truth (Isaac Sim only)")
    parser.add_argument("--truth-topic", default="/isaac/ground_truth/odom")
    parser.add_argument("--truth-yaw-offset", type=float, default=math.pi,
                        help="base_link yaw minus the truth frame yaw (Nova Carter: pi; Carter v1: 0)")
    parser.add_argument("--robot-type", default=DEFAULT_ROBOT_TYPE,
                        help="Robot profile (config/robots/<type>.yaml) whose overrides were "
                             "active while recording and that --apply updates")
    parser.add_argument("--profile-dir", type=Path,
                        default=default_config("slam_localization_3d/config/robots"))
    parser.add_argument("--apply", action="store_true",
                        help="Write the recommended values into the robot profile")
    parser.add_argument("--min-imu-variance", type=float, default=1e-6,
                        help="Minimum gyro measurement variance (rad^2/s^2); use an independently "
                             "measured floor when motion/filter errors defeat the noise fit")
    args = parser.parse_args(argv)
    if args.lio_body_to_base is None:
        args.lio_body_to_base = profile_body_to_base(robot_profile_path(args))
    return args


def main(argv=None):
    args = parse_args(argv)
    data = load_data(args)
    segments = static_segments(data.wheel_t, data.wheel_vx, data.wheel_wz, args.min_static, 1.0)
    static_imu = in_segments(data.imu_t, segments)
    imu_dt = float(np.median(np.diff(data.imu_t)))
    if static_imu.sum() >= 10:
        imu_bias = float(np.mean(data.imu_wz[static_imu]))
        imu_static_variance = float(np.var(data.imu_wz[static_imu], ddof=1))
    else:
        imu_bias, imu_static_variance = 0.0, 0.0
        print("WARNING: no standstill segment; IMU bias assumed zero", file=sys.stderr)
    lever = None if args.keep_lever else fit_lever_arm(data, args.window)
    motion_data = data
    if lever is not None and lever[2] < 0.8 * lever[1]:
        offset = (lever[0][0], lever[0][1], data.lio_body_to_base[2])
        # Wheel/IMU noise uses the fitted offset; PCD keeps the production offset it shares.
        motion_data = replace(data, lio=data.lio_samples.base(offset, data.lio_body_to_base[3]))
    windows = build_windows(motion_data, args.window, imu_bias, max_gap=0.5)
    if len(windows) < 20:
        raise SystemExit(f"Only {len(windows)} usable windows; record a longer drive")
    noise = fit_increment_noise(windows)
    imu_variance = max(
        imu_static_variance,
        max(noise["imu_yaw_variance_per_second"], noise["imu_lio_yaw_variance_per_second"]) / imu_dt,
        args.min_imu_variance,
    )
    systematic_yaw, distance_scale = wheel_systematic(windows)
    profile = robot_profile_path(args)

    def fusion_parameter(key, default):
        value = read_parameter(profile, "global_pose_adapter", key, PROFILE_PREFIX)
        return default if value is None else value

    # Values active while the bag was recorded; they are removed from the published covariance.
    current_scale = fusion_parameter("registration_covariance_scale", 1.0)
    current_floors = (fusion_parameter("min_covariance_xy", 1e-4),
                      fusion_parameter("min_covariance_yaw", 1e-5))
    pcd = pcd_error_variance(data, args.pcd_min_lag, args.pcd_max_lag)
    repeatability = pcd_repeatability(data, segments)
    pcd_floors, pcd_predicted = None, None
    if pcd is not None:
        icp_part = pcd_icp_part(data, current_scale, *current_floors)
        pcd_floors = recommend_pcd_floors(pcd["variance"], icp_part, args.pcd_unobservable_factor)
        pcd_predicted = np.array([
            np.diag(icp) + np.diag((pcd_floors[0], pcd_floors[0], pcd_floors[1]))
            for icp in icp_part
        ])

    print(f"Standstill segments: {len(segments)} ({sum(e - s for s, e in segments):.0f} s)")
    print(f"Motion windows: {len(windows)} x {args.window:.1f} s "
          f"(yaw rows used {noise['yaw_rows_used']}/{noise['yaw_rows']})")
    print(f"IMU gyro z bias {imu_bias:+.2e} rad/s, standstill variance {imu_static_variance:.2e}")
    if lever is not None:
        used = " (used for wheel/IMU fits)" if motion_data is not data else ""
        print(f"LIO body->base xy fitted from wheel motion: {lever[0][0]:.4f}, {lever[0][1]:.4f} m "
              f"(configured {data.lio_body_to_base[0]:.4f}, {data.lio_body_to_base[1]:.4f}; "
              f"residual {lever[1] * 1000:.1f} -> {lever[2] * 1000:.1f} mm){used}")
    print("No-ground-truth estimates:")
    print(f"  wheel distance variance per meter  {noise['distance_variance_per_meter']:.3e} m^2/m")
    print(f"  wheel position variance per radian {noise['position_variance_per_radian']:.3e} m^2/rad")
    print(f"  wheel yaw variance per meter       {noise['yaw_variance_per_meter']:.3e} rad^2/m")
    print(f"  wheel yaw variance per radian      {noise['yaw_variance_per_radian']:.3e} rad^2/rad")
    print(f"  IMU integrated yaw variance        hat {noise['imu_yaw_variance_per_second']:.3e}, "
          f"IMU/LIO bound {noise['imu_lio_yaw_variance_per_second']:.3e} rad^2/s"
          f" -> angular_velocity_variance {imu_variance:.3e}")
    imu_scale = noise["imu_yaw_scale_vs_lio"]
    print(f"  IMU yaw scale vs LIO {imu_scale:.4f}"
          + (" (gyro scale error; the IMU/LIO bound covers it)" if abs(imu_scale - 1) > 0.01 else ""))
    print(f"  LIO yaw / along-track per window   {noise['lio_yaw_variance_per_window']:.3e} rad^2,"
          f" {noise['lio_distance_variance_per_window']:.3e} m^2")
    print(f"  LIO lateral (low-translation)      {noise['lio_lateral_variance_per_window']:.3e} m^2"
          f" + {noise['lio_lateral_variance_per_radian']:.3e} m^2/rad (information)")
    drift = "n/a" if math.isnan(systematic_yaw) else f"{systematic_yaw:+.4f} rad/m"
    print(f"  wheel systematic yaw drift {drift}, distance scale vs LIO "
          f"{distance_scale:.4f} (calibrate wheel radius/base if far from 0 / 1)")
    if pcd is None:
        print("  PCD: not enough pose pairs", file=sys.stderr)
    else:
        print("  PCD vs LIO half squared difference by pair lag (x m^2, y m^2, yaw rad^2):")
        for low, high, count, values in pcd["curve"]:
            print(f"    {low:4.0f}-{high:4.0f} s n={count:6d}  "
                  f"{values[0]:.2e} {values[1]:.2e} {values[2]:.2e}")
        variance = pcd["variance"]
        print(f"  PCD pose variance ({args.pcd_min_lag:.0f}-{args.pcd_max_lag:.0f} s, "
              f"{pcd['pairs']} pairs): x {variance[0]:.2e} m^2, y {variance[1]:.2e} m^2, "
              f"yaw {variance[2]:.2e} rad^2 (sigma {math.sqrt(max(variance[0], variance[1])) * 100:.2f} cm, "
              f"{math.degrees(math.sqrt(variance[2])):.3f} deg)")
        print(f"  x {args.pcd_unobservable_factor:g} unobservable factor -> min_covariance_xy "
              f"{pcd_floors[0]:.3e}, min_covariance_yaw {pcd_floors[1]:.3e} "
              f"(currently {current_floors[0]:.3e}, {current_floors[1]:.3e}, scale {current_scale:g})")
    if repeatability is not None:
        print(f"  PCD standstill spread / published variance (x, y, yaw): "
              f"{', '.join(f'{v:.2f}' for v in repeatability)}")

    passed = True
    if args.ground_truth:
        validation = ground_truth_validation(
            data, windows, noise, imu_variance * imu_dt, pcd_predicted
        )
        print("Ground-truth validation (ratio = true squared error / predicted variance):")
        for name, values in validation.items():
            if name == "pcd":
                continue
            result = verdict(values[0], upper_bound=name in ("wheel_distance", "imu_yaw"))
            passed &= result != "FAIL"
            print(f"  {name:15s} ratio {values[0]:6.2f} {result}; "
                  f"truth-fitted {', '.join(f'{v:.3e}' for v in values[1:])}")
        report = validation["pcd"]
        label = "recommended" if pcd_predicted is not None else "published"
        for axis, ratio in zip(("x", "y", "yaw"), report["ratio"]):
            result = verdict(ratio)
            passed &= result != "FAIL"
            print(f"  pcd_{axis:13s} ratio {ratio:6.2f} {result} ({label} covariance)")
        print(f"  pcd RMSE {report['rmse_xy'] * 100:.2f} cm, {math.degrees(report['rmse_yaw']):.3f} deg "
              f"over {report['count']} poses; constant yaw offset "
              f"{math.degrees(report['yaw_bias']):+.3f} deg (map rotation / LiDAR yaw mount)")

    wheel = {
        "distance_variance_per_meter": noise["distance_variance_per_meter"],
        "position_variance_per_radian": noise["position_variance_per_radian"],
        "yaw_variance_per_meter": noise["yaw_variance_per_meter"],
        "yaw_variance_per_radian": noise["yaw_variance_per_radian"],
    }
    print("\nRecommended parameters:")
    print("wheel_encoder_odometry:\n  ros__parameters:")
    for key, value in wheel.items():
        print(f"    {key}: {value:.6g}")
    print(f"nav_imu_adapter:\n  ros__parameters:\n    angular_velocity_variance: {imu_variance:.6g}")
    pcd_parameters = None
    if pcd_floors is not None:
        pcd_parameters = {
            "registration_covariance_scale": 1.0,
            "min_covariance_xy": pcd_floors[0],
            "min_covariance_yaw": pcd_floors[1],
        }
        print("global_pose_adapter:\n  ros__parameters:")
        for key, value in pcd_parameters.items():
            print(f"    {key}: {value:.6g}")

    if args.apply:
        if not passed:
            raise SystemExit("Ground-truth validation failed; not applying")
        updates = [
            ("wheel_encoder_odometry", wheel),
            ("nav_imu_adapter", {"angular_velocity_variance": imu_variance}),
        ]
        if pcd_parameters is not None:
            updates.append(("global_pose_adapter", pcd_parameters))
        for section, values in updates:
            write_parameters(profile, section, values, PROFILE_PREFIX)
        print(f"Applied to {profile}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
