import math
import sys
import unittest
from contextlib import ExitStack, redirect_stdout
from dataclasses import replace
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import yaml

SCRIPTS = (
    Path(__file__).resolve().parents[1]
    / "ros2_ws/src/slam_localization_3d/scripts"
)
sys.path.insert(0, str(SCRIPTS))
import covariance_calibration as calibration  # noqa: E402


def window(rng, distance, turn, duration, wheel_yaw, imu, lio_yaw, wheel_distance, lio_distance):
    true_yaw = rng.uniform(-1, 1)
    true_distance = distance
    lio_sigma = math.sqrt(lio_distance[0] + lio_distance[1] * turn)
    wheel_yaw_variance = wheel_yaw[0] * distance + wheel_yaw[1] * turn
    return calibration.Window(
        duration=duration,
        distance=distance,
        turn=turn,
        wheel=np.array([
            true_distance
            + rng.normal(0, math.sqrt(wheel_distance[0] * distance + wheel_distance[1] * turn)),
            # Heading error accumulated along the path displaces the end point sideways.
            rng.normal(0, distance * math.sqrt(wheel_yaw_variance / 3.0)),
            true_yaw + rng.normal(0, math.sqrt(wheel_yaw_variance)),
        ]),
        imu_yaw=true_yaw + rng.normal(0, math.sqrt(imu * duration)),
        lio=np.array([
            true_distance + rng.normal(0, lio_sigma), rng.normal(0, lio_sigma),
            true_yaw + rng.normal(0, math.sqrt(lio_yaw)),
        ]),
        truth=np.array([true_distance, 0.0, true_yaw]),
    )


class TestCovarianceCalibration(unittest.TestCase):
    def test_relative_motion(self):
        start, end = (1.0, 2.0, math.pi / 2), (1.0, 3.0, math.pi)
        np.testing.assert_allclose(
            calibration.relative_motion(start, end), [1.0, 0.0, math.pi / 2], atol=1e-12
        )

    def test_three_cornered_hat_recovers_noise_without_truth(self):
        rng = np.random.default_rng(3)
        truth = {
            "wheel_yaw": (2e-4, 1e-3), "imu": 5e-5, "lio_yaw": 2e-5,
            "wheel_distance": (1e-4, 2e-5), "lio_distance": (3e-6, 5e-5),
        }
        windows = []
        for _ in range(3000):
            kind = rng.integers(3)
            distance = (0.0, 0.5, 0.6)[kind]
            turn = (1.6, 0.0, 0.6)[kind]
            windows.append(window(
                rng, distance, turn, 2.0, truth["wheel_yaw"], truth["imu"],
                truth["lio_yaw"], truth["wheel_distance"], truth["lio_distance"],
            ))
        noise = calibration.fit_increment_noise(windows)
        self.assertAlmostEqual(noise["yaw_variance_per_meter"] / 2e-4, 1, delta=0.25)
        self.assertAlmostEqual(noise["yaw_variance_per_radian"] / 1e-3, 1, delta=0.15)
        self.assertAlmostEqual(noise["imu_yaw_variance_per_second"] / 5e-5, 1, delta=0.25)
        self.assertAlmostEqual(noise["distance_variance_per_meter"] / 1e-4, 1, delta=0.25)
        # Along-track LIO turn error cannot be told apart from wheel scrub: upper bound.
        self.assertAlmostEqual(noise["position_variance_per_radian"] / 7e-5, 1, delta=0.15)
        spin_lateral = (noise["lio_lateral_variance_per_window"]
                        + 1.6 * noise["lio_lateral_variance_per_radian"])
        self.assertAlmostEqual(spin_lateral / (3e-6 + 1.6 * 5e-5), 1, delta=0.15)

        report = calibration.ground_truth_validation(
            FakeTruthData(), windows, noise, noise["imu_yaw_variance_per_second"]
        )
        for name in ("wheel_yaw", "imu_yaw"):
            self.assertTrue(0.8 < report[name][0] < 1.25, (name, report[name]))
        # LIO along-track turn error is booked to the wheel, so the prediction is high.
        self.assertLess(report["wheel_distance"][0], 1.0)
        self.assertNotEqual(calibration.verdict(report["wheel_distance"][0], True), "FAIL")

    def test_imu_lio_bound_covers_gyro_scale_error_correlated_with_wheels(self):
        rng = np.random.default_rng(5)
        windows = []
        for _ in range(2000):
            kind = rng.integers(3)
            distance = (0.0, 0.5, 0.6)[kind]
            turn = (1.6, 0.0, 0.6)[kind]
            w = window(rng, distance, turn, 2.0, (2e-4, 1e-4), 5e-6, 2e-6, (1e-4, 2e-5), (3e-6, 5e-5))
            true_yaw = rng.choice((-1.0, 1.0)) * turn
            shift = true_yaw - w.truth[2]
            # Gyro and wheels both overstate turns: their errors are correlated.
            windows.append(replace(
                w,
                imu_yaw=w.imu_yaw + shift + 0.05 * true_yaw,
                wheel=w.wheel + np.array([0.0, 0.0, shift + 0.03 * true_yaw]),
                lio=w.lio + np.array([0.0, 0.0, shift]),
                truth=np.array([w.truth[0], 0.0, true_yaw]),
            ))
        noise = calibration.fit_increment_noise(windows)
        self.assertAlmostEqual(noise["imu_yaw_scale_vs_lio"], 1.05, delta=0.005)
        hat = calibration.ground_truth_validation(
            FakeTruthData(), windows, noise, noise["imu_yaw_variance_per_second"]
        )
        self.assertGreater(hat["imu_yaw"][0], 2.0)
        bound = calibration.ground_truth_validation(
            FakeTruthData(), windows, noise, noise["imu_lio_yaw_variance_per_second"]
        )
        self.assertNotEqual(calibration.verdict(bound["imu_yaw"][0], True), "FAIL")
        self.assertLess(bound["imu_yaw"][0], 1.25)

    def test_weighted_nnls_drops_gross_outliers(self):
        rng = np.random.default_rng(1)
        x = rng.uniform(0.1, 1.0, 400)
        target = 0.01 * x * rng.chisquare(1, 400)
        target[:5] = 100.0
        solution, keep = calibration.weighted_nnls(x[:, None], target)
        self.assertFalse(keep[:5].any())
        self.assertAlmostEqual(solution[0] / 0.01, 1, delta=0.2)

    def test_pcd_variance_sees_slowly_correlated_error(self):
        rng = np.random.default_rng(11)
        data = FakePcdData(rng, sigma=(0.01, 0.006, 0.002), correlation_time=8.0, duration=4000.0)
        estimate = calibration.pcd_error_variance(data, 10.0, 60.0)
        np.testing.assert_allclose(estimate["variance"] / np.square((0.01, 0.006, 0.002)), 1, atol=0.3)
        # Adjacent poses share most of the error, so short lags underestimate it.
        self.assertLess(estimate["curve"][0][3][0], 0.5 * estimate["variance"][0])

    def test_pcd_floors_remove_icp_share(self):
        icp = np.array([[1e-5, 2e-5, 1e-7]] * 5)
        xy, yaw = calibration.recommend_pcd_floors((4e-5, 5e-5, 1e-6), icp, factor=2.0)
        self.assertAlmostEqual(xy, 2 * 5e-5 - 2e-5)
        self.assertAlmostEqual(yaw, 2 * 1e-6 - 1e-7)
        xy, yaw = calibration.recommend_pcd_floors((1e-9, 1e-9, 1e-12), icp, factor=1.0)
        self.assertEqual((xy, yaw), (1e-6, 1e-8))
        published = np.array([np.diag((3e-4, 3e-4, 2e-5))])
        np.testing.assert_allclose(
            calibration.pcd_icp_part(FakeCovariance(published), 2.0, 1e-4, 1e-5), [[5e-5, 5e-5, 0.0]]
        )

    def test_pcd_ground_truth_ratio_per_axis(self):
        data = FakePcdData(np.random.default_rng(3), sigma=(0.01, 0.01, 0.002), correlation_time=0.1)
        data.truth = calibration.Trajectory.create(data.lio.t, data.lio.x, data.lio.y, data.lio.yaw)
        report = calibration.ground_truth_validation(data, [], _noise(), 0.0)["pcd"]
        expected = np.square((0.01, 0.01, 0.002)) / np.diag(data.pcd_covariance[0])
        np.testing.assert_allclose(report["ratio"], expected, rtol=0.25)

    def test_verdict_accepts_overstated_upper_bounds(self):
        self.assertEqual(calibration.verdict(1.0), "PASS")
        self.assertEqual(calibration.verdict(0.4), "CONSERVATIVE")
        self.assertEqual(calibration.verdict(0.2), "FAIL")
        self.assertEqual(calibration.verdict(0.2, upper_bound=True), "UPPER BOUND")
        self.assertEqual(calibration.verdict(3.0, upper_bound=True), "FAIL")

    def test_ground_truth_checks_recommended_imu_variance_including_floors(self):
        data = SimpleNamespace(
            wheel_t=np.arange(0.0, 10.0, 0.1), wheel_vx=np.zeros(100),
            wheel_wz=np.zeros(100), imu_t=np.arange(0.0, 10.0, 0.02),
            imu_wz=np.tile([-0.1, 0.1], 250),
        )
        noise = {**_noise(), **dict.fromkeys((
            "imu_yaw_variance_per_second", "imu_lio_yaw_variance_per_second",
            "lio_yaw_variance_per_window",
            "lio_distance_variance_per_window", "lio_lateral_variance_per_window",
            "lio_lateral_variance_per_radian",
        ), 1e-8), "imu_yaw_scale_vs_lio": 1.0, "yaw_rows_used": 60, "yaw_rows": 60}
        report = {
            "imu_yaw": (1.0, 1e-4),
            "pcd": {"ratio": [1.0] * 3, "rmse_xy": 0.01, "rmse_yaw": 0.001,
                    "yaw_bias": 0.0, "count": 20},
        }
        for minimum in (0.03, 1e-6):
            with self.subTest(minimum=minimum), ExitStack() as stack:
                stack.enter_context(redirect_stdout(StringIO()))
                stack.enter_context(patch.object(calibration, "load_data", return_value=data))
                stack.enter_context(patch.object(calibration, "build_windows",
                                                return_value=[None] * 20))
                stack.enter_context(patch.object(calibration, "fit_increment_noise",
                                                return_value=noise))
                stack.enter_context(patch.object(calibration, "wheel_systematic",
                                                return_value=(0.0, 1.0)))
                stack.enter_context(patch.object(calibration, "pcd_error_variance",
                                                return_value=None))
                stack.enter_context(patch.object(calibration, "pcd_repeatability",
                                                return_value=None))
                validation = stack.enter_context(patch.object(
                    calibration, "ground_truth_validation", return_value=report))
                self.assertEqual(calibration.main([
                    "bag", "--ground-truth", "--keep-lever",
                    "--min-imu-variance", str(minimum),
                ]), 0)
                segments = calibration.static_segments(
                    data.wheel_t, data.wheel_vx, data.wheel_wz, 5.0, 1.0)
                static_variance = np.var(
                    data.imu_wz[calibration.in_segments(data.imu_t, segments)], ddof=1)
                self.assertAlmostEqual(
                    validation.call_args.args[3],
                    max(minimum, static_variance) * np.median(np.diff(data.imu_t)),
                )

    def test_align_se2(self):
        source = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 2.0]])
        angle = 0.3
        rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        target = source @ rotation.T + [2.0, -1.0]
        found, found_rotation, translation = calibration.align_se2(source, target)
        self.assertAlmostEqual(found, angle)
        np.testing.assert_allclose(source @ found_rotation.T + translation, target, atol=1e-12)

    def test_static_segments_trim_edges(self):
        t = np.arange(0.0, 20.0, 0.1)
        vx = np.where((t > 5) & (t < 7), 0.3, 0.0)
        segments = calibration.static_segments(t, vx, vx * 0, 2.0, 0.5)
        self.assertEqual(len(segments), 2)
        self.assertAlmostEqual(segments[0][0], 0.5)
        self.assertAlmostEqual(segments[1][1], t[-1] - 0.5)

    def test_parameter_write_replaces_and_inserts(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text(
                "first:\n  ros__parameters:\n    value: 1.0  # keep\n"
                "second:\n  ros__parameters:\n    value: 2.0\n"
            )
            calibration.write_parameters(path, "first", {"value": 0.5, "added": 3e-5})
            self.assertEqual(calibration.read_parameter(path, "first", "value"), 0.5)
            self.assertEqual(calibration.read_parameter(path, "first", "added"), 3e-5)
            self.assertEqual(calibration.read_parameter(path, "second", "value"), 2.0)
            self.assertIn("# keep", path.read_text())

    def test_profile_override_write_creates_and_updates_nested_sections(self):
        prefix = calibration.PROFILE_PREFIX
        with TemporaryDirectory() as directory:
            path = Path(directory) / "robot.yaml"
            path.write_text(
                "simulation:\n  wheel_radius: 0.1\n"
                "# Overrides.\nparameter_overrides: {}\n"
            )
            calibration.write_parameters(path, "global_pose_adapter", {"min_covariance_xy": 2e-4},
                                         prefix)
            calibration.write_parameters(path, "nav_imu_adapter",
                                         {"angular_velocity_variance": 1e-3}, prefix)
            calibration.write_parameters(path, "global_pose_adapter", {"min_covariance_xy": 3e-4},
                                         prefix)
            self.assertEqual(calibration.read_parameter(
                path, "global_pose_adapter", "min_covariance_xy", prefix), 3e-4)
            self.assertEqual(calibration.read_parameter(
                path, "nav_imu_adapter", "angular_velocity_variance", prefix), 1e-3)
            self.assertIsNone(calibration.read_parameter(path, "global_pose_adapter",
                                                         "min_covariance_xy"))
            profile = yaml.safe_load(path.read_text())
            self.assertEqual(profile["simulation"], {"wheel_radius": 0.1})
            self.assertEqual(profile["parameter_overrides"], {
                "global_pose_adapter": {"ros__parameters": {"min_covariance_xy": 3e-4}},
                "nav_imu_adapter": {"ros__parameters": {"angular_velocity_variance": 1e-3}},
            })

    def test_top_level_section_ignores_nested_keys_with_the_same_name(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text(
                "other:\n  first:\n    ros__parameters:\n      value: 9.0\n"
                "first:\n  ros__parameters:\n    value: 1.0\n"
            )
            self.assertEqual(calibration.read_parameter(path, "first", "value"), 1.0)
            calibration.write_parameters(path, "first", {"value": 2.0})
            self.assertEqual(yaml.safe_load(path.read_text())["other"]["first"],
                             {"ros__parameters": {"value": 9.0}})
            with self.assertRaises(SystemExit):
                calibration.write_parameters(path, "missing", {"value": 1.0})

    def test_robot_profile_supplies_active_values(self):
        root = Path(__file__).resolve().parents[1] / "ros2_ws/src"
        profile = calibration.robot_profile_path(calibration.parse_args(["bag"]))
        self.assertEqual(profile, root / "slam_localization_3d/config/robots/nova_carter.yaml")
        for key in ("registration_covariance_scale", "min_covariance_xy", "min_covariance_yaw"):
            self.assertIsNotNone(calibration.read_parameter(
                profile, "global_pose_adapter", key, calibration.PROFILE_PREFIX))
        for robot_type in ("missing_robot", "../nova_carter"):
            with self.subTest(robot_type=robot_type), self.assertRaises(SystemExit):
                calibration.robot_profile_path(
                    calibration.parse_args(["bag", "--robot-type", robot_type]))

    def test_lio_body_to_base_defaults_to_profile_mount(self):
        from localization_3d_pose import BODY_TO_BASE
        args = calibration.parse_args(["bag"])
        for actual, expected in zip(args.lio_body_to_base, (*BODY_TO_BASE[0], math.pi)):
            self.assertAlmostEqual(actual, expected, places=8)
        with TemporaryDirectory() as directory:
            Path(directory, "other.yaml").write_text(yaml.safe_dump({"sensor_frames": {"imu_link": {
                "x": 0.1, "y": 0.0, "z": 0.3, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}}}))
            args = calibration.parse_args(
                ["bag", "--robot-type", "other", "--profile-dir", directory])
            self.assertEqual(args.lio_body_to_base, [-0.1, 0.0, -0.3, 0.0])
            args = calibration.parse_args(["bag", "--robot-type", "other", "--profile-dir",
                                           directory, "--lio-body-to-base", "1", "2", "3", "0"])
            self.assertEqual(args.lio_body_to_base, [1.0, 2.0, 3.0, 0.0])

    def test_yaml_float_is_always_a_double(self):
        self.assertEqual(calibration.yaml_float(1), "1.0")
        self.assertEqual(calibration.yaml_float(1e-6), "1.0e-06")
        self.assertEqual(calibration.yaml_float(8.3806e-06), "8.3806e-06")
        self.assertEqual(calibration.yaml_float(0.000886391), "0.000886391")

    def test_repository_profiles_define_calibrated_parameters(self):
        root = Path(__file__).resolve().parents[1] / "ros2_ws/src"
        prefix = calibration.PROFILE_PREFIX
        for profile in sorted((root / "slam_localization_3d/config/robots").glob("*.yaml")):
            with self.subTest(profile=profile.name):
                for key in ("distance_variance_per_meter", "position_variance_per_radian",
                            "yaw_variance_per_meter", "yaw_variance_per_radian"):
                    self.assertIsNotNone(calibration.read_parameter(
                        profile, "wheel_encoder_odometry", key, prefix))
                self.assertIsNotNone(calibration.read_parameter(
                    profile, "nav_imu_adapter", "angular_velocity_variance", prefix))
                for key in ("registration_covariance_scale", "min_covariance_xy",
                            "min_covariance_yaw"):
                    self.assertIsNotNone(calibration.read_parameter(
                        profile, "global_pose_adapter", key, prefix))

def _noise():
    return {key: 0.0 for key in (
        "yaw_variance_per_meter", "yaw_variance_per_radian",
        "distance_variance_per_meter", "position_variance_per_radian",
    )}


class FakeTruthData:
    """PCD section of ground_truth_validation with perfectly consistent poses."""

    def __init__(self):
        t = np.arange(0.0, 10.0, 1.0)
        self.truth = calibration.Trajectory.create(t, t, t * 0, t * 0)
        self.pcd_t = t
        self.pcd_pose = np.column_stack((t + 0.01, t * 0, t * 0))
        self.pcd_covariance = np.array([np.diag((1e-4, 1e-4, 1e-4))] * len(t))


class FakeCovariance:
    def __init__(self, covariance):
        self.pcd_covariance = covariance


class FakePcdData:
    """Circular drive with AR(1) PCD pose error and perfect FAST-LIO."""

    def __init__(self, rng, sigma, correlation_time, period=2.0, duration=900.0):
        t = np.arange(0.0, duration, 0.05)
        yaw = 0.4 * np.sin(t / 20.0) + t / 30.0
        self.lio = calibration.Trajectory.create(t, np.cos(t / 40.0), np.sin(t / 25.0), yaw)
        self.pcd_t = np.arange(1.0, duration - 1.0, period)
        rho = math.exp(-period / correlation_time)
        error = np.zeros((len(self.pcd_t), 3))
        error[0] = rng.normal(0, sigma)
        for index in range(1, len(error)):
            error[index] = rho * error[index - 1] + rng.normal(0, sigma) * math.sqrt(1 - rho * rho)
        self.pcd_pose = np.column_stack(self.lio.at(self.pcd_t)) + error
        self.pcd_covariance = np.array([np.diag((2e-4, 2e-4, 1e-5))] * len(self.pcd_t))


if __name__ == "__main__":
    unittest.main()
