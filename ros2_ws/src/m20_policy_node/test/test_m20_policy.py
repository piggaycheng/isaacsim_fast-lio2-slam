import numpy as np

from m20_policy_node import m20_policy as m20


def test_default_pose_obs_is_zero_relative():
    obs = m20.build_observation(
        np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.array([0.5, 0, 0.1]),
        m20.DEFAULT_LEG_POS, np.zeros(12), np.zeros(4), np.zeros(16),
    )
    assert obs.shape == (m20.FLAT_OBS_SIZE,)
    np.testing.assert_allclose(obs[6:9], [0, 0, -1])  # upright: gravity along -z of the base
    np.testing.assert_allclose(obs[9:12], [0.5, 0, 0.1])
    np.testing.assert_allclose(obs[12:24], 0)
    np.testing.assert_allclose(obs[-4:], 0)


def test_gravity_follows_base_tilt():
    # 90 deg roll about x: in the base frame gravity points along -y
    s = np.sqrt(0.5)
    obs = m20.build_observation(
        np.zeros(3), np.zeros(3), np.array([s, s, 0, 0]), np.zeros(3),
        m20.DEFAULT_LEG_POS, np.zeros(12), np.zeros(4), np.zeros(16),
    )
    np.testing.assert_allclose(obs[6:9], [0, -1, 0], atol=1e-6)


def test_action_targets_scaled_and_clipped():
    leg, wheel = m20.action_to_targets(np.concatenate([np.full(12, 4.0), [1, -1, 100, -100]]))
    np.testing.assert_allclose(leg, m20.DEFAULT_LEG_POS + 1.0)
    np.testing.assert_allclose(wheel, [10, -10, 60, -60])
    leg, _ = m20.action_to_targets(np.zeros(16))
    np.testing.assert_allclose(leg, m20.DEFAULT_LEG_POS)


def make_map(z, center=(0.0, 0.0), res=0.1):
    return m20.ElevationMap(z, np.array(center), np.array(z.shape) * res, res)


def test_scan_layout_matches_training_grid():
    assert m20.SCAN_SIZE == 187 and m20.OBS_SIZE == 243
    np.testing.assert_allclose(m20.SCAN_POINTS[:2], [[-0.8, -0.5], [-0.7, -0.5]])  # x varies fastest


def test_height_scan_flat_ground_and_unknown_cells():
    ground = make_map(np.zeros((100, 100), dtype=np.float32))
    scan = m20.height_scan_observation(ground.sample, np.array([0.0, 0.0, 0.49]), 0.0)
    np.testing.assert_allclose(scan, -0.01, atol=1e-6)
    off_map = make_map(np.full((10, 10), np.nan, dtype=np.float32))
    scan = m20.height_scan_observation(off_map.sample, np.array([0.0, 0.0, 3.0]), 0.0)
    np.testing.assert_allclose(scan, m20.STANDING_HEIGHT - 0.5)  # unknown: assumed standing on flat ground


def test_height_scan_follows_yaw_and_clips():
    z = np.zeros((100, 100), dtype=np.float32)
    z[:50, :] = 0.2  # rows 0..49 cover world x in (0, 5]: a 0.2 m step up at x = 0
    ground = make_map(z)
    base = np.array([0.0, 0.0, 0.49])
    scan = m20.height_scan_observation(ground.sample, base, 0.0).reshape(11, 17)
    np.testing.assert_allclose(scan[5, 9:], -0.21, atol=1e-6)  # ahead (+x): ground 0.2 m higher
    np.testing.assert_allclose(scan[5, :8], -0.01, atol=1e-6)
    turned = m20.height_scan_observation(ground.sample, base, np.pi).reshape(11, 17)
    np.testing.assert_allclose(turned[5, :8], -0.21, atol=1e-6)  # the step is now behind the robot
    deep = m20.height_scan_observation(ground.sample, np.array([0.0, 0.0, 5.0]), 0.0)
    assert deep.max() == m20.SCAN_CLIP
