"""Env contract: action/observation spaces, Box-vs-Dict observation modes,
the Gymnasium 5-tuple, truncation at ``max_steps``, and a clean ``close()``.

These lock the observation schema so a regression in shape, dtype, or space
type fails loudly.
"""
import numpy as np
from gymnasium import spaces

from skysim import HoverTask, NavTask


def test_action_space_bounds(make_env):
    env = make_env(task=HoverTask())
    assert isinstance(env.action_space, spaces.Box)
    assert env.action_space.shape == (4,)
    np.testing.assert_array_equal(env.action_space.low, [-1, -1, -1, 0])
    np.testing.assert_array_equal(env.action_space.high, [1, 1, 1, 1])


def test_full_state_is_box_12(make_env):
    env = make_env(task=HoverTask(), state_mode="full")
    assert isinstance(env.observation_space, spaces.Box)
    assert env.observation_space.shape == (12,)
    obs, info = env.reset(seed=0)
    assert obs.shape == (12,)
    assert obs.dtype == np.float32
    assert {"collision", "telemetry", "gt_pos", "t", "dr"}.issubset(info)


def test_gps_denied_state_is_box_9(make_env):
    env = make_env(task=HoverTask(), state_mode="gps_denied")
    assert env.observation_space.shape == (9,)
    obs, _ = env.reset(seed=0)
    assert obs.shape == (9,)


def test_depth_switches_to_dict_space(make_env):
    env = make_env(task=NavTask(), state_mode="gps_denied",
                   include_depth=True, depth_size=(32, 24))
    assert isinstance(env.observation_space, spaces.Dict)
    assert set(env.observation_space.spaces) == {"state", "depth"}
    obs, _ = env.reset(seed=0)
    assert isinstance(obs, dict)
    assert obs["state"].shape == (9,)
    assert obs["depth"].shape == (24, 32)          # (rows, cols)
    assert obs["depth"].min() >= 0.0               # ranges are non-negative


def test_rgb_switches_to_dict_space(make_env):
    env = make_env(task=HoverTask(), include_rgb=True, rgb_size=(128, 96))
    assert isinstance(env.observation_space, spaces.Dict)
    assert "rgb" in env.observation_space.spaces
    obs, _ = env.reset(seed=0)
    assert obs["rgb"].shape == (96, 128, 3)        # (h, w, 3)
    assert obs["rgb"].dtype == np.uint8


def test_step_returns_gym_five_tuple(make_env):
    env = make_env(task=HoverTask(), max_steps=10)
    env.reset(seed=0)
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
    assert obs.shape == (12,)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert isinstance(info, dict)


def test_truncation_fires_exactly_at_max_steps(make_env):
    env = make_env(task=HoverTask(), max_steps=5)
    env.reset(seed=0)
    hover = np.array([0, 0, 0, 0.5], np.float32)
    truncs = [env.step(hover)[3] for _ in range(5)]
    assert truncs[-1] is True
    assert not any(truncs[:-1])


def test_close_is_idempotent(make_env):
    env = make_env(task=HoverTask())
    env.reset(seed=0)
    env.close()
    env.close()   # must not raise
