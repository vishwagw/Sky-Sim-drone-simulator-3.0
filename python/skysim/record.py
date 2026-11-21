"""Record runs to disk and sample domain-randomization parameters.

A run is a per-episode directory containing dependency-light numpy arrays plus a
JSON manifest — the raw material for imitation learning, offline RL, or
sim-to-real dataset building.

    from skysim import SkySimEnv, NavTask, RecordRun, DomainRandomizer
    env = RecordRun(SkySimEnv(task=NavTask(), include_depth=True), out_dir="runs")
    dr = DomainRandomizer(seed=0)
    obs, info = env.reset(seed=0, options={"randomize": dr.sample()})
    ...
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import gymnasium as gym


class DomainRandomizer:
    """Samples a per-episode randomization dict from a seeded RNG. The dict is
    passed to reset(options={"randomize": ...}), applied by the server where
    possible, and logged in full so every run is reproducible."""

    def __init__(self, mass_scale=(0.85, 1.15), wind_max=3.0,
                 sensor_noise=(0.5, 2.0), spawn_jitter=1.0,
                 attitude_jitter_deg=5.0, seed=None):
        self.mass_scale = mass_scale
        self.wind_max = wind_max
        self.sensor_noise = sensor_noise
        self.spawn_jitter = spawn_jitter
        self.attitude_jitter_deg = attitude_jitter_deg
        self.rng = np.random.default_rng(seed)

    def sample(self) -> dict:
        r = self.rng
        ang = r.uniform(0, 2 * np.pi)
        mag = r.uniform(0, self.wind_max)
        return {
            "mass_scale": float(r.uniform(*self.mass_scale)),
            "wind": [float(mag * np.cos(ang)), 0.0, float(mag * np.sin(ang))],
            "sensor_noise": float(r.uniform(*self.sensor_noise)),
            "spawn_jitter": [float(r.uniform(-self.spawn_jitter, self.spawn_jitter)),
                             0.0,
                             float(r.uniform(-self.spawn_jitter, self.spawn_jitter))],
            "attitude_jitter_deg": [float(r.uniform(-self.attitude_jitter_deg, self.attitude_jitter_deg))
                                    for _ in range(3)],
        }


class RecordRun(gym.Wrapper):
    """Gym wrapper that records each episode to out_dir/<name>/.

    Buffers observations (state + depth/rgb if present), actions, rewards,
    ground truth, then writes numpy arrays + manifest.json on episode end.
    """

    def __init__(self, env, out_dir="runs", run_prefix="run", save_rgb=True):
        super().__init__(env)
        self.out_dir = Path(out_dir)
        self.run_prefix = run_prefix
        self.save_rgb = save_rgb
        self._reset_buffers()
        self._seed = None
        self._dr_requested = None
        self._dr_applied = None
        self._run_dir = None

    def _reset_buffers(self):
        self._state, self._depth, self._rgb = [], [], []
        self._gt_pos, self._actions, self._rewards = [], [], []
        self._collision, self._t = [], []

    def reset(self, *, seed=None, options=None):
        obs, info = self.env.reset(seed=seed, options=options)
        self._reset_buffers()
        self._seed = seed
        self._dr_requested = (options or {}).get("randomize")
        self._dr_applied = info.get("dr")
        self._append(obs, None, 0.0, info)
        return obs, info

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        self._append(obs, np.asarray(action, dtype=np.float32), reward, info)
        if terminated or truncated:
            self._flush(terminated, truncated)
        return obs, reward, terminated, truncated, info

    def _append(self, obs, action, reward, info):
        if isinstance(obs, dict):
            self._state.append(np.asarray(obs["state"], np.float32))
            if "depth" in obs:
                self._depth.append(np.asarray(obs["depth"], np.float32))
            if self.save_rgb and "rgb" in obs:
                self._rgb.append(np.asarray(obs["rgb"], np.uint8))
        else:
            self._state.append(np.asarray(obs, np.float32))
        self._gt_pos.append(np.asarray(info.get("gt_pos") or [np.nan] * 3, np.float32))
        self._collision.append(bool(info.get("collision", False)))
        self._t.append(float(info.get("t") or 0.0))
        if action is not None:
            self._actions.append(action)
            self._rewards.append(float(reward))

    def _flush(self, terminated, truncated):
        ts = time.strftime("%Y%m%d-%H%M%S")
        name = f"{self.run_prefix}_{ts}_seed{self._seed}"
        d = self.out_dir / name
        d.mkdir(parents=True, exist_ok=True)
        self._run_dir = d

        np.save(d / "state.npy", np.stack(self._state))
        np.save(d / "actions.npy", np.stack(self._actions) if self._actions else np.zeros((0, 4), np.float32))
        np.save(d / "rewards.npy", np.asarray(self._rewards, np.float32))
        np.save(d / "gt_pos.npy", np.stack(self._gt_pos))
        np.save(d / "collision.npy", np.asarray(self._collision, bool))
        np.save(d / "t.npy", np.asarray(self._t, np.float32))
        if self._depth:
            np.save(d / "depth.npy", np.stack(self._depth))
        if self._rgb:
            np.save(d / "rgb.npy", np.stack(self._rgb))

        manifest = {
            "protocol": 1,
            "created": ts,
            "seed": self._seed,
            "task": type(self.env.unwrapped.task).__name__,
            "state_mode": self.env.unwrapped.state_mode,
            "include_depth": self.env.unwrapped.include_depth,
            "include_rgb": self.env.unwrapped.include_rgb and self.save_rgb,
            "depth_size": [self.env.unwrapped.depth_cols, self.env.unwrapped.depth_rows],
            "rgb_size": [self.env.unwrapped.rgb_w, self.env.unwrapped.rgb_h],
            "n_obs": len(self._state),
            "n_steps": len(self._actions),
            "terminated": bool(terminated),
            "truncated": bool(truncated),
            "dr_requested": self._dr_requested,
            "dr_applied": self._dr_applied,
        }
        (d / "manifest.json").write_text(json.dumps(manifest, indent=2))
        return d


def load_run(run_dir) -> dict:
    """Load a recorded run directory into a dict of arrays + manifest."""
    d = Path(run_dir)
    out = {"manifest": json.loads((d / "manifest.json").read_text())}
    for key in ["state", "actions", "rewards", "gt_pos", "collision", "t", "depth", "rgb"]:
        f = d / f"{key}.npy"
        if f.exists():
            out[key] = np.load(f)
    return out
