"""In-process Gymnasium environment backed by the standalone SkySim C++ core.

Loads ``libskysim_core`` (``web/core/drone_core.cpp`` + ``web/core/capi.cpp``,
the same core that is compiled to WebAssembly for the browser) through ctypes,
so no Godot instance or WebSocket server is needed. Useful for headless,
CPU-only training and for batch evaluation.

Build the library::

    g++ -std=c++20 -O3 -shared -fPIC -Iinclude -Iweb/core \
        web/core/drone_core.cpp web/core/capi.cpp -o libskysim_core.so
"""
from __future__ import annotations

import ctypes
import os

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # pragma: no cover
    gym = None

PHYSICS_DT = 1.0 / 400.0


def load_core(path: str | None = None) -> ctypes.CDLL:
    path = path or os.environ.get("SKYSIM_CORE_LIB", "libskysim_core.so")
    lib = ctypes.CDLL(path)
    P, D, I = ctypes.c_void_p, ctypes.c_double, ctypes.c_int
    lib.skysim_create.restype = P
    lib.skysim_destroy.argtypes = [P]
    lib.skysim_reset.argtypes = [P, D, D, D]
    lib.skysim_arm.argtypes = [P, I]
    lib.skysim_set_attitude.argtypes = [P, D, D, D, D]
    lib.skysim_set_wind.argtypes = [P, D, D, D]
    lib.skysim_step.argtypes = [P, D]
    lib.skysim_step_n.argtypes = [P, D, I]
    lib.skysim_get_obs.argtypes = [P, ctypes.POINTER(D)]
    return lib


class SkySimCoreHoverEnv(gym.Env if gym else object):
    """Reach-and-hold a 3-D position from a random start, with random steady wind.

    Observation (12): position error (3), velocity (3), roll/pitch/yaw (3),
    body rates (3) — Godot world frame (Y up).
    Action (4) in [-1, 1]: roll, pitch, yaw-rate, collective (around hover).
    Control runs at ``control_hz``; physics at 400 Hz.
    """

    metadata = {"render_modes": []}

    def __init__(self, lib_path=None, control_hz=50, episode_s=8.0,
                 target=(0.0, 3.0, 0.0), max_wind=2.0):
        self.lib = load_core(lib_path)
        self.h = self.lib.skysim_create()
        self.n_sub = int(round(1.0 / (control_hz * PHYSICS_DT)))
        self.max_steps = int(episode_s * control_hz)
        self.target = np.asarray(target, dtype=np.float64)
        self.max_wind = max_wind
        self._buf = (ctypes.c_double * 12)()
        if gym:
            self.observation_space = spaces.Box(-np.inf, np.inf, (12,), np.float64)
            self.action_space = spaces.Box(-1.0, 1.0, (4,), np.float64)

    def _obs(self):
        self.lib.skysim_get_obs(self.h, self._buf)
        o = np.frombuffer(self._buf, dtype=np.float64).copy()
        o[0:3] -= self.target
        return o

    def reset(self, seed=None, options=None):
        if gym:
            super().reset(seed=seed)
            r = self.np_random
        else:
            r = np.random.default_rng(seed)
        start = self.target + np.array([r.uniform(-2, 2), r.uniform(-1.5, 2.0), r.uniform(-2, 2)])
        self.lib.skysim_reset(self.h, *start)
        w = r.uniform(-1, 1, 3) * np.array([1, 0.2, 1]) * self.max_wind
        self.lib.skysim_set_wind(self.h, *w)
        self.lib.skysim_arm(self.h, 1)
        self.t = 0
        return self._obs(), {}

    def step(self, a):
        a = np.clip(np.asarray(a, dtype=np.float64), -1, 1)
        self.lib.skysim_set_attitude(self.h, 0.4 * a[0], 0.4 * a[1], 1.0 * a[2], 0.344 + 0.3 * a[3])
        self.lib.skysim_step_n(self.h, PHYSICS_DT, self.n_sub)
        self.t += 1
        o = self._obs()
        err = np.linalg.norm(o[0:3])
        reward = -(err + 0.1 * np.linalg.norm(o[3:6]) + 0.02 * np.dot(a, a)) / self.max_steps
        crashed = abs(o[6]) > 1.2 or abs(o[7]) > 1.2 or (o[1] + self.target[1]) < 0.2
        if crashed:
            # Charge the remaining episode at a 3 m error so that surviving longer
            # always ranks higher (gives search-based learners a usable signal).
            reward -= 3.0 * (self.max_steps - self.t) / self.max_steps
        truncated = self.t >= self.max_steps
        return o, reward, bool(crashed), truncated, {"pos_err": err}

    def close(self):
        if getattr(self, "h", None):
            self.lib.skysim_destroy(self.h)
            self.h = None
