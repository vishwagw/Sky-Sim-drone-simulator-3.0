"""Tests for the in-process Gymnasium environment backed by the C++ core."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env, check_reset_seed_determinism

from skysim import core_env


class _FakeCore:
    """Minimal stand-in for the C API, used to test Python-side env logic offline."""

    def __init__(self):
        self.start = np.zeros(3)
        self.wind = np.zeros(3)
        self.attitude = np.zeros(4)
        self.t = 0
        self.armed = False
        self.destroyed = False

    def skysim_create(self):
        return 1

    def skysim_destroy(self, handle):
        self.destroyed = True

    def skysim_reset(self, handle, x, y, z):
        self.start = np.array([x, y, z], dtype=np.float64)
        self.t = 0

    def skysim_arm(self, handle, armed):
        self.armed = bool(armed)

    def skysim_set_attitude(self, handle, roll, pitch, yaw, collective):
        self.attitude = np.array([roll, pitch, yaw, collective], dtype=np.float64)

    def skysim_set_wind(self, handle, wx, wy, wz):
        self.wind = np.array([wx, wy, wz], dtype=np.float64)

    def skysim_step_n(self, handle, dt, n):
        self.t += 1

    def skysim_get_obs(self, handle, buf):
        obs = np.zeros(12, dtype=np.float64)
        obs[0:3] = self.start
        obs[3:6] = self.wind
        obs[6:10] = self.attitude
        obs[10] = self.t
        for i, value in enumerate(obs):
            buf[i] = float(value)


@pytest.fixture
def fake_core(monkeypatch):
    core = _FakeCore()
    monkeypatch.setattr(core_env, "load_core", lambda path=None: core)
    return core


def _make_env():
    return core_env.SkySimCoreHoverEnv(lib_path="fake")


def test_reset_seeds_gymnasium_rng(fake_core):
    """A seeded reset must initialise Gymnasium's own RNG, not a private one."""
    env = _make_env()
    try:
        check_reset_seed_determinism(env)
    finally:
        env.close()


def test_core_env_passes_gymnasium_checker(fake_core):
    """The env must satisfy Gymnasium's full API contract."""
    env = _make_env()
    try:
        check_env(env, skip_render_check=True)
    finally:
        env.close()


def test_same_seed_repeat_gives_same_observation(fake_core):
    """Repeating reset(seed=1) must reproduce the same initial observation."""
    env = _make_env()
    try:
        first, _ = env.reset(seed=1)
        second, _ = env.reset(seed=1)
        np.testing.assert_array_equal(first, second)
    finally:
        env.close()


@pytest.fixture(scope="session")
def compiled_core_lib(tmp_path_factory):
    """Build libskysim_core once per session; skip if no usable C++ compiler."""
    compiler = shutil.which("g++") or shutil.which("c++")
    if not compiler:
        pytest.skip("no usable C++ compiler on PATH")

    tmp_path = tmp_path_factory.mktemp("skysim-core")
    repo_root = Path(__file__).resolve().parents[2]
    library = tmp_path / "libskysim_core.so"
    command = [
        compiler,
        "-std=c++20",
        "-O3",
        "-shared",
        "-fPIC",
        "-I",
        str(repo_root / "include"),
        "-I",
        str(repo_root / "web" / "core"),
        str(repo_root / "web" / "core" / "drone_core.cpp"),
        str(repo_root / "web" / "core" / "capi.cpp"),
        "-o",
        str(library),
    ]
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    if result.returncode != 0:
        detail = result.stderr.strip().splitlines()
        pytest.skip(
            "no usable C++ compiler: " + (detail[-1] if detail else "build failed")
        )
    return library


@pytest.mark.xfail(
    strict=True,
    reason="needs #38 (DroneCore::reset leaks state)",
)
def test_check_env_against_compiled_core(compiled_core_lib):
    """check_env must pass against the real compiled core once #38 lands."""
    env = core_env.SkySimCoreHoverEnv(lib_path=str(compiled_core_lib))
    try:
        check_env(env, skip_render_check=True)
    finally:
        env.close()
