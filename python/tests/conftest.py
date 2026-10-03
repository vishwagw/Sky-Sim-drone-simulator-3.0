"""Shared pytest fixtures for the SkySim client test-suite.

Everything here is designed to run fully offline: a pure-Python mock agent
server (``tests/mock_server.py``) stands in for Godot, so none of the tests need
a Godot install, a GPU, or real hardware.
"""
from __future__ import annotations

import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

# Make the package importable without an editable install ("import skysim").
_PYROOT = Path(__file__).resolve().parents[1]          # python/
if str(_PYROOT) not in sys.path:
    sys.path.insert(0, str(_PYROOT))


def _free_port() -> int:
    """Ask the OS for an unused localhost port."""
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _wait_until_accepting(port: int, timeout: float = 10.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), 0.25):
                return True
        except OSError:
            time.sleep(0.05)
    return False


@pytest.fixture(scope="session")
def mock_server() -> int:
    """Start ``tests/mock_server.py`` on an ephemeral port; yield the port.

    Session-scoped so the whole suite shares one server process. The server
    speaks the same agent protocol as the real Godot build, which is what lets
    these tests exercise the real client code paths offline.
    """
    port = _free_port()
    server = _PYROOT / "tests" / "mock_server.py"
    proc = subprocess.Popen(
        [sys.executable, str(server), "--port", str(port)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    try:
        if not _wait_until_accepting(port):
            proc.terminate()
            raise RuntimeError("mock server did not start listening in time")
        yield port
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


@pytest.fixture
def make_env(mock_server):
    """Factory building ``SkySimEnv`` instances bound to the mock server.

    Every env created through the factory is closed at test teardown, so tests
    don't leak sockets even when they assert their way out early.
    """
    from skysim import SkySimEnv

    created = []

    def _factory(**kwargs):
        env = SkySimEnv(port=mock_server, **kwargs)
        created.append(env)
        return env

    yield _factory

    for env in created:
        try:
            env.close()
        except Exception:
            pass
