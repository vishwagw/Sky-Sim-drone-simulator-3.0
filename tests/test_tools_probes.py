# Part of SkySim — see repository LICENSE (non-commercial).
"""CLI behaviour of the tools/ probes: --help and clean exit when no sim runs."""

import os
import socket
import subprocess
import sys

ROOT = os.path.join(os.path.dirname(__file__), "..")


def _run(script, *args):
    return subprocess.run(
        [sys.executable, os.path.join(ROOT, "tools", script), *args],
        capture_output=True, text=True, timeout=30,
    )


def _unused_port(kind):
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_probes_help_and_clean_exit_without_sim():
    for script in ("px4_probe.py", "ap_json_probe.py"):
        r = _run(script, "--help")
        assert r.returncode == 0 and "usage:" in r.stdout, r.stdout + r.stderr

    # px4_probe speaks TCP; ap_json_probe speaks UDP, so probe each on its own protocol.
    ports = {
        "px4_probe.py": str(_unused_port(socket.SOCK_STREAM)),
        "ap_json_probe.py": str(_unused_port(socket.SOCK_DGRAM)),
    }
    for script, port in ports.items():
        r = _run(script, "127.0.0.1", port)
        out = r.stdout + r.stderr
        assert r.returncode == 1, out
        assert "Traceback" not in out, out
        assert "No sim listening on 127.0.0.1:" + port in r.stdout, out
