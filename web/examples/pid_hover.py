# PID hover — a browser Python policy for SkySim (Pyodide).
#
# Load this from index.html: switch to the Python tab, click "Load .py…", pick
# this file, then "Run Python ▶". It runs entirely client-side against the WASM
# physics core — no server, no install.
#
# Contract (identical to the JS control hook, but in Python):
#   obs = [px, py, pz, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]   # SkySim full state
#   return [roll, pitch, yaw_rate, throttle]                        # throttle in 0..1
#
# This mirrors python/examples/pid_hover.py, but as a stateless policy(obs) so it
# behaves identically every time you press Run (no integrator to wind up).

TARGET_ALT = 5.0        # metres
HOVER_FF   = 0.34       # measured hover throttle for the shipped quad
TILT_LIMIT = 0.30       # cap on normalised roll/pitch


def _clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def policy(obs):
    x, y, z = obs[0], obs[1], obs[2]
    vx, vy, vz = obs[3], obs[4], obs[5]

    # Altitude hold -> throttle (PD around the hover feed-forward).
    throttle = HOVER_FF + 0.15 * (TARGET_ALT - y) - 0.08 * vy
    throttle = _clip(throttle, 0.0, 1.0)

    # Gentle position hold toward the origin. +roll moves +x and +pitch moves +z
    # in SkySim's frame, so drive them with a negative gain.
    roll = _clip(-(0.35 * x + 0.45 * vx), -TILT_LIMIT, TILT_LIMIT)
    pitch = _clip(-(0.35 * z + 0.45 * vz), -TILT_LIMIT, TILT_LIMIT)

    return [roll, pitch, 0.0, throttle]
