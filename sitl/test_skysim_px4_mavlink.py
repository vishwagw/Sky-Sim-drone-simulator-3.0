# Part of SkySim — see repository LICENSE (non-commercial).
"""Conformance tests for bridges/skysim_px4_mavlink.py.

Field-encoding tests check the SI->MAVLink scaling. The loopback test runs the
real MAVLink-v2-over-TCP path against a mock PX4 client (pymavlink 'tcp:'), so
it exercises the actual server/accept/encode/decode round trip. It does NOT
prove behaviour against the real PX4 flight stack, which requires building PX4
SITL and is out of scope for unit tests.
"""

import os
os.environ.setdefault("MAVLINK20", "1")

import socket
import threading
import time

import pytest
from pymavlink import mavutil

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bridges"))

from skysim_px4_mavlink import (  # noqa: E402
    ActuatorControls, GpsSample, GroundTruth, PX4MavlinkBridge, SensorSample,
    FIELDS_ALL, DEFAULT_PORT,
)


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


# --- field encoding --------------------------------------------------------
def test_default_port_and_mask():
    assert DEFAULT_PORT == 4560
    assert FIELDS_ALL == 0x1FFF          # 13 fields, bits 0..12


def test_gps_scaling():
    g = GpsSample(time_usec=1_000_000, lat_deg=47.397742, lon_deg=8.545594,
                  alt_m=488.0, vel_ned=[3.0, 4.0, 0.0])
    f = g.encode_fields()
    assert f["lat"] == 473977420          # deg * 1e7
    assert f["lon"] == 85455940
    assert f["alt"] == 488000             # mm
    assert f["vn"] == 300 and f["ve"] == 400 and f["vd"] == 0   # cm/s
    assert f["vel"] == 500                # ground speed 5 m/s -> 500 cm/s
    assert f["cog"] == round((__import__("math").degrees(
        __import__("math").atan2(4.0, 3.0))) * 100)   # centidegrees
    assert f["fix_type"] == 3


# --- real TCP loopback: bridge (server) <-> mock PX4 (client) --------------
def test_hil_sensor_roundtrip():
    port = _free_port()
    bridge = PX4MavlinkBridge(host="127.0.0.1", port=port)

    # Mock PX4 connects as TCP client, exactly as PX4 SITL does.
    px4 = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", dialect="common")

    try:
        # Stream sensor frames until the mock PX4 decodes one (covers the
        # lazy TCP accept on the server side).
        got = None
        for i in range(50):
            bridge.send_sensor(SensorSample(
                time_usec=1000 * (i + 1),
                gyro=[0.01, -0.02, 0.03],
                accel=[0.0, 0.0, -9.80665],
                mag=[0.21, 0.0, 0.43],
                abs_pressure=955.0, pressure_alt=500.0, temperature=15.0,
            ))
            got = px4.recv_match(type="HIL_SENSOR", blocking=True, timeout=0.2)
            if got is not None:
                break
        assert got is not None, "mock PX4 never received HIL_SENSOR"
        assert abs(got.zacc + 9.80665) < 1e-4
        assert abs(got.xgyro - 0.01) < 1e-6
        assert got.fields_updated == FIELDS_ALL
        assert abs(got.abs_pressure - 955.0) < 1e-3
    finally:
        px4.close()
        bridge.close()


def test_actuator_controls_roundtrip():
    port = _free_port()
    bridge = PX4MavlinkBridge(host="127.0.0.1", port=port)
    px4 = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", dialect="common")

    try:
        # Prime the connection so the server accepts the client.
        for _ in range(20):
            bridge.send_sensor(SensorSample(1000, [0, 0, 0], [0, 0, -9.8]))
            if px4.recv_match(type="HIL_SENSOR", blocking=True, timeout=0.1):
                break

        # Mock PX4 replies with actuator controls (4 motors at 0.5).
        controls = [0.5, 0.5, 0.5, 0.5] + [0.0] * 12
        px4.mav.hil_actuator_controls_send(
            2000, controls, mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 0
        )

        act = bridge.recv_actuator(timeout=2.0)
        assert isinstance(act, ActuatorControls)
        assert act.time_usec == 2000
        assert act.motors(4) == (0.5, 0.5, 0.5, 0.5)
        assert len(act.controls) == 16
    finally:
        px4.close()
        bridge.close()


def test_recv_actuator_timeout_returns_none():
    port = _free_port()
    bridge = PX4MavlinkBridge(host="127.0.0.1", port=port)
    try:
        assert bridge.recv_actuator(timeout=0.2) is None
    finally:
        bridge.close()


def test_state_quaternion_encodes_and_sends():
    port = _free_port()
    bridge = PX4MavlinkBridge(host="127.0.0.1", port=port)
    px4 = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", dialect="common")
    try:
        got = None
        for _ in range(50):
            bridge.send_state(GroundTruth(
                time_usec=3000, quaternion=[1.0, 0.0, 0.0, 0.0],
                rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0,
                lat_deg=47.4, lon_deg=8.5, alt_m=500.0,
                vel_ned=[1.0, 0.0, 0.0], accel=[0.0, 0.0, -9.80665],
            ))
            got = px4.recv_match(type="HIL_STATE_QUATERNION",
                                 blocking=True, timeout=0.2)
            if got is not None:
                break
        assert got is not None
        assert got.lat == 474000000
        assert got.vx == 100                       # 1 m/s -> 100 cm/s
        assert abs(got.zacc + 1000) < 2            # -9.80665 m/s^2 -> ~-1000 mg
    finally:
        px4.close()
        bridge.close()
