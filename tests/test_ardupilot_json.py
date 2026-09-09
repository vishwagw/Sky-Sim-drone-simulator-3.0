# Part of SkySim — see repository LICENSE (non-commercial).
"""Protocol-conformance tests for bridges/ardupilot_json.py.

These validate SkySim's output against the byte layout and field contract in
ardupilot/libraries/SITL/examples/JSON/readme.md. The socket test exercises
the real UDP auto-detect/reply loop against a mock ArduPilot (no firmware
build needed); it does NOT prove behaviour against a live SITL binary, which
requires building ArduPilot and is out of scope for unit tests.
"""

import json
import socket
import struct

import pytest

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bridges"))

from ardupilot_json import (  # noqa: E402
    JSONBackend, ProtocolError, ServoPacket, StateMessage,
    _MAGIC_16, _MAGIC_32, _SIZE_16, _SIZE_32,
)


# --- servo packet (ArduPilot -> backend) ----------------------------------
def test_sizes_match_spec():
    assert _SIZE_16 == 40
    assert _SIZE_32 == 72


def test_parse_16ch_roundtrip():
    pwm = tuple(range(1000, 1000 + 16))
    raw = struct.pack("<HHI16H", _MAGIC_16, 400, 42, *pwm)
    pkt = ServoPacket.parse(raw)
    assert pkt.channels == 16
    assert pkt.frame_rate == 400
    assert pkt.frame_count == 42
    assert pkt.pwm == pwm
    assert pkt.pack() == raw  # exact byte round-trip


def test_parse_32ch_roundtrip():
    pwm = tuple(range(1000, 1000 + 32))
    raw = struct.pack("<HHI32H", _MAGIC_32, 400, 7, *pwm)
    pkt = ServoPacket.parse(raw)
    assert pkt.channels == 32
    assert pkt.pwm == pwm
    assert pkt.pack() == raw


def test_parse_rejects_bad_magic():
    raw = struct.pack("<HHI16H", 12345, 400, 1, *([1500] * 16))
    with pytest.raises(ProtocolError):
        ServoPacket.parse(raw)


def test_parse_rejects_bad_length():
    with pytest.raises(ProtocolError):
        ServoPacket.parse(b"\x00" * 41)


# --- state message (backend -> ArduPilot) ---------------------------------
def _minimal(**kw):
    base = dict(
        timestamp=2.5,
        gyro=[0.0, 0.0, 0.0],
        accel_body=[0.0, 0.0, -9.80665],
        position=[1.0, 2.0, -3.0],
        velocity=[0.1, 0.2, 0.3],
        attitude=[0.0, 0.0, 1.57],
    )
    base.update(kw)
    return StateMessage(**base)


def test_frame_is_newline_delimited_and_parses():
    frame = _minimal().to_frame()
    assert frame.startswith(b"\n") and frame.endswith(b"\n")
    obj = json.loads(frame.decode("ascii"))
    assert obj["timestamp"] == 2.5
    assert obj["imu"]["gyro"] == [0.0, 0.0, 0.0]
    assert obj["imu"]["accel_body"] == [0.0, 0.0, -9.80665]
    assert obj["position"] == [1.0, 2.0, -3.0]
    assert obj["velocity"] == [0.1, 0.2, 0.3]
    assert obj["attitude"] == [0.0, 0.0, 1.57]


def test_required_fields_present():
    obj = _minimal().to_dict()
    for key in ("timestamp", "imu", "position", "velocity"):
        assert key in obj
    assert set(obj["imu"]) == {"gyro", "accel_body"}


def test_quaternion_mode():
    obj = _minimal(attitude=None, quaternion=[1, 0, 0, 0]).to_dict()
    assert obj["quaternion"] == [1.0, 0.0, 0.0, 0.0]
    assert "attitude" not in obj


def test_exactly_one_attitude_required():
    with pytest.raises(ProtocolError):
        _minimal(attitude=None, quaternion=None).to_dict()      # neither
    with pytest.raises(ProtocolError):
        _minimal(quaternion=[1, 0, 0, 0]).to_dict()             # both


def test_optional_sensors_serialize():
    obj = _minimal(
        rangefinders=[0.4, 1.2],
        airspeed=12.3,
        windvane=(0.0, 3.5),
        velocity_wind=[3.2, 0.0, -0.7],
        rc=[1500, 1500, 1000, 1500],
        battery=(50.39, 64.01),
    ).to_dict()
    assert obj["rng_1"] == 0.4 and obj["rng_2"] == 1.2 and "rng_3" not in obj
    assert obj["airspeed"] == 12.3
    assert obj["windvane"] == {"direction": 0.0, "speed": 3.5}
    assert obj["velocity_wind"] == [3.2, 0.0, -0.7]
    assert obj["rc"] == {"rc_1": 1500, "rc_2": 1500, "rc_3": 1000, "rc_4": 1500}
    assert obj["battery"] == {"voltage": 50.39, "current": 64.01}


def test_vector_length_validated():
    with pytest.raises(ProtocolError):
        StateMessage(timestamp=0, gyro=[0, 0], accel_body=[0, 0, 0],
                     position=[0, 0, 0], velocity=[0, 0, 0],
                     attitude=[0, 0, 0]).to_dict()


# --- real UDP loop: mock ArduPilot <-> backend ----------------------------
def test_udp_autodetect_and_reply():
    """Mock ArduPilot sends a servo frame; backend must reply to the sender."""
    be = JSONBackend(host="127.0.0.1", port=0, timeout=2.0)
    be_port = be.sock.getsockname()[1]

    ap = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ap.settimeout(2.0)
    ap.bind(("127.0.0.1", 0))

    try:
        pwm = tuple([1500] * 16)
        ap.sendto(ServoPacket(400, 1, pwm).pack(), ("127.0.0.1", be_port))

        servo = be.recv_servo()
        assert servo is not None
        assert servo.pwm == pwm
        assert be.peer == ap.getsockname()      # auto-detected the sender
        assert be.frame_reset is False

        be.send_state(_minimal(timestamp=0.0025))
        frame, src = ap.recvfrom(1024)
        assert src == ("127.0.0.1", be_port)
        assert json.loads(frame.decode("ascii"))["timestamp"] == 0.0025
    finally:
        be.close()
        ap.close()


def test_frame_reset_detected_on_count_rollback():
    be = JSONBackend(host="127.0.0.1", port=0, timeout=2.0)
    be_port = be.sock.getsockname()[1]
    ap = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ap.sendto(ServoPacket(400, 100, tuple([1500] * 16)).pack(),
                  ("127.0.0.1", be_port))
        be.recv_servo()
        assert be.frame_reset is False
        # SITL restarts -> frame_count resets to 0
        ap.sendto(ServoPacket(400, 0, tuple([1500] * 16)).pack(),
                  ("127.0.0.1", be_port))
        be.recv_servo()
        assert be.frame_reset is True
    finally:
        be.close()
        ap.close()


def test_timeout_returns_none():
    be = JSONBackend(host="127.0.0.1", port=0, timeout=0.2)
    try:
        assert be.recv_servo() is None
    finally:
        be.close()
