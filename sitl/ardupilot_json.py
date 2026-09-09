# Part of SkySim — see repository LICENSE (non-commercial).
#
# Implements the wire protocol documented at:
#   ardupilot/libraries/SITL/examples/JSON/readme.md
#
# This module speaks a published network protocol; it copies no ArduPilot
# source and creates no derivative of GPL code. It is the reference the
# SkySim physics core conforms to when acting as an ArduPilot SITL physics
# backend ("--model JSON:<ip>").
"""ArduPilot JSON SITL physics-backend protocol for SkySim.

Data flow (ArduPilot is the flight controller, SkySim is the physics):

    ArduPilot SITL  --UDP-->  SkySim (binds :9002)     servo PWM out
    SkySim          --UDP-->  ArduPilot                JSON state in

ArduPilot auto-detects the backend: SkySim replies to whatever (ip, port)
the servo packet arrived from, so no target address needs configuring.
"""

from __future__ import annotations

import json
import socket
import struct
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

# --- SITL output (ArduPilot -> backend), binary, little-endian -------------
# 16-channel frame:  uint16 magic=18458, uint16 frame_rate, uint32 frame_count,
#                    uint16 pwm[16]
# 32-channel frame:  magic=29569, pwm[32]  (SERVO_32_ENABLE=1)
_MAGIC_16 = 18458
_MAGIC_32 = 29569
_FMT_16 = "<HHI16H"
_FMT_32 = "<HHI32H"
_SIZE_16 = struct.calcsize(_FMT_16)   # 40
_SIZE_32 = struct.calcsize(_FMT_32)   # 72

DEFAULT_LISTEN_PORT = 9002
# ArduPilot re-sends the last output frame (same frame_count) if it gets no
# reply for this long, so the backend can restart and re-detect.
KEEPALIVE_TIMEOUT_S = 10.0


class ProtocolError(ValueError):
    """Raised when a received packet is not a valid ArduPilot servo frame."""


@dataclass(frozen=True)
class ServoPacket:
    """A decoded servo-output frame from ArduPilot SITL."""

    frame_rate: int          # Hz, == SIM_RATE_HZ; advisory, may be ignored
    frame_count: int         # increments per frame; resets to 0 on SITL restart
    pwm: Tuple[int, ...]     # 16 or 32 servo values in microseconds (1000-2000)

    @property
    def channels(self) -> int:
        return len(self.pwm)

    @classmethod
    def parse(cls, data: bytes) -> "ServoPacket":
        """Decode a raw UDP payload. Raises ProtocolError on a bad frame."""
        n = len(data)
        if n == _SIZE_16:
            magic, rate, count, *pwm = struct.unpack(_FMT_16, data)
            if magic != _MAGIC_16:
                raise ProtocolError(f"bad 16ch magic {magic} (want {_MAGIC_16})")
        elif n == _SIZE_32:
            magic, rate, count, *pwm = struct.unpack(_FMT_32, data)
            if magic != _MAGIC_32:
                raise ProtocolError(f"bad 32ch magic {magic} (want {_MAGIC_32})")
        else:
            raise ProtocolError(f"unexpected length {n} (want {_SIZE_16} or {_SIZE_32})")
        return cls(frame_rate=rate, frame_count=count, pwm=tuple(pwm))

    def pack(self) -> bytes:
        """Re-encode (used by tests / mock ArduPilot). Round-trips parse()."""
        if self.channels == 16:
            return struct.pack(_FMT_16, _MAGIC_16, self.frame_rate,
                               self.frame_count, *self.pwm)
        if self.channels == 32:
            return struct.pack(_FMT_32, _MAGIC_32, self.frame_rate,
                               self.frame_count, *self.pwm)
        raise ProtocolError(f"channels must be 16 or 32, got {self.channels}")


# --- SITL input (backend -> ArduPilot), JSON text --------------------------
Vec3 = Sequence[float]


@dataclass
class StateMessage:
    """Physics state returned to ArduPilot for one step.

    Required by ArduPilot: timestamp, imu.gyro, imu.accel_body, position,
    velocity, and exactly one attitude representation (euler or quaternion).
    All optional sensors default to unset and are omitted from the frame.
    """

    timestamp: float              # s, ABSOLUTE physics time (not the timestep)
    gyro: Vec3                    # rad/s, body frame (roll, pitch, yaw)
    accel_body: Vec3             # m/s^2, body frame (x, y, z)
    position: Vec3               # m, earth NED (north, east, down)
    velocity: Vec3               # m/s, earth NED (north, east, down)
    attitude: Optional[Vec3] = None            # rad euler (roll, pitch, yaw)
    quaternion: Optional[Sequence[float]] = None   # (q1, q2, q3, q4)

    # Optional sensors (require the matching SITL sensor param upstream)
    rangefinders: Sequence[float] = field(default_factory=tuple)  # rng_1..rng_6, m
    airspeed: Optional[float] = None                              # m/s
    windvane: Optional[Tuple[float, float]] = None               # (dir rad, speed m/s)
    velocity_wind: Optional[Vec3] = None                          # m/s NED
    rc: Optional[Sequence[int]] = None                           # up to 12 chans, us
    battery: Optional[Tuple[float, float]] = None                # (volts, amps)

    def to_dict(self) -> dict:
        if (self.attitude is None) == (self.quaternion is None):
            # ArduPilot needs at least one; if both are sent it uses the
            # quaternion. We require the caller to pick exactly one to keep
            # SkySim's output unambiguous.
            raise ProtocolError("provide exactly one of attitude / quaternion")

        out: dict = {
            "timestamp": self.timestamp,
            "imu": {"gyro": _f3(self.gyro), "accel_body": _f3(self.accel_body)},
            "position": _f3(self.position),
            "velocity": _f3(self.velocity),
        }
        if self.attitude is not None:
            out["attitude"] = _f3(self.attitude)
        else:
            q = list(self.quaternion)  # type: ignore[arg-type]
            if len(q) != 4:
                raise ProtocolError("quaternion must have 4 elements")
            out["quaternion"] = [float(v) for v in q]

        for i, d in enumerate(self.rangefinders[:6], start=1):
            out[f"rng_{i}"] = float(d)
        if self.airspeed is not None:
            out["airspeed"] = float(self.airspeed)
        if self.windvane is not None:
            out["windvane"] = {"direction": float(self.windvane[0]),
                               "speed": float(self.windvane[1])}
        if self.velocity_wind is not None:
            out["velocity_wind"] = _f3(self.velocity_wind)
        if self.rc is not None:
            out["rc"] = {f"rc_{i}": int(v) for i, v in enumerate(self.rc[:12], start=1)}
        if self.battery is not None:
            out["battery"] = {"voltage": float(self.battery[0]),
                              "current": float(self.battery[1])}
        return out

    def to_frame(self) -> bytes:
        """Serialize to the on-wire frame: newline-delimited compact JSON."""
        body = json.dumps(self.to_dict(), separators=(",", ":"))
        return ("\n" + body + "\n").encode("ascii")


def _f3(v: Vec3):
    v = list(v)
    if len(v) != 3:
        raise ProtocolError(f"expected 3 elements, got {len(v)}")
    return [float(v[0]), float(v[1]), float(v[2])]


# --- socket loop -----------------------------------------------------------
class JSONBackend:
    """UDP endpoint that plays the physics backend to ArduPilot SITL.

    Typical use::

        be = JSONBackend()
        while True:
            servo = be.recv_servo()          # blocks until ArduPilot sends
            if servo is None:                # timeout keepalive
                continue
            if be.frame_reset:               # SITL restarted -> reset vehicle
                world.reset()
            state = physics.step(servo.pwm)  # your SkySim step
            be.send_state(state)
    """

    def __init__(self, host: str = "0.0.0.0", port: int = DEFAULT_LISTEN_PORT,
                 timeout: Optional[float] = KEEPALIVE_TIMEOUT_S):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        if timeout is not None:
            self.sock.settimeout(timeout)
        self._peer: Optional[Tuple[str, int]] = None
        self._last_count: Optional[int] = None
        self.frame_reset: bool = False

    @property
    def peer(self) -> Optional[Tuple[str, int]]:
        """The auto-detected ArduPilot (ip, port), or None before first frame."""
        return self._peer

    def recv_servo(self) -> Optional[ServoPacket]:
        """Receive and decode one servo frame. Returns None on socket timeout.

        Sets ``frame_reset`` when the frame_count goes backwards (ArduPilot
        restarted). Invalid frames are skipped, not raised, so a noisy port
        can't wedge the loop.
        """
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                return None
            try:
                pkt = ServoPacket.parse(data)
            except ProtocolError:
                continue  # ignore stray / malformed datagrams
            self._peer = addr  # auto-detect: reply target is the sender
            self.frame_reset = (
                self._last_count is not None and pkt.frame_count < self._last_count
            )
            self._last_count = pkt.frame_count
            return pkt

    def send_state(self, state: StateMessage) -> None:
        """Send one JSON state frame back to the detected ArduPilot peer."""
        if self._peer is None:
            raise ProtocolError("no ArduPilot peer yet; call recv_servo() first")
        self.sock.sendto(state.to_frame(), self._peer)

    def close(self) -> None:
        self.sock.close()

    def __enter__(self) -> "JSONBackend":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
