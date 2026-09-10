# Part of SkySim — see repository LICENSE (non-commercial).
#
# Implements PX4's Simulator MAVLink API, documented at:
#   PX4-Autopilot/docs/en/simulation/index.md  ("Simulator MAVLink API")
#
# This module speaks a published MAVLink protocol via pymavlink; it copies no
# PX4 source and is not a derivative of PX4. Connection direction per the PX4
# docs: the SIMULATOR listens on TCP 4560 and PX4 connects to it, so SkySim is
# the TCP server. Messages: sim -> PX4 sends HIL_SENSOR / HIL_GPS / (optional)
# HIL_STATE_QUATERNION; PX4 -> sim sends HIL_ACTUATOR_CONTROLS. Lockstep is on:
# PX4 advances its clock from the sim's timestamps and waits for sensor data,
# while the sim waits for the actuator reply before stepping physics.
"""PX4 Simulator MAVLink API bridge for SkySim."""

from __future__ import annotations

import os
os.environ.setdefault("MAVLINK20", "1")  # PX4 uses MAVLink v2; set before import

import math
import select
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

from pymavlink import mavutil

DEFAULT_PORT = 4560
GRAVITY_MSS = 9.80665

# HIL_SENSOR.fields_updated is a bitmask of which fields are valid. Bits 0..12
# map to xacc,yacc,zacc,xgyro,ygyro,zgyro,xmag,ymag,zmag,abs_pressure,
# diff_pressure,pressure_alt,temperature. 0x1FFF marks all 13 as present.
FIELDS_ALL = 0x1FFF


# --- sim -> PX4 -------------------------------------------------------------
@dataclass
class SensorSample:
    """HIL_SENSOR payload: IMU/mag/baro in SI units, NED body frame."""

    time_usec: int
    gyro: Sequence[float]            # rad/s  (x, y, z) body
    accel: Sequence[float]           # m/s^2  (x, y, z) body
    mag: Sequence[float] = (0.0, 0.0, 0.0)      # Gauss (x, y, z) body
    abs_pressure: float = 1013.25    # hPa (mbar)
    diff_pressure: float = 0.0       # hPa
    pressure_alt: float = 0.0        # m
    temperature: float = 20.0        # degC
    fields_updated: int = FIELDS_ALL
    sensor_id: int = 0


@dataclass
class GpsSample:
    """HIL_GPS payload built from SI inputs (scaling handled here)."""

    time_usec: int
    lat_deg: float
    lon_deg: float
    alt_m: float
    vel_ned: Sequence[float] = (0.0, 0.0, 0.0)   # m/s (north, east, down)
    fix_type: int = 3                            # 3D fix
    satellites_visible: int = 10
    eph_m: float = 1.0
    epv_m: float = 1.0

    def encode_fields(self) -> dict:
        vn, ve, vd = (float(v) for v in self.vel_ned)
        ground_speed = math.hypot(vn, ve)
        cog_deg = (math.degrees(math.atan2(ve, vn)) + 360.0) % 360.0
        return dict(
            time_usec=self.time_usec,
            fix_type=self.fix_type,
            lat=int(round(self.lat_deg * 1e7)),      # degE7
            lon=int(round(self.lon_deg * 1e7)),
            alt=int(round(self.alt_m * 1000.0)),     # mm (MSL)
            eph=int(round(self.eph_m * 100.0)),      # cm
            epv=int(round(self.epv_m * 100.0)),
            vel=int(round(ground_speed * 100.0)),    # cm/s
            vn=int(round(vn * 100.0)),               # cm/s
            ve=int(round(ve * 100.0)),
            vd=int(round(vd * 100.0)),
            cog=int(round(cog_deg * 100.0)),         # centidegrees
            satellites_visible=self.satellites_visible,
        )


@dataclass
class GroundTruth:
    """HIL_STATE_QUATERNION payload: optional ground-truth for estimator eval."""

    time_usec: int
    quaternion: Sequence[float]      # (w, x, y, z)
    rollspeed: float                 # rad/s
    pitchspeed: float
    yawspeed: float
    lat_deg: float
    lon_deg: float
    alt_m: float
    vel_ned: Sequence[float]         # m/s (n, e, d)
    accel: Sequence[float]           # m/s^2 body (x, y, z)
    ind_airspeed: float = 0.0        # m/s
    true_airspeed: float = 0.0       # m/s


# --- PX4 -> sim -------------------------------------------------------------
@dataclass(frozen=True)
class ActuatorControls:
    """Decoded HIL_ACTUATOR_CONTROLS: normalized outputs from PX4."""

    time_usec: int
    controls: Tuple[float, ...]      # 16 values; motors normalized 0..1
    mode: int
    flags: int

    def motors(self, n: int) -> Tuple[float, ...]:
        """First n control channels (the motor outputs for an n-rotor)."""
        return self.controls[:n]


# --- bridge -----------------------------------------------------------------
class PX4MavlinkBridge:
    """TCP server (SkySim side) for PX4's Simulator MAVLink API.

    SkySim listens on ``tcpin:<host>:<port>``; PX4 SITL connects to it. The
    sim streams HIL_SENSOR (and HIL_GPS at a lower rate) and reads
    HIL_ACTUATOR_CONTROLS back. Typical lockstep loop::

        with PX4MavlinkBridge() as bridge:
            t = 0
            while True:
                bridge.send_sensor(SensorSample(t, gyro, accel, mag, ...))
                if t % 100 == 0:
                    bridge.send_gps(GpsSample(t, lat, lon, alt, vel_ned))
                act = bridge.recv_actuator(timeout=1.0)   # PX4 replies
                if act is None:
                    continue
                world.step(act.motors(4))                 # apply + advance
                t += dt_usec
    """

    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_PORT):
        # 'tcpin:' makes this end the server; PX4 connects as the client.
        self.conn = mavutil.mavlink_connection(
            f"tcpin:{host}:{port}", dialect="common", autoreconnect=True
        )
        self.host = host
        self.port = port

    @property
    def mav(self):
        return self.conn.mav

    @property
    def connected(self) -> bool:
        """True once PX4's TCP connection has been accepted."""
        return self.conn.port is not None

    def _pump_accept(self) -> None:
        # pymavlink's tcpin accepts only inside recv(); its write() silently
        # drops while unconnected. Since the sim streams sensors before it ever
        # receives an actuator (PX4 waits for sensors first), we accept the
        # pending client here so those first writes actually land.
        if self.conn.port is None:
            r, _, _ = select.select([self.conn.listen], [], [], 0)
            if r:
                self.conn.recv()  # triggers accept(); sets conn.port

    def wait_for_px4(self, timeout: Optional[float] = None) -> bool:
        """Block until PX4 connects (accepted). Returns False on timeout."""
        deadline = None if timeout is None else time.time() + timeout
        while self.conn.port is None:
            budget = 0.1 if deadline is None else max(0.0, deadline - time.time())
            r, _, _ = select.select([self.conn.listen], [], [], budget)
            if r:
                self.conn.recv()
            if deadline is not None and time.time() >= deadline:
                return self.conn.port is not None
        return True

    def send_sensor(self, s: SensorSample) -> None:
        self._pump_accept()
        gx, gy, gz = (float(v) for v in s.gyro)
        ax, ay, az = (float(v) for v in s.accel)
        mx, my, mz = (float(v) for v in s.mag)
        self.mav.hil_sensor_send(
            s.time_usec, ax, ay, az, gx, gy, gz, mx, my, mz,
            float(s.abs_pressure), float(s.diff_pressure),
            float(s.pressure_alt), float(s.temperature),
            s.fields_updated, s.sensor_id,
        )

    def send_gps(self, g: GpsSample) -> None:
        self._pump_accept()
        self.mav.hil_gps_send(**g.encode_fields())

    def send_state(self, gt: GroundTruth) -> None:
        self._pump_accept()
        vn, ve, vd = (int(round(v * 100.0)) for v in gt.vel_ned)   # cm/s
        ax, ay, az = (int(round(v * 1000.0 / GRAVITY_MSS)) for v in gt.accel)  # mg
        self.mav.hil_state_quaternion_send(
            gt.time_usec, list(float(q) for q in gt.quaternion),
            float(gt.rollspeed), float(gt.pitchspeed), float(gt.yawspeed),
            int(round(gt.lat_deg * 1e7)), int(round(gt.lon_deg * 1e7)),
            int(round(gt.alt_m * 1000.0)),
            vn, ve, vd,
            int(round(gt.ind_airspeed * 100.0)),
            int(round(gt.true_airspeed * 100.0)),
            ax, ay, az,
        )

    def recv_actuator(self, timeout: Optional[float] = 1.0
                      ) -> Optional[ActuatorControls]:
        """Block for one HIL_ACTUATOR_CONTROLS. None on timeout."""
        msg = self.conn.recv_match(
            type="HIL_ACTUATOR_CONTROLS", blocking=True, timeout=timeout
        )
        if msg is None:
            return None
        return ActuatorControls(
            time_usec=msg.time_usec,
            controls=tuple(float(c) for c in msg.controls),
            mode=msg.mode,
            flags=msg.flags,
        )

    def close(self) -> None:
        self.conn.close()

    def __enter__(self) -> "PX4MavlinkBridge":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
