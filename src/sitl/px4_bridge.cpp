// ===========================================================================
// PX4 Simulator MAVLink API bridge
//
// Reference: docs.px4.io "Simulator MAVLink API".
// The sim is a TCP server on port 4560; PX4 SITL ("make px4_sitl none_iris")
// connects as a client. PX4's lockstep scheduler advances on the time_usec
// we stamp into HIL_SENSOR, so timestamps must be monotonic sim time.
// ===========================================================================
#include "sitl/firmware_bridge.hpp"
#include <cmath>
#include <cstring>

using namespace dronesim;
using namespace dronesim::sitl;
using namespace dronesim::sitl::mavlink;

// ---------------------------------------------------------------------------
bool PX4Bridge::tick(const SimState& s, ActuatorOutput& out) noexcept {
    if (!_transport || !_transport->is_open()) return false;

    // recv first: for a TCP server this also polls accept() for a pending
    // PX4 connection, so it must run even while disconnected.
    int n = _transport->recv(_recv_buf, sizeof(_recv_buf));
    if (!_transport->is_connected()) return false;

    // ---- Send sensor stream (sim-time-stamped -> drives PX4 lockstep) ----
    _send_hil_sensor(s);
    if ((_tick % GPS_DIVIDER) == 0) _send_hil_gps(s);
    if ((_tick % HB_DIVIDER)  == 0) _send_heartbeat();
    ++_tick;

    // ---- Parse actuator replies -------------------------------------------
    bool got_actuator = false;
    if (n > 0) {
        for (auto& frame : _parser.parse(_recv_buf, n)) {
            if (frame.msg_id == MSG_ID_HIL_ACTUATOR_CONTROLS)
                got_actuator = _parse_hil_actuator_controls(frame, out);
            // HEARTBEAT etc. swallowed silently
        }
    }
    return got_actuator;
}

// ---------------------------------------------------------------------------
// HIL_SENSOR — full IMU/mag/baro set every physics tick.
// PX4 wants this at 250 Hz+ for a healthy EKF; set Godot's physics tick
// rate accordingly (project setting physics/common/physics_ticks_per_second).
// ---------------------------------------------------------------------------
void PX4Bridge::_send_hil_sensor(const SimState& s) noexcept {
    HilSensorPayload p{};
    p.time_usec = s.timestamp_us;

    p.xacc  = static_cast<float>(s.accel_ms2.x);   // FRD body, m/s²
    p.yacc  = static_cast<float>(s.accel_ms2.y);
    p.zacc  = static_cast<float>(s.accel_ms2.z);
    p.xgyro = static_cast<float>(s.gyro_rads.x);   // FRD body, rad/s
    p.ygyro = static_cast<float>(s.gyro_rads.y);
    p.zgyro = static_cast<float>(s.gyro_rads.z);

    // µT -> Gauss (1 Gauss = 100 µT)
    p.xmag = static_cast<float>(s.mag_ut.x * 0.01);
    p.ymag = static_cast<float>(s.mag_ut.y * 0.01);
    p.zmag = static_cast<float>(s.mag_ut.z * 0.01);

    p.abs_pressure  = static_cast<float>(s.pressure_hpa);
    p.diff_pressure = 0.0f;
    p.pressure_alt  = static_cast<float>(s.pressure_alt_m);
    p.temperature   = static_cast<float>(s.temperature_c);
    p.fields_updated = 0x1FFF; // accel + gyro + mag + baro + diff + temp
    p.id = 0;

    auto buf = make_hil_sensor(p, _seq);
    _transport->send(buf);
}

// ---------------------------------------------------------------------------
void PX4Bridge::_send_hil_gps(const SimState& s) noexcept {
    HilGpsPayload p{};
    p.time_usec = s.timestamp_us;
    p.fix_type  = static_cast<uint8_t>(s.fix_type);
    p.lat = static_cast<int32_t>(s.lat_deg * 1e7);
    p.lon = static_cast<int32_t>(s.lon_deg * 1e7);
    p.alt = static_cast<int32_t>(s.alt_msl_m * 1000.0);
    p.eph = static_cast<uint16_t>(s.eph_m * 100.0);
    p.epv = static_cast<uint16_t>(s.epv_m * 100.0);
    const double gs = std::sqrt(s.vel_ned_ms.x*s.vel_ned_ms.x +
                                s.vel_ned_ms.y*s.vel_ned_ms.y);
    p.vel = static_cast<uint16_t>(gs * 100.0);
    p.vn  = static_cast<int16_t>(s.vel_ned_ms.x * 100.0);
    p.ve  = static_cast<int16_t>(s.vel_ned_ms.y * 100.0);
    p.vd  = static_cast<int16_t>(s.vel_ned_ms.z * 100.0);
    p.cog = static_cast<uint16_t>(static_cast<int>(
        std::atan2(s.vel_ned_ms.y, s.vel_ned_ms.x) * RAD2DEG * 100.0 + 36000.0) % 36000);
    p.satellites_visible = static_cast<uint8_t>(s.sats);
    p.id  = 0;
    p.yaw = 0; // not available

    auto buf = make_hil_gps(p, _seq);
    _transport->send(buf);
}

// ---------------------------------------------------------------------------
void PX4Bridge::_send_heartbeat() noexcept {
    auto buf = make_heartbeat(_seq);
    _transport->send(buf);
}

// ---------------------------------------------------------------------------
// HIL_ACTUATOR_CONTROLS: 16 floats; motor channels arrive in [0,1] in HIL.
// ---------------------------------------------------------------------------
bool PX4Bridge::_parse_hil_actuator_controls(
    const mavlink::Frame& f, ActuatorOutput& out) const noexcept
{
    const auto p = unpack_payload<HilActuatorControlsPayload>(f);

    out.n_channels = 16;
    out.source = ActuatorOutput::Source::PX4;
    for (int i = 0; i < 16; ++i) {
        double v = static_cast<double>(p.controls[i]);
        out.channels[i] = (v < 0.0) ? 0.0 : (v > 1.0 ? 1.0 : v);
        // Back-convert to PWM for logging / display
        out.pwm_us[i] = static_cast<uint16_t>(1000 + out.channels[i] * 1000);
    }
    return true;
}
