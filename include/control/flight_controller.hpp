#pragma once
#include "core/math_types.hpp"
#include <array>

namespace dronesim {

// -------------------------------------------------------------------------
// PID controller — generic scalar, anti-windup via clamping
// -------------------------------------------------------------------------
struct PID {
    double kp{}, ki{}, kd{};
    double integral_limit{50.0}; // anti-windup clamp

    double _integral{};
    double _prev_error{};
    bool   _first{true};

    void reset() noexcept { _integral = 0; _prev_error = 0; _first = true; }

    [[nodiscard]] double update(double error, double dt) noexcept {
        _integral += error * dt;
        _integral  = clamp(_integral, -integral_limit, integral_limit);

        double deriv = 0.0;
        if (!_first) deriv = (error - _prev_error) / std::max(dt, 1e-6);
        _prev_error = error;
        _first = false;

        return kp*error + ki*_integral + kd*deriv;
    }
};

// -------------------------------------------------------------------------
// FlightController
//
// Two-loop cascade (matches PX4 MC_RATE / MC_ATT design):
//   outer: attitude error → rate setpoint
//   inner: rate error     → moment setpoint
//
// Outputs: desired angular acceleration in body frame [roll, pitch, yaw]
// -------------------------------------------------------------------------
class FlightController {
public:
    struct Setpoints {
        double roll_rad{};      // desired roll
        double pitch_rad{};     // desired pitch
        double yaw_rate_rads{}; // desired yaw rate
        double thrust_norm{};   // [0,1]
    };

    struct Gains {
        // Attitude loop (outer)
        double att_roll_p{8.0};
        double att_pitch_p{8.0};
        // Rate loop (inner)
        double rate_roll_p{0.15}, rate_roll_i{0.05}, rate_roll_d{0.003};
        double rate_pitch_p{0.15}, rate_pitch_i{0.05}, rate_pitch_d{0.003};
        double rate_yaw_p{0.20},  rate_yaw_i{0.1},  rate_yaw_d{0.0};
        // Output limits (rad/s²)
        double max_roll_rate{8.0};
        double max_pitch_rate{8.0};
        double max_yaw_rate{3.14};
    };

    FlightController() noexcept : FlightController(Gains{}) {}
    explicit FlightController(Gains g) noexcept : _g(g) {
        _rate_roll.kp  = g.rate_roll_p;  _rate_roll.ki  = g.rate_roll_i;  _rate_roll.kd  = g.rate_roll_d;
        _rate_pitch.kp = g.rate_pitch_p; _rate_pitch.ki = g.rate_pitch_i; _rate_pitch.kd = g.rate_pitch_d;
        _rate_yaw.kp   = g.rate_yaw_p;   _rate_yaw.ki   = g.rate_yaw_i;   _rate_yaw.kd   = g.rate_yaw_d;
    }

    // Tick: returns [roll_torque, pitch_torque, yaw_torque, thrust_norm]
    // Current attitude from orientation quaternion
    [[nodiscard]] std::array<double,4> update(
        const Setpoints& sp,
        const Quat&      orientation,
        const Vec3d&     angular_vel_bf, // body frame rad/s
        double           dt
    ) noexcept {
        Vec3d rpy = orientation.to_euler_rpy();

        // ----- Outer attitude loop ----------------------------------------
        double roll_rate_sp  = _g.att_roll_p  * (sp.roll_rad  - rpy.x);
        double pitch_rate_sp = _g.att_pitch_p * (sp.pitch_rad - rpy.y);
        double yaw_rate_sp   = sp.yaw_rate_rads;

        roll_rate_sp  = clamp(roll_rate_sp,  -_g.max_roll_rate,  _g.max_roll_rate);
        pitch_rate_sp = clamp(pitch_rate_sp, -_g.max_pitch_rate, _g.max_pitch_rate);
        yaw_rate_sp   = clamp(yaw_rate_sp,   -_g.max_yaw_rate,   _g.max_yaw_rate);

        // ----- Inner rate loop --------------------------------------------
        double roll_err  = roll_rate_sp  - angular_vel_bf.x;
        double pitch_err = pitch_rate_sp - angular_vel_bf.y;
        double yaw_err   = yaw_rate_sp   - angular_vel_bf.z;

        double tau_roll  = _rate_roll.update(roll_err,  dt);
        double tau_pitch = _rate_pitch.update(pitch_err, dt);
        double tau_yaw   = _rate_yaw.update(yaw_err,   dt);

        return { tau_roll, tau_pitch, tau_yaw, sp.thrust_norm };
    }

    void reset() noexcept { _rate_roll.reset(); _rate_pitch.reset(); _rate_yaw.reset(); }
    void set_gains(Gains g) noexcept { _g = g; }

private:
    Gains _g;
    PID   _rate_roll, _rate_pitch, _rate_yaw;
};

/* 
// -------------------------------------------------------------------------
// MixerMatrix — maps [thrust, roll, pitch, yaw] demand to per-rotor RPM
//
// Default: quad-X configuration
//   Motor layout (top view, X = front):
//       1(CCW)  2(CW)
//          \ /
//          / \
//       3(CW)  4(CCW)
// ------------------------------------------------------------------------- 
*/
class MixerMatrix {
public:
    // Each row: [thrust_coef, roll_coef, pitch_coef, yaw_coef]
    using Row = std::array<double,4>;

    explicit MixerMatrix(std::vector<Row> mix) noexcept : _mix(std::move(mix)) {}

    // Returns per-rotor normalised throttle [0,1]
    [[nodiscard]] std::vector<double> mix(
        double thrust_n, double roll_n, double pitch_n, double yaw_n
    ) const noexcept {
        std::vector<double> out(_mix.size());
        for (size_t i = 0; i < _mix.size(); ++i) {
            const auto& r = _mix[i];
            out[i] = r[0]*thrust_n + r[1]*roll_n + r[2]*pitch_n + r[3]*yaw_n;
        }
        // Normalise so max is 1, then rescale all proportionally
        double mx = 0;
        for (double v : out) mx = std::max(mx, std::abs(v));
        if (mx > 1.0) for (double& v : out) v /= mx;
        for (double& v : out) v = clamp(v, 0.0, 1.0);
        return out;
    }

    [[nodiscard]] static MixerMatrix quad_x() {
        // Motor order matches ArduPilot/PX4 quad-X (see DroneBody layout):
        //   M1 front-right CCW, M2 back-left CCW, M3 front-left CW, M4 back-right CW
        // Demands are FRD-frame: +roll right-down, +pitch nose-up, +yaw nose-right.
        return MixerMatrix({
            { 1.0, -1.0,  1.0,  1.0 }, // M1 front-right CCW
            { 1.0,  1.0, -1.0,  1.0 }, // M2 back-left   CCW
            { 1.0,  1.0,  1.0, -1.0 }, // M3 front-left  CW
            { 1.0, -1.0, -1.0, -1.0 }, // M4 back-right  CW
        });
    }

    [[nodiscard]] static MixerMatrix hex_x() {
        const double s = 0.866; // sin(60°)
        return MixerMatrix({
            { 1.0,  0.0,  1.0, -1.0 }, // front CCW
            { 1.0, -s,    0.5,  1.0 }, // front-right CW
            { 1.0, -s,   -0.5, -1.0 }, // rear-right  CCW
            { 1.0,  0.0, -1.0,  1.0 }, // rear CW
            { 1.0,  s,   -0.5, -1.0 }, // rear-left CCW
            { 1.0,  s,    0.5,  1.0 }, // front-left CW
        });
    }

private:
    std::vector<Row> _mix;
};

} // namespace dronesim
