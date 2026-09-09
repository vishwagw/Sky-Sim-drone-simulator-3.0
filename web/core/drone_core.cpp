#include "drone_core.hpp"
#include <cmath>

namespace dronesim {

void DroneCore::build_rotors() {
    _rotors  = std::make_unique<RotorArray>();
    _effects = std::make_unique<AeroEffectsBundle>(_rotor_radius);
    _mixer   = std::make_unique<MixerMatrix>(MixerMatrix::quad_x());
    _fc      = std::make_unique<FlightController>();

    // Quad-X layout matching drone_body.cpp (ArduPilot/PX4 motor order).
    const double arm = _rotor_radius * 2.1;
    struct P { double x, z; int dir; };
    const P layout[4] = {
        {  arm, -arm, +1 }, { -arm,  arm, +1 },
        { -arm, -arm, -1 }, {  arm,  arm, -1 },
    };
    for (auto& p : layout) {
        RotorConfig cfg;
        cfg.radius      = _rotor_radius;
        cfg.motor_kv    = _motor_kv;
        cfg.max_voltage = _max_voltage;
        cfg.position    = { p.x, 0, p.z };
        cfg.spin_dir    = p.dir;
        _rotors->add_rotor(std::move(cfg));
    }
}

void DroneCore::step(double dt) {
    if (dt <= 0.0) return;
    _time += dt;

    const double alt = _s.position.y;
    _atm.set_wind_global(_wind);
    const Vec3d wind_w = _atm.sample_wind(alt, dt);
    const AtmosphericState atm_s = _atm.at_altitude(alt);

    // ---- Control source: direct motors, armed PID, or off ----
    std::vector<double> throttles;
    if (_direct) {
        throttles = _direct_thr;
        throttles.resize(_rotors->size(), 0.0);
    } else if (_armed) {
        const Quat  q_frd = frames::godot_to_ned(_s.orientation);
        const Vec3d w_frd = frames::godot_to_ned(_s.angular_velocity);
        auto out = _fc->update(_setpoints, q_frd, w_frd, dt);
        const double torque_scale = 0.3;
        throttles = _mixer->mix(out[3], out[0]*torque_scale,
                                out[1]*torque_scale, out[2]*torque_scale);
    } else {
        throttles.assign(_rotors->size(), 0.0);
    }
    _rotors->set_throttles(throttles);

    // Feed the battery's terminal voltage (from the previous tick — this breaks
    // the thrust->power->voltage algebraic loop) into the motors, so a sagging
    // pack loses thrust authority. A healthy pack (>= nominal) leaves it at 1.0.
    if (_battery_thrust_coupling)
        _rotors->set_supply_voltage(_battery.terminal_voltage());
    else
        _rotors->set_supply_voltage(_max_voltage);

    // ---- Rotor aerodynamics (body frame) ----
    Wrench wrench = _rotors->solve_all(_s, _atm, wind_w, dt);

    // ---- Ground effect ----
    const double h_agl = std::max(alt - _ground_height, 0.0);
    double ge = _effects->ground_effect.effective_multiplier(h_agl);
    wrench.force.y *= ge;

    // ---- Vortex ring state ----
    const double descent_rate  = -_s.velocity.y;
    const double lateral_speed = std::sqrt(_s.velocity.x*_s.velocity.x +
                                           _s.velocity.z*_s.velocity.z);
    double vc = 0.0;
    if (!_rotors->states().empty()) {
        const auto& rs = _rotors->states()[0];
        const double A = PI * _rotor_radius * _rotor_radius;
        vc = std::sqrt(std::max(rs.thrust, 0.0) / (2.0 * atm_s.density * A));
    }
    auto vrs = _effects->vrs.evaluate(vc, descent_rate, lateral_speed, dt);
    if (vrs.active) wrench.force.y *= vrs.thrust_factor;

    // ---- Assemble world force (gravity + drag) ----
    Vec3d force_world = _s.orientation.rotate(wrench.force);
    const double g = 9.80665;
    force_world += Vec3d{0.0, -g * _s.mass, 0.0};

    const double drag_lin = 0.02, drag_quad = 0.15;
    Vec3d vel_rel = _s.velocity - wind_w;
    double v2 = vel_rel.norm2();
    if (v2 > 1e-6)
        force_world += -(drag_lin + drag_quad*atm_s.density*std::sqrt(v2)) * vel_rel;

    // ---- Integrate linear (world frame, semi-implicit Euler) ----
    Vec3d accel = force_world / _s.mass;
    _s.velocity += accel * dt;
    _s.position += _s.velocity * dt;

    // ---- Integrate angular (body frame) ----
    // torque is body-frame (from blade element); I is diagonal in body axes.
    Vec3d tau = wrench.torque;
    Vec3d w = _s.angular_velocity;
    Vec3d Iw{ _inertia.x*w.x, _inertia.y*w.y, _inertia.z*w.z };
    Vec3d gyro = w.cross(Iw);
    Vec3d wdot{ (tau.x - gyro.x)/_inertia.x,
                (tau.y - gyro.y)/_inertia.y,
                (tau.z - gyro.z)/_inertia.z };
    _s.angular_velocity += wdot * dt;

    // quaternion kinematics: q_dot = 0.5 * q * (w,0)   (w body frame)
    Quat wq{ _s.angular_velocity.x, _s.angular_velocity.y, _s.angular_velocity.z, 0.0 };
    Quat qd = _s.orientation * wq;
    _s.orientation.x += 0.5 * qd.x * dt;
    _s.orientation.y += 0.5 * qd.y * dt;
    _s.orientation.z += 0.5 * qd.z * dt;
    _s.orientation.w += 0.5 * qd.w * dt;
    _s.orientation = _s.orientation.normalized();

    // ---- Ground contact ----
    if (_s.position.y < _ground_radius) {
        _s.position.y = _ground_radius;
        if (_s.velocity.y < 0.0) _s.velocity.y = 0.0;
        _s.velocity.x *= 0.85;
        _s.velocity.z *= 0.85;
        _s.angular_velocity *= 0.8;
    }

    // ---- Telemetry ----
    const Quat q_frd = frames::godot_to_ned(_s.orientation);
    const Vec3d w_frd = frames::godot_to_ned(_s.angular_velocity);
    Vec3d rpy = q_frd.to_euler_rpy();
    _t.altitude = alt; _t.vertical_speed = _s.velocity.y; _t.ground_speed = lateral_speed;
    _t.roll_deg = rpy.x*RAD2DEG; _t.pitch_deg = rpy.y*RAD2DEG; _t.yaw_deg = rpy.z*RAD2DEG;
    _t.roll_rate = w_frd.x; _t.pitch_rate = w_frd.y; _t.yaw_rate = w_frd.z;
    _t.total_thrust = wrench.force.norm();
    _t.ground_effect_factor = ge; _t.air_density = atm_s.density;
    _t.vrs_active = vrs.active; _t.vrs_severity = vrs.severity;
    double power = 0; for (const auto& rs : _rotors->states()) power += rs.power;
    _t.power_draw = power;

    // ---- Battery: integrate SoC / voltage sag from the drawn power ----------
    _battery.update(power, dt);
    publish_battery();
}

} // namespace dronesim
