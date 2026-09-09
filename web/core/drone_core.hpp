#pragma once
// DroneCore — the SkySim physics, decoupled from Godot.
// Reuses the exact pure subsystems (blade element, atmosphere, aero effects,
// flight controller, mixer) and adds a standalone 6-DOF integrator in place of
// Godot's RigidBody3D. Compiles natively and to WebAssembly (Emscripten).
//
// Frames follow the Godot convention used throughout the sim: world Y-up,
// body -Z forward / +X right. Orientation is body->world.

// Include prelude: the reused subsystem headers assume these are already
// pulled in (Godot's build provides them transitively). Supply them here so
// the core compiles standalone / under Emscripten without editing those headers.
#include <cstdint>
#include <cstddef>
#include <vector>
#include <array>
#include <span>
#include <cmath>
#include <algorithm>

#include "core/math_types.hpp"
#include "core/frames.hpp"
#include "aero/atmosphere.hpp"
#include "aero/blade_element.hpp"
#include "aero/aero_effects.hpp"
#include "control/flight_controller.hpp"
#include "power/battery.hpp"

#include <memory>
#include <vector>

namespace dronesim {

struct TelemetryLite {
    double altitude{}, ground_speed{}, vertical_speed{};
    double roll_deg{}, pitch_deg{}, yaw_deg{};
    double roll_rate{}, pitch_rate{}, yaw_rate{};
    double total_thrust{}, power_draw{}, air_density{};
    double ground_effect_factor{1.0}, vrs_severity{};
    bool   vrs_active{};
    // Battery (shared model with the native DroneBody).
    double battery_voltage{}, battery_voltage_ocv{}, battery_current{};
    double battery_soc{1.0}, battery_mah_used{};
    bool   battery_low_warn{}, battery_cutoff{};
};

class DroneCore {
public:
    DroneCore() { build_rotors(); reset(0, 2, 0); }

    void reset(double x, double y, double z) {
        _s = RigidBodyState{};
        _s.position = {x, y, z};
        _s.mass = _mass;
        _armed = false;
        _direct = false;
        _setpoints = {};
        _time = 0.0;
        if (_fc) _fc->reset();
        _battery.reset();
        publish_battery();
    }

    void arm()    { _armed = true; _direct = false; if (_fc) _fc->reset(); _battery.reset(); publish_battery(); }
    void disarm() { _armed = false; }

    void set_attitude_setpoint(double roll, double pitch, double yaw_rate, double throttle) {
        _setpoints.roll_rad      = clamp(roll,     -0.785, 0.785);
        _setpoints.pitch_rad     = clamp(pitch,    -0.785, 0.785);
        _setpoints.yaw_rate_rads = clamp(yaw_rate, -3.14,  3.14);
        _setpoints.thrust_norm   = clamp(throttle,  0.0,   1.0);
        _direct = false;
    }

    void set_motors(const double* thr, int n) {
        _direct_thr.assign(thr, thr + n);
        _direct = true;
    }

    void set_wind(double x, double y, double z) { _wind = {x, y, z}; }

    // Toggle whether battery-voltage sag droops motor thrust (default on).
    void set_battery_thrust_coupling(bool on) { _battery_thrust_coupling = on; }
    bool battery_thrust_coupling() const { return _battery_thrust_coupling; }

    void step(double dt);

    const RigidBodyState& state()   const { return _s; }
    const TelemetryLite&  telem()   const { return _t; }
    const Battery&        battery() const { return _battery; }
    double time() const { return _time; }

    // out[12] = pos(3), vel(3), euler(3, Godot rpy rad), body_rates(3)
    void get_obs(double* out) const {
        Vec3d rpy = _s.orientation.to_euler_rpy();
        out[0]=_s.position.x; out[1]=_s.position.y; out[2]=_s.position.z;
        out[3]=_s.velocity.x; out[4]=_s.velocity.y; out[5]=_s.velocity.z;
        out[6]=rpy.x; out[7]=rpy.y; out[8]=rpy.z;
        out[9]=_s.angular_velocity.x; out[10]=_s.angular_velocity.y; out[11]=_s.angular_velocity.z;
    }

    // Copy the battery model's state into the telemetry snapshot.
    void publish_battery() {
        _t.battery_voltage     = _battery.terminal_voltage();
        _t.battery_voltage_ocv = _battery.ocv_voltage();
        _t.battery_current     = _battery.current();
        _t.battery_soc         = _battery.soc();
        _t.battery_mah_used    = _battery.mah_used();
        _t.battery_low_warn    = _battery.low_warning();
        _t.battery_cutoff      = _battery.cutoff();
    }

private:
    void build_rotors();

    RigidBodyState _s{};
    Atmosphere _atm{};
    std::unique_ptr<RotorArray>        _rotors;
    std::unique_ptr<AeroEffectsBundle> _effects;
    std::unique_ptr<FlightController>  _fc;
    std::unique_ptr<MixerMatrix>       _mixer;

    FlightController::Setpoints _setpoints{};
    TelemetryLite _t{};
    Battery       _battery{};

    bool   _armed{false}, _direct{false};
    bool   _battery_thrust_coupling{true};
    std::vector<double> _direct_thr;
    double _time{0.0};

    // config
    double _rotor_radius{0.127};
    double _motor_kv{920.0}, _max_voltage{14.8};
    double _mass{1.5};
    Vec3d  _wind{};
    double _ground_height{0.0};

    // diagonal inertia in Godot body axes (x=right, y=up, z=back);
    // yaw is about the up axis (y), so Iyy is the larger term.
    Vec3d _inertia{0.02, 0.035, 0.02};
    double _ground_radius{0.15};
};

} // namespace dronesim
