#include "core/drone_body.hpp"
#include "sitl/sitl_manager.hpp"
#include <godot_cpp/classes/physics_direct_body_state3d.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <cmath>

using namespace godot;
using namespace dronesim;

// ============================================================================
// Construction / Godot lifecycle
// ============================================================================
DroneBody::DroneBody() {
    _rotors  = std::make_unique<RotorArray>();
    _effects = std::make_unique<AeroEffectsBundle>(_rotor_radius);
    _fc      = std::make_unique<FlightController>(_fc_gains);
    _mixer   = std::make_unique<MixerMatrix>(MixerMatrix::quad_x());
    _sensors = std::make_unique<SensorSuite>();
}

void DroneBody::_ready() {
    _rebuild_rotors();
    // Match RigidBody3D mass to our sim mass
    set_mass(static_cast<real_t>(1.5));
    // Disable Godot's built-in gravity — we handle it ourselves
    set_gravity_scale(0.0);
    // Enable continuous collision detection for small-body accuracy
    set_use_continuous_collision_detection(true);
    // Find the SITLManager (explicit path first, then direct children)
    _resolve_sitl_manager();
    // Build the battery pack from the editor properties.
    _configure_battery();
}

void DroneBody::_configure_battery() {
    Battery::Config c;
    c.cells               = std::max(1, _battery_cells);
    c.capacity_mah        = std::max(1.0, _battery_capacity_mah);
    c.internal_resistance = std::max(1e-4, _battery_internal_resistance);
    // Tie the OCV curve's nominal knot to the existing max_voltage property
    // (which represents the nominal pack voltage), when it's in a sane range.
    double nominal_cell = _max_voltage / c.cells;
    if (nominal_cell > c.empty_cell_v && nominal_cell < c.full_cell_v)
        c.nominal_cell_v = nominal_cell;
    _battery.configure(c);   // configure() also resets to full
}

void DroneBody::_physics_process(double /*delta*/) {
    // Nothing in the GDScript tick — all physics happens in _integrate_forces
}

void DroneBody::_resolve_sitl_manager() {
    _sitl = nullptr;
    if (!_sitl_manager_path.is_empty())
        _sitl = Object::cast_to<SITLManager>(get_node_or_null(_sitl_manager_path));
    if (!_sitl) {
        for (int i = 0; i < get_child_count(); ++i) {
            if (auto* m = Object::cast_to<SITLManager>(get_child(i))) {
                _sitl = m;
                break;
            }
        }
    }
}

void DroneBody::_integrate_forces(PhysicsDirectBodyState3D* gstate) {
    const double dt = gstate->get_step();
    if (dt <= 0.0) return;
    _sim_time_s += dt;

    // ---- Sample atmosphere --------------------------------------------------
    const double alt = static_cast<double>(gstate->get_transform().origin.y);
    _atm.set_turbulence({ _turbulence_intensity, 200.0 });
    _atm.set_wind_global(_wind_world);
    const Vec3d wind_w = _atm.sample_wind(alt, dt);
    const AtmosphericState atm_s = _atm.at_altitude(alt);

    // ---- Build sim state from Godot -----------------------------------------
    RigidBodyState body = _godot_to_sim_state(gstate);

    // ---- SITL exchange ------------------------------------------------------
    // Sends last tick's sensor readings (1-tick latency, standard for HIL)
    // and picks up any fresh actuator frame from the firmware.
    if (_sitl) {
        sitl::ActuatorOutput out;
        if (_sitl->physics_tick(_sim_time_s, body, wind_w, _atm,
                                _last_imu, _last_baro, _last_gps, out)) {
            _sitl_out = out;
            _sitl_last_rx_s = _sim_time_s;
        }
    }
    const bool sitl_active =
        (_sim_time_s - _sitl_last_rx_s) < SITL_TIMEOUT_S &&
        _sitl_out.source != sitl::ActuatorOutput::Source::None;

    // ---- Select control source ---------------------------------------------
    std::vector<double> throttles;
    if (sitl_active) {
        // Firmware owns the vehicle: map its motor channels 1:1 onto rotors
        throttles.assign(_rotors->size(), 0.0);
        for (size_t i = 0; i < throttles.size() &&
             i < static_cast<size_t>(sitl::ActuatorOutput::MAX_CHANNELS); ++i)
            throttles[i] = _sitl_out.channels[i];
    } else if (_direct_throttle_mode) {
        throttles = _direct_throttles;
    } else if (_armed) {
        // Internal cascade PID, running in aerospace FRD/NED convention
        const Quat  q_frd = frames::godot_to_ned(body.orientation);
        const Vec3d w_frd = frames::godot_to_ned(body.angular_velocity);
        auto [tau_r, tau_p, tau_y, thr] = _fc->update(_setpoints, q_frd, w_frd, dt);

        // Normalise torque demands (crude scaling — tune per airframe)
        const double torque_scale = 0.3;
        throttles = _mixer->mix(
            thr,
            tau_r * torque_scale,
            tau_p * torque_scale,
            tau_y * torque_scale
        );
    } else {
        throttles.assign(_rotors->size(), 0.0);
    }
    _rotors->set_throttles(throttles);

    // Battery -> thrust coupling: droop the motors' supply voltage with the
    // pack's terminal voltage (previous tick, to break the algebraic loop). A
    // healthy pack (>= nominal) leaves motor authority unchanged.
    _rotors->set_supply_voltage(_battery_thrust_coupling ? _battery.terminal_voltage()
                                                         : _max_voltage);

    // ---- Rotor aerodynamics (body frame) ------------------------------------
    Wrench rotor_wrench = _rotors->solve_all(body, _atm, wind_w, dt);

    // ---- Ground Effect ------------------------------------------------------
    const double h_agl = std::max(alt - _ground_height, 0.0);
    double ge_factor = _effects->ground_effect.effective_multiplier(h_agl);
    rotor_wrench.force.y *= ge_factor;   // thrust boost near ground

    // ---- Vortex Ring State --------------------------------------------------
    const double descent_rate  = -body.velocity.y;  // positive = descending
    const double lateral_speed = std::sqrt(body.velocity.x*body.velocity.x
                                         + body.velocity.z*body.velocity.z);
    // Use first rotor's induced velocity as representative hover Vc
    double vc = 0.0;
    if (!_rotors->states().empty()) {
        const auto& rs = _rotors->states()[0];
        const double A = PI * _rotor_radius * _rotor_radius;
        vc = std::sqrt(std::max(rs.thrust, 0.0) / (2.0 * atm_s.density * A));
    }
    auto vrs = _effects->vrs.evaluate(vc, descent_rate, lateral_speed, dt);
    if (vrs.active) {
        rotor_wrench.force.y *= vrs.thrust_factor;
    }

    // ---- Assemble world-frame force -----------------------------------------
    // Rotor wrench is body-frame; gravity and airframe drag act in world frame.
    Vec3d force_world  = body.orientation.rotate(rotor_wrench.force);
    Vec3d torque_world = body.orientation.rotate(rotor_wrench.torque);

    const double g = 9.80665;
    force_world += Vec3d{0.0, -g * body.mass, 0.0};

    const double drag_lin  = 0.02;  // kg/s  — skin friction approx
    const double drag_quad = 0.15;  // kg/m  — form drag coefficient
    Vec3d vel_rel = body.velocity - wind_w;
    double v2     = vel_rel.norm2();
    if (v2 > 1e-6) {
        force_world += -(drag_lin + drag_quad * atm_s.density * std::sqrt(v2))
                       * vel_rel;
    }

    // ---- Apply to Godot physics engine --------------------------------------
    gstate->apply_central_force(_sim_to_gv3(force_world));
    gstate->apply_torque(_sim_to_gv3(torque_world));

    // ---- Sensor simulation (stored; sent to firmware next tick) -------------
    // IMU input = total kinematic acceleration in body frame (incl. gravity);
    // the IMU model subtracts gravity itself to produce specific force.
    Vec3d accel_bf_total =
        body.orientation.conjugate().rotate(force_world / body.mass);
    _last_imu  = _sensors->imu.measure(accel_bf_total, body.angular_velocity,
                                       body.orientation, dt);
    _last_baro = _sensors->baro.measure(atm_s, alt, dt);
    _last_gps  = _sensors->gps.measure(body.position, body.velocity, dt);

    // ---- Telemetry update ---------------------------------------------------
    const Quat q_frd = frames::godot_to_ned(body.orientation);
    const Vec3d w_frd = frames::godot_to_ned(body.angular_velocity);
    Vec3d rpy = q_frd.to_euler_rpy();
    _telem.altitude           = alt;
    _telem.vertical_speed     = body.velocity.y;
    _telem.ground_speed       = lateral_speed;
    _telem.roll_deg           = rpy.x * RAD2DEG;
    _telem.pitch_deg          = rpy.y * RAD2DEG;
    _telem.yaw_deg            = rpy.z * RAD2DEG;
    _telem.roll_rate          = w_frd.x;
    _telem.pitch_rate         = w_frd.y;
    _telem.yaw_rate           = w_frd.z;
    _telem.total_thrust       = rotor_wrench.force.norm();
    _telem.vrs_active         = vrs.active;
    _telem.vrs_severity       = vrs.severity;
    _telem.ground_effect_factor = ge_factor;
    _telem.air_density        = atm_s.density;
    _telem.wind_world         = wind_w;
    _telem.sitl_active        = sitl_active;
    _telem.sitl_source        = sitl_active ? _sitl_out.source
                                            : sitl::ActuatorOutput::Source::None;

    double total_power = 0;
    for (const auto& rs : _rotors->states()) total_power += rs.power;
    _telem.power_draw = total_power;

    // Battery: integrate state-of-charge and terminal-voltage sag from the
    // electrical power the rotors are drawing. Shared model with the WASM core.
    _battery.update(_telem.power_draw, dt);
    _telem.battery_voltage = _battery.terminal_voltage();
}

// ============================================================================
// Control API
// ============================================================================
void DroneBody::arm()   { _armed = true;  _fc->reset(); _direct_throttle_mode = false; _battery.reset(); }
void DroneBody::disarm(){ _armed = false; }

void DroneBody::set_attitude_setpoint(double roll, double pitch,
                                      double yaw_rate, double throttle) {
    _setpoints.roll_rad      = clamp(roll,      -0.785, 0.785); // ±45°
    _setpoints.pitch_rad     = clamp(pitch,     -0.785, 0.785);
    _setpoints.yaw_rate_rads = clamp(yaw_rate,  -3.14,  3.14);
    _setpoints.thrust_norm   = clamp(throttle,   0.0,   1.0);
    _direct_throttle_mode    = false;
}

void DroneBody::set_rotor_throttles(PackedFloat64Array throttles) {
    _direct_throttles.resize(static_cast<size_t>(throttles.size()));
    for (int i = 0; i < throttles.size(); ++i)
        _direct_throttles[static_cast<size_t>(i)] = throttles[i];
    _direct_throttle_mode = true;
}

// ============================================================================
// Private helpers
// ============================================================================
void DroneBody::_rebuild_rotors() {
    _rotors = std::make_unique<RotorArray>();
    _effects = std::make_unique<AeroEffectsBundle>(_rotor_radius);

    if (_n_rotors == 4) {
        _build_quad_x_layout();
        _mixer = std::make_unique<MixerMatrix>(MixerMatrix::quad_x());
    } else if (_n_rotors == 6) {
        // Hex-X would go here — similar pattern
        _mixer = std::make_unique<MixerMatrix>(MixerMatrix::hex_x());
    }
}

void DroneBody::_build_quad_x_layout() {
    // Motor order & spin match the ArduPilot/PX4 quad-X convention so SITL
    // firmware channels map 1:1 onto rotors:
    //   M1 front-right CCW, M2 back-left CCW, M3 front-left CW, M4 back-right CW
    // Godot body axes: front = -Z, right = +X; spin_dir +1 = CCW from above.
    const double arm = _rotor_radius * 2.1;  // arm length heuristic
    struct MotorPlacement { double x; double z; int dir; };
    const MotorPlacement layout[4] = {
        {  arm, -arm, +1 },  // M1 front-right CCW
        { -arm,  arm, +1 },  // M2 back-left   CCW
        { -arm, -arm, -1 },  // M3 front-left  CW
        {  arm,  arm, -1 },  // M4 back-right  CW
    };
    for (auto& p : layout) {
        RotorConfig cfg;
        cfg.radius    = _rotor_radius;
        cfg.motor_kv  = _motor_kv;
        cfg.max_voltage = _max_voltage;
        cfg.position  = { p.x, 0, p.z };
        cfg.spin_dir  = p.dir;
        _rotors->add_rotor(std::move(cfg));
    }
}

RigidBodyState DroneBody::_godot_to_sim_state(
    PhysicsDirectBodyState3D* gs) const noexcept
{
    RigidBodyState s;
    Transform3D xf = gs->get_transform();
    s.position = _gv3_to_sim(xf.origin);
    s.velocity = _gv3_to_sim(gs->get_linear_velocity());
    // Angular velocity from Godot is in local (body) frame
    s.angular_velocity = _gv3_to_sim(gs->get_angular_velocity());
    s.mass = static_cast<double>(get_mass());

    // Convert Godot Basis to quaternion (Shepperd method).
    // Each sqrt argument is mathematically >=0 for an orthonormal rotation
    // matrix, but RigidBody3D's basis can drift slightly non-orthonormal
    // over many ticks of continuous force/torque application (most visible
    // after long resting contact) — clamp to 0 to avoid sqrt(negative)
    // silently producing a NaN quaternion that poisons the whole tick.
    Basis b = xf.basis;
    double tr = b[0][0] + b[1][1] + b[2][2];
    if (tr > 0) {
        double s4 = std::sqrt(std::max(tr + 1.0, 0.0)) * 2;
        s.orientation = { (b[2][1]-b[1][2])/s4, (b[0][2]-b[2][0])/s4,
                          (b[1][0]-b[0][1])/s4, 0.25*s4 };
    } else if ((b[0][0]>b[1][1]) && (b[0][0]>b[2][2])) {
        double s4 = std::sqrt(std::max(1.0+b[0][0]-b[1][1]-b[2][2], 0.0))*2;
        s.orientation = { 0.25*s4, (b[0][1]+b[1][0])/s4,
                          (b[0][2]+b[2][0])/s4, (b[2][1]-b[1][2])/s4 };
    } else if (b[1][1] > b[2][2]) {
        double s4 = std::sqrt(std::max(1.0+b[1][1]-b[0][0]-b[2][2], 0.0))*2;
        s.orientation = { (b[0][1]+b[1][0])/s4, 0.25*s4,
                          (b[1][2]+b[2][1])/s4, (b[0][2]-b[2][0])/s4 };
    } else {
        double s4 = std::sqrt(std::max(1.0+b[2][2]-b[0][0]-b[1][1], 0.0))*2;
        s.orientation = { (b[0][2]+b[2][0])/s4, (b[1][2]+b[2][1])/s4,
                          0.25*s4, (b[1][0]-b[0][1])/s4 };
    }
    s.orientation = s.orientation.normalized();
    return s;
}

// ============================================================================
// Dictionary telemetry export
// ============================================================================
Dictionary DroneBody::get_telemetry() const {
    Dictionary d;
    d["altitude"]             = _telem.altitude;
    d["ground_speed"]         = _telem.ground_speed;
    d["vertical_speed"]       = _telem.vertical_speed;
    d["roll_deg"]             = _telem.roll_deg;
    d["pitch_deg"]            = _telem.pitch_deg;
    d["yaw_deg"]              = _telem.yaw_deg;
    d["roll_rate"]            = _telem.roll_rate;
    d["pitch_rate"]           = _telem.pitch_rate;
    d["yaw_rate"]             = _telem.yaw_rate;
    d["total_thrust"]         = _telem.total_thrust;
    d["power_draw"]           = _telem.power_draw;

    // Battery pack (shared model with the WASM core). These feed battery_hud.gd.
    d["battery_voltage"]           = _telem.battery_voltage;      // terminal, sagged
    d["battery_voltage_ocv"]       = _battery.ocv_voltage();
    d["battery_current"]           = _battery.current();
    d["battery_soc"]               = _battery.soc();
    d["battery_mah_used"]          = _battery.mah_used();
    d["battery_temp_c"]            = _battery.temperature_c();
    d["battery_flight_time_min"]   = _battery.flight_time_min();
    d["battery_peukert_eff"]       = _battery.peukert_efficiency();
    d["battery_cell_imbalance_mv"] = _battery.cell_imbalance_mv();
    d["battery_low_warn"]          = _battery.low_warning();
    d["battery_cutoff"]            = _battery.cutoff();
    {
        PackedFloat64Array cells;
        for (double cv : _battery.cell_voltages()) cells.push_back(cv);
        d["cell_voltages"] = cells;
    }

    d["vrs_active"]           = _telem.vrs_active;
    d["vrs_severity"]         = _telem.vrs_severity;
    d["ground_effect_factor"] = _telem.ground_effect_factor;
    d["air_density"]          = _telem.air_density;
    d["wind"]                 = _sim_to_gv3(_telem.wind_world);
    d["sitl_active"]          = _telem.sitl_active;
    switch (_telem.sitl_source) {
        case sitl::ActuatorOutput::Source::ArduPilot:  d["sitl_source"] = "ardupilot";  break;
        case sitl::ActuatorOutput::Source::PX4:        d["sitl_source"] = "px4";        break;
        case sitl::ActuatorOutput::Source::Betaflight: d["sitl_source"] = "betaflight"; break;
        default:                                       d["sitl_source"] = "none";       break;
    }
    return d;
}

// ============================================================================
// Property accessors
// ============================================================================
void   DroneBody::set_rotor_radius(double r)  { _rotor_radius = r; if (is_inside_tree()) _rebuild_rotors(); }
double DroneBody::get_rotor_radius() const     { return _rotor_radius; }

void   DroneBody::set_n_rotors(int n)          { _n_rotors = n; if (is_inside_tree()) _rebuild_rotors(); }
int    DroneBody::get_n_rotors() const         { return _n_rotors; }

void   DroneBody::set_motor_kv(double kv)      { _motor_kv = kv; if (is_inside_tree()) _rebuild_rotors(); }
double DroneBody::get_motor_kv() const         { return _motor_kv; }

void   DroneBody::set_max_voltage(double v)    { _max_voltage = v; if (is_inside_tree()) { _rebuild_rotors(); _configure_battery(); } }
double DroneBody::get_max_voltage() const      { return _max_voltage; }

void   DroneBody::set_battery_capacity_mah(double mah) { _battery_capacity_mah = std::max(1.0, mah); _configure_battery(); }
double DroneBody::get_battery_capacity_mah() const     { return _battery_capacity_mah; }

void   DroneBody::set_battery_cells(int n)             { _battery_cells = std::max(1, n); _configure_battery(); }
int    DroneBody::get_battery_cells() const            { return _battery_cells; }

void   DroneBody::set_battery_internal_resistance(double ohm) { _battery_internal_resistance = std::max(1e-4, ohm); _configure_battery(); }
double DroneBody::get_battery_internal_resistance() const     { return _battery_internal_resistance; }

void   DroneBody::set_battery_thrust_coupling(bool on) { _battery_thrust_coupling = on; }
bool   DroneBody::get_battery_thrust_coupling() const  { return _battery_thrust_coupling; }

void   DroneBody::set_turbulence_intensity(double i) { _turbulence_intensity = std::max(0.0,i); }
double DroneBody::get_turbulence_intensity() const   { return _turbulence_intensity; }

void   DroneBody::set_wind(Vector3 w)          { _wind_world = _gv3_to_sim(w); }
Vector3 DroneBody::get_wind() const            { return _sim_to_gv3(_wind_world); }

void   DroneBody::set_ground_height(double h)  { _ground_height = h; }
double DroneBody::get_ground_height() const    { return _ground_height; }

void DroneBody::set_sitl_manager_path(NodePath p) {
    _sitl_manager_path = p;
    if (is_inside_tree()) _resolve_sitl_manager();
}
NodePath DroneBody::get_sitl_manager_path() const { return _sitl_manager_path; }

void DroneBody::set_rate_roll_pid(double p, double i, double d) {
    _fc_gains.rate_roll_p = p; _fc_gains.rate_roll_i = i; _fc_gains.rate_roll_d = d;
    _fc->set_gains(_fc_gains);
}
void DroneBody::set_rate_pitch_pid(double p, double i, double d) {
    _fc_gains.rate_pitch_p = p; _fc_gains.rate_pitch_i = i; _fc_gains.rate_pitch_d = d;
    _fc->set_gains(_fc_gains);
}
void DroneBody::set_rate_yaw_pid(double p, double i, double d) {
    _fc_gains.rate_yaw_p = p; _fc_gains.rate_yaw_i = i; _fc_gains.rate_yaw_d = d;
    _fc->set_gains(_fc_gains);
}

// ============================================================================
// GDClass binding
// ============================================================================
void DroneBody::_bind_methods() {
    // Control
    ClassDB::bind_method(D_METHOD("arm"),    &DroneBody::arm);
    ClassDB::bind_method(D_METHOD("disarm"), &DroneBody::disarm);
    ClassDB::bind_method(D_METHOD("set_attitude_setpoint","roll","pitch","yaw_rate","throttle"),
                         &DroneBody::set_attitude_setpoint);
    ClassDB::bind_method(D_METHOD("set_rotor_throttles","throttles"),
                         &DroneBody::set_rotor_throttles);
    ClassDB::bind_method(D_METHOD("get_telemetry"), &DroneBody::get_telemetry);

    // PID
    ClassDB::bind_method(D_METHOD("set_rate_roll_pid","p","i","d"),  &DroneBody::set_rate_roll_pid);
    ClassDB::bind_method(D_METHOD("set_rate_pitch_pid","p","i","d"), &DroneBody::set_rate_pitch_pid);
    ClassDB::bind_method(D_METHOD("set_rate_yaw_pid","p","i","d"),   &DroneBody::set_rate_yaw_pid);

    // Properties
    ClassDB::bind_method(D_METHOD("set_rotor_radius","r"), &DroneBody::set_rotor_radius);
    ClassDB::bind_method(D_METHOD("get_rotor_radius"),     &DroneBody::get_rotor_radius);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"rotor_radius",PROPERTY_HINT_RANGE,"0.05,0.5,0.001"),
                 "set_rotor_radius","get_rotor_radius");

    ClassDB::bind_method(D_METHOD("set_n_rotors","n"), &DroneBody::set_n_rotors);
    ClassDB::bind_method(D_METHOD("get_n_rotors"),     &DroneBody::get_n_rotors);
    ADD_PROPERTY(PropertyInfo(Variant::INT,"n_rotors",PROPERTY_HINT_ENUM,"4,6"),
                 "set_n_rotors","get_n_rotors");

    ClassDB::bind_method(D_METHOD("set_motor_kv","kv"), &DroneBody::set_motor_kv);
    ClassDB::bind_method(D_METHOD("get_motor_kv"),      &DroneBody::get_motor_kv);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"motor_kv",PROPERTY_HINT_RANGE,"100,3000,1"),
                 "set_motor_kv","get_motor_kv");

    ClassDB::bind_method(D_METHOD("set_max_voltage","v"), &DroneBody::set_max_voltage);
    ClassDB::bind_method(D_METHOD("get_max_voltage"),     &DroneBody::get_max_voltage);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"max_voltage",PROPERTY_HINT_RANGE,"7.4,44.4,0.1"),
                 "set_max_voltage","get_max_voltage");

    ClassDB::bind_method(D_METHOD("set_battery_capacity_mah","mah"), &DroneBody::set_battery_capacity_mah);
    ClassDB::bind_method(D_METHOD("get_battery_capacity_mah"),       &DroneBody::get_battery_capacity_mah);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"battery_capacity_mah",PROPERTY_HINT_RANGE,"250,30000,10"),
                 "set_battery_capacity_mah","get_battery_capacity_mah");

    ClassDB::bind_method(D_METHOD("set_battery_cells","n"), &DroneBody::set_battery_cells);
    ClassDB::bind_method(D_METHOD("get_battery_cells"),     &DroneBody::get_battery_cells);
    ADD_PROPERTY(PropertyInfo(Variant::INT,"battery_cells",PROPERTY_HINT_RANGE,"1,12,1"),
                 "set_battery_cells","get_battery_cells");

    ClassDB::bind_method(D_METHOD("set_battery_internal_resistance","ohm"), &DroneBody::set_battery_internal_resistance);
    ClassDB::bind_method(D_METHOD("get_battery_internal_resistance"),       &DroneBody::get_battery_internal_resistance);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"battery_internal_resistance",PROPERTY_HINT_RANGE,"0.005,0.5,0.001"),
                 "set_battery_internal_resistance","get_battery_internal_resistance");

    ClassDB::bind_method(D_METHOD("set_battery_thrust_coupling","on"), &DroneBody::set_battery_thrust_coupling);
    ClassDB::bind_method(D_METHOD("get_battery_thrust_coupling"),      &DroneBody::get_battery_thrust_coupling);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL,"battery_thrust_coupling"),
                 "set_battery_thrust_coupling","get_battery_thrust_coupling");

    ClassDB::bind_method(D_METHOD("set_turbulence_intensity","i"), &DroneBody::set_turbulence_intensity);
    ClassDB::bind_method(D_METHOD("get_turbulence_intensity"),     &DroneBody::get_turbulence_intensity);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"turbulence_intensity",PROPERTY_HINT_RANGE,"0,5,0.01"),
                 "set_turbulence_intensity","get_turbulence_intensity");

    ClassDB::bind_method(D_METHOD("set_wind","wind"), &DroneBody::set_wind);
    ClassDB::bind_method(D_METHOD("get_wind"),        &DroneBody::get_wind);
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3,"wind"),
                 "set_wind","get_wind");

    ClassDB::bind_method(D_METHOD("set_ground_height","h"), &DroneBody::set_ground_height);
    ClassDB::bind_method(D_METHOD("get_ground_height"),     &DroneBody::get_ground_height);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,"ground_height",PROPERTY_HINT_RANGE,"-500,5000,0.1"),
                 "set_ground_height","get_ground_height");

    ClassDB::bind_method(D_METHOD("set_sitl_manager_path","path"), &DroneBody::set_sitl_manager_path);
    ClassDB::bind_method(D_METHOD("get_sitl_manager_path"),        &DroneBody::get_sitl_manager_path);
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH,"sitl_manager_path",
                              PROPERTY_HINT_NODE_PATH_VALID_TYPES,"SITLManager"),
                 "set_sitl_manager_path","get_sitl_manager_path");
}
