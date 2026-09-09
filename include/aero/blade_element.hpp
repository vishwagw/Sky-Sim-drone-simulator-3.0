#pragma once
#include "core/math_types.hpp"
#include "aero/atmosphere.hpp"
#include <array>
#include <span>

namespace dronesim {

// -------------------------------------------------------------------------
// Rotor configuration (SI units throughout)
// -------------------------------------------------------------------------
struct RotorConfig {
    // Geometry
    double radius{0.127};          // m — rotor tip radius
    double chord{0.016};           // m — mean chord (constant for simplicity)
    int    n_blades{2};
    double twist_root_deg{14.0};   // deg — geometric twist at root
    double twist_tip_deg{6.0};     // deg — geometric twist at tip (linear interpolation)
    double hub_radius{0.015};      // m — cutout

    // Aerodynamic coefficients (flat plate / NACA thin-section approximations)
    double cl_alpha{5.73};         // per rad — 2π for thin aerofoil
    double cd0{0.012};             // profile drag coefficient at zero lift
    double cd2{0.080};             // induced drag factor (CDi = cd2 * CL²)

    // Motor / ESC first-order model
    double motor_kv{920.0};        // RPM/V
    double motor_resistance{0.12}; // Ω
    double motor_inertia{1.5e-5};  // kg·m²
    double max_voltage{14.8};      // V (4S LiPo)
    double esc_tau{0.015};         // s — first-order ESC time constant

    // Position in body frame (metres) and spin direction (+1 CCW, -1 CW from above)
    Vec3d  position{};
    int    spin_dir{1};
};

// -------------------------------------------------------------------------
// Per-rotor runtime state
// -------------------------------------------------------------------------
struct RotorState {
    double omega{0.0};        // rad/s — current angular velocity
    double omega_cmd{0.0};    // rad/s — commanded (filtered by ESC model)
    double thrust{0.0};       // N
    double torque_reaction{0.0}; // N·m
    double power{0.0};        // W
};

// -------------------------------------------------------------------------
// BET result for one rotor
// -------------------------------------------------------------------------
struct BETResult {
    Vec3d  thrust_world{};    // N in world frame
    Vec3d  torque_body{};     // N·m in body frame
    double induced_velocity{};// m/s axial inflow
    double power{};           // W shaft power
};

// -------------------------------------------------------------------------
// BladeElementSolver
//
// Integrates the blade from hub to tip using N annuli.  For each annulus:
//   - Effective AoA = θ(r) - φ(r)   where φ = atan(vi / (Ω·r))
//   - dT = ½ρ (Ω·r)² c·Cl·dr
//   - dQ = ½ρ (Ω·r)² c·Cd·r·dr
//
// Induced velocity vi is solved iteratively via momentum theory:
//   T = 2 ρ A (vi + v_inf) · vi    (Rankine-Froude)
// -------------------------------------------------------------------------
class BladeElementSolver {
public:
    static constexpr int N_ANNULI = 24; // accuracy vs speed tradeoff

    explicit BladeElementSolver(RotorConfig cfg) noexcept : _cfg(std::move(cfg)) {}

    // Main solve — call every physics tick
    // body_vel_world: body velocity in world frame
    // body_omega: body angular velocity in body frame
    // orient: body orientation quaternion (world-to-body)
    // atm: atmospheric state at rotor altitude
    [[nodiscard]] BETResult solve(
        RotorState&           state,
        const Vec3d&          body_vel_world,
        const Vec3d&          body_omega_bf,
        const Quat&           orient,
        const AtmosphericState& atm,
        const Vec3d&          wind_world,
        double                dt
    ) noexcept {
        // Advance motor model
        _integrate_motor(state, dt);

        const double omega   = state.omega;
        const double rho     = atm.density;
        const double R       = _cfg.radius;
        const double hub     = _cfg.hub_radius;
        const double dr      = (R - hub) / static_cast<double>(N_ANNULI);

        // Rotor hub velocity in world frame
        Vec3d hub_pos_bf = _cfg.position;
        Vec3d hub_vel_w  = body_vel_world +
            orient.rotate(body_omega_bf.cross(hub_pos_bf));

        // Axial component (up = rotor thrust axis, -Y in Godot body frame)
        Vec3d rotor_up_w = orient.rotate({0,1,0});
        double v_axial   = -(hub_vel_w - wind_world).dot(rotor_up_w);

        // Iterative hover inflow solve (3 Newton steps, enough for real-time)
        double vi = _prev_vi;
        for (int iter = 0; iter < 3; ++iter) {
            double T_est = _last_thrust;
            double A     = PI * R * R;
            double vi_new = T_est / (2.0 * rho * A * std::max(std::abs(v_axial + vi), 0.5));
            vi = lerp(vi, vi_new, 0.5); // damp iteration
        }

        // Integrate blade annuli
        double total_thrust = 0.0;
        double total_torque = 0.0;
        double total_power  = 0.0;

        for (int i = 0; i < N_ANNULI; ++i) {
            const double r  = hub + (i + 0.5) * dr;
            const double Vt = omega * r;                       // tangential velocity
            const double Va = v_axial + vi;                    // axial inflow
            const double V2 = Vt*Vt + Va*Va;
            if (V2 < 1e-6) continue;

            // Local blade pitch angle (linear twist)
            const double t = (r - hub) / (R - hub);
            const double theta_rad = lerp(_cfg.twist_root_deg, _cfg.twist_tip_deg, t) * DEG2RAD;

            // Inflow angle φ
            const double phi   = std::atan2(Va, Vt);
            const double alpha = theta_rad - phi;

            // Lift and drag coefficients
            const double Cl    = _cfg.cl_alpha * alpha;
            const double Cd    = _cfg.cd0 + _cfg.cd2 * Cl * Cl;

            // 2D section forces (per unit span, then times chord and dr)
            const double q = 0.5 * rho * V2;
            const double dL = q * _cfg.chord * Cl * dr * _cfg.n_blades;
            const double dD = q * _cfg.chord * Cd * dr * _cfg.n_blades;

            // Resolve into thrust / torque axes
            const double cosph = std::cos(phi), sinph = std::sin(phi);
            const double dT =  dL * cosph - dD * sinph;
            const double dQ = (dL * sinph + dD * cosph) * r;

            total_thrust += dT;
            total_torque += dQ;
            total_power  += dQ * omega;
        }

        // Update cached inflow for next tick
        _prev_vi     = vi;
        _last_thrust = std::max(total_thrust, 0.0);

        state.thrust          = _last_thrust;
        state.torque_reaction = total_torque;
        state.power           = total_power;

        // Project into world frame
        Vec3d thrust_w = rotor_up_w * _last_thrust;
        // Torque reaction about rotor axis (counter-reaction to motor spin)
        Vec3d torque_b = Vec3d{0,1,0} * (-static_cast<double>(_cfg.spin_dir) * total_torque);
        // Add gyroscopic precession: τ_gyro = Ω_rotor × I_rotor·ω_body
        // (simplified single-axis approximation)
        double gyro_factor = _cfg.motor_inertia * omega * static_cast<double>(_cfg.spin_dir);
        Vec3d gyro_b = body_omega_bf.cross({0, gyro_factor, 0});
        torque_b += gyro_b;

        return { thrust_w, torque_b, vi, total_power };
    }

    const RotorConfig& config() const noexcept { return _cfg; }

    // Runtime supply-voltage scale (1.0 = pack at/above nominal; <1 = sagged).
    // Lets a battery model droop available motor RPM without touching config.
    void   set_supply_scale(double s) noexcept { _supply_scale = clamp(s, 0.0, 1.0); }
    double supply_scale() const noexcept { return _supply_scale; }
    double omega_max() const noexcept {
        return _cfg.motor_kv * _cfg.max_voltage * _supply_scale * (PI / 30.0); // RPM -> rad/s
    }

private:
    RotorConfig _cfg;
    double _prev_vi{0.0};
    double _last_thrust{0.0};
    double _supply_scale{1.0};

    void _integrate_motor(RotorState& s, double dt) noexcept {
        // First-order ESC lag
        double tau   = _cfg.esc_tau;
        double decay = std::exp(-dt / tau);
        s.omega = s.omega * decay + s.omega_cmd * (1.0 - decay);
        // Clamp to physical limits (scaled by the current supply voltage)
        s.omega = clamp(s.omega, 0.0, omega_max());
    }
};

// -------------------------------------------------------------------------
// RotorArray — manages N rotors, computes combined wrench
// -------------------------------------------------------------------------
class RotorArray {
public:
    void add_rotor(RotorConfig cfg) {
        _solvers.emplace_back(std::move(cfg));
        _states.emplace_back();
    }

    // Set normalised throttle [0,1] per rotor — converted to omega_cmd
    void set_throttles(std::span<const double> throttles) noexcept {
        for (size_t i = 0; i < _solvers.size() && i < throttles.size(); ++i) {
            double omega_max = _solvers[i].omega_max();     // reflects supply voltage
            // Square-root mapping: thrust ∝ ω², so linear throttle → linear thrust
            _states[i].omega_cmd = omega_max * std::sqrt(std::max(throttles[i], 0.0));
        }
    }

    // Set the live pack voltage feeding the motors. A pack at or above its
    // nominal voltage leaves behaviour unchanged (scale clamps to 1.0); a
    // sagged pack droops the achievable RPM (and hence thrust) proportionally.
    void set_supply_voltage(double volts) noexcept {
        for (auto& s : _solvers) {
            const double nominal = s.config().max_voltage;
            s.set_supply_scale(nominal > 0.0 ? volts / nominal : 1.0);
        }
    }

    // Returns aggregate wrench in body frame + updates rotor states
    [[nodiscard]] Wrench solve_all(
        const RigidBodyState& body,
        const Atmosphere&     atm_model,
        const Vec3d&          wind_world,
        double                dt
    ) noexcept {
        Wrench total{};
        const double alt = body.position.y;
        const AtmosphericState atm = atm_model.at_altitude(alt);

        for (size_t i = 0; i < _solvers.size(); ++i) {
            BETResult r = _solvers[i].solve(
                _states[i],
                body.velocity,
                body.angular_velocity,
                body.orientation,
                atm,
                wind_world,
                dt
            );
            // Force: already in world frame — convert to body
            Vec3d f_body = body.orientation.conjugate().rotate(r.thrust_world);
            // Moment arm from CoM
            Vec3d arm  = _solvers[i].config().position;
            Vec3d tau  = arm.cross(f_body) + r.torque_body;
            total.force  += f_body;
            total.torque += tau;
        }
        return total;
    }

    [[nodiscard]] const std::vector<RotorState>& states() const noexcept { return _states; }
    [[nodiscard]] size_t size() const noexcept { return _solvers.size(); }

private:
    std::vector<BladeElementSolver> _solvers;
    std::vector<RotorState>         _states;
};

} // namespace dronesim
