#pragma once
#include "core/math_types.hpp"

namespace dronesim {

// -------------------------------------------------------------------------
// ISA (International Standard Atmosphere) — troposphere only
// -------------------------------------------------------------------------
struct AtmosphericState {
    double pressure{101325.0};    // Pa
    double temperature{288.15};   // K
    double density{1.225};        // kg/m³
    double speed_of_sound{340.3}; // m/s
    double dynamic_viscosity{1.789e-5}; // Pa·s
};

class Atmosphere {
public:
    // ISA sea-level constants
    static constexpr double SEA_LEVEL_P   = 101325.0;  // Pa
    static constexpr double SEA_LEVEL_T   = 288.15;    // K
    static constexpr double SEA_LEVEL_RHO = 1.225;     // kg/m³
    static constexpr double LAPSE_RATE    = 0.0065;    // K/m
    static constexpr double R_SPECIFIC    = 287.058;   // J/(kg·K)
    static constexpr double GAMMA         = 1.4;
    static constexpr double G0            = 9.80665;   // m/s²

    // Custom sea-level offset (e.g. simulate high-altitude airfield)
    void set_sea_level_offset(double altitude_m) noexcept { _offset_m = altitude_m; }

    // -----------------------------------------------------------------------
    // Primary ISA query — all quantities from altitude
    // -----------------------------------------------------------------------
    [[nodiscard]] AtmosphericState at_altitude(double alt_m) const noexcept {
        alt_m += _offset_m;
        alt_m = std::max(alt_m, 0.0);

        const double T = SEA_LEVEL_T - LAPSE_RATE * alt_m;
        const double ratio = T / SEA_LEVEL_T;
        const double exp   = G0 / (R_SPECIFIC * LAPSE_RATE);

        AtmosphericState s;
        s.temperature = T;
        s.pressure    = SEA_LEVEL_P * std::pow(ratio, exp);
        s.density     = s.pressure / (R_SPECIFIC * T);
        s.speed_of_sound = std::sqrt(GAMMA * R_SPECIFIC * T);
        // Sutherland's law for dynamic viscosity
        const double S = 110.4; // Sutherland constant K
        s.dynamic_viscosity = 1.716e-5 * std::pow(T/273.15, 1.5) * (273.15+S)/(T+S);
        return s;
    }

    // -----------------------------------------------------------------------
    // Dryden continuous turbulence model (MIL-HDBK-1797)
    // Must be called every physics tick; integrates internal state with dt
    // -----------------------------------------------------------------------
    struct DrydenParams {
        double intensity{0.1};    // turbulence intensity σ (m/s)
        double scale_length{200}; // turbulence scale length L (m)
    };

    void set_turbulence(DrydenParams p) noexcept { _turb = p; }
    void set_wind_global(Vec3d w)       noexcept { _wind_global = w; }

    // Returns total wind vector (steady + turbulent) in world frame
    [[nodiscard]] Vec3d sample_wind(double alt_m, double dt) noexcept {
        if (_turb.intensity < 1e-9) return _wind_global;

        // Low-altitude scale lengths (MIL-F-8785C, h < 1000 ft). The formulas use
        // feet, so h is converted to ft and back; h is height above ground, clamped to
        // >= 10 ft. Above 1000 ft fall back to the configured scale_length (m).
        constexpr double M_PER_FT = 0.3048;
        const double h_ft = std::max(alt_m / M_PER_FT, 10.0);
        double Lu = _turb.scale_length, Lv = Lu, Lw = Lu;
        if (h_ft < 1000.0) {
            Lw = h_ft * M_PER_FT;
            Lu = Lv = h_ft / std::pow(0.177 + 0.000823 * h_ft, 1.2) * M_PER_FT;
        }
        const double sigma = _turb.intensity;

        // First-order Dryden filter: dx = -x/τ·dt + σ·√(2/τ)·n(0,1)
        // τ = L / V_ref  (use nominal 10 m/s reference)
        const double V_ref = 10.0;
        auto step = [&](double& state, double L) -> double {
            double tau = L / V_ref;
            double decay = std::exp(-dt / tau);
            double noise = _rng_next() * sigma * std::sqrt(2.0 / tau) * std::sqrt(dt);
            state = state * decay + noise;
            return state;
        };

        // World frame is Y-up (include/core/frames.hpp): Y is vertical, Z is north.
        // Vertical gusts take the short height scale Lw; north-south gusts take Lv.
        step(_turb_state.x, Lu);
        step(_turb_state.y, Lw);
        step(_turb_state.z, Lv);

        return _wind_global + _turb_state;
    }

    [[nodiscard]] Vec3d wind_global() const noexcept { return _wind_global; }

private:
    double  _offset_m{0.0};
    Vec3d   _wind_global{};
    Vec3d   _turb_state{};
    DrydenParams _turb{};

    // Minimal LCG random — replaced with a proper RNG in production
    uint64_t _lcg_state{0xDEADBEEFCAFEBABEULL};
    double _rng_next() noexcept {
        _lcg_state = _lcg_state * 6364136223846793005ULL + 1442695040888963407ULL;
        // Box-Muller single-step approximation via polar method
        double u = static_cast<double>(_lcg_state >> 11) / (1ULL << 53);
        return (u - 0.5) * 3.4641; // approximate N(0,1) — replace with proper impl
    }
};

} // namespace dronesim
