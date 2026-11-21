#pragma once
// -----------------------------------------------------------------------------
// Battery — a first-order Li-ion / LiPo pack discharge model.
//
// Header-only and dependency-light (only <cmath>/<vector>/<cstdint>) so the
// *same* model is shared by the Godot GDExtension (DroneBody) and the standalone
// WebAssembly core (DroneCore) — browser and native therefore agree.
//
// What it models, and how honest each part is:
//   * State of charge (SoC)  — integrated from the current actually drawn.
//                              Exact bookkeeping of amp-seconds; monotone while
//                              discharging.
//   * Open-circuit voltage   — a monotonic per-cell SoC->voltage curve shaped
//                              like a real LiPo (flat middle, steep knees),
//                              anchored to the configured full/nominal/empty.
//   * Terminal voltage sag   — V_term = V_ocv - I*R, with the current solved
//                              *consistently* from the commanded electrical
//                              power P = V_term * I (a small quadratic). This is
//                              why voltage sags under load and recovers when the
//                              load drops.
//   * Peukert derating       — high discharge rates remove charge slightly
//                              faster (Peukert's law). First-order but real.
//   * Lumped thermal         — I^2R self-heating vs Newtonian cooling to
//                              ambient. First-order stand-in, clearly simplified.
//   * Per-cell imbalance     — small, *deterministic* fixed per-cell offsets so
//                              the per-cell HUD looks alive without breaking
//                              reproducibility. A cosmetic stand-in, not a real
//                              cell-aging model.
//
// The model is deterministic: no per-tick RNG, so runs remain reproducible.
//
// Defaults describe a 4S 4000 mAh LiPo (pack ESR 30 mOhm) driving the shipped
// ~1.5 kg quad. At the sim's measured hover power the pack reaches low-voltage
// cutoff after a realistic hover endurance — see docs/battery.md for the
// calibration and the exact number produced by this build.
// -----------------------------------------------------------------------------

#include <cmath>
#include <vector>
#include <cstdint>
#include <algorithm>

namespace dronesim {

class Battery {
public:
    struct Config {
        int    cells{4};                    // series cell count (S)
        double capacity_mah{4000.0};        // rated pack capacity (4S 4000 mAh LiPo)
        double internal_resistance{0.030};  // pack equivalent series resistance, Ohm
        double full_cell_v{4.20};           // OCV at 100% SoC
        double nominal_cell_v{3.70};        // OCV anchor at ~40% SoC (label/curve)
        double empty_cell_v{3.30};          // OCV at 0% usable SoC
        double warn_cell_v{3.50};           // low-voltage warning (terminal, per cell)
        double cutoff_cell_v{3.30};         // hard cutoff (terminal, per cell)
        double peukert_exponent{1.05};      // >1 => capacity falls at high current
        double peukert_ref_c{1.0};          // C-rate below which efficiency == 1
        double thermal_mass_j_per_k{450.0};       // lumped pack heat capacity, J/K
        double thermal_dissipation_w_per_k{0.9};  // cooling coefficient, W/K
        double ambient_temp_c{25.0};
        double initial_soc{1.0};
        std::uint32_t imbalance_seed{0xB0FFEEu};
        double imbalance_spread_v{0.010};   // per-cell OCV spread (~10 mV)
    };

    Battery() : Battery(Config{}) {}
    explicit Battery(Config c) { configure(c); }

    void configure(const Config& c) {
        _c = c;
        _c.cells = std::max(1, _c.cells);
        _c.capacity_mah = std::max(1.0, _c.capacity_mah);
        _capacity_as = _c.capacity_mah / 1000.0 * 3600.0;   // amp-seconds
        _build_cell_offsets();
        reset();
    }

    // Reset to the configured initial charge and clear integrators.
    void reset() {
        _soc = clamp01(_c.initial_soc);
        _charge_used_as = (1.0 - _soc) * _capacity_as;
        _temp_c = _c.ambient_temp_c;
        _current = 0.0;
        _v_ocv  = ocv_pack(_soc);
        _v_term = _v_ocv;
        _cutoff_latched = false;
    }

    // Advance one tick given the electrical power the propulsion demands (W).
    void update(double power_draw_w, double dt) {
        if (dt <= 0.0) return;

        const double vocv = ocv_pack(_soc);
        _v_ocv = vocv;
        const double R = std::max(1e-4, _c.internal_resistance);
        const double P = std::max(0.0, power_draw_w);

        // Solve  P = V_term * I  with  V_term = Vocv - I*R
        //   =>   R*I^2 - Vocv*I + P = 0
        double I = 0.0;
        if (P > 0.0 && vocv > 0.0) {
            const double disc = vocv * vocv - 4.0 * R * P;
            if (disc <= 0.0) {
                // Demand exceeds deliverable power; clamp at max-power-transfer.
                I = vocv / (2.0 * R);
            } else {
                I = (vocv - std::sqrt(disc)) / (2.0 * R);   // physical (smaller) root
            }
        }
        _current = I;
        _v_term  = std::max(0.0, vocv - I * R);

        // Peukert-derated charge removal: high current empties the pack faster.
        const double eff = peukert_efficiency(I);
        const double dq  = (I / std::max(1e-6, eff)) * dt;  // amp-seconds
        _charge_used_as  = std::min(_capacity_as, _charge_used_as + dq);
        _soc = clamp01(1.0 - _charge_used_as / _capacity_as);

        // Lumped thermal: I^2R heating, Newtonian cooling toward ambient.
        const double q_gen  = I * I * R;
        const double q_cool = _c.thermal_dissipation_w_per_k * (_temp_c - _c.ambient_temp_c);
        _temp_c += (q_gen - q_cool) / std::max(1.0, _c.thermal_mass_j_per_k) * dt;

        if (per_cell_terminal() <= _c.cutoff_cell_v) _cutoff_latched = true;
    }

    // -------- queries -------------------------------------------------------
    double soc()              const { return _soc; }
    double terminal_voltage() const { return _v_term; }
    double ocv_voltage()      const { return _v_ocv; }
    double current()          const { return _current; }
    double power()            const { return _v_term * _current; }
    double mah_used()         const { return _charge_used_as / 3600.0 * 1000.0; }
    double temperature_c()    const { return _temp_c; }
    int    cell_count()       const { return _c.cells; }
    double capacity_mah()     const { return _c.capacity_mah; }

    double per_cell_ocv()      const { return _v_ocv  / _c.cells; }
    double per_cell_terminal() const { return _v_term / _c.cells; }

    bool low_warning() const { return per_cell_terminal() <= _c.warn_cell_v; }
    bool cutoff()      const { return _cutoff_latched || per_cell_terminal() <= _c.cutoff_cell_v; }

    // Instantaneous Peukert efficiency (<= 1); ~1 at or below the reference C-rate.
    double peukert_efficiency(double I) const {
        const double c_now = I / (_c.capacity_mah / 1000.0);   // C-rate
        const double c_ref = std::max(1e-6, _c.peukert_ref_c);
        if (c_now <= c_ref) return 1.0;
        return std::pow(c_ref / c_now, _c.peukert_exponent - 1.0);
    }
    double peukert_efficiency() const { return peukert_efficiency(_current); }

    // Estimated remaining hover/flight time (minutes) at the present draw.
    double flight_time_min() const {
        if (_current <= 1e-3) return 1.0e4;               // effectively infinite
        return (_soc * _capacity_as / _current) / 60.0;
    }

    // Max-min per-cell OCV, millivolts (from the deterministic imbalance model).
    double cell_imbalance_mv() const {
        if (_cell_offset.empty()) return 0.0;
        double lo = _cell_offset.front(), hi = _cell_offset.front();
        for (double o : _cell_offset) { lo = std::min(lo, o); hi = std::max(hi, o); }
        return (hi - lo) * 1000.0;
    }

    // Per-cell terminal voltages (base terminal + fixed offset) for the HUD.
    std::vector<double> cell_voltages() const {
        std::vector<double> v(static_cast<std::size_t>(_c.cells));
        const double base = per_cell_terminal();
        for (int i = 0; i < _c.cells; ++i) v[static_cast<std::size_t>(i)] = base + _cell_offset[static_cast<std::size_t>(i)];
        return v;
    }

    double ocv_pack(double soc) const { return ocv_cell(soc) * _c.cells; }

    const Config& config() const { return _c; }

private:
    static double clamp01(double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }

    // Per-cell OCV curve: monotonic, LiPo-shaped. Voltage is interpolated over
    // [empty, full] using a normalized shape; the nominal voltage anchors the
    // 40%-SoC knot so a chemistry change (via Config) reshapes the curve too.
    double ocv_cell(double soc) const {
        soc = clamp01(soc);
        const double e = _c.empty_cell_v, n = _c.nominal_cell_v, f = _c.full_cell_v;
        const double span = std::max(1e-6, f - e);
        // Fraction of the [empty,full] span the nominal sits at (clamped so the
        // curve stays monotonic for odd configs).
        double fn = (n - e) / span;
        fn = std::min(0.80, std::max(0.30, fn));
        // Knots: (SoC, fraction-of-span).
        const double S[6] = {0.00, 0.05, 0.15, 0.40, 0.75, 1.00};
        const double F[6] = {0.00, 0.12, 0.28, fn,   0.82, 1.00};
        for (int i = 1; i < 6; ++i) {
            if (soc <= S[i]) {
                const double t = (soc - S[i-1]) / (S[i] - S[i-1]);
                const double frac = F[i-1] + t * (F[i] - F[i-1]);
                return e + frac * span;
            }
        }
        return f;
    }

    void _build_cell_offsets() {
        _cell_offset.assign(static_cast<std::size_t>(_c.cells), 0.0);
        std::uint32_t st = _c.imbalance_seed ? _c.imbalance_seed : 1u;
        for (int i = 0; i < _c.cells; ++i) {
            st ^= st << 13; st ^= st >> 17; st ^= st << 5;      // xorshift32
            const double u = (st / 4294967296.0) - 0.5;         // [-0.5, 0.5)
            _cell_offset[static_cast<std::size_t>(i)] = u * _c.imbalance_spread_v;
        }
    }

    Config _c{};
    double _capacity_as{1.0};
    double _soc{1.0};
    double _charge_used_as{0.0};
    double _v_term{0.0}, _v_ocv{0.0}, _current{0.0};
    double _temp_c{25.0};
    bool   _cutoff_latched{false};
    std::vector<double> _cell_offset;
};

} // namespace dronesim
