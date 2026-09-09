// Battery -> thrust coupling (contribution follow-up to issue #2).
//
//   g++ -std=c++20 -Iinclude -Iweb/core \
//       web/core/drone_core.cpp tests/cpp/test_battery_thrust_coupling.cpp -o /tmp/t && /tmp/t
//
// Two levels of check:
//   1. Rotor model: a sagged supply voltage droops max thrust as ~voltage^2,
//      and a healthy pack (>= nominal) leaves thrust unchanged.
//   2. Through DroneCore: with coupling ON, a pack drained below nominal delivers
//      less full-throttle thrust than a full pack; with coupling OFF it doesn't.

#include "drone_core.hpp"
#include "aero/blade_element.hpp"

#include <cstdio>
#include <cmath>
#include <vector>

using namespace dronesim;

namespace { int g_fail = 0;
void check(bool c, const char* what) {
    std::printf(c ? "  ok:   %s\n" : "  FAIL: %s\n", what);
    if (!c) ++g_fail;
}
constexpr double DT = 1.0 / 400.0;
constexpr double NOMINAL = 14.8;     // default pack nominal voltage

// Build a quad rotor array like DroneCore::build_rotors and measure the steady
// full-throttle thrust at a given supply voltage.
double steady_full_throttle_thrust(double supply_v) {
    RotorArray rotors;
    const double R = 0.127, arm = R * 2.1;
    struct P { double x, z; int dir; };
    const P layout[4] = {{arm,-arm,+1},{-arm,arm,+1},{-arm,-arm,-1},{arm,arm,-1}};
    for (auto& p : layout) {
        RotorConfig c; c.radius = R; c.motor_kv = 920.0; c.max_voltage = NOMINAL;
        c.position = {p.x, 0, p.z}; c.spin_dir = p.dir;
        rotors.add_rotor(std::move(c));
    }
    rotors.set_supply_voltage(supply_v);
    RigidBodyState body{}; body.position = {0, 5, 0}; body.mass = 1.5;
    Atmosphere atm{};
    std::vector<double> full(4, 1.0);
    double thrust = 0.0;
    for (int i = 0; i < 1600; ++i) {                // settle omega under ESC lag
        rotors.set_supply_voltage(supply_v);        // (throttle setter reads it)
        rotors.set_throttles(full);
        Wrench w = rotors.solve_all(body, atm, {0,0,0}, DT);
        thrust = w.force.norm();
    }
    return thrust;
}
}

int main() {
    std::printf("test_battery_thrust_coupling\n");

    // 1. Rotor-level voltage law.
    const double T_nom = steady_full_throttle_thrust(NOMINAL);
    const double T_hi  = steady_full_throttle_thrust(NOMINAL * 1.10);   // healthy/over
    const double T_70  = steady_full_throttle_thrust(NOMINAL * 0.70);   // sagged
    std::printf("  full-throttle thrust: nom=%.2fN  hi=%.2fN  0.7x=%.2fN\n", T_nom, T_hi, T_70);

    check(std::abs(T_hi - T_nom) < 1e-6, "healthy pack (>= nominal) leaves thrust unchanged");
    check(T_70 < T_nom, "sagged pack reduces max thrust");
    // thrust ~ omega^2 ~ voltage^2  => ratio ~ 0.7^2 = 0.49
    const double ratio = T_70 / T_nom;
    std::printf("  thrust ratio at 0.70x voltage = %.3f (expect ~0.49)\n", ratio);
    check(std::abs(ratio - 0.49) < 0.03, "thrust scales as ~voltage^2");

    // 2. Through DroneCore, altitude-controlled so only the pack state varies.
    // A simple 5 m alt-hold; return the *steady* throttle it settles at, averaged
    // over a window once the craft is settled AND the stated condition is met
    // (a minimum hover time, or the pack has drained past a voltage). As a pack
    // sags, holding hover thrust needs more throttle -> "sluggish on a low pack".
    auto steady_hover_throttle = [](DroneCore& d, double drain_to_v, double min_time_s) {
        const double TARGET = 5.0;
        double thr = 0.34, thr_sum = 0.0; int cnt = 0;
        for (int i = 0; i < 400 * 1600; ++i) {
            double alt = d.state().position.y, vy = d.telem().vertical_speed;
            thr = 0.34 + 0.5 * (TARGET - alt) - 0.10 * vy;
            if (thr < 0) thr = 0; if (thr > 1) thr = 1;
            d.set_attitude_setpoint(0, 0, 0, thr); d.step(DT);
            bool cond = (d.time() >= min_time_s) || (d.telem().battery_voltage <= drain_to_v);
            if (cond && std::fabs(alt - TARGET) < 0.25 && std::fabs(vy) < 0.2) {
                thr_sum += thr; ++cnt;
                if (cnt >= 800) break;
            }
        }
        return cnt ? thr_sum / cnt : thr;
    };

    // Full charge (measured after the same 20 s settle): ON must equal OFF,
    // because a healthy pack keeps the supply scale at 1.0.
    DroneCore full_on;  full_on.reset(0,5,0);  full_on.arm();
    DroneCore full_off; full_off.reset(0,5,0); full_off.arm(); full_off.set_battery_thrust_coupling(false);
    double thr_full_on  = steady_hover_throttle(full_on,  -1.0, 20.0);
    double thr_full_off = steady_hover_throttle(full_off, -1.0, 20.0);
    std::printf("  steady hover throttle @ full charge: on=%.3f off=%.3f\n", thr_full_on, thr_full_off);
    check(std::fabs(thr_full_on - thr_full_off) < 0.01, "healthy pack: coupling ON == OFF");

    // Drained below nominal: ON needs more throttle to hover; OFF is unchanged.
    DroneCore low_on;  low_on.reset(0,5,0);  low_on.arm();
    DroneCore low_off; low_off.reset(0,5,0); low_off.arm(); low_off.set_battery_thrust_coupling(false);
    double thr_low_on  = steady_hover_throttle(low_on,  14.2, 1e9);
    double thr_low_off = steady_hover_throttle(low_off, 14.2, 1e9);
    std::printf("  steady hover throttle @ ~14.2V:      on=%.3f off=%.3f\n", thr_low_on, thr_low_off);
    check(thr_low_on > thr_full_on + 0.008, "coupling ON: sagged pack needs more hover throttle");
    check(std::fabs(thr_low_off - thr_full_off) < 0.01, "coupling OFF: hover throttle independent of pack state");

    if (g_fail == 0) { std::printf("ALL PASSED\n"); return 0; }
    std::printf("%d CHECK(S) FAILED\n", g_fail);
    return 1;
}
