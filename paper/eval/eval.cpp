// SkySim paper evaluation harness.
// Build (fixed core):   g++ -std=c++20 -O3 -Iinclude -Iweb/core web/core/drone_core.cpp paper/eval/eval.cpp -o eval -pthread
// Build (original core): prepend -I<dir with original aero/blade_element.hpp>
// Usage: ./eval <mode> [args]   — writes CSV to stdout.
#include "drone_core.hpp"
#include "aero/blade_element.hpp"
#include "aero/aero_effects.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#ifndef __wasi__
#include <thread>
#endif
#include <vector>
#include <atomic>

using namespace dronesim;
static constexpr double DT = 1.0 / 400.0;
static constexpr double RHO = 1.225;
static constexpr double G = 9.80665;

// --------------------------------------------------------------------------
// Rotor configurations
// --------------------------------------------------------------------------
static RotorConfig default_rotor() { RotorConfig c; c.radius = 0.127; return c; }

// APC Slow Flyer 10x4.7 approximated in SkySim's parametrisation
// (constant chord = c(0.75R), linear twist fitted to UIUC geometry over r/R>=0.3).
static RotorConfig apc_rotor() {
    RotorConfig c;
    c.radius = 0.127;
    c.hub_radius = 0.15 * 0.127;
    c.chord = 0.197 * 0.127;
    c.twist_root_deg = 27.14;  // LSQ line over r/R>=0.3, evaluated at the hub cut-out
    c.twist_tip_deg = 4.05;
    c.max_voltage = 25.0;   // allow sweep beyond default RPM ceiling
    return c;
}

struct Steady { double T, Q, P, vi; };

// Run the BET solver at fixed RPM and axial velocity until steady.
static Steady steady_rotor(RotorConfig cfg, double omega, double v_axial_world_up) {
    BladeElementSolver s(cfg);
    RotorState st; st.omega = omega; st.omega_cmd = omega;
    AtmosphericState atm; atm.density = RHO;
    Quat q{}; // identity: rotor up = +Y
    BETResult r{};
    for (int i = 0; i < 8000; ++i)
        r = s.solve(st, Vec3d{0, v_axial_world_up, 0}, Vec3d{0, 0, 0}, q, atm, Vec3d{0, 0, 0}, DT);
    return {st.thrust, st.torque_reaction, st.power, r.induced_velocity};
}

// --------------------------------------------------------------------------
// Simple altitude-hold wrapper around DroneCore (attitude mode)
// --------------------------------------------------------------------------
struct AltHold {
    double kp = 0.10, kd = 0.08, ki = 0.02, hover = 0.344;
    double integ = 0.0;
    double thr = 0.0;
    double update(double z_sp, double z, double vz) {
        double e = z_sp - z;
        integ = clamp(integ + e * DT, -5.0, 5.0);
        thr = clamp(hover + kp * e - kd * vz + ki * integ, 0.0, 1.0);
        return thr;
    }
};

static std::uint64_t fnv(std::uint64_t h, double v) {
    unsigned char b[8]; std::memcpy(b, &v, 8);
    for (unsigned char c : b) { h ^= c; h *= 1099511628211ULL; }
    return h;
}

// --------------------------------------------------------------------------
int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "mode?\n"); return 1; }
    std::string mode = argv[1];

    if (mode == "static") {
        // CT/CP vs RPM for both rotor configs. UIUC convention: n in rev/s, D diameter.
        // optional argv[2]: collective pitch offset (deg) added to the APC config,
        // equivalent to a zero-lift angle alpha0 = -offset for a cambered section.
        const double off = argc > 2 ? std::atof(argv[2]) : 0.0;
        std::printf("config,rpm,T,Q,P,CT,CP,FM\n");
        for (int which = 0; which < 2; ++which) {
            RotorConfig c = which ? apc_rotor() : default_rotor();
            if (which) { c.twist_root_deg += off; c.twist_tip_deg += off; }
            const char* name = which ? "apc10x4.7" : "default";
            const double D = 2 * c.radius, A = PI * c.radius * c.radius;
            for (int rpm = 2000; rpm <= 8000; rpm += 250) {
                double w = rpm * PI / 30.0, n = rpm / 60.0;
                Steady s = steady_rotor(c, w, 0.0);
                double CT = s.T / (RHO * n * n * std::pow(D, 4));
                double CP = s.P / (RHO * n * n * n * std::pow(D, 5));
                double FM = std::pow(s.T, 1.5) / (s.P * std::sqrt(2 * RHO * A));
                std::printf("%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", name, rpm, s.T, s.Q, s.P, CT, CP, FM);
            }
        }
        return 0;
    }

    if (mode == "hoveromega") {
        // omega that gives weight/4 thrust per rotor (default config)
        double lo = 100, hi = 1400, target = 1.5 * G / 4.0;
        for (int i = 0; i < 50; ++i) {
            double m = 0.5 * (lo + hi);
            (steady_rotor(default_rotor(), m, 0).T > target ? hi : lo) = m;
        }
        Steady s = steady_rotor(default_rotor(), 0.5 * (lo + hi), 0);
        std::printf("omega_hover,%.4f\nrpm,%.2f\nT,%.5f\nvi,%.5f\nvh_momentum,%.5f\nP_rotor,%.4f\n",
                    0.5 * (lo + hi), 0.5 * (lo + hi) * 30 / PI, s.T, s.vi,
                    std::sqrt(s.T / (2 * RHO * PI * 0.127 * 0.127)), s.P);
        return 0;
    }

    if (mode == "axial") {
        // Thrust at fixed hover RPM vs axial (climb +) velocity.
        double w = std::atof(argv[2]);
        std::printf("Vc,T,P,vi\n");
        for (int k = -40; k <= 60; ++k) {
            double Vc = 0.1 * k;
            Steady s = steady_rotor(default_rotor(), w, Vc);
            std::printf("%.2f,%.6f,%.6f,%.6f\n", Vc, s.T, s.P, s.vi);
        }
        return 0;
    }

    if (mode == "step") {
        // Closed loop: take off from ground, step to 10 m. Log altitude + body rates.
        DroneCore d; d.reset(0, 0.15, 0); d.arm();
        AltHold ah;
        std::printf("t,alt,vz,roll_rate,pitch_rate,yaw_rate,thr\n");
        for (int i = 0; i < 400 * 20; ++i) {
            double z_sp = 10.0;
            double thr = ah.update(z_sp, d.state().position.y, d.state().velocity.y);
            d.set_attitude_setpoint(0, 0, 0, thr);
            d.step(DT);
            if (i % 4 == 0) {
                auto& t = d.telem();
                std::printf("%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", d.time(), t.altitude, t.vertical_speed,
                            t.roll_rate, t.pitch_rate, t.yaw_rate, thr);
            }
        }
        return 0;
    }

    if (mode == "rate_step") {
        // Attitude step: hold 5 m, command 10 deg roll at t=5 s for 1 s.
        DroneCore d; d.reset(0, 5, 0); d.arm();
        AltHold ah;
        std::printf("t,alt,roll_deg,roll_sp_deg,roll_rate\n");
        for (int i = 0; i < 400 * 8; ++i) {
            double t = i * DT;
            double roll_sp = (t > 5.0 && t < 6.0) ? 10.0 * DEG2RAD : 0.0;
            double thr = ah.update(5.0, d.state().position.y, d.state().velocity.y);
            d.set_attitude_setpoint(roll_sp, 0, 0, thr);
            d.step(DT);
            if (i % 2 == 0)
                std::printf("%.4f,%.5f,%.5f,%.5f,%.5f\n", d.time(), d.telem().altitude, d.telem().roll_deg,
                            roll_sp * RAD2DEG, d.telem().roll_rate);
        }
        return 0;
    }

    if (mode == "ground") {
        // (a) model multiplier vs h/R; (b) closed-loop steady hover throttle vs commanded height.
        GroundEffectModel ge(0.127);
        std::printf("kind,h_over_R,value,value_cb\n");
        for (int k = 25; k <= 400; k += 5) {
            double hR = k / 100.0, h = hR * 0.127;
            double r = 1.0 / (4.0 * hR);
            std::printf("mult,%.3f,%.6f,%.6f\n", hR, ge.effective_multiplier(h), 1.0 / (1.0 - r * r));
        }
        for (double z : {0.18, 0.2, 0.25, 0.3, 0.35, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5, 2.0, 3.0}) {
            DroneCore d; d.reset(0, z, 0); d.arm();
            AltHold ah; double acc = 0; int n = 0;
            for (int i = 0; i < 400 * 30; ++i) {
                double thr = ah.update(z, d.state().position.y, d.state().velocity.y);
                d.set_attitude_setpoint(0, 0, 0, thr); d.step(DT);
                if (i > 400 * 20) { acc += thr; ++n; }
            }
            std::printf("hover_thr,%.3f,%.6f,%.6f\n", z / 0.127, acc / n, d.state().position.y);
        }
        return 0;
    }

    if (mode == "vrs") {
        // Steady VRS thrust factor vs normalised descent rate (lateral 0 and 0.25 vh).
        std::printf("mu_l,mu_d,factor_mean,severity\n");
        const double vh = 4.0;
        for (double mul : {0.0, 0.25}) {
            for (int k = 0; k <= 200; k += 2) {
                double mud = k / 100.0;
                VortexRingStateModel m; double acc = 0, sev = 0; int n = 0;
                for (int i = 0; i < 400 * 20; ++i) {
                    auto s = m.evaluate(vh, mud * vh, mul * vh, DT);
                    if (i > 400 * 10) { acc += s.thrust_factor; sev += s.severity; ++n; }
                }
                std::printf("%.2f,%.2f,%.6f,%.6f\n", mul, mud, acc / n, sev / n);
            }
        }
        return 0;
    }

    if (mode == "battery") {
        DroneCore d; d.reset(0, 5, 0); d.arm();
        AltHold ah;
        std::printf("t_min,alt,v_term,v_ocv,soc,current,thr,mah\n");
        for (long i = 0; i < 400L * 60 * 40; ++i) {
            double thr = ah.update(5.0, d.state().position.y, d.state().velocity.y);
            d.set_attitude_setpoint(0, 0, 0, thr); d.step(DT);
            auto& t = d.telem();
            if (i % (400 * 10) == 0)
                std::printf("%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f\n", d.time() / 60, t.altitude,
                            t.battery_voltage, t.battery_voltage_ocv, t.battery_soc, t.battery_current, thr,
                            t.battery_mah_used);
            if (t.battery_cutoff) {
                std::printf("%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f\n", d.time() / 60, t.altitude,
                            t.battery_voltage, t.battery_voltage_ocv, t.battery_soc, t.battery_current, thr,
                            t.battery_mah_used);
                std::fprintf(stderr, "cutoff at %.2f min\n", d.time() / 60);
                break;
            }
        }
        return 0;
    }

    if (mode == "perf") {
        // Single-thread steps/s, then multi-instance throughput.
        long N = argc > 2 ? std::atol(argv[2]) : 400L * 600;   // 10 sim-minutes
        auto run = [&](long n) {
            DroneCore d; d.reset(0, 5, 0); d.arm(); AltHold ah; double sink = 0;
            for (long i = 0; i < n; ++i) {
                double thr = ah.update(5.0, d.state().position.y, d.state().velocity.y);
                d.set_attitude_setpoint(0, 0, 0, thr); d.step(DT);
                sink += d.state().position.y;
            }
            return sink;
        };
        std::printf("threads,steps_total,seconds,steps_per_s,rtf_400hz\n");
#ifdef __wasi__
        unsigned hw = 1;   // WASM build: single-threaded (no pthreads)
#else
        unsigned hw = std::max(1u, std::thread::hardware_concurrency());
#endif
        for (unsigned k = 1; k <= hw; k *= 2) {
            for (int rep = 0; rep < 3; ++rep) {
                std::atomic<double> sink{0};
                auto t0 = std::chrono::steady_clock::now();
#ifdef __wasi__
                sink = run(N);
#else
                std::vector<std::thread> th;
                for (unsigned j = 0; j < k; ++j) th.emplace_back([&] { double s = run(N); sink = sink + s; });
                for (auto& x : th) x.join();
#endif
                double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                double sps = (double)N * k / sec;
                std::printf("%u,%ld,%.4f,%.1f,%.2f\n", k, N * k, sec, sps, sps / 400.0);
            }
        }
        return 0;
    }

    if (mode == "trace") {
        // Deterministic 60 s flight (default Dryden turbulence on) with a manoeuvre;
        // prints a hash of the full state trajectory plus final state.
        DroneCore d; d.reset(0, 0.15, 0); d.arm(); AltHold ah;
        std::uint64_t h = 1469598103934665603ULL;
        for (int i = 0; i < 400 * 60; ++i) {
            double t = i * DT;
            double roll = (t > 20 && t < 22) ? 0.15 : 0.0;
            double pitch = (t > 30 && t < 33) ? -0.1 : 0.0;
            double thr = ah.update(8.0, d.state().position.y, d.state().velocity.y);
            d.set_attitude_setpoint(roll, pitch, 0.2 * (t > 40), thr); d.step(DT);
            double o[12]; d.get_obs(o);
            for (double v : o) h = fnv(h, v);
            if (argc > 2 && i % 40 == 0) std::fprintf(stderr, "%.17g,%.17g,%.17g,%.17g\n", t, o[0], o[1], o[2]);
        }
        double o[12]; d.get_obs(o);
        std::printf("hash,%016llx\n", (unsigned long long)h);
        std::printf("pos,%.17g,%.17g,%.17g\n", o[0], o[1], o[2]);
        return 0;
    }

    std::fprintf(stderr, "unknown mode\n");
    return 1;
}
