#pragma once
#include "core/math_types.hpp"
#include "aero/atmosphere.hpp"
#include <optional>
#include <cmath>

namespace dronesim {

// -------------------------------------------------------------------------
// Gaussian noise generator (Box-Muller)
// -------------------------------------------------------------------------
class GaussianNoise {
public:
    explicit GaussianNoise(double sigma, uint64_t seed = 0xC0FFEE)
        : _sigma(sigma), _state(seed) {}

    // Change the standard deviation at runtime. RNG state is untouched, so
    // sigma = 0 still advances the generator and the sequence stays aligned.
    void set_sigma(double sigma) noexcept { _sigma = sigma; }

    [[nodiscard]] double sample() noexcept {
        if (_has_spare) {
            _has_spare = false;
            return _spare * _sigma;
        }
        double u, v, s;
        do {
            u = 2.0 * _uniform() - 1.0;
            v = 2.0 * _uniform() - 1.0;
            s = u*u + v*v;
        } while (s >= 1.0 || s == 0.0);
        double m = std::sqrt(-2.0 * std::log(s) / s);
        _spare    = v * m;
        _has_spare = true;
        return u * m * _sigma;
    }

private:
    double   _sigma;
    uint64_t _state;
    double   _spare{};
    bool     _has_spare{false};

    double _uniform() noexcept {
        _state ^= _state >> 12;
        _state ^= _state << 25;
        _state ^= _state >> 27;
        return static_cast<double>((_state * 0x2545F4914F6CDD1DULL) >> 11) / (1ULL<<53);
    }
};

// -------------------------------------------------------------------------
// IMU — accelerometer + gyroscope with bias drift + white noise
// -------------------------------------------------------------------------
struct IMUReading {
    Vec3d accel_body{};    // m/s² in body frame (includes gravity)
    Vec3d gyro_body{};     // rad/s in body frame
};

class IMU {
public:
    struct Params {
        double accel_noise{0.005};    // m/s² / √Hz
        double accel_bias{0.01};      // m/s² DC bias
        double gyro_noise{0.0003};    // rad/s / √Hz
        double gyro_bias{0.001};      // rad/s DC bias
        double bias_drift{0.0001};    // bias random walk rate
    };

    explicit IMU(Params p = {}) noexcept
        : _p(p)
        , _an_x(p.accel_noise), _an_y(p.accel_noise), _an_z(p.accel_noise)
        , _gn_x(p.gyro_noise),  _gn_y(p.gyro_noise),  _gn_z(p.gyro_noise)
    {
        _accel_bias = { p.accel_bias, p.accel_bias*0.8, p.accel_bias*1.1 };
        _gyro_bias  = { p.gyro_bias,  p.gyro_bias*0.9,  p.gyro_bias*1.05 };
    }

    [[nodiscard]] IMUReading measure(
        const Vec3d& accel_true_bf, // true linear accel in body frame
        const Vec3d& omega_bf,      // true angular vel in body frame
        const Quat&  orient,
        double       dt
    ) noexcept {
        // Gravity in body frame
        Vec3d g_world{0, -9.80665, 0};
        Vec3d g_body = orient.conjugate().rotate(g_world);

        // Drift bias slowly
        _drift_bias(dt);

        IMUReading r;
        r.accel_body = accel_true_bf - g_body + _accel_bias + Vec3d{
            _an_x.sample(), _an_y.sample(), _an_z.sample()
        };
        r.gyro_body = omega_bf + _gyro_bias + Vec3d{
            _gn_x.sample(), _gn_y.sample(), _gn_z.sample()
        };
        return r;
    }

    // Scales the white-noise sigmas relative to the construction-time Params:
    // 0 = no white noise, 1 = default, 2 = double. DC bias and bias drift are
    // not affected.
    void set_noise_scale(double s) noexcept {
        _an_x.set_sigma(_p.accel_noise * s);
        _an_y.set_sigma(_p.accel_noise * s);
        _an_z.set_sigma(_p.accel_noise * s);
        _gn_x.set_sigma(_p.gyro_noise * s);
        _gn_y.set_sigma(_p.gyro_noise * s);
        _gn_z.set_sigma(_p.gyro_noise * s);
    }

private:
    Params       _p;
    Vec3d        _accel_bias{}, _gyro_bias{};
    GaussianNoise _an_x, _an_y, _an_z;
    GaussianNoise _gn_x, _gn_y, _gn_z;
    GaussianNoise _drift{0.001};

    void _drift_bias(double dt) noexcept {
        double scale = std::sqrt(dt) * _p.bias_drift;
        _gyro_bias.x += _drift.sample() * scale;
        _gyro_bias.y += _drift.sample() * scale;
        _gyro_bias.z += _drift.sample() * scale;
    }
};

// -------------------------------------------------------------------------
// Barometer — pressure altitude with noise and lag
// -------------------------------------------------------------------------
struct BaroReading {
    double pressure_pa{};
    double altitude_m{};
    double temperature_k{};
};

class Barometer {
public:
    explicit Barometer(double noise_pa = 2.0, double tau = 0.05) noexcept
        : _noise(noise_pa), _tau(tau), _base_noise_pa(noise_pa) {}

    // Scales the pressure-noise sigma relative to the constructor value.
    void set_noise_scale(double s) noexcept { _noise.set_sigma(_base_noise_pa * s); }

    [[nodiscard]] BaroReading measure(
        const AtmosphericState& atm,
        double true_alt,
        double dt
    ) noexcept {
        double true_p = atm.pressure;
        double decay  = std::exp(-dt / _tau);
        _filtered_p   = _filtered_p * decay + true_p * (1.0 - decay);
        double measured_p = _filtered_p + _noise.sample();

        // Invert ISA for altitude estimate
        const double T0 = 288.15, L = 0.0065, P0 = 101325.0;
        const double R = 287.058, g = 9.80665;
        double alt = (T0/L) * (1.0 - std::pow(measured_p/P0, R*L/g));

        return { measured_p, alt, atm.temperature };
    }

private:
    GaussianNoise _noise;
    double        _tau;
    double        _base_noise_pa;
    double        _filtered_p{101325.0};
};

// -------------------------------------------------------------------------
// GPS — position + velocity with typical consumer-grade noise
// -------------------------------------------------------------------------
struct GPSReading {
    Vec3d  position{};     // m (NED)
    Vec3d  velocity{};     // m/s (NED)
    int    fix_type{3};    // 0=none,1=dead-reck,2=2D,3=3D
    double hdop{1.2};
    double vdop{2.0};
};

class GPS {
public:
    struct Params {
        double pos_noise_h{0.5};  // m horizontal 1σ
        double pos_noise_v{1.0};  // m vertical 1σ
        double vel_noise{0.05};   // m/s 1σ
        double update_rate{10.0}; // Hz
    };

    explicit GPS(Params p = {}) noexcept
        : _p(p)
        , _nh(p.pos_noise_h), _nv(p.pos_noise_v), _nvel(p.vel_noise)
    {}

    // Scales position and velocity noise relative to the construction-time Params.
    void set_noise_scale(double s) noexcept {
        _nh.set_sigma(_p.pos_noise_h * s);
        _nv.set_sigma(_p.pos_noise_v * s);
        _nvel.set_sigma(_p.vel_noise * s);
    }

    // Returns nullopt if no update this tick
    [[nodiscard]] std::optional<GPSReading> measure(
        const Vec3d& true_pos,
        const Vec3d& true_vel,
        double dt
    ) noexcept {
        _accum += dt;
        if (_accum < 1.0 / _p.update_rate) return std::nullopt;
        _accum = 0;

        GPSReading r;
        r.position = true_pos + Vec3d{_nh.sample(), _nv.sample(), _nh.sample()};
        r.velocity = true_vel + Vec3d{_nvel.sample(), _nvel.sample(), _nvel.sample()};
        r.fix_type = 3;
        return r;
    }

private:
    Params        _p;
    GaussianNoise _nh, _nv, _nvel;
    double        _accum{};
};

// -------------------------------------------------------------------------
// SensorSuite — composite
// -------------------------------------------------------------------------
struct SensorSuite {
    IMU         imu;
    Barometer   baro;
    GPS         gps;

    // Scales all random sensor noise: 0 = none, 1 = default, 2 = double.
    void set_noise_scale(double s) noexcept {
        imu.set_noise_scale(s);
        baro.set_noise_scale(s);
        gps.set_noise_scale(s);
    }
};

} // namespace dronesim
