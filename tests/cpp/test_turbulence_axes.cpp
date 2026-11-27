// Turbulence axis assignment (issue #30).
//
//   g++ -std=c++20 -Iinclude tests/cpp/test_turbulence_axes.cpp -o /tmp/t && /tmp/t
//
// The world frame is Y-up, so the vertical gust (Y) must use the short
// low-altitude height scale Lw, and the north gust (Z) the lateral scale Lv.
// At 5 m AGL, Lw ~= 5 m and Lv ~= 37 m, so the Y lag-1 autocorrelation must be
// clearly smaller than the Z one. The check is made with the reference speed
// and step from Atmosphere::sample_wind, with the steady wind set to zero.

#include <cstdint>
#include "aero/atmosphere.hpp"
#include <cmath>
#include <cstdio>
#include <vector>

using namespace dronesim;

// Lag-1 autocorrelation of a zero-mean-ized series.
static double lag1(const std::vector<double>& v) {
    double mean = 0.0;
    for (double x : v) mean += x;
    mean /= static_cast<double>(v.size());
    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < v.size(); ++i) {
        const double a = v[i] - mean;
        den += a * a;
        if (i + 1 < v.size()) num += a * (v[i + 1] - mean);
    }
    return num / den;
}

int main() {
    Atmosphere atm;
    atm.set_wind_global({0.0, 0.0, 0.0});
    atm.set_turbulence({1.0, 200.0});

    const double alt = 5.0, dt = 0.01;
    const int n = 20000;
    std::vector<double> ys, zs;
    ys.reserve(n);
    zs.reserve(n);
    for (int i = 0; i < n; ++i) {
        const Vec3d w = atm.sample_wind(alt, dt);
        ys.push_back(w.y);
        zs.push_back(w.z);
    }

    const double phi_y = lag1(ys);
    const double phi_z = lag1(zs);
    printf("alt=5m lag-1 autocorr: phi_y(up)=%.4f phi_z(north)=%.4f\n", phi_y, phi_z);

    if (!(phi_y < phi_z)) {
        printf("FAIL: vertical (Y) decorrelates slower than north (Z): phi_y=%.4f >= phi_z=%.4f\n",
               phi_y, phi_z);
        return 1;
    }
    printf("PASS: vertical (Y) decorrelates faster than north (Z)\n");
    return 0;
}
