// Turbulence uses height above ground (regression for issue #31).
//
//   c++ -std=c++20 -Iinclude -Iweb/core \
//       web/core/drone_core.cpp tests/cpp/test_turbulence_agl.cpp -o /tmp/t && /tmp/t
//
// DroneCore has no public ground-height or turbulence setter, so this test
// reaches the private members to set the ground height and to inspect the
// Dryden turbulence state after one physics step.
//
// With ground height 50 m and the drone at world height 52 m, the turbulence
// must be sampled at height above ground (2 m), not at world height (52 m).

#define private public
#include "drone_core.hpp"
#undef private

#include <cstdio>
#include <cmath>

using namespace dronesim;

int main() {
    constexpr double dt = 1.0 / 400.0;

    DroneCore d;
    d.reset(0, 52, 0);
    d._ground_height = 50.0;

    // Same RNG state and turbulence state as the atmosphere DroneCore will use.
    Atmosphere ref = d._atm;
    d.step(dt);

    // Expected: one sample at h_agl = 2 m from the same starting state.
    (void)ref.sample_wind(2.0, dt);

    const bool ok = std::fabs(d._atm._turb_state.x - ref._turb_state.x) < 1e-12 &&
                    std::fabs(d._atm._turb_state.y - ref._turb_state.y) < 1e-12 &&
                    std::fabs(d._atm._turb_state.z - ref._turb_state.z) < 1e-12;
    if (!ok) {
        std::printf("FAIL: turbulence sampled at world height, expected height above ground "
                    "(x got %.9g expected %.9g)\n",
                    d._atm._turb_state.x, ref._turb_state.x);
        return 1;
    }
    std::printf("ok: turbulence sampled at height above ground\n");
    return 0;
}
