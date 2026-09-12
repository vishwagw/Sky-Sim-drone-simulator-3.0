// ===========================================================================
// px4_hitl_roundtrip_test — drive the refactored PX4Bridge over SerialTransport.
//
// A PTY stands in for the USB CDC-ACM link to a physical PX4 board:
//   slave  -> handed to SerialTransport (the SkySim side)
//   master -> this test, playing the "board": it reads HIL_SENSOR that the
//             bridge streams, and replies with HIL_ACTUATOR_CONTROLS.
//
// Proves the transport refactor carries the HIL loop both directions with the
// bridge's send/parse logic unchanged.
//
// Build: see docs/hitl_bridge.md (compile with src/sitl/px4_bridge.cpp)
// ===========================================================================
#include "sitl/firmware_bridge.hpp"
#include "sitl/transport.hpp"
#include "sitl/mavlink/mavlink_types.hpp"

#include <cstdio>
#include <cstring>
#include <memory>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdlib>

using namespace dronesim::sitl;
using namespace dronesim::sitl::mavlink;

static int g_fail = 0;
#define CHECK(c, m) do { if (c) std::printf("  PASS  %s\n", m); \
    else { std::printf("  FAIL  %s\n", m); ++g_fail; } } while (0)

static void make_raw(int fd) {
    termios t{};
    if (tcgetattr(fd, &t) == 0) { cfmakeraw(&t); tcsetattr(fd, TCSANOW, &t); }
}

int main() {
    // ---- PTY setup ---------------------------------------------------------
    int master = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || grantpt(master) != 0 || unlockpt(master) != 0) {
        std::printf("  SKIP  PTY unavailable in this sandbox\n");
        return 0;
    }
    const char* slave = ::ptsname(master);
    make_raw(master);
    ::fcntl(master, F_SETFL, O_NONBLOCK);

    // ---- SkySim side: PX4Bridge over a SerialTransportAdapter --------------
    auto adapter = std::make_unique<SerialTransportAdapter>(
        SerialTransport::Config{ std::string(slave), 921600 });
    PX4Bridge bridge(std::move(adapter));
    CHECK(bridge.connect(), "PX4Bridge.connect() over serial adapter");
    CHECK(bridge.is_connected(), "bridge reports connected (serial link open)");
    CHECK(bridge.port() == 0, "port() == 0 for a serial (non-socket) transport");

    // ---- Board side: parse HIL_SENSOR, reply with HIL_ACTUATOR_CONTROLS ----
    Parser board_parser;
    auto drain_master = [&](int& sensors, uint64_t& last_ts) {
        uint8_t buf[1024];
        int n = ::read(master, buf, sizeof(buf));
        if (n <= 0) return;
        for (auto& f : board_parser.parse(buf, n)) {
            if (f.msg_id == MSG_ID_HIL_SENSOR) {
                ++sensors;
                last_ts = unpack_payload<HilSensorPayload>(f).time_usec;
            }
        }
    };

    auto make_state = [](uint64_t tick) {
        SimState s;
        s.timestamp_us = tick * 4000;      // 250 Hz sim time
        s.accel_ms2 = { 0.0, 0.0, -9.81 }; // at rest
        return s;
    };

    int      sensors_seen = 0;
    uint64_t last_sensor_ts = 0;
    ActuatorOutput out;

    // Warm-up: bridge streams sensors; board reads them.
    for (uint64_t t = 0; t < 5; ++t) {
        bridge.tick(make_state(t), out);
        usleep(1000);
        drain_master(sensors_seen, last_sensor_ts);
    }
    CHECK(sensors_seen > 0, "board received HIL_SENSOR stream from bridge (sim->board)");
    CHECK(last_sensor_ts == 4 * 4000, "HIL_SENSOR time_usec carries sim time (lockstep stamp)");

    // Board replies with a known actuator command.
    const float cmd[4] = { 0.10f, 0.20f, 0.30f, 0.40f };
    {
        HilActuatorControlsPayload p{};
        p.time_usec = last_sensor_ts;
        for (int i = 0; i < 4; ++i) p.controls[i] = cmd[i];
        uint8_t seq = 0;
        auto frame_bytes = frame(MSG_ID_HIL_ACTUATOR_CONTROLS,
                                 reinterpret_cast<const uint8_t*>(&p), sizeof(p),
                                 CRC_EXTRA_HIL_ACTUATOR_CONTROLS, seq, 1, 1);
        ::write(master, frame_bytes.data(), frame_bytes.size());
        usleep(2000);
    }

    // Next ticks: bridge should recv+parse the actuator frame (board->sim).
    bool got_actuator = false;
    ActuatorOutput act_out;
    for (uint64_t t = 5; t < 20 && !got_actuator; ++t) {
        if (bridge.tick(make_state(t), act_out)) got_actuator = true;
        usleep(1000);
        drain_master(sensors_seen, last_sensor_ts);
    }
    CHECK(got_actuator, "bridge received+parsed HIL_ACTUATOR_CONTROLS (board->sim)");
    CHECK(act_out.source == ActuatorOutput::Source::PX4, "actuator source tagged PX4");
    bool vals_ok = true;
    for (int i = 0; i < 4; ++i)
        if (std::abs(act_out.channels[i] - cmd[i]) > 1e-4) vals_ok = false;
    CHECK(vals_ok, "actuator channel values round-trip 0.10/0.20/0.30/0.40");

    bridge.disconnect();
    ::close(master);

    std::printf("\n%s (%d failure%s)\n", g_fail ? "FAILURES" : "ALL PASSED",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
