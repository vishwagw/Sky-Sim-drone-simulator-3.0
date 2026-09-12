// ===========================================================================
// serial_router_selftest — native, no-Godot test for the HITL delta.
//
// Covers:
//   1. AutopilotProbe on MAVLink v2 heartbeats (PX4, ArduPilot)
//   2. AutopilotProbe on a MAVLink v1 heartbeat (ArduPilot legacy link)
//   3. Our own sim heartbeat (autopilot=INVALID) is ignored
//   4. resolve_link() for every cell of the connection matrix
//   5. SerialTransport open/send/recv round-trip over a PTY (POSIX),
//      end-to-end into the probe.
//
// Build:  g++ -std=c++20 -Iinclude tools/serial_router_selftest.cpp -o /tmp/st
// Run:    /tmp/st         (exit code 0 = all passed)
// ===========================================================================
#include "sitl/serial_transport.hpp"
#include "sitl/autopilot_probe.hpp"
#include "sitl/hil_link_router.hpp"
#include "sitl/mavlink/mavlink_types.hpp"

#include <cstdio>
#include <cstring>
#include <vector>
#include <string>

#ifndef _WIN32
#  include <fcntl.h>
#  include <unistd.h>
#  include <cstdlib>   // posix_openpt, grantpt, unlockpt, ptsname
#endif

using namespace dronesim::sitl;

static int g_fail = 0;
#define CHECK(cond, msg) do {                                  \
    if (cond) { std::printf("  PASS  %s\n", msg); }            \
    else      { std::printf("  FAIL  %s\n", msg); ++g_fail; }  \
} while (0)

// --- heartbeat builders -----------------------------------------------------
static std::vector<uint8_t> hb_v2(uint8_t autopilot) {
    mavlink::HeartbeatPayload p{};
    p.type      = 2;            // MAV_TYPE_QUADROTOR
    p.autopilot = autopilot;
    uint8_t seq = 0;
    return mavlink::frame(mavlink::MSG_ID_HEARTBEAT,
                          reinterpret_cast<const uint8_t*>(&p), sizeof(p),
                          mavlink::CRC_EXTRA_HEARTBEAT, seq,
                          /*sys*/1, /*comp*/1);
}

static std::vector<uint8_t> hb_v1(uint8_t autopilot) {
    mavlink::HeartbeatPayload p{};
    p.type = 2; p.autopilot = autopilot;
    const uint8_t len = sizeof(p);            // 9
    std::vector<uint8_t> b = {0xFE, len, /*seq*/0, /*sys*/1, /*comp*/1, /*msg*/0};
    const uint8_t* pp = reinterpret_cast<const uint8_t*>(&p);
    for (uint8_t i = 0; i < len; ++i) b.push_back(pp[i]);
    mavlink::CRC c;
    c.accumulate(b.data() + 1, static_cast<size_t>(5 + len)); // len..payload
    c.accumulate(mavlink::CRC_EXTRA_HEARTBEAT);
    b.push_back(static_cast<uint8_t>(c.value & 0xFF));
    b.push_back(static_cast<uint8_t>(c.value >> 8));
    return b;
}

static AutopilotKind probe_all(const std::vector<uint8_t>& bytes) {
    AutopilotProbe pr;
    pr.feed(bytes.data(), static_cast<int>(bytes.size()));
    return pr.result();
}

int main() {
    std::printf("== 1/2/3: AutopilotProbe ==\n");
    CHECK(probe_all(hb_v2(MAV_AUTOPILOT_PX4))           == AutopilotKind::PX4,       "v2 PX4 heartbeat -> PX4");
    CHECK(probe_all(hb_v2(MAV_AUTOPILOT_ARDUPILOTMEGA)) == AutopilotKind::ArduPilot, "v2 ArduPilot heartbeat -> ArduPilot");
    CHECK(probe_all(hb_v1(MAV_AUTOPILOT_ARDUPILOTMEGA)) == AutopilotKind::ArduPilot, "v1 ArduPilot heartbeat -> ArduPilot");
    CHECK(probe_all(hb_v1(MAV_AUTOPILOT_PX4))           == AutopilotKind::PX4,       "v1 PX4 heartbeat -> PX4");
    CHECK(probe_all(hb_v2(MAV_AUTOPILOT_INVALID))       == AutopilotKind::Unknown,   "sim heartbeat (INVALID) -> ignored");

    // Resilience: a stray STX (0xFD) in line noise desyncs the byte parser
    // for exactly one frame. A real link streams a heartbeat ~1 Hz, so the
    // probe recovers on the next beat — mirror that (noise + two beats).
    {
        std::vector<uint8_t> noisy = {0x11,0x22,0xFD,0x00,0xFF};   // junk incl. stray STX
        auto good = hb_v2(MAV_AUTOPILOT_PX4);
        noisy.insert(noisy.end(), good.begin(), good.end());       // beat #1 (may desync)
        noisy.insert(noisy.end(), good.begin(), good.end());       // beat #2 (recovers)
        CHECK(probe_all(noisy) == AutopilotKind::PX4, "PX4 detected across repeated beats despite noise");
    }

    std::printf("== 4: resolve_link matrix ==\n");
    {
        LinkInputs in; in.serial_present = true; in.serial_kind = AutopilotKind::PX4;
        auto p = resolve_link(in);
        CHECK(p.kind == LinkKind::PX4_Hitl_Serial && p.needs_hitl_precheck,
              "PX4 board -> PX4_Hitl_Serial (+HITL precheck)");
    }
    {
        LinkInputs in; in.serial_present = true; in.serial_kind = AutopilotKind::ArduPilot;
        auto p = resolve_link(in);
        CHECK(p.kind == LinkKind::ArduPilot_Sitl_Udp && p.ardupilot_board_rejected && p.port == 9002,
              "ArduPilot board -> rejected, fall back to ArduPilot SITL :9002");
    }
    {
        LinkInputs in; in.serial_present = true; in.serial_kind = AutopilotKind::Unknown;
        auto p = resolve_link(in);
        CHECK(p.kind == LinkKind::None, "serial open, no heartbeat -> None");
    }
    {
        LinkInputs in; in.serial_present = false; in.sitl_fallback = SitlPref::PX4;
        auto p = resolve_link(in);
        CHECK(p.kind == LinkKind::PX4_Sitl_Tcp && p.port == 4560, "no board, pref PX4 -> PX4 SITL :4560");
    }
    {
        LinkInputs in; in.serial_present = false; in.sitl_fallback = SitlPref::ArduPilot;
        auto p = resolve_link(in);
        CHECK(p.kind == LinkKind::ArduPilot_Sitl_Udp && p.port == 9002, "no board, pref ArduPilot -> ArduPilot SITL :9002");
    }

#ifndef _WIN32
    std::printf("== 5: SerialTransport PTY round-trip ==\n");
    {
        int master = ::posix_openpt(O_RDWR | O_NOCTTY);
        bool pty_ok = (master >= 0) && (::grantpt(master) == 0) && (::unlockpt(master) == 0);
        const char* slave = pty_ok ? ::ptsname(master) : nullptr;
        if (!slave) {
            std::printf("  SKIP  PTY unavailable in this sandbox\n");
        } else {
            SerialTransport st(SerialTransport::Config{ std::string(slave), 115200 });
            CHECK(st.open(), "SerialTransport.open() on PTY slave");

            auto beat = hb_v2(MAV_AUTOPILOT_PX4);
            ssize_t w = ::write(master, beat.data(), beat.size());
            CHECK(w == static_cast<ssize_t>(beat.size()), "wrote heartbeat to PTY master");

            AutopilotProbe pr;
            uint8_t buf[512];
            AutopilotKind got = AutopilotKind::Unknown;
            for (int tries = 0; tries < 200 && got == AutopilotKind::Unknown; ++tries) {
                int n = st.recv(buf, sizeof(buf));
                if (n > 0) if (auto k = pr.feed(buf, n)) got = *k;
                if (n <= 0) ::usleep(1000);
            }
            CHECK(got == AutopilotKind::PX4, "recv over SerialTransport -> probe detects PX4");
            st.close();
        }
        if (master >= 0) ::close(master);
    }
#endif

    std::printf("\n%s (%d failure%s)\n", g_fail ? "FAILURES" : "ALL PASSED",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail ? 1 : 0;
}
