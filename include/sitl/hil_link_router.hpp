#pragma once
// ===========================================================================
// hil_link_router — decide the transport + protocol from what's present.
//
// This is pure logic (no I/O), so it is trivially unit-testable and can be
// called from SITLManager (native bridge) or surfaced to the browser UI.
//
// The connection matrix it encodes:
//
//   Serial device runs PX4        -> PX4 HITL over serial     (real board)
//   Serial device runs ArduPilot  -> REJECTED: ArduPilot has no HITL;
//                                     fall back to ArduPilot SITL (UDP/JSON)
//   Serial device unknown/other   -> None (no valid heartbeat seen yet)
//   No serial device              -> configured SITL fallback:
//                                       PX4 SITL (TCP/MAVLink) or
//                                       ArduPilot SITL (UDP/JSON)
//
// Precondition the router cannot enforce and must surface to the user: a PX4
// board must already be in HITL mode (SYS_HITL=1 with a HIL airframe), set
// once in QGroundControl. The router flags this in `note`.
// ===========================================================================
#include "sitl/autopilot_probe.hpp"
#include <cstdint>
#include <string>

namespace dronesim::sitl {

// What the caller wants to fall back to when no board is plugged in.
enum class SitlPref { PX4, ArduPilot };

enum class LinkKind {
    None,               // nothing to do (no board detected, no usable plan)
    PX4_Hitl_Serial,    // physical PX4 over USB, MAVLink HIL
    PX4_Sitl_Tcp,       // PX4 firmware process, MAVLink HIL over TCP :4560
    ArduPilot_Sitl_Udp, // ArduPilot firmware process, JSON over UDP :9002
};

[[nodiscard]] inline const char* to_string(LinkKind k) noexcept {
    switch (k) {
        case LinkKind::PX4_Hitl_Serial:    return "PX4_Hitl_Serial";
        case LinkKind::PX4_Sitl_Tcp:       return "PX4_Sitl_Tcp";
        case LinkKind::ArduPilot_Sitl_Udp: return "ArduPilot_Sitl_Udp";
        default:                           return "None";
    }
}

// Inputs to the decision.
struct LinkInputs {
    bool          serial_present{false};  // a serial port was opened
    AutopilotKind serial_kind{AutopilotKind::Unknown}; // probe result on it
    SitlPref      sitl_fallback{SitlPref::PX4};         // used when no board

    uint16_t      px4_tcp_port{4560};
    uint16_t      ardupilot_udp_port{9002};
};

// The resolved plan.
struct LinkPlan {
    LinkKind    kind{LinkKind::None};
    uint16_t    port{0};                 // socket port for the SITL kinds
    bool        ardupilot_board_rejected{false}; // true iff an APM board was
                                                 // detected but can't do HITL
    bool        needs_hitl_precheck{false};      // remind user re: SYS_HITL=1
    std::string note{};                  // one-line human explanation
};

[[nodiscard]] inline LinkPlan resolve_link(const LinkInputs& in) noexcept {
    LinkPlan p{};

    if (in.serial_present) {
        switch (in.serial_kind) {
            case AutopilotKind::PX4:
                p.kind = LinkKind::PX4_Hitl_Serial;
                p.needs_hitl_precheck = true;
                p.note = "PX4 board detected — running hardware-in-the-loop. "
                         "Ensure the board is in HITL mode (SYS_HITL=1) with a "
                         "HIL airframe selected in QGroundControl.";
                return p;

            case AutopilotKind::ArduPilot:
                // The dead cell in the matrix, made explicit.
                p.kind = LinkKind::ArduPilot_Sitl_Udp;
                p.port = in.ardupilot_udp_port;
                p.ardupilot_board_rejected = true;
                p.note = "ArduPilot board detected, but ArduPilot has no "
                         "hardware-in-the-loop support (archived years ago). "
                         "Unplug it and run ArduPilot SITL instead, or use SIH "
                         "on the board. Falling back to ArduPilot SITL.";
                return p;

            default:
                p.kind = LinkKind::None;
                p.note = "Serial port open, but no valid autopilot HEARTBEAT "
                         "seen yet. Check the cable/port and that the firmware "
                         "is running.";
                return p;
        }
    }

    // No board: use the configured SITL fallback.
    if (in.sitl_fallback == SitlPref::PX4) {
        p.kind = LinkKind::PX4_Sitl_Tcp;
        p.port = in.px4_tcp_port;
        p.note = "No board — waiting for PX4 SITL to connect "
                 "(make px4_sitl none_iris).";
    } else {
        p.kind = LinkKind::ArduPilot_Sitl_Udp;
        p.port = in.ardupilot_udp_port;
        p.note = "No board — waiting for ArduPilot SITL "
                 "(sim_vehicle.py -v ArduCopter --model JSON:<sim-ip>).";
    }
    return p;
}

} // namespace dronesim::sitl
