#pragma once
// ===========================================================================
// AutopilotProbe — identify what's on a freshly-opened serial link.
//
// Before entering HIL, a physical flight controller streams a HEARTBEAT
// (~1 Hz) whose `autopilot` field says which stack it runs. We read that to
// decide the plan:
//     MAV_AUTOPILOT_PX4          (12) -> PX4  -> HITL is possible
//     MAV_AUTOPILOT_ARDUPILOTMEGA (3) -> ArduPilot -> NO HITL (see router)
//     MAV_AUTOPILOT_GENERIC       (0) -> Generic
//
// Handles BOTH MAVLink v2 (0xFD) and v1 (0xFE) framing, because ArduPilot
// links commonly open in v1 until a v2 message is seen. HEARTBEAT payload
// layout and crc_extra (50) are identical across versions.
//
// Our own simulator heartbeats carry autopilot = MAV_AUTOPILOT_INVALID (8),
// so they are naturally ignored here (we only report real stacks).
//
// Robustness note: a stray STX byte in line noise can desync the byte parser
// for a single frame. Heartbeats stream at ~1 Hz, so detection self-heals on
// the next beat; callers should keep feeding until resolved() (typically <2 s).
// ===========================================================================
#include "sitl/mavlink/mavlink_types.hpp"
#include <cstdint>
#include <optional>

namespace dronesim::sitl {

// MAV_AUTOPILOT enum (MAVLink common) — the subset we care about.
enum : uint8_t {
    MAV_AUTOPILOT_GENERIC       = 0,
    MAV_AUTOPILOT_ARDUPILOTMEGA = 3,
    MAV_AUTOPILOT_INVALID       = 8,   // used by GCS / our sim heartbeat
    MAV_AUTOPILOT_PX4           = 12,
};

enum class AutopilotKind { Unknown, PX4, ArduPilot, Generic, Other };

[[nodiscard]] inline const char* to_string(AutopilotKind k) noexcept {
    switch (k) {
        case AutopilotKind::PX4:       return "PX4";
        case AutopilotKind::ArduPilot: return "ArduPilot";
        case AutopilotKind::Generic:   return "Generic";
        case AutopilotKind::Other:     return "Other";
        default:                       return "Unknown";
    }
}

[[nodiscard]] inline AutopilotKind kind_from_autopilot_field(uint8_t ap) noexcept {
    switch (ap) {
        case MAV_AUTOPILOT_PX4:           return AutopilotKind::PX4;
        case MAV_AUTOPILOT_ARDUPILOTMEGA: return AutopilotKind::ArduPilot;
        case MAV_AUTOPILOT_GENERIC:       return AutopilotKind::Generic;
        case MAV_AUTOPILOT_INVALID:       return AutopilotKind::Unknown; // ignore
        default:                          return AutopilotKind::Other;
    }
}

// ---------------------------------------------------------------------------
// AutopilotProbe
// ---------------------------------------------------------------------------
class AutopilotProbe {
public:
    // Feed received bytes. Returns a definite kind the first time a HEARTBEAT
    // from a real stack (PX4/ArduPilot/Generic/Other) is validated; returns
    // nullopt while still waiting. Once resolved it latches `result()`.
    std::optional<AutopilotKind> feed(const uint8_t* buf, int n) noexcept {
        for (int i = 0; i < n; ++i) {
            if (auto k = _feed_byte(buf[i]); k && *k != AutopilotKind::Unknown) {
                _result = *k;
                return _result;
            }
        }
        return std::nullopt;
    }

    [[nodiscard]] AutopilotKind result() const noexcept { return _result; }
    [[nodiscard]] bool resolved()       const noexcept {
        return _result != AutopilotKind::Unknown;
    }
    void reset() noexcept { *this = AutopilotProbe{}; }

private:
    AutopilotKind        _result{AutopilotKind::Unknown};
    mavlink::Parser      _v2;                 // reuse the tree's validated v2 parser

    // Minimal MAVLink v1 heartbeat recognizer (0xFE). We only need to pull the
    // autopilot byte out of a CRC-valid HEARTBEAT; other v1 msgs are skipped.
    enum class V1 { IDLE, LEN, SEQ, SYS, COMP, MSGID, PAYLOAD, CRC0, CRC1 };
    V1              _v1{V1::IDLE};
    mavlink::CRC    _v1crc{};
    uint8_t         _v1len{}, _v1msg{}, _v1idx{}, _v1lo{};
    uint8_t         _v1pay[255]{};

    std::optional<AutopilotKind> _feed_byte(uint8_t b) noexcept {
        // ---- v2 branch: hand to the validated parser, inspect HEARTBEATs ---
        if (auto f = _v2.parse_byte(b)) {
            if (f->msg_id == mavlink::MSG_ID_HEARTBEAT) {
                auto hb = mavlink::unpack_payload<mavlink::HeartbeatPayload>(*f);
                return kind_from_autopilot_field(hb.autopilot);
            }
            return AutopilotKind::Unknown; // a validated non-heartbeat: keep waiting
        }

        // ---- v1 branch: run in parallel so we catch v1-only links ----------
        return _feed_v1(b);
    }

    std::optional<AutopilotKind> _feed_v1(uint8_t b) noexcept {
        switch (_v1) {
            case V1::IDLE:
                if (b == 0xFE) { _v1crc = mavlink::CRC{}; _v1 = V1::LEN; }
                break;
            case V1::LEN:   _v1len = b; _v1crc.accumulate(b); _v1 = V1::SEQ;  break;
            case V1::SEQ:               _v1crc.accumulate(b); _v1 = V1::SYS;  break;
            case V1::SYS:               _v1crc.accumulate(b); _v1 = V1::COMP; break;
            case V1::COMP:              _v1crc.accumulate(b); _v1 = V1::MSGID;break;
            case V1::MSGID:
                _v1msg = b; _v1crc.accumulate(b); _v1idx = 0;
                _v1 = (_v1len > 0) ? V1::PAYLOAD : V1::CRC0;
                break;
            case V1::PAYLOAD:
                if (_v1idx < sizeof(_v1pay)) _v1pay[_v1idx] = b;
                _v1crc.accumulate(b);
                if (++_v1idx >= _v1len) _v1 = V1::CRC0;
                break;
            case V1::CRC0: _v1lo = b; _v1 = V1::CRC1; break;
            case V1::CRC1: {
                _v1 = V1::IDLE;
                const uint16_t recv = static_cast<uint16_t>(_v1lo)
                                    | (static_cast<uint16_t>(b) << 8);
                const auto extra = mavlink::crc_extra_for(_v1msg);
                if (!extra) break;                 // unknown v1 msg — skip
                mavlink::CRC fin = _v1crc; fin.accumulate(*extra);
                if (fin.value != recv) break;      // corrupt — skip
                if (_v1msg == mavlink::MSG_ID_HEARTBEAT) {
                    mavlink::HeartbeatPayload hb{};
                    std::memcpy(&hb, _v1pay,
                                std::min<size_t>(_v1len, sizeof(hb)));
                    return kind_from_autopilot_field(hb.autopilot);
                }
                break;
            }
        }
        return std::nullopt;
    }
};

} // namespace dronesim::sitl
