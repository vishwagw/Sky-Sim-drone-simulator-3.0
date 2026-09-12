# HITL bridge — hardware-in-the-loop over USB

This delta lets SkySim drive a **physical PX4 flight controller** over USB using
the same MAVLink HIL loop the tree already uses for PX4 SITL — plus the logic to
pick the right link automatically and fall back to SITL when no board is present.

## Connection matrix

| What the user has                | Transport        | Protocol    | Result |
|----------------------------------|------------------|-------------|--------|
| Physical **PX4** board on USB    | Serial (CDC-ACM) | MAVLink HIL | HITL   |
| **PX4** firmware as a process    | TCP :4560        | MAVLink HIL | SITL   |
| **ArduPilot** firmware process   | UDP :9002        | JSON        | SITL   |
| Physical **ArduPilot** board USB | —                | —           | ❌ ArduPilot removed HITL; use SITL or SIH |

## What's in this delta (all drop-in, no changes to existing files)

- `include/sitl/serial_transport.hpp` — `SerialTransport`, mirroring
  `SocketTransport`'s method surface (`open/close/is_open/is_connected/send/recv/config`)
  so a bridge can carry HIL over USB with **no change to its send/parse logic**.
  POSIX termios (Linux/macOS) + Win32 COM. Non-blocking `recv()`.
- `include/sitl/autopilot_probe.hpp` — `AutopilotProbe`: reads a HEARTBEAT
  (MAVLink **v1 and v2**) and reports PX4 vs ArduPilot from the `autopilot`
  field. Ignores our own sim heartbeats (autopilot = INVALID).
- `include/sitl/hil_link_router.hpp` — `resolve_link()`: pure decision logic
  implementing the matrix above, including the explicit ArduPilot-board reject
  and the `SYS_HITL=1` precheck reminder.
- `tools/serial_router_selftest.cpp` — native, no-Godot self-test.

## Validation (run locally, GCC-13, `-std=c++20 -Wall -Wextra`)

```
g++ -std=c++20 -Iinclude tools/serial_router_selftest.cpp -o /tmp/st && /tmp/st
```

All 14 checks pass: probe on v1/v2 PX4+ArduPilot heartbeats; sim heartbeat
ignored; noise-resilience across repeated beats; all five router cells; and a
**real SerialTransport open/send/recv round-trip over a PTY** feeding the probe.
Not exercised here: the Win32 COM path (API-accurate, needs a Windows toolchain)
and a live PX4 board (needs hardware).

## Next increment — wire the serial transport into PX4Bridge

`PX4Bridge` currently owns a `SocketTransport` by value. To let it run over
either transport without duplicating the HIL loop, introduce a thin transport
interface and hold it by pointer (recommended — keeps `px4_bridge.cpp` as a
`.cpp`, avoids header-templating):

```cpp
// include/sitl/transport.hpp
struct ITransport {
    virtual ~ITransport() = default;
    virtual bool is_open()      const noexcept = 0;
    virtual bool is_connected() const noexcept = 0;
    virtual bool send(const uint8_t*, size_t) noexcept = 0;
    virtual int  recv(uint8_t*, int)          noexcept = 0;
};
// SocketTransportAdapter / SerialTransportAdapter forward to the concrete type.
```

Then `PX4Bridge` holds `std::unique_ptr<ITransport>`; its `tick()` body is
unchanged (same `send`/`recv`/`is_connected` calls). Per-tick virtual dispatch
is a handful of calls — negligible against the physics step.

`SITLManager` then gains one path: when a serial port is opened and the probe
returns PX4, build the PX4 bridge over a `SerialTransportAdapter`; otherwise use
the socket adapter exactly as today. `resolve_link()` already returns the plan
to drive that switch.

(Alternative: template `PX4Bridge` on the transport type — cleaner call sites but
requires moving the `px4_bridge.cpp` body into the header. The adapter path is
the smaller, lower-risk change.)

---

## Increment 2 — serial transport wired into PX4Bridge + SITLManager (done)

Implemented and validated:

- **`include/sitl/transport.hpp`** — `ITransport` + `SocketTransportAdapter` +
  `SerialTransportAdapter`. Adapters own their transport by `unique_ptr`, so an
  already-open link can be handed to a bridge without reopening it.
- **`PX4Bridge`** now holds `std::unique_ptr<ITransport>`. Two constructors:
  the original `SocketTransport::Config` (SITL, unchanged call sites) and a new
  `std::unique_ptr<ITransport>` (HITL over serial). `tick()`/`_send_*`/`_parse_*`
  are byte-for-byte unchanged apart from `.` → `->`.
- **`SITLManager`** gains `enabled_hitl`, `serial_port`, `serial_baud` inspector
  properties and a serial state machine driven from `_process()`:
  `Idle → Probing → (Active | Rejected | NoHeartbeat)`. On a PX4 board it moves
  the open link into a `SerialTransportAdapter`, builds the PX4 bridge over it,
  and `physics_tick()` streams HIL unchanged. An ArduPilot board is rejected
  (message points to SITL). `get_sitl_status()["hitl"]` reports state/autopilot/note.
  When HITL owns the PX4 slot, the socket PX4 build is skipped and an active
  serial bridge survives bridge rebuilds.

### Validation (local, GCC-13, `-std=c++20 -Wall -Wextra`)

- `tools/serial_router_selftest.cpp` — 14/14 pass (probe v1/v2, router matrix, PTY).
- `tools/px4_hitl_roundtrip_test.cpp` — 8/8 pass: **PX4Bridge over
  `SerialTransportAdapter` across a PTY**, streaming HIL_SENSOR (sim→board, sim
  time in `time_usec`) and parsing HIL_ACTUATOR_CONTROLS (board→sim) with the
  0.10/0.20/0.30/0.40 controls round-tripping.
- Backward-compat: the `SocketTransport::Config` constructor still compiles and
  links against the refactored header.
- **Not run here:** `SITLManager` (needs `godot_cpp`) — validated by inspection;
  every symbol it references is present in headers. Win32 COM path (needs MSVC).
  A live PX4 board over USB (needs hardware + `SYS_HITL=1`).

### Using it (GDScript)

```gdscript
var m := $DroneBody/SITLManager
m.serial_port = "/dev/ttyACM0"   # or "COM3"
m.serial_baud = 921600
m.enabled_hitl = true            # probes; promotes to PX4 HITL automatically
# ... each frame:
var s := m.get_sitl_status()
print(s["hitl"]["state"], " ", s["hitl"]["note"])
```

Precondition the software can't set for you: put the PX4 board in HITL mode
(`SYS_HITL=1`) with a HIL airframe in QGroundControl, once.
