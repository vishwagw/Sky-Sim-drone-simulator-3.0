# SkySim — open-source drone simulator

[![CI](https://github.com/OWNER/REPO/actions/workflows/ci.yml/badge.svg)](https://github.com/OWNER/REPO/actions/workflows/ci.yml)
<!-- Replace OWNER/REPO above with this repository's GitHub slug once it's public. -->

A cross-platform drone simulator with real aerodynamics computed in C++20, and
the ability to hand the vehicle to actual flight-controller firmware —
ArduPilot (JSON SITL), PX4 (MAVLink HIL), and Betaflight (MSP).

Physics runs in a Godot 4 GDExtension via `_integrate_forces` — **zero GDScript
in the hot path**. Runs on Windows, Linux, and macOS (Intel + Apple Silicon).

## Status

SkySim now spans the full stack from a fly-it-yourself sim to a browser tier and
a benchmark suite. What's **tested end-to-end**: the Python agent interface,
perception, data pipeline, benchmarks, and determinism harness; the Godot agent
server, perception, and domain randomization (validated against Godot 4.3); and
the web physics core (decoupled from Godot, validated natively).

What still needs the compiled C++ extension + hardware: real-flight aerodynamics,
RGB rendering, cross-machine determinism numbers, and — the one thing no code can
produce — a **sim-to-real transfer result** (see `docs/sim_to_real.md`).

The pitch, honestly stated: every simulator that lets you test drone algorithms
needs Linux, ROS, or a game engine and a GPU. SkySim's goal is to make it a URL.
See [`ROADMAP.md`](ROADMAP.md) for phase-by-phase progress.

---

## Get it running — pick your level

### 0. Fly it in your browser — nothing to install (Phase 4, preview)

The physics core compiles to WebAssembly and runs entirely client-side. Open the
hosted page and fly with the keyboard, or drop in a JavaScript control function.
See [`web/`](web/) to build and host it (`web/build_wasm.sh`, then serve `web/`).
Heavy ML and firmware SITL use the native agent server; the browser tier is for
interactive flight and classical/light control.

### 1. Just want to try it? Download and run. No install.

Grab a standalone build from the [**Releases**](../../releases) page:

| Your OS | Download | How to run |
|---------|----------|------------|
| Windows | `SkySim-windows.zip` | Unzip, double-click `SkySim.exe` |
| Linux | `SkySim-linux.zip` | Unzip, `chmod +x SkySim.x86_64`, run it |
| macOS | `SkySim-macos.zip` | Unzip, right-click the app → **Open** (unsigned) |

No Godot, no compiler, no dependencies. Everything is bundled.

### 2. Want to open it in the Godot editor (to modify scenes/scripts)?

1. Install **[Godot 4.3+](https://godotengine.org/download)** — one portable
   download, no admin rights, no installer needed.
2. From [Releases](../../releases), download `SkySim-godot-project.zip` and
   unzip it. **Prebuilt extension binaries for every OS are already inside** —
   you don't compile anything.
3. Open Godot → *Import* → select the unzipped folder → **Play**.

### 3. Developer / modder — build the C++ extension from source

You only need this if you're changing the C++ physics. It's one command and
**godot-cpp is fetched automatically — no git submodules**.

```bash
# Linux / macOS
./scripts/build.sh

# Windows
scripts\build.bat
```

<details>
<summary>Prefer raw CMake?</summary>

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
# Output: gdextension/bin/(lib)drone_sim.<platform>.release.<arch>.(dll|so|dylib)
```

Requirements: CMake ≥ 3.22, a C++20 compiler (MSVC 2022 / GCC 12+ / Clang 15+),
Python 3.8+. Match `godot-cpp` to your editor with
`-DGODOTCPP_GIT_TAG=godot-4.3-stable`.

**Portability note:** binaries are built without `-march=native` on purpose, so
they run on any CPU of the same architecture. Turn on `-DDRONE_SIM_NATIVE=ON`
*only* for a private single-machine build — it bakes in your CPU's instruction
set and will crash elsewhere.
</details>

> **How the cross-platform binaries are made:** you don't make them. Pushing a
> `v*` tag triggers GitHub Actions (`.github/workflows/`), which compiles the
> extension on Windows, Linux, and macOS (x86_64 + arm64), exports the
> standalone apps, and attaches everything to a Release automatically.

---

## Architecture

```
DroneBody (RigidBody3D subclass)
├── RotorArray → BladeElementSolver × N   ← BET thrust/torque per rotor
├── Atmosphere        ← ISA density/pressure + Dryden turbulence (MIL-HDBK-1797)
├── AeroEffectsBundle ← Cheeseman-Bennett ground effect + Leishman VRS
├── FlightController  ← PX4-style cascade PID (attitude + rate)
├── MixerMatrix       ← wrench → per-rotor throttle allocation
├── SensorSuite       ← IMU / barometer / GPS with noise + bias
└── SITLManager       ← ArduPilot / PX4 / Betaflight bridges
```

Control priority each physics tick: **SITL firmware** (fresh actuator frame
< 0.5 s) → `set_rotor_throttles()` → internal PID → motors off.

### Aerodynamics

- **Blade Element Theory** — 24 radial annuli per rotor; induced velocity via 3
  Newton iterations of Rankine-Froude momentum theory; 1st-order ESC lag
  (`esc_tau = 15 ms`) and gyroscopic precession.
- **Ground effect** — Cheeseman-Bennett `T_IGE/T_OGE = 1/(1 − (R/4h)²)`,
  blended out above `h/R = 3`.
- **Vortex Ring State** — Leishman onset envelope with hysteresis (0.5 s
  build-up, 1.2 s recovery), up to 30% thrust loss with ~2.3 Hz buffeting.
- **ISA atmosphere** — standard troposphere, Sutherland's-law viscosity.
- **Dryden turbulence** — per-axis 1st-order shaping filter, runtime-tunable.

---

## Using SkySim in your own Godot project

1. Copy the `gdextension/` folder into your project root (Godot 4 auto-detects
   it — no plugin to enable).
2. Add a `DroneBody` node (appears under `RigidBody3D`) with a
   `CollisionShape3D` and a visual `MeshInstance3D` child.
3. For firmware-in-the-loop, add a `SITLManager` as a child of `DroneBody`.
4. Set **Project Settings → Physics → Physics Ticks per Second = 250–400**
   (400 for ArduCopter). This project already ships at 400.

---

## Flying with real flight-controller firmware (SITL)

| Firmware | Transport | Port | Who connects |
|----------|-----------|------|--------------|
| ArduPilot | UDP (sim = server) | 9002 | AP sends servo packets to us |
| PX4 | TCP (sim = server) | 4560 | PX4 SITL connects to us |
| Betaflight | TCP (sim = client) | 5760 | We connect to Betaflight |

```bash
# ArduPilot
cd ardupilot && sim_vehicle.py -v ArduCopter -f JSON:127.0.0.1 --console --map

# PX4
cd PX4-Autopilot && make px4_sitl none_iris

# Betaflight
cd betaflight && make TARGET=SITL && ./obj/main/betaflight_SITL.elf
```

Run the SkySim scene first, then start the firmware. Arming, modes, and
missions come from your GCS (Mission Planner / QGroundControl). Frames are
aerospace-standard (NED world / FRD body, see `include/core/frames.hpp`); rotor
order follows the ArduPilot/PX4 quad-X convention.

`tools/ap_json_probe.py` and `tools/px4_probe.py` let you exercise the bridges
without a full firmware checkout.

---

## Telemetry

`drone.get_telemetry()` returns every physics tick: `altitude`, `ground_speed`,
`vertical_speed`, `roll/pitch/yaw_deg`, `roll/pitch/yaw_rate`, `total_thrust`,
`power_draw`, `vrs_active`, `vrs_severity`, `ground_effect_factor`,
`air_density`, `wind`, `sitl_active`, `sitl_source`. Live PID tuning:

```gdscript
drone.set_rate_roll_pid(0.15, 0.05, 0.003)
drone.set_rate_pitch_pid(0.15, 0.05, 0.003)
drone.set_rate_yaw_pid(0.20, 0.10, 0.0)
```

---

## Benchmarks & reproducibility

A small, stable suite of seeded tasks (`hover`, `waypoint`, `gps_denied_nav`)
lets algorithms be compared and cited:

```bash
python examples/run_benchmark.py --port 5557 --out scorecard.json
python examples/check_determinism.py --port 5557
```

For reference-quality reproducible numbers, build the extension with
`-DDRONE_SIM_DETERMINISTIC=ON` (strict floating point). Details and how to submit
results: [`docs/benchmarks.md`](docs/benchmarks.md).

## Contributing

Issues and PRs welcome. CI builds every PR on all three platforms, so if it
compiles in the workflow it works for everyone. Physics stays in C++
(`src/`, `include/`); scene logic and dynamic-world helpers stay in GDScript
(`demo/`).

## License

MIT — see [LICENSE](LICENSE).
