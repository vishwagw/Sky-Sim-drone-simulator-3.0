<div align="center">

# 🛸 SkySim


https://github.com/user-attachments/assets/57c2010c-84c4-4cbf-9fed-58a11da762e7


**Test drone autonomy in your browser — zero install.**

Real aerodynamics in C++20. Real flight-controller firmware in the loop.
ArduPilot · PX4 · Betaflight · Windows · Linux · macOS · WebAssembly

[![CI](https://github.com/OWNER/REPO/actions/workflows/ci.yml/badge.svg)](https://github.com/OWNER/REPO/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Godot 4.3+](https://img.shields.io/badge/Godot-4.3%2B-478cbf?logo=godotengine&logoColor=white)](https://godotengine.org)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C?logo=cplusplus)](src/)
[![PRs welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](#-contributing)
[![Live demo](https://img.shields.io/badge/▶_live_demo-fly_in_your_browser-ff6b6b)](https://OWNER.github.io/REPO/)
<!-- Replace OWNER/REPO (badges and demo links) with this repository's GitHub slug. -->

<a href="https://OWNER.github.io/REPO/">
<img src="docs/media/skysim-demo.gif" alt="SkySim running in the browser: a quadcopter flying an obstacle course with live altitude, speed and mode telemetry. Click to fly it." width="820">
</a>

### [▶ Fly it now in your browser — no install](https://OWNER.github.io/REPO/)

**[⬇ Download](../../releases)** ·
**[⬇ Download](../../releases)** ·
**[🧪 Benchmarks](#-benchmarks--reproducibility)** ·
**[🗺 Roadmap](ROADMAP.md)** ·
**[🤝 Contribute](#-contributing)**

</div>

---

## Why SkySim?

Every simulator that lets you test drone algorithms needs Linux, ROS, or a game
engine and a GPU. **SkySim's goal is to make it a URL.**

| | |
|---|---|
| 🌐 **Runs in a browser** | The physics core compiles to WebAssembly — open a page, fly with the keyboard, or drop in a JavaScript control function. |
| ✈️ **Real aerodynamics** | Blade Element Theory, ground effect, vortex ring state, ISA atmosphere, Dryden turbulence — all in C++20, zero GDScript in the hot path. |
| 🔌 **Real firmware in the loop** | Hand the vehicle to ArduPilot (JSON SITL), PX4 (MAVLink HIL), or Betaflight (MSP) and fly it from Mission Planner or QGroundControl. |
| 🤖 **Built for algorithms** | Python agent interface, perception, data pipeline, domain randomization, and a seeded benchmark suite for comparable, citable results. |
| 💻 **Cross-platform** | Windows, Linux, and macOS (Intel + Apple Silicon). Prebuilt binaries for every OS — nothing to compile. |

## Status — honestly stated

**Tested end-to-end:** the Python agent interface, perception, data pipeline,
benchmarks, and determinism harness; the Godot agent server, perception, and
domain randomization (validated against Godot 4.3); and the web physics core
(decoupled from Godot, validated natively).

**Still needs the compiled C++ extension + hardware:** real-flight aerodynamics,
RGB rendering, cross-machine determinism numbers, and — the one thing no code can
produce — a **sim-to-real transfer result** (see [`docs/sim_to_real.md`](docs/sim_to_real.md)).
That last one is the single most valuable contribution open right now.

See [`ROADMAP.md`](ROADMAP.md) for phase-by-phase progress.

---

## 🚀 Get it running — pick your level

### 0. Fly it in your browser — nothing to install

*(Phase 4, preview)*

**👉 [Open the live demo](https://OWNER.github.io/REPO/)** — works in any modern
desktop browser (Chrome, Firefox, Edge, Safari).

The physics core compiles to WebAssembly and runs entirely client-side. Fly with
the keyboard, or drop in a JavaScript control function. The demo is rebuilt and
redeployed automatically on every push to `main`
(`.github/workflows/pages.yml`). To host your own copy, run `web/build_wasm.sh`
and serve `web/`.
Heavy ML and firmware SITL use the native agent server; the browser tier is for
interactive flight and classical/light control.

<table>
<tr>
<td width="50%"><img src="docs/media/browser-controls.png" alt="Browser tier: keyboard controls and live HUD"></td>
<td width="50%"><img src="docs/media/browser-js-control.png" alt="Browser tier: writing a JavaScript control function"></td>
</tr>
<tr>
<td align="center"><sub>Fly with the keyboard — HUD shows altitude, speed and mode</sub></td>
<td align="center"><sub>Or drop in your own JavaScript controller</sub></td>
</tr>
</table>

### 1. Just want to try it? Download and run

Grab a standalone build from the [**Releases**](../../releases) page:

| Your OS | Download | How to run |
|---------|----------|------------|
| Windows | `SkySim-windows.zip` | Unzip, double-click `SkySim.exe` |
| Linux | `SkySim-linux.zip` | Unzip, `chmod +x SkySim.x86_64`, run it |
| macOS | `SkySim-macos.zip` | Unzip, right-click the app → **Open** (unsigned) |

No Godot, no compiler, no dependencies. Everything is bundled.

<p align="center">
<img src="docs/media/releases-page.png" alt="GitHub Releases page with the SkySim download for each OS" width="700"><br>
<sub>Pick the zip for your OS from the Assets list on the latest release</sub>
</p>

<details>
<summary>macOS says the app "can't be opened"?</summary>

The build is unsigned, so Gatekeeper blocks a normal double-click. Right-click
(or Control-click) the app and choose **Open**, then confirm once.

<img src="docs/media/macos-open.png" alt="macOS right-click menu with Open highlighted" width="480">
</details>

### 2. Open it in the Godot editor (to modify scenes/scripts)

1. Install **[Godot 4.3+](https://godotengine.org/download)** — one portable
   download, no admin rights, no installer needed.
2. From [Releases](../../releases), download `SkySim-godot-project.zip` and
   unzip it. **Prebuilt extension binaries for every OS are already inside** —
   you don't compile anything.
3. Open Godot → *Import* → select the unzipped folder → **Play**.

<table>
<tr>
<td width="33%"><img src="docs/media/godot-import.png" alt="Godot Project Manager with the Import button"></td>
<td width="33%"><img src="docs/media/godot-select-folder.png" alt="Selecting the unzipped SkySim folder"></td>
<td width="33%"><img src="docs/media/godot-play.png" alt="Godot editor with the Play button highlighted"></td>
</tr>
<tr>
<td align="center"><sub>① Click <b>Import</b></sub></td>
<td align="center"><sub>② Select the unzipped folder</sub></td>
<td align="center"><sub>③ Press <b>Play</b> (F5)</sub></td>
</tr>
</table>

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

## 🏗 Architecture

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

<details>
<summary><b>Aerodynamics in detail</b></summary>

- **Blade Element Theory** — 24 radial annuli per rotor; induced velocity via 3
  Newton iterations of Rankine-Froude momentum theory; 1st-order ESC lag
  (`esc_tau = 15 ms`) and gyroscopic precession.
- **Ground effect** — Cheeseman-Bennett `T_IGE/T_OGE = 1/(1 − (R/4h)²)`,
  blended out above `h/R = 3`.
- **Vortex Ring State** — Leishman onset envelope with hysteresis (0.5 s
  build-up, 1.2 s recovery), up to 30% thrust loss with ~2.3 Hz buffeting.
- **ISA atmosphere** — standard troposphere, Sutherland's-law viscosity.
- **Dryden turbulence** — per-axis 1st-order shaping filter, runtime-tunable.
</details>

---

## 🧩 Using SkySim in your own Godot project

1. Copy the `gdextension/` folder into your project root (Godot 4 auto-detects
   it — no plugin to enable).
2. Add a `DroneBody` node (appears under `RigidBody3D`) with a
   `CollisionShape3D` and a visual `MeshInstance3D` child.
3. For firmware-in-the-loop, add a `SITLManager` as a child of `DroneBody`.
4. Set **Project Settings → Physics → Physics Ticks per Second = 250–400**
   (400 for ArduCopter). This project already ships at 400.

<table>
<tr>
<td width="50%"><img src="docs/media/godot-scene-tree.png" alt="Godot scene tree showing DroneBody with CollisionShape3D, MeshInstance3D and SITLManager children"></td>
<td width="50%"><img src="docs/media/godot-physics-ticks.png" alt="Project Settings: Physics Ticks per Second set to 400"></td>
</tr>
<tr>
<td align="center"><sub>A minimal <code>DroneBody</code> scene tree</sub></td>
<td align="center"><sub>Physics ticks set to 400</sub></td>
</tr>
</table>

---

## 🔌 Flying with real flight-controller firmware (SITL)

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

<p align="center">
<img src="docs/media/sitl-qgc.png" alt="SkySim flying under PX4 SITL with QGroundControl connected side by side" width="820"><br>
<sub>SkySim on the left, QGroundControl on the right — the drone is flown by real PX4 firmware</sub>
</p>

`tools/ap_json_probe.py` and `tools/px4_probe.py` let you exercise the bridges
without a full firmware checkout.

---

## 📈 Telemetry

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

## 🧪 Benchmarks & reproducibility

A small, stable suite of seeded tasks (`hover`, `waypoint`, `gps_denied_nav`)
lets algorithms be compared and cited:

```bash
python examples/run_benchmark.py --port 5557 --out scorecard.json
python examples/check_determinism.py --port 5557
```

For reference-quality reproducible numbers, build the extension with
`-DDRONE_SIM_DETERMINISTIC=ON` (strict floating point). Details and how to submit
results: [`docs/benchmarks.md`](docs/benchmarks.md).

<p align="center">
<img src="docs/media/benchmark-scorecard.png" alt="Terminal output of run_benchmark.py showing per-task scores" width="700"><br>
<sub>Example scorecard from <code>run_benchmark.py</code></sub>
</p>

---

## 🤝 Contributing

Issues and PRs welcome — and you don't need to be a C++ physics person to help.
A few places to start:

- **🎯 Sim-to-real transfer** — fly a SkySim-trained policy on a real drone and
  report what happened. The highest-impact open item ([`docs/sim_to_real.md`](docs/sim_to_real.md)).
- **🏁 Submit a benchmark result** — run the suite with your algorithm and send
  the scorecard ([`docs/benchmarks.md`](docs/benchmarks.md)).
- **🌐 Browser tier** — new scenes, control examples, or UI polish in [`web/`](web/).
- **🔌 Firmware bridges** — test against your ArduPilot / PX4 / Betaflight setup
  and file what breaks.
- **📝 Docs & examples** — a tutorial or example script you wish had existed.

CI builds every PR on all three platforms, so if it compiles in the workflow it
works for everyone. Physics stays in C++ (`src/`, `include/`); scene logic and
dynamic-world helpers stay in GDScript (`demo/`).

If SkySim is useful to you, a ⭐ helps others find it.

## License

MIT — see [LICENSE](LICENSE).
