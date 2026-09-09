# Battery model

SkySim ships a first-order Li-ion / LiPo pack discharge model
(`include/power/battery.hpp`). It replaces the old placeholder that pinned
`battery_voltage` to a constant, so the HUD now shows a pack that actually
drains and sags under load. The model is header-only and shared verbatim by the
Godot GDExtension (`DroneBody`) and the WebAssembly core (`DroneCore`), so the
browser and native builds agree.

## What it models

| Quantity            | How it works                                                                 |
|---------------------|------------------------------------------------------------------------------|
| State of charge     | Integrated from the current actually drawn (amp-seconds); monotone in discharge. |
| Open-circuit voltage| Monotonic per-cell SoC→voltage curve, LiPo-shaped, anchored to full/nominal/empty. |
| Terminal voltage    | `V_term = V_ocv − I·R`, with current solved consistently from `P = V_term·I`. |
| Peukert derating    | High discharge rates remove charge slightly faster (Peukert's law).          |
| Temperature         | Lumped thermal: `I²R` self-heating vs Newtonian cooling to ambient.          |
| Cell imbalance      | Small, deterministic per-cell offsets so the per-cell HUD looks alive.       |

The terminal-voltage solve is the reason voltage **sags under high throttle and
recovers when the load drops**: for a commanded electrical power `P`,

```
P = V_term · I ,  V_term = V_ocv − I·R   ⟹   R·I² − V_ocv·I + P = 0
```

is solved for the physical (smaller) current root each tick. Higher power ⇒
higher current ⇒ larger `I·R` drop.

The model is **deterministic** — no per-tick RNG — so runs stay reproducible.
The first-order thermal and cell-imbalance sub-models are deliberately simple
stand-ins for the HUD, not validated cell physics; that is called out in the
header comments rather than hidden.

## Defaults & calibration (the shipped quad)

Defaults describe a **4S 4000 mAh LiPo**, pack ESR **30 mΩ**, driving the shipped
~1.5 kg quad:

- Full charge: **16.8 V** (4.20 V/cell) · Nominal: 14.8 V (tied to `max_voltage`)
- Low-voltage warning at **3.50 V/cell** (terminal), hard cutoff at **3.30 V/cell**

The quad's **steady hover power measured from the sim's own rotor model is
≈ 154 W** (thrust ≈ weight ≈ 14.7 N near 34 % throttle). Flying a hover-hold to
cutoff with those defaults gives:

```
LOW WARN  ~18.1 min   SoC 20%   14.00 V
CUTOFF    ~21.3 min   SoC  4%   13.20 V   (~3840 mAh used, ~4% reserve)
```

That ~21 min hover endurance is the honest consequence of the pack energy
(4000 mAh × 14.8 V ≈ 59 Wh) and the sim's hover power (≈ 154 W). The sim's rotor
model is on the efficient side, so a heavier real-world equivalent would see
shorter times; scale `battery_capacity_mah` down (e.g. 3000 mAh → ~16 min) if you
want a tighter budget.

Reproduce the numbers:

```bash
g++ -std=c++20 -Iinclude -Iweb/core \
    web/core/drone_core.cpp tests/cpp/test_battery_core.cpp -o /tmp/t && /tmp/t
```

## Editor properties (`DroneBody`)

Bound like the existing `max_voltage` property, with sane quad defaults:

- `battery_capacity_mah` — rated pack capacity (default 4000)
- `battery_cells` — series cell count / S (default 4)
- `battery_internal_resistance` — pack ESR in ohms (default 0.030)

`max_voltage` continues to represent the nominal pack voltage and anchors the OCV
curve's nominal knot. SoC resets to full on `arm()` (and on episode reset, which
re-arms).

## Telemetry keys (`get_telemetry()` / `obs.telemetry`)

`battery_voltage` (terminal), `battery_voltage_ocv`, `battery_current`,
`battery_soc`, `battery_mah_used`, `battery_temp_c`, `battery_flight_time_min`,
`battery_peukert_eff`, `battery_cell_imbalance_mv`, `battery_low_warn`,
`battery_cutoff`, and `cell_voltages` (per-cell terminal). `demo/scripts/battery_hud.gd`
consumes these directly — the HUD drains over a hover with no changes to it.

The WASM core exposes `skysim_battery_voltage`, `skysim_battery_soc`,
`skysim_battery_current`, `skysim_battery_cutoff`, and `skysim_power` via the C API.

## Battery → thrust coupling

The pack's terminal voltage feeds back into the motors: each rotor's available
RPM ceiling is `motor_kv · V_supply`, and the supply voltage is the battery's
live terminal voltage (from the previous tick, which breaks the
thrust→power→voltage algebraic loop). Because thrust ∝ ω², a pack sagging to a
fraction *f* of nominal delivers ≈ *f*² of its full-throttle thrust.

To keep existing flight tuning intact, the coupling only ever *reduces* authority:
a healthy pack at or above nominal (14.8 V) leaves the supply scale at 1.0, so
short flights and benchmarks are unchanged. As the pack drains and terminal
voltage falls below nominal (roughly the second half of a hover), the craft grows
sluggish and eventually can't hold altitude — the realistic end-of-charge
behaviour. Terminal voltage crosses nominal near ~50 % SoC for the shipped 4S
pack, so the effect appears only on sustained flights.

Toggle it with the `battery_thrust_coupling` editor property (native, default on),
`DroneCore::set_battery_thrust_coupling(bool)` / `skysim_set_battery_thrust_coupling`
(web). The behaviour is a deterministic function of battery state — no RNG — so
determinism is preserved. Shared verbatim by the native and web tiers via
`RotorArray::set_supply_voltage()`.

## Not modelled / possible follow-ups

- No charge/regeneration, self-discharge, or temperature-dependent ESR.
