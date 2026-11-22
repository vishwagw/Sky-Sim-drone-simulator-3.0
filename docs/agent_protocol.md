# SkySim Agent Protocol v1

The contract between a running SkySim instance (the **server**) and an external
algorithm (the **client**, e.g. a Python RL policy). It's deliberately
transport-simple and language-agnostic so the same algorithm can later target a
real vehicle: design your algorithm against *this* interface, not against Godot.

## Transport

- **WebSocket**, one JSON object per text frame. (WebSocket — not raw TCP — so
  the browser front-end can reuse this exact protocol later. On localhost the
  overhead is negligible.)
- Server listens on a port (default **5557**), single client.
- **Lockstep:** the simulation advances exactly **one physics tick per action**.
  The server blocks between ticks waiting for the client, so a slow algorithm
  never desyncs from physics and runs are reproducible.

## Lifecycle

```
client connects
server → hello
loop:
  client → reset            (start/restart an episode)
  server → obs              (initial observation)
  loop:
    client → action
    server → obs            (result of stepping one tick)
  until episode ends (client decides, using obs)
client → close
```

There is a **1-tick actuation latency**: the effect of `action` at step *t* is
reflected in the observation returned at step *t+1* (as on real hardware). This
is consistent and RL-friendly.

## Messages: server → client

### `hello` (once, on connect)
```json
{ "type": "hello", "protocol": 1, "physics_hz": 400, "dt": 0.0025,
  "control_modes": ["attitude", "motors"] }
```

### `obs` (reply to every `reset` and `action`)
```json
{ "type": "obs",
  "t": 1.25, "step": 500,
  "gt": {                         // ground truth (the sim's advantage; use for reward/labels)
    "pos":  [x, y, z],            // metres, Godot world frame (Y up)
    "vel":  [vx, vy, vz],         // m/s
    "quat": [x, y, z, w],
    "euler":[roll, pitch, yaw],   // radians
    "ang_vel":[wx, wy, wz]        // rad/s, body
  },
  "telemetry": { ... },           // full get_telemetry() dict (altitude, thrust,
                                  // power, vrs_active, ground_effect_factor,
                                  // air_density, wind, ...)
  "depth": {                      // raycast depth+semantics (see below), or null
    "cols": 32, "rows": 24, "max_range": 40.0,
    "ranges": [ ... cols*rows floats, metres, row-major, row 0 = top ... ],
    "classes":[ ... cols*rows ints: 0 none,1 obstacle,2 ground,3 goal,4 other ]
  },
  "camera": {                     // RGB (needs GPU context), or null
    "w": 128, "h": 96, "encoding": "raw_rgb8", "data": "<base64 raw RGB bytes>"
  },
  "collision": false,
  "out_of_bounds": false }
```

**Sensor rate.** `depth`/`camera` are produced at the server's `camera_hz`
(default 30), a fraction of the physics rate. On ticks in between they are
`null`; the client should **hold the previous frame**. Both stay `null` if no
perception is configured (Phase-1 behaviour).

**Depth vs RGB.** Depth+semantics come from raycasts — they work in `--headless`,
are deterministic, and suit obstacle/nav learning. RGB comes from a rendered
viewport and requires a real GPU/display context (a windowed run, or Xvfb on a
server) — it is `{}`/`null` under the dummy `--headless` renderer.
```

### `error`
```json
{ "type": "error", "message": "..." }
```

## Messages: client → server

### `reset`
```json
{ "type": "reset", "seed": 0, "spawn": [0, 2, 0], "arm": true,
  "randomize": {                         // optional domain randomization
    "mass_scale": 1.07,
    "wind": [1.2, 0.0, -0.4],
    "sensor_noise": 1.5,
    "spawn_jitter": [0.3, 0.0, -0.6],
    "attitude_jitter_deg": [2.1, -1.0, 3.4]
  } }
```
`seed` (nullable) seeds sim randomisation. `spawn` (nullable) overrides the spawn
position. `arm` arms the flight controller. `randomize` (optional) perturbs the
episode; the server applies what it can (mass, spawn, initial attitude; wind and
sensor noise where the extension exposes setters) and **echoes the applied dict
back as `obs.dr`** so every run is logged and reproducible. Server teleports the
drone, zeroes velocities, resets `step`/`t`, and replies with the initial `obs`.

`sensor_noise` is a multiplier on the default sensor noise (IMU, barometer and GPS), applied through `DroneBody.set_sensor_noise_scale`: `0` gives noise-free sensors, `1` keeps the default noise, and `2` roughly doubles the spread. Negative values are treated as `0`.

### `action`
Attitude/rate setpoint (maps to `DroneBody.set_attitude_setpoint`):
```json
{ "type": "action", "mode": "attitude",
  "roll": 0.0, "pitch": 0.1, "yaw_rate": 0.0, "throttle": 0.55 }
```
`roll`/`pitch` in radians, `yaw_rate` in rad/s, `throttle` in [0,1].

Direct per-motor (maps to `DroneBody.set_rotor_throttles`):
```json
{ "type": "action", "mode": "motors", "throttles": [0.5, 0.5, 0.5, 0.5] }
```

### `close`
```json
{ "type": "close" }
```

## Versioning

`hello.protocol` is an integer. Additive fields (new obs keys, new control modes)
do **not** bump it; breaking changes do. Clients should ignore unknown obs keys
so Phase 2 (`camera`/`depth`) is backward-compatible.

## Notes for a faithful implementation

- Ground truth is read directly from the `DroneBody` (a `RigidBody3D`): position,
  `linear_velocity`, `angular_velocity`, orientation. No extra C++ bindings
  needed.
- The reference server is `demo/scripts/agent_server.gd`. A minimal reference
  **mock** server (pure Python, fake physics) lives in
  `python/tests/mock_server.py` — useful for developing clients without Godot,
  and as an executable spec of this document.
