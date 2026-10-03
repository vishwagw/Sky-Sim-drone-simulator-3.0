# SkySim Python client

Point any control or RL algorithm at a simulated drone through a standard
[Gymnasium](https://gymnasium.farama.org/) interface. Talks to a running SkySim
instance over WebSocket (protocol: [`../docs/agent_protocol.md`](../docs/agent_protocol.md)).

## Install

```bash
pip install -e .            # core (numpy, gymnasium, websocket-client)
pip install -e ".[rl]"      # + stable-baselines3 for training
pip install -e ".[dev]"     # + websockets, pytest (for the mock server)
```

## Run

Start the simulator's headless agent server (needs the built extension):

```bash
godot --headless demo/agent_server.tscn -- --port 5557
```

Then drive it:

```python
from skysim import SkySimEnv, HoverTask

env = SkySimEnv(port=5557, task=HoverTask(target=(0, 3, 0)))
obs, info = env.reset(seed=0)
for _ in range(1000):
    action = env.action_space.sample()          # [roll, pitch, yaw_rate, throttle]
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        obs, info = env.reset()
env.close()
```

- **Action** (Box, 4-d): `[roll, pitch, yaw_rate, throttle]`, first three in
  [-1,1] (scaled to max tilt/rate), throttle in [0,1].
- **Observation** (Box, 12-d): ground-truth `pos`, `vel`, `euler`, body rates.
- **Reward/termination**: from the pluggable `Task` (`HoverTask`, `WaypointTask`,
  or your own subclass).

## Examples

```bash
python examples/random_agent.py --port 5557      # smoke test, no ML deps
python examples/pid_hover.py --port 5557          # classical PID hover
python examples/train_rl.py --port 5557 --timesteps 200000   # PPO (needs [rl])
```

## No Godot yet? Develop against the mock

`tests/mock_server.py` is a pure-Python fake drone that speaks the same protocol,
so you can build and test clients before the simulator is running:

```bash
python tests/mock_server.py --port 5557    # terminal 1
python examples/pid_hover.py --port 5557   # terminal 2
```

## Record data, build datasets, replay (Phase 3)

Record randomized episodes to disk, then load them for imitation learning or
offline RL:

```python
from skysim import SkySimEnv, NavTask, RecordRun, DomainRandomizer, SkySimDataset

env = RecordRun(SkySimEnv(port=5557, task=NavTask(), state_mode="gps_denied",
                          include_depth=True), out_dir="runs")
dr = DomainRandomizer(seed=1234)
for ep in range(20):
    obs, info = env.reset(seed=ep, options={"randomize": dr.sample()})
    done = False
    while not done:
        obs, r, term, trunc, info = env.step(env.action_space.sample())
        done = term or trunc

ds = SkySimDataset("runs", keys=("state", "depth"))   # imitation-learning pairs
obs, action = ds[0]
```

Each run directory holds `state/depth/rgb/actions/rewards/gt_pos.npy` plus a
`manifest.json` recording the seed, task, env config, and the exact
randomization used — so every dataset is reproducible.

- `python examples/collect_dataset.py --port 5557 --episodes 20`
- `python examples/replay_run.py runs/<run_dir> --port 5557`  (determinism check)

## Running the tests (fully offline — no Godot, no GPU)

The client ships an automated [`pytest`](https://pytest.org) suite that runs
against the pure-Python mock server (`tests/mock_server.py`) — it needs no Godot
install and no hardware. A session-scoped fixture starts the mock on an ephemeral
port, so nothing has to be launched by hand.

```bash
cd python
pip install -e ".[dev]"     # numpy, gymnasium, websocket-client, websockets, pytest
pytest                      # ~1 s; discovers tests/ automatically
```

What's covered:

- **Env contract** — action/observation spaces, `full` vs `gps_denied` state,
  Box→Dict switching for depth/RGB, the Gymnasium 5-tuple, truncation at
  `max_steps`, and a clean `close()`.
- **Tasks** — `HoverTask` / `WaypointTask` / `NavTask` reward and termination,
  as pure units on hand-built observations (no server).
- **Benchmark** — `run_benchmark` yields a well-formed scorecard with the
  expected keys and one episode per seed.
- **Determinism** — `check_determinism` reports `deterministic=True` against the
  (deterministic) mock server.
- **Dataset round-trip** — `RecordRun → load_run → SkySimDataset` produces
  `(obs, action)` pairs aligned as `obs[:-1] → action`, and the domain-
  randomization dict recorded in the manifest survives the round-trip.

The tests fail loudly if the observation schema, task rewards, or dataset
alignment regress. Wiring `pytest` into CI on pull requests is a natural
follow-up.
