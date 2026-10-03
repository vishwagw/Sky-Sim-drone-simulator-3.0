"""Dataset round-trip: RecordRun writes an episode, load_run reads it back, and
SkySimDataset yields (obs, action) pairs aligned as obs[:-1] -> action. Also
checks that the domain-randomization dict recorded in the manifest survives the
round-trip (this is the coverage that makes Issue 1's DR bug catchable).
"""
import numpy as np
import pytest

from skysim import (SkySimEnv, HoverTask, RecordRun, DomainRandomizer,
                    load_run, SkySimDataset)


def test_record_load_dataset_roundtrip(mock_server, tmp_path):
    inner = SkySimEnv(port=mock_server, task=HoverTask(), max_steps=15)
    env = RecordRun(inner, out_dir=str(tmp_path))

    requested_dr = DomainRandomizer(seed=0).sample()
    env.reset(seed=0, options={"randomize": requested_dr})

    hover = np.array([0, 0, 0, 0.5], np.float32)
    steps, terminated, truncated = 0, False, False
    while not (terminated or truncated):
        _, _, terminated, truncated, _ = env.step(hover)
        steps += 1
    env.close()

    # An episode directory with a manifest was written.
    run_dir = env._run_dir
    assert run_dir is not None
    assert (run_dir / "manifest.json").exists()

    data = load_run(run_dir)
    m = data["manifest"]

    # Buffer/manifest alignment: n_obs == n_steps + 1 (initial obs included).
    assert m["n_steps"] == steps
    assert m["n_obs"] == steps + 1
    assert data["state"].shape[0] == steps + 1
    assert data["actions"].shape == (steps, 4)

    # Domain-randomization survived the round-trip.
    assert m["dr_requested"] is not None
    assert set(m["dr_requested"]) == set(requested_dr)
    assert m["dr_requested"]["mass_scale"] == pytest.approx(requested_dr["mass_scale"])
    # Whatever the server reports as *applied* must be consistent with what we
    # asked for (equal here; a subset would also be acceptable and still pass).
    applied = m["dr_applied"]
    assert applied is not None
    for key, value in applied.items():
        assert requested_dr[key] == pytest.approx(value) \
            if isinstance(value, (int, float)) else requested_dr[key] == value

    # SkySimDataset aligns obs[:-1] with actions.
    ds = SkySimDataset(str(tmp_path), keys=("state",))
    assert len(ds) == steps
    o0, a0 = ds[0]
    assert np.asarray(o0).shape == (12,)
    assert np.asarray(a0).shape == (4,)


def test_dataset_discovers_multiple_runs(mock_server, tmp_path):
    hover = np.array([0, 0, 0, 0.5], np.float32)
    total = 0
    for seed in (0, 1):
        env = RecordRun(SkySimEnv(port=mock_server, task=HoverTask(), max_steps=10),
                        out_dir=str(tmp_path))
        env.reset(seed=seed)
        terminated = truncated = False
        while not (terminated or truncated):
            _, _, terminated, truncated, _ = env.step(hover)
            total += 1
        env.close()

    ds = SkySimDataset(str(tmp_path), keys=("state",))
    assert len(ds) == total       # both episodes concatenated
