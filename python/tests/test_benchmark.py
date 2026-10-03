"""The benchmark runner must produce a well-formed, stable scorecard: the right
keys, one episode per seed, and per-episode fields the community can compare.

Uses a reduced-step copy of the standard ``hover`` benchmark so the structural
assertions run fast while exercising the real ``run_benchmark`` code path.
"""
import dataclasses

import numpy as np

from skysim import run_benchmark, SUITE


def hover_policy(_obs):
    # Constant mid throttle — hovers in the mock's simple dynamics.
    return np.array([0, 0, 0, 0.5], np.float32)


def test_run_benchmark_scorecard_is_well_formed(mock_server):
    bench = dataclasses.replace(SUITE["hover"], seeds=(0, 1, 2), max_steps=40)
    card = run_benchmark(hover_policy, bench, port=mock_server)

    assert card["benchmark"] == "hover"
    assert card["n_episodes"] == 3
    for key in ("description", "reward_mean", "reward_std", "collision_rate", "episodes"):
        assert key in card
    assert isinstance(card["reward_mean"], float)
    assert 0.0 <= card["collision_rate"] <= 1.0

    assert len(card["episodes"]) == 3
    seeds_seen = sorted(ep["seed"] for ep in card["episodes"])
    assert seeds_seen == [0, 1, 2]
    for ep in card["episodes"]:
        for key in ("seed", "steps", "total_reward", "terminated",
                    "truncated", "collision", "final_pos"):
            assert key in ep
        assert len(ep["final_pos"]) == 3
