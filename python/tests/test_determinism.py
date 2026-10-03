"""The determinism harness must confirm that the same seed + same actions yield
the same trajectory. The mock server is deterministic, so this both checks the
harness and guards the reproducibility guarantee the datasets depend on.
"""
from skysim import check_determinism


def test_mock_server_is_reported_deterministic(mock_server):
    result = check_determinism(port=mock_server, seed=0, steps=80)
    assert result["deterministic"] is True
    assert result["steps_compared"] >= 1
    assert result["max_divergence"] < result["atol"]


def test_determinism_result_has_expected_fields(mock_server):
    result = check_determinism(port=mock_server, seed=3, steps=40)
    for key in ("seed", "steps_compared", "max_divergence",
                "mean_divergence", "deterministic", "atol"):
        assert key in result
    assert result["seed"] == 3
