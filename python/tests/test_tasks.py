"""Task reward/termination logic, tested as pure units on hand-built obs
messages — no server, no sockets. These pin the reward shaping so a change in
task semantics is caught immediately.
"""
import pytest

from skysim import HoverTask, WaypointTask, NavTask


def obs(pos, vel=(0.0, 0.0, 0.0), collision=False):
    return {"gt": {"pos": list(pos), "vel": list(vel)}, "collision": collision}


# ---- HoverTask -------------------------------------------------------------
def test_hover_reward_higher_at_target():
    t = HoverTask(target=(0, 3, 0))
    r_at, done = t.reward(obs((0, 3, 0)))
    r_off, _ = t.reward(obs((0, 0, 0)))
    assert r_at > r_off
    assert not done


def test_hover_terminates_on_low_altitude_and_collision():
    t = HoverTask(target=(0, 3, 0), crash_alt=0.18)
    _, done_low = t.reward(obs((0, 0.1, 0)))
    _, done_col = t.reward(obs((0, 3, 0), collision=True))
    assert done_low
    assert done_col


def test_hover_penalises_velocity():
    t = HoverTask(target=(0, 3, 0))
    r_still, _ = t.reward(obs((0, 3, 0), vel=(0, 0, 0)))
    r_fast, _ = t.reward(obs((0, 3, 0), vel=(5, 0, 0)))
    assert r_still > r_fast


# ---- WaypointTask ----------------------------------------------------------
def test_waypoint_advances_index_and_awards_bonus():
    t = WaypointTask(waypoints=[(0, 3, 0), (5, 3, 0)], reach_radius=0.6, reach_bonus=10.0)
    t.reset(obs((0, 0, 0)))

    r_far, done = t.reward(obs((0, 0, 0)))     # far from wp0
    assert t._i == 0 and not done

    r_reach, done = t.reward(obs((0, 3, 0)))   # at wp0 -> bonus, advance
    assert r_reach > r_far
    assert t._i == 1 and not done

    _, done = t.reward(obs((5, 3, 0)))         # at last wp -> finished
    assert t._i == 2 and done


def test_waypoint_terminates_on_collision():
    t = WaypointTask(waypoints=[(0, 3, 0)])
    t.reset(obs((0, 0, 0)))
    _, done = t.reward(obs((0, 0, 0), collision=True))
    assert done


# ---- NavTask ---------------------------------------------------------------
def test_nav_first_step_is_step_penalty_only():
    t = NavTask(goal=(0, 0, -10), reach_radius=1.5, step_penalty=0.01)
    t.reset(obs((0, 0, 0)))
    r0, done0 = t.reward(obs((0, 0, 0)))       # prev_dist unset -> progress 0
    assert not done0
    assert r0 == pytest.approx(-0.01)


def test_nav_rewards_progress_toward_goal():
    t = NavTask(goal=(0, 0, -10), reach_radius=1.5, step_penalty=0.01)
    t.reset(obs((0, 0, 0)))
    t.reward(obs((0, 0, 0)))                    # seed prev_dist at 10
    r1, done1 = t.reward(obs((0, 0, -5)))       # moved 5 m closer
    assert r1 > 0 and not done1


def test_nav_reach_gives_bonus_and_terminates():
    t = NavTask(goal=(0, 0, -10), reach_radius=1.5, reach_bonus=100.0)
    t.reset(obs((0, 0, 0)))
    t.reward(obs((0, 0, 0)))
    r, done = t.reward(obs((0, 0, -10)))
    assert done and r > 50


def test_nav_collision_penalises_and_terminates():
    t = NavTask(goal=(0, 0, -10), collision_penalty=50.0)
    t.reset(obs((0, 0, 0)))
    t.reward(obs((0, 0, 0)))
    r, done = t.reward(obs((0, 0, -1), collision=True))
    assert done and r < 0
