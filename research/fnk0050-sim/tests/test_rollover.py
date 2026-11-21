from dataclasses import replace
from math import cos, pi, sin

import pytest

from rollover import DRIVE_TICKS, RECOVERY_TICKS, SETTLE_TICKS, Sample, evaluate, run_search


def trace():
    ticks = SETTLE_TICKS + DRIVE_TICKS + RECOVERY_TICKS
    return [Sample((i + 1) * .02, (1, 0, 0, 0), 1, .09, 4, 3, 0, 0, 0)
            for i in range(ticks)]


def test_final_pose_without_rotation_does_not_qualify():
    assert evaluate(trace())["reason"] == "incomplete_roll"


def test_quaternion_sign_changes_are_not_rotation():
    samples = [replace(row, orientation=((-1) ** i, 0, 0, 0)) for i, row in enumerate(trace())]
    assert evaluate(samples)["roll_rad"] == pytest.approx(0)


def test_full_rotation_fixture_and_clock_discontinuity():
    rows = trace()
    for i in range(DRIVE_TICKS):
        theta = (i + 1) * 2 * pi / DRIVE_TICKS
        index = SETTLE_TICKS + i
        rows[index] = replace(rows[index], orientation=(cos(theta / 2), sin(theta / 2), 0, 0),
                              up=cos(theta), supports=0)
    assert evaluate(rows)["accepted"]
    rows[-2] = replace(rows[-2], time=0)
    result = evaluate(rows)
    assert not result["accepted"]
    assert "clock_discontinuity" in result["reason"]


def test_teleport_and_excessive_force_disqualify():
    rows = trace()
    rows[100] = replace(rows[100], orientation=(0, 1, 0, 0), force=500)
    result = evaluate(rows)
    assert not result["accepted"]
    assert "rotation_discontinuity" in result["reason"]
    assert "physics_limits" in result["reason"]


def test_actual_baseline_and_separate_holdout():
    summary = run_search(2)
    assert summary["attempts"] == 10
    assert not summary["accepted"]
    assert [item["seed"] for item in summary["holdout"]] == list(range(500, 508))
