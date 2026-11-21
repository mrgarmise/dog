"""Simulation-only roll-over experiments on the existing FNK0050 model."""

from collections import Counter
from dataclasses import dataclass
from math import pi
from random import Random

import mujoco
import numpy as np

from core import DT, servo_degrees, slew
from mujoco_env import DogEnv

VERSION = "fnk0050-roll-v1"
# Thresholds are research screening parameters for this model version.
UPRIGHT = 0.97
MIN_BASE_HEIGHT = 0.075
MAX_STEP_ROTATION = 0.30
MAX_CONTACT_FORCE = 100.0
JOINT_SLACK = 0.03
SETTLE_TICKS = 30
DRIVE_TICKS = 100
RECOVERY_TICKS = 70


@dataclass(frozen=True)
class Sample:
    time: float
    orientation: tuple
    up: float
    base_height: float
    supports: int
    force: float
    joint_excess: float
    speed: float
    warnings: int


def sample(env):
    model, data = env.model, env.data
    support = set()
    force = 0.0
    for index in range(data.ncon):
        contact = data.contact[index]
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, wrench)
        force = max(force, float(np.linalg.norm(wrench[:3])))
        if model.geom("floor").id in (contact.geom1, contact.geom2):
            foot = contact.geom2 if contact.geom1 == model.geom("floor").id else contact.geom1
            if model.geom(foot).name.endswith("_foot") and wrench[0] > 0.1:
                support.add(foot)
    joint = data.qpos[7:]
    excess = np.maximum(model.jnt_range[1:, 0] - joint, joint - model.jnt_range[1:, 1])
    return Sample(float(data.time), tuple(map(float, data.qpos[3:7])),
                  float(data.xmat[1, 8]), float(data.qpos[2]), len(support), force,
                  max(0.0, float(excess.max())), float(np.linalg.norm(data.qvel[:6])),
                  int(sum(data.warning.number)))


def evaluate(trace):
    """Independent full-path screening, irrespective of candidate parameters."""
    if len(trace) != SETTLE_TICKS + DRIVE_TICKS + RECOVERY_TICKS:
        return {"accepted": False, "reason": "incomplete_trace", "roll_rad": 0.0}
    if any(not np.isfinite([row.time] + list(row.orientation)
                         + [row.up, row.base_height, row.force, row.joint_excess, row.speed]).all()
           for row in trace):
        return {"accepted": False, "reason": "nonfinite", "roll_rad": 0.0}

    def stance(rows):
        return all(row.up >= UPRIGHT and row.base_height >= MIN_BASE_HEIGHT
                   and row.supports >= 3 and row.speed < 0.5 for row in rows)

    roll = 0.0
    reasons = []
    if not stance(trace[SETTLE_TICKS - 5:SETTLE_TICKS]):
        reasons.append("unstable_start")
    if not stance(trace[-5:]):
        reasons.append("unstable_finish")
    if any(row.warnings or row.force > MAX_CONTACT_FORCE or row.joint_excess > JOINT_SLACK
           or row.base_height < 0.01 for row in trace):
        reasons.append("physics_limits")
    for before, after in zip(trace, trace[1:]):
        if not np.isclose(after.time - before.time, DT, rtol=0, atol=1e-7):
            reasons.append("clock_discontinuity")
            break
        previous, current = np.array(before.orientation), np.array(after.orientation)
        if not all(abs(np.linalg.norm(q) - 1) < 1e-6 for q in (previous, current)):
            reasons.append("invalid_quaternion")
            break
        delta = np.zeros(3)
        # MuJoCo returns the local rotation taking previous to current.
        mujoco.mju_subQuat(delta, current, previous)
        if np.linalg.norm(delta) > MAX_STEP_ROTATION:
            reasons.append("rotation_discontinuity")
            break
        roll += float(delta[0])
    if abs(abs(roll) - 2 * pi) > .45:
        reasons.append("incomplete_roll")
    return {"accepted": not reasons, "reason": ",".join(reasons) or "accepted",
            "roll_rad": roll, "peak_force_n": max(row.force for row in trace)}


def attempt(env, seed, amplitudes):
    env.reset(seed=seed, options={"randomize": True})
    neutral = env.q[:]
    target = [angle + offset for angle, offset in zip(neutral, amplitudes, strict=True)]
    try:
        servo_degrees(target)
    except ValueError:
        return {"accepted": False, "reason": "command_envelope", "roll_rad": 0.0}
    trace = []
    for tick in range(SETTLE_TICKS + DRIVE_TICKS + RECOVERY_TICKS):
        active = SETTLE_TICKS <= tick < SETTLE_TICKS + DRIVE_TICKS
        env.queue.append(target if active else neutral)
        env.q = slew(env.q, env.queue.pop(0), env.rate)
        env.data.ctrl[:] = env.q
        mujoco.mj_step(env.model, env.data, nstep=round(DT / env.model.opt.timestep))
        trace.append(sample(env))
    return evaluate(trace)


def run_search(attempts=128):
    if type(attempts) is not int or not 2 <= attempts <= 512:
        raise ValueError("expected 2–512 search attempts")
    env, rng = DogEnv(mode="stand"), Random(20261121)
    results, best = [], [0.0] * 12
    best_value = 0.0
    try:
        for index in range(attempts):
            radius = .35 if index < attempts // 2 else .12
            candidate = ([0.0] * 12 if index == 0 else
                         [offset + rng.uniform(-radius, radius) for offset in best])
            result = attempt(env, 100, candidate)
            result["candidate"] = candidate
            results.append(result)
            # Screening first considers recorded achievements, then continuous roll.
            value = 10 * result["accepted"] + min(2 * pi, abs(result["roll_rad"]))
            if not any(flag in result["reason"] for flag in ("limits", "discontinuity", "unstable_start")):
                if value > best_value:
                    best, best_value = candidate, value
        holdout = [dict(seed=seed, **attempt(env, seed, best)) for seed in range(500, 508)]
    finally:
        env.close()
    return {"version": VERSION, "attempts": attempts + len(holdout), "training_seed": 100,
            "training_outcomes": dict(Counter(result["reason"] for result in results)),
            "max_training_roll_rad": max(abs(result["roll_rad"]) for result in results),
            "best_candidate": best, "holdout": holdout,
            "accepted": any(result["accepted"] for result in results)
                        and all(result["accepted"] for result in holdout)}


if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempts", type=int, default=128)
    print(json.dumps(run_search(parser.parse_args().attempts), indent=2, allow_nan=False))
