"""Benchmark joint mapping, pitch coupling and seeded domain adaptation."""

import argparse
from pathlib import Path

import mujoco
import numpy as np

from uel_adapter import DogTrial
from research.uel.core import Journal, Session, encode
from research.uel.learner import Model


def capture(twin, path, name, actions, seeds, revision):
    journal = Journal(path, name, twin.profile, revision)
    session = Session(twin, journal, trials=len(actions), seconds=10)
    values = []
    for action, seed in zip(actions, seeds, strict=True):
        before, after = session.run(tuple(map(float, action)), .16, int(seed))
        values.append(after.values)
    session.cancel("complete")
    journal.db.close()
    return np.asarray(values)


def demo(path, revision):
    nominal = DogTrial()
    changed = DogTrial(randomized=True, delay_ticks=2)
    basis = np.eye(12) * .75
    train = np.concatenate((basis, -basis))
    rng = np.random.default_rng(451)
    holdout = rng.uniform(-.6, .6, (16, 12))
    base = capture(nominal, path, "neutral", [np.zeros(12)], [1], revision)[0]
    varied_base = capture(changed, path, "changed-neutral", [np.zeros(12)], [42], revision)[0]
    train_y = capture(nominal, path, "isolated", train, [1]*24, revision) - base
    random_actions = rng.uniform(-.75, .75, (24, 12))
    random_y = capture(nominal, path, "random", random_actions, [1]*24, revision) - base
    nominal_y = capture(nominal, path, "heldout", holdout, [1]*16, revision) - base
    shifted_y = capture(changed, path, "changed-holdout", holdout, [42]*16, revision) - varied_base
    adapt_y = capture(changed, path, "adaptation", train, [42]*24, revision) - varied_base
    model = Model.fit(train, train_y)
    random_model = Model.fit(random_actions, random_y)
    adapted = Model.fit(train, adapt_y)
    predictions, _ = model.predict(holdout)
    random_predictions, _ = random_model.predict(holdout)
    adapted_predictions, _ = adapted.predict(holdout)
    diagonal = model.weights[np.arange(12), nominal.permutation]
    pitch_coupling = model.weights[:, 12]
    strongest = int(np.argmax(np.abs(pitch_coupling)))
    correction = np.zeros(12)
    correction[strongest] = np.clip(-base[12]/pitch_coupling[strongest], -.75, .75)
    counter = capture(nominal, path, "pitch-counteraction", [correction], [1], revision)[0]
    latency_steps = []
    for duration in (.02, .04, .06, .08):
        changed.reset(42)
        excited = changed.step(tuple(map(float, train[0])), duration)
        changed.reset(42)
        neutral = changed.step(changed.profile.neutral, duration)
        if np.max(np.abs(np.subtract(excited.values[:12], neutral.values[:12]))) > 1e-5:
            latency_steps.append(round(duration/.02))
    changed.stop("complete")
    result = {
        "mujoco": mujoco.__version__, "physics_timestep_s": nominal.env.model.opt.timestep,
        "matched_positive_joints": int(np.sum(diagonal > 0)),
        "nominal_joint_mae": float(np.mean(np.abs(predictions[:, :12]-nominal_y[:, :12]))),
        "random_joint_mae": float(np.mean(np.abs(random_predictions[:, :12]-nominal_y[:, :12]))),
        "zero_joint_mae": float(np.mean(np.abs(nominal_y[:, :12]))),
        "changed_joint_mae_before": float(np.mean(np.abs(predictions[:, :12]-shifted_y[:, :12]))),
        "changed_joint_mae_adapted": float(np.mean(np.abs(adapted_predictions[:, :12]-shifted_y[:, :12]))),
        "changed_simulator_delay_control_ticks": changed.env.delay,
        "first_detected_response_control_tick": min(latency_steps, default=None),
        "strongest_pitch_channel": nominal.profile.actuators[strongest].name,
        "neutral_pitch_rad": float(base[12]), "counteraction_pitch_rad": float(counter[12]),
        "neutral_contacts": int(base[13]), "counteraction_contacts": int(counter[13]),
    }
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("database", type=Path)
    parser.add_argument("--revision", required=True)
    args = parser.parse_args()
    print(encode(demo(args.database, args.revision)))
