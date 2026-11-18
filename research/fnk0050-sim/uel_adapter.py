"""Connect UEL to the existing CPU-only Gymnasium simulator."""

from random import Random
import time

import numpy as np

from mujoco_env import DT, DogEnv
from research.uel.core import Channel, Observation, Profile


class DogTrial:
    def __init__(self, *, labels_seed=17, randomized=False, delay_ticks=None):
        self.env = DogEnv(mode="stand")
        self.randomized = randomized
        self.delay_ticks = delay_ticks
        self.permutation = list(range(12))
        Random(labels_seed).shuffle(self.permutation)
        self.profile = Profile(
            "fnk0050-mujoco", f"1:{labels_seed}:{randomized}:{delay_ticks}",
            tuple(Channel(f"channel_{i}", "gym normalized", -.8, .8) for i in range(12)),
            tuple(Channel(f"joint_angle_{i}", "rad", -4, 4) for i in range(12))
            + (Channel("pitch", "rad", -1, 1), Channel("contacts", "count", 0, 100)),
            (0.,) * 12, max_duration=.20,
        )
        self.stopped = True

    def reset(self, seed):
        self.env.reset(seed=seed, options={"randomize": self.randomized})
        if self.delay_ticks is not None:
            self.env.delay = self.delay_ticks
            self.env.queue = [self.env.q[:] for _ in range(self.delay_ticks)]
        self.stopped = False
        return self.read(0.)

    def read(self, pitch):
        return Observation((*map(float, self.env.data.qpos[7:]), float(pitch),
                            int(self.env.data.ncon)), time.monotonic())

    def step(self, action, duration):
        self.profile.command(action, duration)
        count = round(duration / DT)
        if self.stopped or count <= 0 or abs(count * DT - duration) > 1e-8:
            raise ValueError("simulator stopped or duration off control grid")
        commands = np.zeros(12)
        commands[self.permutation] = action
        for _ in range(count):
            _, _, terminated, truncated, info = self.env.step(commands)
            if terminated or truncated:
                self.stop("simulator terminated")
                raise RuntimeError("simulator terminated")
        return self.read(info["rpy"][1])

    def stop(self, reason):
        self.stopped = True
