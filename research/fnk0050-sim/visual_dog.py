"""Read simulated front-left knee position and IMU from the existing Gym environment."""

import hashlib
from pathlib import Path

import numpy as np
from mujoco_env import DogEnv, DT
from research.uel.visual.contracts import BodyDescription, Trajectory, validate_trial
from research.uel.visual.rig import waveform


class VisualDog:
    def __init__(self):
        self.env = DogEnv()
        model = self.env.model
        self.body = BodyDescription(
            "fnk0050",
            tuple(model.actuator(i).name for i in range(model.nu)),
            "FL_knee_link",
            hashlib.sha256(
                (Path(__file__).parent / "models/dog.xml").read_bytes()
            ).hexdigest(),
        )

    def predict(self, family, seed, delay):
        validate_trial(family, seed, delay)
        env = self.env
        env.reset(seed=seed)
        env.queue = [env.q[:] for _ in range(delay)]
        xyz, commands, imu, limbs = [], [], [], []
        for amplitude in waveform(family, seed):
            action = np.zeros(12)
            action[1] = amplitude
            _, _, done, truncated, _ = env.step(action)
            if done or truncated:
                raise ValueError("simulator terminated during probe")
            limbs.append(
                [
                    [
                        env.data.body(leg + "_" + joint + "_link").xpos.copy()
                        for joint in ("roll", "hip", "knee")
                    ]
                    + [env.data.site(leg + "_tip").xpos.copy()]
                    for leg in ("FL", "RL", "RR", "FR")
                ]
            )
            commands.append(action)
            xyz.append(env.data.body(self.body.marker).xpos.copy())
            imu.append(env.data.sensordata.copy())
        return Trajectory(
            np.array(xyz),
            np.array(commands),
            DT * (1 + np.arange(len(xyz))),
            np.array(imu),
            np.array(limbs),
        )

    def practice(self, delay, mode, seed):
        """Roll out conservative perturbations around the fitted delay. No motor code."""
        import json
        import mujoco

        if mode not in (
            "stand",
            "squat",
            "step",
            "walk",
            "old-gait",
            "unstable-start",
            "leg-excursion",
            "tripod",
            "recovery",
        ):
            raise ValueError("unknown curriculum entry")
        validate_trial("pulse", seed, delay)
        env = DogEnv(
            mode=(
                "stand"
                if mode in ("unstable-start", "leg-excursion", "tripod", "recovery")
                else mode
            )
        )
        params = {}
        if mode == "old-gait":
            params = json.loads(
                (Path(__file__).parent / "gait_policy.json").read_text()
            )["params"]
        env.reset(seed=seed, options={"randomize": True, "params": params})
        env.queue = [env.q[:] for _ in range(delay)]
        rng = np.random.default_rng(seed)
        angle = (
            0.85
            if mode == "unstable-start"
            else (0.32 if mode == "recovery" else rng.uniform(-0.04, 0.04))
        )
        env.data.qpos[3:7] = [np.cos(angle / 2), np.sin(angle / 2), 0, 0]
        mujoco.mj_forward(env.model, env.data)
        margin = 1.0
        energy = 0.0
        clearance = []
        action = np.zeros(12)
        tripod_ticks = 0
        for i in range(100):
            if mode in ("leg-excursion", "tripod"):
                action[2] = 0.8 * (np.sin(i * 0.1) if mode == "leg-excursion" else 1.0)
            before = env.q[:]
            _, _, done, _, info = env.step(action)
            foot_heights = [
                env.data.site(leg + "_tip").xpos[2] - 0.008
                for leg in ("FL", "RL", "RR", "FR")
            ]
            tripod_ticks += int(
                foot_heights[0] > 0.002 and max(foot_heights[1:]) < 0.001
            )
            energy += float(np.sum((np.array(before) - env.q) ** 2))
            margin = min(margin, 0.7 - max(abs(np.asarray(info["rpy"][:2]))))
            clearance.append(float(env.data.site("FL_tip").xpos[2] - 0.008))
            if done:
                break
        return dict(
            seed=seed,
            delay=delay,
            start_roll_rad=float(angle),
            steps=i + 1,
            tilt_margin_rad=float(margin),
            distance_m=info["distance_x"],
            peak_FL_clearance_m=max(clearance),
            energy_proxy_rad2=energy,
            tripod_ticks=tripod_ticks,
            accepted=bool(
                not done
                and margin > 0.1
                and (mode != "tripod" or tripod_ticks >= 10)
                and (mode != "recovery" or abs(info["rpy"][0]) < 0.05)
                and (mode not in ("walk", "old-gait") or info["distance_x"] > 0)
            ),
        )
