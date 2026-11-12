"""Virtual IMU / model-estimated COM -> seven visual LED geom colors.

The IMU sensor already exists in dog.xml. Use true simulated body orientation
as a temporary oracle; this is NOT an MPU6050 sensor-fusion validation.
COM estimate uses MuJoCo's known per-body mass/position (not observable
directly on physical Dog). Contact-support estimate requires >=3 grounded feet.
"""
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from dog_runtime.balance_compass import BalanceCompass

_LEGS = ("FL", "RL", "RR", "FR")


def attitude(model, data):
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    rotation = data.xmat[base].reshape((3, 3))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1, 1)))
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return roll, pitch, yaw


def com_support_vector(model, data, *, scale_m=0.060):
    """COM offset from centroid of currently contacting feet, body-horizontal XY.

    This is an exploratory direction cue, not support-polygon stability margin.
    """
    masses = model.body_mass
    positive = masses > 0
    if not np.any(positive):
        return (0.0, 0.0), False
    com_world = np.average(data.xipos[positive], axis=0, weights=masses[positive])
    foot_ids = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{leg}_foot")
        for leg in _LEGS
    }
    foot_contacts = set()
    for i in range(data.ncon):
        contact = data.contact[i]
        if contact.dist <= 0:
            foot_contacts.add(contact.geom1)
            foot_contacts.add(contact.geom2)
    active = foot_ids.intersection(foot_contacts)
    if len(active) < 3:
        return (0.0, 0.0), False
    centers = np.array([data.geom_xpos[geom_id, :2] for geom_id in active])
    support_center = centers.mean(axis=0)
    delta = com_world[:2] - support_center
    _, _, yaw = attitude(model, data)
    c, s = math.cos(yaw), math.sin(yaw)
    body_x = c * delta[0] + s * delta[1]
    body_y = -s * delta[0] + c * delta[1]
    return (float(np.clip(body_x / scale_m, -1, 1)),
            float(np.clip(body_y / scale_m, -1, 1))), True


class SimBalanceCompass:
    def __init__(self, model, *, led_order=None):
        self.model = model
        self.compass = BalanceCompass(led_order=led_order)
        self.geom_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"compass_led_{i}")
            for i in range(7)
        ]
        if any(index < 0 for index in self.geom_ids):
            raise RuntimeError("MuJoCo model missing seven compass LED geoms")

    def update(self, data):
        roll, pitch, yaw = attitude(self.model, data)
        com_vector, valid = com_support_vector(self.model, data)
        frame = self.compass.frame(
            data.time, roll=roll, pitch=pitch,
            com_vector=com_vector, com_valid=valid,
        )
        for geom_id, rgb in zip(self.geom_ids, frame.rgb):
            self.model.geom_rgba[geom_id] = [*(v / 255 for v in rgb), 1.0]
        return {
            "time_s": float(data.time),
            "mode": frame.mode,
            "com_valid": frame.qualified if frame.mode == "green-com" else valid,
            "com_vector": list(com_vector),
            "rpy": [roll, pitch, yaw],
            "imu_accel_m_s2": data.sensor("accel").data.tolist(),
            "imu_gyro_rad_s": data.sensor("gyro").data.tolist(),
            "rgb": [list(v) for v in frame.rgb],
        }
