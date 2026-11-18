import numpy as np

from uel_adapter import DogTrial
from uel_demo import demo


def test_reset_reproduces_after_parameter_change():
    twin = DogTrial(randomized=True)
    before = twin.reset(42)
    first = twin.step((.3,) * 12, .16)
    twin.reset(99)
    assert twin.reset(42).values == before.values
    assert np.allclose(twin.step((.3,) * 12, .16).values, first.values)


def test_mapping_and_adaptation(tmp_path):
    data = demo(tmp_path / "dog.sqlite", "test")
    assert data["matched_positive_joints"] == 12
    assert data["nominal_joint_mae"] < data["zero_joint_mae"]
    assert data["changed_joint_mae_adapted"] < data["changed_joint_mae_before"]
    assert data["physics_timestep_s"] == .002
