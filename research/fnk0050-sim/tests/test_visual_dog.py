from visual_dog import VisualDog
from research.uel.visual.experiment import run
from research.uel.visual.qualification import practice


def test_dog_visual_curriculum(tmp_path):
    dog = VisualDog()
    result = run(dog, tmp_path)
    assert result["passed"] and result["delay"] == 3
    assert result["rms_px"] < result["fixed_rms_px"]
    assert result["ambiguous"] == list(range(5))
    _, skills = practice(dog, tmp_path, result["qualification"])
    assert skills["stand"]["simulation_accepted"]
    assert not skills["unstable-start"]["simulation_accepted"]
    assert all(not v["hardware_accepted"] for v in skills.values())
