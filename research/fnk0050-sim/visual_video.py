"""Export annotated simulated full-body frames and command timeline, fully headless."""

import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from visual_dog import VisualDog
from research.uel.visual.camera import fixture
from research.uel.visual.perception import render


def generate(folder, seed=47):
    folder.mkdir(exist_ok=False, parents=True)
    dog = VisualDog()
    trajectory = dog.predict("sine", seed, 3)
    cam = fixture("fnk0050")
    cam.K[:2, :2] *= 0.34
    cam.t[0] = 0
    rng = np.random.default_rng(seed)
    rows = []
    for i, t in enumerate(trajectory.time):
        limbs = cam.project(trajectory.limbs[i]).reshape(4, 4, 2)
        uv = cam.project(trajectory.xyz[i])[0]
        jitter = rng.normal(0, 0.4, 2)
        visible = i % 17 != 7
        frame = render(uv + jitter, rng, visible, limbs + jitter)
        for j, axis in enumerate(np.eye(3) * 0.02):
            origin, end = cam.project([np.zeros(3), axis]) + jitter
            cv2.arrowedLine(
                frame,
                tuple(origin.astype(int)),
                tuple(end.astype(int)),
                (120, 180, 180),
                1,
            )
            cv2.putText(
                frame, "XYZ"[j], tuple(end.astype(int)), 0, 0.35, (200, 200, 200)
            )
        name = f"frame-{i:03d}.png"
        if not cv2.imwrite(str(folder / name), frame):
            raise IOError(name)
        import hashlib

        rows.append(
            dict(
                path=name,
                sha256=hashlib.sha256((folder / name).read_bytes()).hexdigest(),
                time_s=float(t),
                command=trajectory.commands[i].tolist(),
                truth_m=trajectory.xyz[i].tolist(),
                camera_jitter_px=jitter.tolist(),
                visible=visible,
            )
        )
    profile = {
        k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in vars(cam).items()
    }
    manifest = dict(
        version=1,
        sources=[
            dict(
                source=dict(
                    camera_id="primary",
                    backend="synthetic",
                    calibration_id="wide-dog-fixture",
                ),
                calibration=profile,
                frames=rows,
            )
        ],
    )
    (folder / "recording.json").write_text(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    generate(p.parse_args().folder)
