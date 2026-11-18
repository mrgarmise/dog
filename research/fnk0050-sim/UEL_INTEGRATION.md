# Dog UEL integration

Base is Dog research commit 2d6af9192a69868454f260e8cb3851c98727e623. The integration adds only research files alongside the existing CPU-only Gymnasium model. Use a sibling Charlie checkout pinned at a4108e297c87ad39b6127ddc398d06955b7dce16 (draft PR #41). Run from this simulator directory:

```sh
export PYTHONPATH=.:../../../charlie
export OPENBLAS_NUM_THREADS=1
python -m pytest -q tests
python uel_demo.py "$(mktemp)" --revision "$(git rev-parse HEAD)"
```

The development host used Python 3.12, Gymnasium 1.4.0, NumPy 2.3.5 and MuJoCo 4.0.1. Use matching platform wheels with pip --no-index for air-gapped use. The original model's physics interval is 2 ms and its Gymnasium control interval is 20 ms. The trial uses only the stand reset and normalized actions inside ±0.8. Joint measurements come from physical state in the simulator rather than command echoes. Candidate stabilization needs independent robustness tests before a simulation skill is exported. No gait, snapshots, vendor code or hardware controls were changed.
