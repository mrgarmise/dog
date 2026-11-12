# Dog: FNK0050 simulation research prototype

**Decision: BUILD a minimal FNK0050 model in MuJoCo.** This is an executable, uncalibrated research prototype—not a physically accepted digital twin or a deployment package. See `REPORT.md` for evidence, candidate decisions, limitations and the full sim-to-real pathway. The mission's simulator-agnostic revision of 12 November 2026 is followed.

## Start on Mint

Unzip this package in a new folder, open a terminal there, then:

```bash
bash mint_check.sh > mint-machine.txt
bash setup.sh
source .venv/bin/activate
python evaluate.py --steps 1000 --seeds 10
python dry_run.py --seconds 2 > dry-run.jsonl
```

The setup uses a separate virtual environment and installs pinned dependencies; it does not contact Dog. If venv is missing, install Mint's `python3-venv` package first. Python 3.11–3.13 is the intended range; Python 3.12 was tested. Stop and inspect errors rather than falling back to an unpinned source build.

Expected: the unit/physics tests pass, evaluation writes `logs/evaluation.json`, and dry-run output contains `hardware_write: false`. These are prototype checks, not physical acceptance.

## Learn and inspect

```bash
python train_gait.py --trials 32 --steps 1000
python validate_policy.py
python view.py
```

The last command opens an optional desktop viewer. Close its window to stop it. Training and evaluation require no display, GPU, ROS or Webots. The search is a small optimizer for gait parameters; it is not a trained neural balance controller. The generated `gait_policy.json` is portable plain JSON. `core.py` and `dry_run.py` use only Python's standard library.

`validate_policy.py` deliberately probes 1, 2 and 4 ms timesteps. Coarse-timestep failures are retained in the evidence; **4 ms is not approved for training**. Read results, do not interpret a zero process exit code as locomotion acceptance.

## Files

- `models/dog.xml`, `generate_mjcf.py`: primary model and editable generator.
- `core.py`: kinematics, bounded gait, slew limiter, official servo mapping and export.
- `mujoco_env.py`: working Gymnasium API with position actuators.
- `evaluate.py`, `train_gait.py`, `validate_policy.py`: baseline, optimization and held-out evaluation.
- `tests/`, `logs/`: automated checks and recorded evidence.
- `docs/measurements.md`: calibration inventory and channel table.
- `docs/sim-to-real.md`: transfer design and acceptance sequence.
- `worlds/dog.wbt`, `generate.py`, `env.py`, `controllers/`: earlier Webots draft, **not executed or validated**. Retained only as an optional port reference; use MuJoCo for the tested workflow.
- `reference/`: attributed Freenove source used for comparison, not imported by simulation.

No hardware driver is included in the execution path. Do not run the vendor reference files on a robot as part of this mission.

License: CC BY-NC-SA 3.0; see `LICENSE.txt` and `NOTICE.md`.
