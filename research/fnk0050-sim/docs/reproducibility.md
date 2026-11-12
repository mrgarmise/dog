# Reproducibility and provenance

Source revisions are pinned in `source-manifest.json`; vendor baseline was cloned and its HEAD confirmed as `73892c05dde13e1c2af24b7e1237f682af82ee66`. This is also the baseline recorded in mrgarmise/dog/vendor/freenove/UPSTREAM_COMMIT.txt. Existing Dog hardware and simulation files were left untouched; this work is a separate research subdirectory.

Main commands executed in Work (from project root unless shown):

```bash
python -m pip install gymnasium==1.2.2 pytest
python -m pip install mujoco==3.3.7
python generate_mjcf.py
python -m unittest discover -s tests -v
python evaluate.py --steps 1000 --seeds 10
python train_gait.py --trials 32 --steps 1000
python validate_policy.py
python dry_run.py --seconds .1
python -m compileall -q core.py mujoco_env.py generate_mjcf.py evaluate.py train_gait.py validate_policy.py dry_run.py
```

Search was run twice: first with a 6–10 mm lift range and without a clearance penalty; then 9–13 mm lift, 0.3–0.7 Hz, 97–103 mm height and a per-foot 2 mm minimum peak clearance penalty after the first 2 s. The initial results/policy remain in `logs/*initial*`; the current source implements the revised search. Random generator seed is 20261112; training seeds 0–2; held-out seeds 100–109; long-validation seeds 200–202. No continued tuning on held-out seeds was performed after the second search.

Supplemental engine compatibility used a separate Python venv with mujoco==4.0.1, gymnasium==1.2.2 and numpy==2.3.5. The full test suite and evaluate.py were rerun there; logs have `mujoco4` in their names. All other numerical logs are 3.3.7 unless explicitly marked.

MuJoCo EGL Renderer generated a 640×480 still; that is actual model rendering, not generated artwork. The optional interactive viewer is supplied but was not opened interactively in Work. The user independently reported opening MuJoCo 4.0.1's viewer on Mint.

Attempts not counted as successful simulation:

- Webots R2025a amd64 DEB downloaded from its official release, extracted to scratch; libsndio7.0 from Ubuntu supplied locally. Xvfb and Qt/X connection attempts did not establish a usable display. No world-load result exists.
- PyBullet 3.2.7 installation attempted together with MuJoCo; its build failed because clang++ was absent. MuJoCo was then installed separately. No PyBullet validation is claimed.
- Gazebo not installed because no better FNK0050 model was identified and its larger integration stack offered no demonstrated benefit.

Pass criteria and failed gates are in RESULTS.md and REPORT.md. Process exit status from evaluation/search means the experiment completed, not that walking was accepted. Benchmark scripts must be assessed by displacement, direction, clearance, falls and timestep sensitivity.

Expected next benchmark on Mint: run tests, then evaluate.py with 1000 steps and 10 seeds under `/usr/bin/time -v`. Keep local RAM/disk/GPU information with results. Do not infer laptop speed from the Work-host measurements. No SSH connection to Dog or Mint was used.
