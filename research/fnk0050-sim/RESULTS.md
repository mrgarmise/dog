# Executed results — uncalibrated model

MuJoCo 3.3.7, Python 3.12.14; supplemental compatibility check on MuJoCo 4.0.1. No hardware trials.

**12 automated tests passed on each engine version.** Gymnasium check_env passed; infinite observation-bound warnings were expected. EGL rendering produced `docs/prototype.png`.

| Baseline stage | Simulated seconds | Fall | Forward displacement | Peak tilt |
|---|---:|---|---:|---:|
| stand | 20.0 | False | -0.00385 m | 0.0082 rad |
| squat | 20.0 | False | -0.00205 m | 0.0061 rad |
| step | 20.0 | False | -0.00948 m | 0.0214 rad |
| walk | 20.0 | False | 0.00817 m | 0.0214 rad |
| turn | 20.0 | False | 0.00856 m | 0.0213 rad |
| stand + perturbation | 20.0 | False | -0.00385 m | 0.0082 rad |

Ten randomized baseline walks survived; forward displacement was tiny and one was slightly negative. These are diagnostics, not gait acceptance.

## Search with foot-clearance criterion

32 candidates × 3 training seeds, 20 s per episode; elapsed 28.38 s on this Work host. Ten held-out seeds: 0 falls, mean forward displacement 0.0107 m, minimum -0.2087 m. Minimum per-foot peak clearance across held-out trials: 3.21 mm.

**Forward robustness FAILED:** 9/10 advanced; 1/10 moved backward. Export is a demonstration only, with simulation_accepted=false and hardware_accepted=false. The initial search without clearance criterion is retained separately as rejected shuffling.

## Longer and timestep checks

| Timestep | Seed | Duration | Forward distance | Fall |
|---|---:|---:|---:|---|
| 1 ms | 200 | 100 s | 0.2236 m | False |
| 1 ms | 201 | 100 s | 0.1905 m | False |
| 1 ms | 202 | 100 s | 0.2839 m | False |
| 2 ms | 200 | 100 s | 0.2335 m | False |
| 2 ms | 201 | 100 s | 0.1890 m | False |
| 2 ms | 202 | 100 s | 0.2809 m | False |
| 4 ms | 200 | 100 s | -6.1827 m | False |
| 4 ms | 201 | 100 s | -6.7432 m | False |
| 4 ms | 202 | 100 s | -6.8294 m | False |

**4 ms rejected.** The 1/2 ms results are reasonably close on these three seeds; contact integration at 4 ms changes direction dramatically. It must not be used to accelerate learning.

## Resource evidence

Initial baseline batch: 70.7–86.2× real time; peak process RSS 52924 KiB. This is NOT a Mint or Pi benchmark. Installation, graphics and the Python environment consume additional disk/RAM.

MuJoCo 4.0.1 compatibility batch: 16 stage/seed episodes, 0 falls. These check execution and baseline stability, not equivalence of trained policies.

## Execution limits

Webots: package extracted and dependency supplied, but display connection failed; world never executed. PyBullet: installation failed in source build (missing clang++); never simulated. Gazebo: reviewed only. MH-FLOCKE XML: parsed/compiled only, no author training results reproduced. Mint: separately reported viewer/simple benchmark success; this exact model has not been timed on Mint.
