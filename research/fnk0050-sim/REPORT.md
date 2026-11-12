# Dog FNK0050: research, simulator selection and prototype handoff

**12 November 2026 — simulator-agnostic mission revision**

## Decision and outcome

**BUILD a minimal, source-grounded FNK0050 model in MuJoCo.** No adequate, ready-to-use FNK0050 training model was established in this search. This is a bounded search finding, not proof that none exists. Confidence is high in rejecting the inspected models as drop-in twins; confidence in this prototype's physical dynamics is low until Dog is measured.

A working CPU/headless model, Gymnasium interface, gait search, servo mapping, dry-run exporter and tests are supplied. The model's kinematic chain was checked against the official forward equations and independently against the physics engine's site positions. Standing works in simulation. **Robust forward locomotion and sim-to-real acceptance remain unachieved.** One held-out gait trial moved backward; coarse-timestep trials show substantial contact sensitivity. The failed evidence is retained, and the exported demonstration policy is marked unaccepted.

The most relevant existing candidate is MH-FLOCKE's Freenove MJCF, but inspection found different first-joint axes, first-link placement and channel labeling. Replacing its essential mechanics would be comparable to generating a small model directly. We therefore chose BUILD rather than preserving a misleading “exact robot” label or converting it to Webots.

## Candidate evidence and decision matrix

Scores are engineering screening judgments (0–5), **not measured fidelity scores**: K = match to official kinematic model; L = evidenced reusable license; E = execution evidence in this mission. “0 execution” means untested, not necessarily broken.

| Candidate / exact revision | K/L/E | Relevant properties and issues | Decision |
|---|---:|---|---|
| [Official Freenove source](https://github.com/Freenove/Freenove_Robot_Dog_Kit_for_Raspberry_Pi/tree/73892c05dde13e1c2af24b7e1237f682af82ee66) | 5/5/2 | Nominal 23/55/55 mm leg chain, 136×76 mm anchors, mirrored servo formulas; CC BY-NC-SA 3.0. Math executed in isolation. No complete dynamic simulator model found in its tree. Only head DWG, documentation, code and other kit resources. | Authoritative starting point for BUILD; physical dimensions still need confirmation. |
| [MH-FLOCKE paper checkpoint](https://github.com/MarcHesse/mhflocke/tree/94b258475e02fc99ec0fd146c3de0af339099498) | 2/5/2 | Apache-2.0; Freenove-labelled MJCF loads in MuJoCo 3.3.7. 12 actuators plus passive head. First joint is Z-axis yaw; first link is a lateral 23 mm offset. Official equations instead yield X-axis roll and radial link. Channels also conflict. Position actuators have no explicit force cap in the inspected XML. | Closest named candidate, rejected as a faithful drop-in. Reference for future adaptation, not a trained policy to transfer. |
| [Wavego Webots](https://github.com/green-creeper/wavego_webots/tree/246932897563146da435a3b78d3dc6b4f40ca847) | 2/0/0 | R2025a, different anchors/link geometry, default motor limits, complex STL collision meshes in inspected PROTO, repeated guessed inertia values. README claims primitive collisions, but inspected PROTO still references Mesh bounding objects. No license file/grant found in tree or README. | Reject as derivative base without permission; different robot and dynamics. |
| [RoboDog CAD](https://github.com/stanislavbritanishskii/Dog/tree/aba260d2a94c26b275bf72c824c39634dc95a6f3) | 2/5/0 | MIT; FreeCAD/STL, hobby servo/PCA9685, 12 leg + 2 head DOF. Custom printed robot, not acrylic FNK0050. No ready physics model found. | Legally useful reference; recreating FNK0050 mechanics is cheaper than adapting its cosmetics. |
| [TuanLinh quadruped](https://github.com/TuanLinh05/Quadruped-robot-12DOF/tree/a21ba1bdc530b842c99701daa813e992584326ae) | 1/0/0 | Four yaw position joints, eight torque-controlled pitch/knee joints, ±5 N m controller bounds; ROS/MATLAB bridge. No repository-wide license file found. | Reject: torque interface and scale differ; unnecessary dependencies. |
| [CHAMP descriptions](https://github.com/chvmp/robots/tree/83d59811d262d52c16fe8b6da8f269ae312e1c5f) | 1/1/0 | Pupper, Mini Pupper, SpotMicro and other links/configurations; heterogeneous asset licenses must be checked at origins. Gazebo support does not establish FNK0050 matching. | Generic locomotion reference; not a better FNK0050 base. |

Additional screening: [Coolnesss/dog-env](https://github.com/Coolnesss/dog-env/tree/8be28fdedbf78822e359e2db6e5c7a776de1fd43) is a **real-robot TCP client**, six discrete motion commands, camera/distance observations and an outdated reset signature. It is not a physics model, has no license found, and was not run because it would connect to hardware. Spot is a different commercial quadruped; adopting its model would retain the wrong geometry and actuator assumptions. MuJoCo Menagerie's root inventory (92 entries at retrieval) contained no Freenove entry. Targeted searches of GitHub, GitLab, CAD/Printables/Onshape/GrabCAD and simulator collections did not yield a licensed, calibrated exact alternative.

MH-FLOCKE's later `v0.7-go2-freenove-final` checkpoint is `ba15e9796f0dba6bde6bfd2d329902ff738a306f`; its `freenove.xml` is byte-identical by Git blob SHA to the paper checkpoint. Its `creature.xml` was also inspected and retains the same yaw/radial mismatch. Current main is Bittle-focused. The preprint's performance claims were treated as author reports, not reproduced evidence. Its reported platform is Pi 4, not Dog's Pi Zero W.

Exact paths and revisions are in `docs/source-manifest.json`. No unlicensed candidate assets were incorporated. Attribution and the full Freenove license are included.

## Simulator choice

| Pairing | Execution here | License / OS fit | Headless / training | Required work and recommendation |
|---|---|---|---|---|
| **New FNK0050 + MuJoCo 3.3.7** | Model compiled, physics run, Gymnasium checked, EGL still rendered | MuJoCo Apache-2.0; prototype CC BY-NC-SA 3.0. Linux x86-64 pip package used. | CPU physics without GL/display; direct Python API | **Selected.** Small primitive model and clear servo mapping; physical calibration remains. |
| New FNK0050 URDF + PyBullet | Installation attempted; 3.2.7 source build failed because clang++ was missing; no model run | Bullet zlib; Linux wheels/builds depend on Python version | DIRECT mode can run without display; custom Gym wrapper | **Fallback.** Export equivalent URDF and revalidate contacts. No fidelity advantage established to justify conversion now. |
| New FNK0050 Webots draft / Wavego | R2025a package downloaded/extracted; missing libsndio supplied locally; X display initialization failed. No world executed. | Webots Apache-2.0; some assets separately licensed. R2025a Ubuntu 22.04/24.04 packages; Mint compatibility needs testing | Python Supervisor API; no-rendering still needs Qt/display setup | Keep draft only. Desktop tooling is useful, but conversion adds work without improving mechanics. |
| CHAMP/Pupper + Gazebo | Source/reference review only | Gazebo Apache-2.0, model licenses vary; Harmonic has Ubuntu packages | Server/CPU operation possible; integration wrapper needed | Not chosen. Different robot and greater setup scope; ROS migration is unnecessary. |

Primary simulator sources: [MuJoCo repository](https://github.com/google-deepmind/mujoco), [Python API](https://mujoco.readthedocs.io/en/stable/python.html), [PyBullet guide](https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html), [Gazebo Harmonic install](https://gazebosim.org/docs/harmonic/install_ubuntu/), [Webots R2025a release](https://github.com/cyberbotics/webots/releases/tag/R2025a), [Webots system requirements](https://github.com/cyberbotics/webots/blob/master/docs/guide/system-requirements.md). The tested version is pinned for reproducibility, not asserted to be the latest.

## What the model represents

Twelve independent positional actuators: leg roll, hip pitch, knee pitch. No kinematic closure is required by the official analytic equations. If physical inspection reveals an important linkage transmission not represented by those equations, the model must be revised before transfer. Left/right mirroring is in the command map and initial roll signs; pitch/knee geometry shares a canonical coordinate frame.

Body/leg dimensions come from official source. Masses, body height, COM, collision radii, friction, gains, torque, rate and delay are hypotheses with explicit labels in `docs/measurements.md`. Inertias are generated from primitive mass geometry. Head and electronics are lumped into the body. Foot contacts are spheres on a plane; nonadjacent self-collision is enabled by MuJoCo defaults, while immediate parent-child contacts are filtered. This is not a CAD collision certification.

Physics uses gravity −9.81 m/s², 2 ms integration, position control with ±0.15 N m placeholder cap, and 20 ms agent commands. Tests demonstrate command envelopes and force bounds, not that the cap matches the actual servos. Explicit backlash and acrylic flexibility are absent. Camera/ultrasonic/head actuation are excluded from the initial learning task.

## Evaluation and failure criteria

The final numeric evidence is in `RESULTS.md` and the JSON logs. Pass/fail meanings:

- Kinematics: inverse/forward consistency to 1e−12 m; independently transformed chain and MuJoCo foot-site positions agree. Official forward equations also agree to floating-point precision.
- Standing: finish the 10 s regression without fall, retain at least four contacts, terminal joint tracking error <0.06 rad. Passed.
- Baseline stages: standing, squat, stepping, forward command, turn command, 0.3 N lateral force for 0.1 s, and ten randomized episodes were executed. Small motion and survival are measured; commanded turning is not an accepted steering controller.
- Training: two 32-candidate searches, three training seeds each and ten held-out seeds. First search produced shuffling and failed timestep robustness. Second search explicitly penalized <2 mm peak clearance per foot after settling. All ten held-out second-search trials cleared that threshold and stayed upright, but one moved backward. **Robust forward-walking gate failed.**
- Long validation: nine 100 s episodes across 1/2/4 ms and three fresh seeds. No fall is insufficient: direction and displacement change with timestep; 4 ms is rejected. For the revised gait, 1 and 2 ms produced similar forward distances on these three seeds (about 0.19–0.28 m over 100 s); this limited agreement does not resolve the held-out backward-motion failure.
- Physical transfer: not tested; no hardware access, actuation, flash, configuration change or deployment.

Do not expand training to millions of steps on this uncalibrated model. The next useful work is contact/actuator identification and a gait objective that rewards consistent support and progress, not more optimization of uncertain physics.

## Mint feasibility and next commands

Known laptop: AMD A12-9700P, integrated graphics, 1366×768. Actual RAM/free disk remain unverified; `bash mint_check.sh` records them. This environment is not the laptop and has no direct access to it. The user's separately reported MuJoCo 4.0.1 headless test and viewer success are encouraging; the reported 73,017 steps/s / 146× is a **simple-model benchmark**, not this 12-servo contact model. No Dog-model speed extrapolation is made from it.

The supplied model ran at roughly 71–86× real time in the initial Work-host batch (about 53 MB process peak RSS, excluding installation/cache and later rendering). A 32-candidate, three-seed, 20-second-episode search plus held-out runs took about 28 seconds here. Host CPU identification was unavailable. These numbers are measurements of this environment only.

Use one environment, no camera, no renderer during tuning; start with 1,000 control steps and ten seeds. Measure elapsed time/RSS locally:

```bash
bash mint_check.sh > mint-machine.txt
bash setup.sh
source .venv/bin/activate
/usr/bin/time -v python evaluate.py --steps 1000 --seeds 10
/usr/bin/time -v python train_gait.py --trials 8 --steps 500
```

Avoid changing the existing MuJoCo 4.0.1 environment merely to try this package: `setup.sh` creates an isolated `.venv` in the package directory with the evidence version. Compatibility was additionally checked here with MuJoCo 4.0.1: the same test suite and baseline evaluation ran successfully. The optimizer and long-validation evidence remain from 3.3.7; before retraining under 4.0.1, retain separate held-out logs because contact behavior can change across versions. Viewer use is optional. Start with a small window; nothing relies on NVIDIA/CUDA.

After the local benchmark, estimate a proposed search budget using measured episode wall time × candidates × training seeds, plus held-out episodes. If slow, reduce search candidates/horizon first; do not coarsen the physics timestep to 4 ms. A parameterized gait and optional small IMU residual are the practical low-compute path. Pi Zero W runs only policy inference and command filtering; simulation stays on Mint.

## Remaining work and smallest next action

**Next:** run the supplied tests and 20-second benchmark on Mint, then record Dog's mass, pivot-to-pivot dimensions and servo markings. The full measurement checklist and transfer plan are in `docs/measurements.md` and `docs/sim-to-real.md`. Current physical power/shield restrictions are not used as a reason to omit the pathway; actual trials simply belong to later supervised commissioning.

The mission delivers a feasible implementation and an honest failed robustness gate. It does not deliver a reliable walking policy, a learned balance/recovery controller or a calibrated digital twin. The report, tests, failed experiments and executable scripts make the next iteration concrete.
