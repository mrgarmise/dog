# Mapping and measurements

## Coordinates and channels

World/body coordinates: X forward, Y left, Z up. Official foot coordinates are X forward, Y down, Z lateral; convert `(x,down,lateral)` to `(x,lateral,-down)`. Positive simulated roll is about X. Hip and knee axes are local negative Y. For analytic angles `(a,b,c)` in the vendor code, our joint angles are `(pi/2-a,b,c)`. This matches the vendor forward kinematics, including the **23 mm radial first link**, rather than a fixed lateral offset.

Leg names below are inferred from `postureBalance` and `changeCoordinates` ordering. Physically confirm them against Dog before commissioning. Channels themselves are directly established by `run()`.

| Official index | Inferred position | Channels: a / b / c | Servo angles in degrees, including calibration |
|---|---|---|---|
| 0 | Front left (FL) | 4 / 3 / 2 | a+cal_a; 90−(b+cal_b); c+cal_c |
| 1 | Rear left (RL) | 7 / 6 / 5 | a+cal_a; 90−(b+cal_b); c+cal_c |
| 2 | Rear right (RR) | 8 / 9 / 10 | a+cal_a; 90+(b+cal_b); 180−(c+cal_c) |
| 3 | Front right (FR) | 11 / 12 / 13 | a+cal_a; 90+(b+cal_b); 180−(c+cal_c) |

The lower-level vendor Servo.py bounds commands to 18–162 degrees, despite Control.py's 0–180 restriction. This prototype rejects out-of-range output. That envelope is software evidence, **not proof of collision-free travel**. Derive narrower per-channel limits from Dog and feed them into a future hardware adapter.

The MH-FLOCKE servo_config incorrectly attributes 5/6/7 as a/b/c for its second leg, whereas official index 1 uses 7/6/5. It also labels the official ordering differently. Do not copy that file to Dog.

## Inventory

| Item | Current source/value | Status / action |
|---|---|---|
| First, upper, lower link | 23, 55, 55 mm from official function defaults | Source-backed nominal; measure pivot-to-pivot on Dog |
| Hip anchor rectangle | 136 × 76 mm from postureBalance | Source-backed nominal; confirm actual rotation centers |
| Default foot center | Down 99 mm; lateral ±10 mm from hip | Source-backed nominal; not floor clearance |
| Body collision height | 30 mm | Placeholder; measure belly, battery and shield extents |
| Body mass | 0.400 kg | Placeholder, includes omitted head/electronics; weigh assembled Dog |
| Per-leg masses | 0.012 + 0.018 + 0.014 + 0.003 kg | Placeholders; total model 0.588 kg |
| COM / inertia | Primitive geometric distribution | Placeholder; measure COM, estimate link inertia from measured parts |
| Foot radius | 8 mm sphere | Placeholder; measure pad shape and compression |
| Torque cap | 0.15 N m | Placeholder, not a servo specification; identify servo marking and supply voltage |
| Position gain / velocity damping | 2 / 0.04 in MuJoCo | Placeholder; identify loaded step response |
| Slew | 2 rad/s, randomized 1.5–2.5 | Placeholder; identify realistic speed under load |
| Delay | 0–40 ms during randomization | Hypothesis; measure command and sensor latency |
| Contact sliding friction | 0.7, randomized 0.4–1.0 | Hypothesis; measure on intended floor |
| Deadband, backlash, compliance | Not implemented explicitly | Measure, then add to actuator model before transfer training |
| IMU mounting | Aligned body frame in simulation | Unknown on Dog; determine rotation/signs with six stationary orientations |
| Servo calibration | Zero offsets in demonstration | Use Dog's actual point.txt and verified horn neutral positions |
| Pi / batteries / head | Lumped body mass; head not articulated | Match actual Pi Zero installation and battery position |

Measure passive geometry now if convenient. Loaded servo characterization belongs to the later supervised commissioning session. Do not import vendor Control just to obtain its math: its constructor initializes hardware.

## Concrete implementation sequence

1. Record Dog's actual board revision and servo labels. Fill this table with measured values and uncertainty.
2. Edit `core.py` link constants and `generate_mjcf.py` anchor positions together. Add regression targets for measured foot positions. If measurements contradict the serial-chain approximation, correct the topology first; do not compensate with policy training.
3. Replace primitive body mass/COM and per-link mass estimates. Retain simple collision geometry unless physical interference requires more detail.
4. Identify loaded actuator speed, torque envelope, delay and hysteresis. Edit `position` attributes and command filter; make supply sag a randomized torque/speed scale.
5. Fit ground contacts to stationary sag and measured foot sliding. Keep timestep at 2 ms initially and compare against 1 ms.
6. Re-run tests, stable standing, slow squat, single-foot lifting, forward motion and both turn directions. Only then repeat gait optimization.
7. Add a small IMU-feedback balance residual after the open-loop gait is repeatable. Recovery behavior is a later task requiring fall/recovery poses and a different action envelope.

Effort estimate, not measured labor: geometry/weight inventory 1–2 hours; model calibration and servo/IMU identification several supervised sessions; dependable locomotion and transfer may take days of iteration. No promise of walking after one tuning run.
