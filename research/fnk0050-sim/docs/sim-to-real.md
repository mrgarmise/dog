# Practical sim-to-real pathway

## What transfers

Start with `gait_policy.json`: stride in metres, foot lift in metres, cadence in Hz and foot-center height in metres, plus schema and leg order. `core.gait(t, **params)` produces 12 joint positions. Apply `slew`, then `servo_degrees` with **Dog-specific** calibration to obtain channel→degree commands. This is standard-library Python suitable for benchmarking on the Pi Zero W; no NumPy, MuJoCo, torch or neural runtime is needed on the robot.

`dry_run.py` already exercises that path and emits JSON without touching hardware. The actual I²C adapter is deliberately not implemented: commissioning should wrap the already-qualified vendor driver and verify its pulse mapping, calibration and board revision. The existing vendor example uses PCA9685 address 0x40, 50 Hz and counts 102–512 across nominal 0–180 degrees; do not assume this is an independently measured pulse calibration.

A future balance policy can be a small 20→16→12 tanh network or linear residual. Export weights and normalization as JSON/float arrays; implement and benchmark straightforward inference on the Pi before choosing ONNX or another runtime. Use the same 12 residuals (±0.03 rad) as this Gymnasium environment. Small size is plausible, but Pi Zero timing and RAM remain unmeasured. If deadlines fail, lower the feedback rate while retaining a timed local command loop, or use a precomputed phase table with interpolation. Do not depend on Wi-Fi for the servo deadline.

## Observation/action/reward contract

- Actions: 12 finite float values in [−1,1], order FL, RL, RR, FR; roll, hip, knee within each leg. Scale by 0.03 radians and add to gait target, validate the servo envelope, then apply delay and slew.
- Observations: 20 floats—body-aligned accelerometer specific force / 9.81 (3), gyro rad/s (3), previous **commanded** joint positions in radians (12), sine/cosine of gait phase (2).
- No measured joint positions, absolute world position, velocity or foot-contact sensors appear in the policy observation. Stock hobby servos do not supply encoder feedback. Simulation joint encoders are used only for validation.
- Reward at 50 Hz: dt × [1 + 5 × clipped forward velocity (±0.15 m/s) − 2(roll²+pitch²)] − 0.1 × squared command change; falling subtracts 5. This baseline reward is a starter, not a complete locomotion objective. Gait search uses distance minus lateral/heading penalties and rejects insufficient foot clearance.
- Termination: body height below 55 mm, roll/pitch beyond 0.7 rad, or nonfinite observation. Truncation: episode horizon. `reset(seed=...)` returns observation and info; `step` returns the Gymnasium five-tuple.
- Simulation-only position/orientation/contact data can score training. They must not be fed into a physical policy unless equivalent sensing is actually installed.

The current gait search does not use IMU feedback; it tunes four open-loop parameters. Standing stability currently comes from geometry and position servos, not learned balance.

## Calibration and validation gates

1. **Simulation model:** measured link/anchor dimensions, correct joint signs, calibrated mass distribution and foot contact; predicted foot positions within a chosen measurement tolerance (suggest 2 mm, refine to instrument precision).
2. **Actuator identification:** after physical power qualification, supported unloaded single-joint sweeps within approved travel. Log command, external measured angle and time. Fit lag/slew/deadband and torque limits; never infer torque from the model's placeholder cap.
3. **IMU:** bias, units and mounting transform. Validate gravity axes and gyro signs; use elapsed monotonic time. Do not copy the vendor estimator's fixed halfT assumptions without checking actual sampling timing.
4. **Matched replay:** record a supervised supported sequence; compare predicted/measured joint trajectories and IMU. Expand domain randomization around observed uncertainty, not arbitrarily wide ranges.
5. **Standing then gait:** short supported standing, squat, foot-lift tests; then supervised floor trials with immediate stop. Compare displacement and foot slip; reject policies that depend on simulator sliding, interpenetration or timing artifacts.
6. **Transfer acceptance:** multiple surfaces/battery states and repeated starts; validate stop latency and recovery to a known supported posture. Maintain the previous working gait/config for rollback. Uncontrolled recovery/fall exploration is not part of this mission.

## Controller behavior to implement at commissioning

Use a monotonic 20 ms scheduler initially, sequence numbers, per-joint calibrated hard limits and slew rates. A missed deadline or stale IMU must stop generating new gait targets; the physical stop path should be a prequalified hold/relax/disable action appropriate to support conditions. Abruptly commanding all channels to 90° is not an emergency-stop design. Include an operator stop and independent watchdog; document whether disable means PWM off or power off for the actual board. No deployment, motor command or physical trial was performed here.

## Remaining reality gap

Randomized mass, friction, rate and 0–40 ms delay are implemented. Explicit backlash, deadband, flexible acrylic, servo heating, voltage sag, unequal leg parameters, IMU noise/bias and time jitter remain to be fitted and implemented. Physical foot contact and servo angles are not observed. A model matching nominal kinematics alone cannot establish transfer success.
