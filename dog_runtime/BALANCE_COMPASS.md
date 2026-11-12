# Balance Compass — shared runtime core (not deployed)

Dog's seven RGB LEDs will alternate **RED tilt** and **GREEN estimated center-of-gravity/support displacement**, changing modes every 0.5 seconds without turning the ring off. Balanced/centered = seven equally dim lights. A directional difference brightens the LEDs in that direction and dims those opposite. The default output cap is intentionally low.

Implemented in `dog_runtime/balance_compass.py` as pure Python with no hardware imports; both Mint Pose Lab and the Pi Zero can reuse it. Logical LED order is **front, then six other equally spaced directions counterclockwise viewed from above**; physical indices require calibration. A physical COM estimate cannot come from the MPU6050 alone. An unknown estimate produces a different sparse green pattern; it must never look like a valid balanced reading.

Status: the renderer and alternation are implemented and unit-testable. **Not yet wired to Freenove's `Led.py`, the runtime's IMU sensor fusion, or the physical Dog. No merge, motor actuation, deployment, or LED actuation occurred.**

Next integration gates:
1. Confirm actual LED ring GPIO driver compatibility under Bookworm and establish physical LED index/orientation (front, clockwise/counterclockwise).
2. Read an orientation estimate with timestamp and validated signs from MPU6050 fusion, rather than raw accelerometer tilt while moving; show a distinct invalid/stale indication on failure.
3. Implement a model-based COM and support-region estimator with joint geometry, mass distribution, and available contact inference. Publish confidence/validity rather than claiming direct measurement.
4. Connect adapter to `Led.py` with a capped refresh rate (e.g. 20 Hz), current/brightness ceiling, and fault/status priority over regular balance indication.
5. Integrate with Pose Lab, saving time-stamped pose, IMU, COM validity, and seven RGB outputs for sim-to-real comparisons.
6. Validate hardware on the LED ring only before supervised power and motion commissioning.

Run: `python -m unittest discover -s tests -p 'test_balance_compass.py' -v`
