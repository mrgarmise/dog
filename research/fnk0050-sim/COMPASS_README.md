# Seven-LED Balance Compass: simulation and physical integration

**Status:** MuJoCo geometry, computed RGB output, visual replay, and a guarded Freenove driver adapter are implemented in this research branch. **No physical hardware was connected, no LED has been lit by this code, and the real-time IMU/runtime wiring is not yet commissioned.** Never merge or deploy to real Dog solely on this evidence.

## Optical mapping

Freenove's current vendor `Code/Server/Led.py` explicitly configures **7** RGB LEDs. This branch follows that count. Logical LED 0 is assumed **front**, and 1..6 proceed toward the **left** around the 360-degree ring, spaced by 360/7 degrees. Actual hardware order can be corrected with `led_order` in `BalanceCompass`.

All LEDs are on at low equal brightness when a measurement is centered. Red tilt and green model-estimated center-of-mass (COM) offset alternate every **0.5 simulated or monotonic seconds**. Red indicates the low side of the body. Green indicates horizontal COM offset from the centroid of supporting feet, **not** a rigorously determined tipping margin or a direct MPU6050 measurement. If support cannot be determined, the green readout shows a distinct sparse low-brightness *unknown* pattern; it must not indicate centered.

## MuJoCo on Mint

The original `dog.xml` contains a virtual accelerometer and gyroscope. This branch adds seven massless, non-contact LED geoms on the back and `compass_sim.py`, which calculates COM from simulated mass distribution and grounded foot contacts and updates the markers.

From repo root with your simulation venv active:

```bash
cd research/fnk0050-sim
python view.py
```

Pose Lab: copy `pose_lab_compass_7led.py` from the companion patch file into the same folder as `pose_lab.py`, preserving the original backup, then rename the patch to `pose_lab.py`. The launcher still uses its existing skill catalog, now logging mode, validity, RGB values, simulated accelerometer, and gyro for each frame. Non-rendered trials also log the compass.

Compare old and new gait traces using the same 2ms physics timestep; the decorative LED geoms have mass 0 and no collision.

## Physical adapter

The Freenove LED driver selects WS281x or SPI based on the board revision. Do not manually assume GPIO18 or GPIO10; call the existing vendor `Led()` only on the correct installed dog and after verifying its setup. The new `FreenoveCompassOutput` class writes to `Led.strip.set_led_rgb_data(i, rgb)` for i=0..6 followed by `show()`. It requires `permission=True`, refuses writes under `fault_active=True`, limits update rate to 20Hz, and caps per-channel values at 60. Integrate only under a single LED owner/arbiter, never alongside an existing animation process.

Physical orientation requires MPU6050 orientation fusion and a verified sign mapping. Physical COM estimation needs mass, joint pose, support-state inference, and confidence. Raw MPU6050 readings alone cannot supply physical COM. Provide a distinct warning when IMU data or model inputs are stale. For hardware bring-up, first run a single-LED indexing test at low brightness and correct mapping. Then test red orientation with Dog restrained and motors off. Green remains unknown until a real estimator is validated.

```bash
PYTHONPATH=../.. python -m unittest discover -s ../../tests -p 'test_balance_compass.py' -v
PYTHONPATH=../.. python -m unittest discover -s ../../tests -p 'test_freenove_led_adapter.py' -v
```

No motion commands or firmware changes are part of this feature.
