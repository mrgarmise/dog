# Attribution and license

Freenove Robot Dog Kit for Raspberry Pi, Freenove Creative Technology Co., Ltd.
https://github.com/Freenove/Freenove_Robot_Dog_Kit_for_Raspberry_Pi
Revision 73892c05dde13e1c2af24b7e1237f682af82ee66.

The official `Control.py` and `Servo.py` are included unchanged as `reference/Freenove_Control.py` and `reference/Freenove_Servo.py`. `reference/official_kinematics.py` extracts and reformats only the two mathematical methods for isolated comparison. Original material is CC BY-NC-SA 3.0. Full license included in `LICENSE.txt`.

The new model generators, simulation interfaces, gait, mapping, tests and reports were created for Alex's Dog research mission on 2026-11-12. They are also provided under CC BY-NC-SA 3.0. They implement simplified geometry based on the official equations; they are not official Freenove models. Changes include SI units, continuous inverse kinematics without the vendor's rounding, analytic roll/pitch/knee representation, primitive contacts, bounded position actuators and an offline learning environment.

No Wavego, TuanLinh, dog-env, CHAMP, RoboDog or MH-FLOCKE code/assets are redistributed in this package. They were evaluated as research candidates. Their licenses do not transfer to this package. MuJoCo and Gymnasium are installed separately under their own licenses. No endorsement by Freenove or other authors is implied.
