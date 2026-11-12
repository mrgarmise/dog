"""Minimal FNK0050 chain from official equations; new CC BY-NC-SA 3.0 model."""
from pathlib import Path
from core import *
def build():
    limbs=[]; actuators=[]
    for i,(name,x,y) in enumerate(zip(LEGS,(.068,-.068,-.068,.068),(.038,.038,-.038,-.038))):
        limbs.append(f'''<body name="{name}_roll_link" pos="{x} {y} 0">
<joint name="{name}_roll" axis="1 0 0" range="-1.256637 1.256637"/>
<geom type="capsule" fromto="0 0 0 0 0 -.023" size=".006" mass=".012"/>
<body name="{name}_hip_link" pos="0 0 -.023"><joint name="{name}_hip" axis="0 -1 0" range="-1.256637 1.256637"/>
<geom type="capsule" fromto="0 0 0 0 0 -.055" size=".005" mass=".018"/>
<body name="{name}_knee_link" pos="0 0 -.055"><joint name="{name}_knee" axis="0 -1 0" range=".314159 2.827433"/>
<geom type="capsule" fromto="0 0 0 0 0 -.055" size=".004" mass=".014"/>
<geom name="{name}_foot" type="sphere" pos="0 0 -.055" size=".008" mass=".003" friction=".7 .005 .0001" rgba=".1 .1 .1 1"/>
<site name="{name}_tip" pos="0 0 -.055" size=".002"/>
</body></body></body>''')
        for j,lo,hi in [('roll',-1.256637,1.256637),('hip',-1.256637,1.256637),('knee',.314159,2.827433)]:
            actuators.append(f'<position name="{name}_{j}_servo" joint="{name}_{j}" kp="2" kv=".04" ctrlrange="{lo} {hi}" forcerange="-.15 .15"/>')
    return '''<mujoco model="FNK0050 uncalibrated research prototype">
<compiler angle="radian"/>
<option timestep=".002" gravity="0 0 -9.81" integrator="implicitfast" iterations="50"/>
<default><joint damping=".01" armature=".00001"/><geom friction=".7 .005 .0001" rgba=".25 .4 .65 1" condim="3"/></default>
<worldbody><light pos="0 0 2"/><geom name="floor" type="plane" size="5 5 .1" rgba=".6 .6 .6 1"/>
<body name="base" pos="0 0 .109"><freejoint/>
<geom name="body" type="box" size=".068 .038 .015" mass=".40" rgba=".2 .2 .2 1"/>
<site name="imu" pos="0 0 0" size=".002"/>
<geom name="compass_led_0" type="sphere" pos="0.024000 0.000000 .020" size=".0045" contype="0" conaffinity="0" mass="0" rgba=".03 0 0 1"/>
<geom name="compass_led_1" type="sphere" pos="0.014964 0.018764 .020" size=".0045" contype="0" conaffinity="0" mass="0" rgba=".03 0 0 1"/>
<geom name="compass_led_2" type="sphere" pos="-0.005341 0.023398 .020" size=".0045" contype="0" conaffinity="0" mass="0" rgba=".03 0 0 1"/>
<geom name="compass_led_3" type="sphere" pos="-0.021623 0.010413 .020" size=".0045" contype="0" conaffinity="0" mass="0" rgba=".03 0 0 1"/>
<geom name="compass_led_4" type="sphere" pos="-0.021623 -0.010413 .020" size=".0045" contype="0" conaffinity="0" mass="0" rgba=".03 0 0 1"/>
<geom name="compass_led_5" type="sphere" pos="-0.005341 -0.023398 .020" size=".0045" contype="0" conaffinity="0" mass="0" rgba=".03 0 0 1"/>
<geom name="compass_led_6" type="sphere" pos="0.014964 -0.018764 .020" size=".0045" contype="0" conaffinity="0" mass="0" rgba=".03 0 0 1"/>
'''+''.join(limbs)+'''</body></worldbody><actuator>'''+''.join(actuators)+'''</actuator><sensor><accelerometer name="accel" site="imu"/><gyro name="gyro" site="imu"/></sensor></mujoco>'''
if __name__=='__main__':
    Path(__file__).parent.joinpath('models/dog.xml').write_text(build()+'\n')
