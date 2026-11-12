"""Generate a primitive Webots model matching the documented analytic chain.
All masses, servo dynamics, collision thicknesses are provisional hypotheses.
"""
from pathlib import Path
from core import *

def shape(length):
    return f'Pose {{ translation 0 0 {-length/2} children [ Shape {{ appearance PBRAppearance {{ baseColor 0.25 0.4 0.65 }} geometry Box {{ size .012 .012 {length} }} }} ] }}'
def collision(length):
    return f'Pose {{ translation 0 0 {-length/2} children [ Box {{ size .012 .012 {length} }} ] }}'
def joint(name,axis,anchor,q,lo,hi,length,child,mass):
    # Initial endpoint rotation matches HingeJointParameters.position.
    return f'''HingeJoint {{ jointParameters HingeJointParameters {{ axis {axis} anchor {anchor} position {q} dampingConstant .005 minStop {lo} maxStop {hi} }}
    device [ RotationalMotor {{ name "{name}" minPosition {lo} maxPosition {hi} maxVelocity 2 maxTorque .15 controlPID 8 0 0 }} PositionSensor {{ name "{name}_sensor" }} ]
    endPoint Solid {{ translation {anchor} rotation {axis} {q} name "{name}_link" children [ {shape(length)} {child} ]
    boundingObject {collision(length)} physics Physics {{ density -1 mass {mass} }} }} }}'''
legs=[]
for i,(name,x,y) in enumerate(zip(LEGS,(.068,-.068,-.068,.068),(.038,.038,-.038,-.038))):
    r,b,c=ik(0,.099,.010 if i<2 else -.010)
    foot='Solid { translation 0 0 -.055 name "'+name+'_foot" contactMaterial "foot" children [ Shape { appearance PBRAppearance { baseColor .1 .1 .1 } geometry Sphere { radius .008 } } ] boundingObject Sphere { radius .008 } physics Physics { density -1 mass .003 } }'
    knee=joint(name+'_knee','0 -1 0','0 0 -.055',c,math.radians(18),math.radians(162),L3,foot,.014)
    hip=joint(name+'_hip','0 -1 0','0 0 -.023',b,-math.radians(72),math.radians(72),L2,knee,.018)
    legs.append(joint(name+'_roll','1 0 0',f'{x} {y} 0',r,-math.radians(72),math.radians(72),L1,hip,.012))
world='''#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 5 coordinateSystem "ENU" contactProperties [ ContactProperties { material1 "foot" material2 "floor" coulombFriction [ .7 ] bounce 0 } ] }
Viewpoint { orientation .4 .8 .4 2.4 position .4 -.5 .35 }
Background { skyColor [ .7 .8 .9 ] }
DirectionalLight { direction -.2 .3 -1 }
Solid { translation 0 0 -.025 contactMaterial "floor" children [ Shape { appearance PBRAppearance { baseColor .6 .6 .6 } geometry Box { size 10 10 .05 } } ] boundingObject Box { size 10 10 .05 } }
DEF DOG Robot { translation 0 0 .109 controller "dog" supervisor TRUE selfCollision TRUE name "Dog research prototype"
children [
Shape { appearance PBRAppearance { baseColor .2 .2 .2 } geometry Box { size .136 .076 .03 } }
Accelerometer { name "accel" } Gyro { name "gyro" } InertialUnit { name "imu" }
'''+ '\n'.join(legs)+'''
] boundingObject Box { size .136 .076 .03 } physics Physics { density -1 mass .40 centerOfMass [ 0 0 0 ] }
}
'''
Path(__file__).parent.joinpath('worlds/dog.wbt').write_text(world)
print('Generated worlds/dog.wbt: 12 joints, provisional dynamics')
