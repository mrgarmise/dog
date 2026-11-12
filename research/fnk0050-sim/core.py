"""FNK0050 kinematic prototype; SI units, radians. No hardware imports.
New implementation informed by Freenove Control.py; CC BY-NC-SA 3.0.
"""
import math
import json
from pathlib import Path
LEGS = ('FL', 'RL', 'RR', 'FR')
CHANNELS = ((4,3,2),(7,6,5),(8,9,10),(11,12,13))
L1,L2,L3 = .023,.055,.055
DT=.02

def ik(x, down, lateral):
    if not all(math.isfinite(v) for v in (x,down,lateral)) or down<=0:
        raise ValueError('invalid foot target')
    roll=math.atan2(lateral,down)
    radial=math.hypot(down,lateral)-L1
    d=math.hypot(x,radial)
    if not 1e-8 < d < L2+L3: raise ValueError('unreachable target')
    knee=math.acos(max(-1,min(1,(d*d-L2*L2-L3*L3)/(2*L2*L3))))
    hip=math.atan2(x,radial)-math.atan2(L3*math.sin(knee),L2+L3*math.cos(knee))
    return [roll,hip,knee]

def fk(q):
    r,b,c=q
    radial=L1+L2*math.cos(b)+L3*math.cos(b+c)
    return [L2*math.sin(b)+L3*math.sin(b+c),radial*math.cos(r),radial*math.sin(r)]

def servo_degrees(q, calibration=None):
    """Return channel->degrees. Reject, never silently clip. Calibration in IK degrees."""
    if len(q)!=12 or not all(math.isfinite(v) for v in q): raise ValueError('12 finite angles required')
    calibration=calibration or [[0,0,0] for _ in LEGS]
    result={}
    for i, channels in enumerate(CHANNELS):
        r,b,c=q[3*i:3*i+3]
        a=90-math.degrees(r)+calibration[i][0]
        b=math.degrees(b)+calibration[i][1]; c=math.degrees(c)+calibration[i][2]
        angles=(a,90-b,c) if i<2 else (a,90+b,180-c)
        for ch,v in zip(channels,angles):
            if not math.isfinite(v) or not 18<=v<=162: raise ValueError('servo envelope exceeded')
            result[ch]=v
    return result

def gait(t,stride=.008,lift=.004,hz=.5,height=.099,turn=0):
    """Crawl starter, not a validated walking policy. turn scales left/right stride."""
    if not (0<=stride<=.016 and 0<=lift<=.020 and .1<=hz<=1 and .09<=height<=.105 and abs(turn)<=.5):
        raise ValueError('gait parameter envelope')
    q=[]
    for i,offset in enumerate((0,.75,.25,.5)):
        phase=(t*hz+offset)%1
        s=stride*(1-turn if i<2 else 1+turn)
        if phase<.75: x=s*(.5-phase/.75); up=0
        else:
            u=(phase-.75)/.25; x=s*(-.5+u); up=lift*math.sin(math.pi*u)
        q.extend(ik(x,height-up,.010 if i<2 else -.010))
    servo_degrees(q)
    return q

def slew(previous,target,rate=2.0,dt=DT):
    if len(previous)!=12 or len(target)!=12 or not all(math.isfinite(x) for x in previous+target): raise ValueError('bad angles')
    if rate<=0 or dt<=0: raise ValueError('bad rate')
    return [p+max(-rate*dt,min(rate*dt,t-p)) for p,t in zip(previous,target)]

def export_policy(path, params):
    gait(0,**params)
    Path(path).write_text(json.dumps({'schema':'dog-gait-v1','units':'SI/radians','leg_order':LEGS,'params':params,'simulation_accepted':False,'hardware_accepted':False},indent=2)+'\n')
