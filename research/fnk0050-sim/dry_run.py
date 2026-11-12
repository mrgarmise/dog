"""Stdlib-only policy playback to JSON lines. Intentionally has no I2C/network backend."""
import json,argparse
from core import *
p=argparse.ArgumentParser();p.add_argument('--policy',default='gait_policy.json');p.add_argument('--seconds',type=float,default=2);args=p.parse_args()
policy=json.loads(Path(args.policy).read_text())
if policy.get('schema')!='dog-gait-v1' or policy.get('leg_order')!=list(LEGS):raise ValueError('unsupported policy schema/order')
q=gait(0,stride=0,lift=0)
for k in range(int(args.seconds/DT)):
 q=slew(q,gait(k*DT,**policy['params']))
 print(json.dumps({'sequence':k,'t':k*DT,'servo_degrees':servo_degrees(q),'hardware_write':False}))
