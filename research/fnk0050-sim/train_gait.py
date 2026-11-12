"""Low-compute random search over four gait parameters, held-out validation.
This optimizes an UNCALIBRATED simulator, never authorizes hardware deployment.
"""
import argparse,json,time
from pathlib import Path
import numpy as np
from mujoco_env import DogEnv
from core import export_policy
p=argparse.ArgumentParser();p.add_argument('--trials',type=int,default=32);p.add_argument('--steps',type=int,default=500);p.add_argument('--output',default='logs/search.json');args=p.parse_args()
rng=np.random.default_rng(20261112);e=DogEnv('walk',args.steps)
def episode(params,seed):
 e.reset(seed=seed,options={'params':params,'randomize':True})
 clearance=np.zeros(4)
 tips=[e.model.site(n+'_tip').id for n in ('FL','RL','RR','FR')]
 for k in range(args.steps):
  o,r,done,trunc,info=e.step(np.zeros(12))
  if k>100: clearance=np.maximum(clearance,e.data.site_xpos[tips,2]-.008)
  if done or trunc:break
 score=info['distance_x']-abs(info['position'][1])-.02*abs(info['rpy'][2])-(1 if done else 0)-(1 if min(clearance)<.002 else 0)
 return {'score':score,'min_peak_clearance':float(min(clearance)),'fell':done,'steps':k+1,**info}
start=time.perf_counter();trials=[]
for i in range(args.trials):
 params={'stride':float(rng.uniform(.006,.016)),'lift':float(rng.uniform(.009,.013)),'hz':float(rng.uniform(.3,.7)),'height':float(rng.uniform(.097,.103))}
 runs=[episode(params,s) for s in (0,1,2)]
 score=float(np.mean([r['score'] for r in runs]));trials.append({'params':params,'score':score,'runs':runs})
best=max(trials,key=lambda t:t['score'])
heldout=[episode(best['params'],s) for s in range(100,110)]
Path(args.output).write_text(json.dumps({'seed':20261112,'elapsed_s':time.perf_counter()-start,'trials':trials,'best':best,'heldout':heldout},indent=2)+'\n')
export_policy('gait_policy.json',best['params'])
print(json.dumps({'best':best['params'],'training_score':best['score'],'heldout_mean_distance':float(np.mean([r['distance_x'] for r in heldout])),'heldout_falls':sum(r['fell'] for r in heldout),'elapsed_s':time.perf_counter()-start}))
