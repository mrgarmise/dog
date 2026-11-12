import json,time
from pathlib import Path
import numpy as np
from mujoco_env import DogEnv
params=json.loads(Path('gait_policy.json').read_text())['params']; results=[]
for dt in (.001,.002,.004):
 for seed in (200,201,202):
  e=DogEnv('walk',5000);e.model.opt.timestep=dt;e.reset(seed=seed,options={'params':params,'randomize':True})
  start=time.perf_counter();peak_tilt=peak_error=0.;max_clearance=np.zeros(4)
  tips=[e.model.site(n+'_tip').id for n in ('FL','RL','RR','FR')]
  for k in range(5000):
   o,r,d,t,info=e.step(np.zeros(12));max_clearance=np.maximum(max_clearance,e.data.site_xpos[tips,2]-.008)
   peak_tilt=max(peak_tilt,*map(abs,info['rpy'][:2]));peak_error=max(peak_error,info['joint_max_error'])
   if d or t:break
  results.append({'timestep':dt,'seed':seed,'steps':k+1,'fell':d,'peak_tilt':peak_tilt,'peak_joint_error':peak_error,'max_foot_clearance_m':max_clearance.tolist(),'elapsed_s':time.perf_counter()-start,**info})
Path('logs/long-validation.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps({'episodes':len(results),'falls':sum(r['fell'] for r in results),'min_distance':min(r['distance_x'] for r in results),'max_distance':max(r['distance_x'] for r in results),'max_abs_yaw':max(abs(r['rpy'][2]) for r in results)}))
