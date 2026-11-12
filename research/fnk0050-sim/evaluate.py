import argparse,json,time,resource,platform
from pathlib import Path
import numpy as np
import mujoco
from mujoco_env import DogEnv
p=argparse.ArgumentParser();p.add_argument('--steps',type=int,default=500);p.add_argument('--seeds',type=int,default=5);p.add_argument('--output',default='logs/evaluation.json');a=p.parse_args()
results=[]
for mode,randomized,perturb in [('stand',False,False),('squat',False,False),('step',False,False),('walk',False,False),('turn',False,False),('stand',False,True),('walk',True,False)]:
 e=DogEnv(mode,a.steps)
 for seed in range(a.seeds if randomized else 1):
  e.reset(seed=seed,options={'randomize':randomized});start=time.perf_counter();peak_error=peak_tilt=0
  for k in range(a.steps):
   if perturb and k==100:e.data.xfrc_applied[1,1]=.3
   if perturb and k==105:e.data.xfrc_applied[:]=0
   o,r,d,t,info=e.step(np.zeros(12));peak_error=max(peak_error,info['joint_max_error']);peak_tilt=max(peak_tilt,*map(abs,info['rpy'][:2]))
   if d or t:break
  elapsed=time.perf_counter()-start
  result={'mode':mode,'randomized':randomized,'perturbed':perturb,'seed':seed,'steps':k+1,'fell':d,'return':e.return_,'wall_seconds':elapsed,'sim_seconds':(k+1)*.02,'realtime_factor':(k+1)*.02/elapsed,'peak_joint_error_rad':peak_error,'peak_tilt_rad':peak_tilt,**info}
  results.append(result);print(json.dumps(result))
report={'environment':{'python':platform.python_version(),'mujoco':mujoco.__version__,'host':platform.machine(),'peak_rss_kib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,'not_mint_benchmark':True},'results':results}
Path(a.output).parent.mkdir(parents=True,exist_ok=True);Path(a.output).write_text(json.dumps(report,indent=2)+'\n')
