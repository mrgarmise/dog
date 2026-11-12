import sys, os, json, time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from controller import Supervisor
from env import DogEnv
import numpy as np
s=Supervisor()
mode=os.environ.get('DOG_MODE','stand'); horizon=int(os.environ.get('DOG_STEPS','500'))
e=DogEnv(s,mode,horizon)
results=[]
try:
    for seed in range(int(os.environ.get('DOG_SEEDS','1'))):
        obs,info=e.reset(seed=seed,options={'randomize':os.environ.get('DOG_RANDOMIZE')=='1'})
        start=time.perf_counter(); peak_error=0; peak_tilt=0
        for k in range(horizon):
            if os.environ.get('DOG_PERTURB')=='1' and k==100: e.node.addForce([.1,0,0],False)
            obs,reward,done,trunc,info=e.step(np.zeros(12,dtype=np.float32))
            peak_error=max(peak_error,info['joint_max_error']); peak_tilt=max(peak_tilt,*map(abs,info['rpy'][:2]))
            if done or trunc: break
        elapsed=time.perf_counter()-start
        result={'mode':mode,'seed':seed,'steps':k+1,'fell':done,'return':e.return_,'seconds':elapsed,
                'sim_seconds':(k+1)*.02,'realtime_factor':(k+1)*.02/elapsed,
                'peak_joint_error_rad':peak_error,'peak_tilt_rad':peak_tilt,**info}
        results.append(result); print('DOG_RESULT '+json.dumps(result),flush=True)
    out=Path(os.environ.get('DOG_OUTPUT',str(ROOT/'logs'/f'{mode}.json'))); out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(results,indent=2)+'\n')
    s.simulationQuit(0)
except Exception:
    import traceback
    traceback.print_exc(); s.simulationQuit(1)
