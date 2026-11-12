"""Optional desktop viewer. Headless training does not import this module."""
import time,json
from pathlib import Path
import numpy as np
import mujoco.viewer
from mujoco_env import DogEnv
e=DogEnv('walk',5000);params=json.loads(Path('gait_policy.json').read_text())['params'];e.reset(seed=0,options={'params':params})
with mujoco.viewer.launch_passive(e.model,e.data) as viewer:
 while viewer.is_running():
  start=time.monotonic();o,r,d,t,info=e.step(np.zeros(12));viewer.sync()
  if d or t:e.reset(seed=0,options={'params':params})
  time.sleep(max(0,.02-(time.monotonic()-start)))
