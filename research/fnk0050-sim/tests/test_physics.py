import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import mujoco
from core import *
from mujoco_env import DogEnv
class PhysicsTests(unittest.TestCase):
 def test_model_fk(self):
  e=DogEnv();e.reset(seed=0)
  rng=np.random.default_rng(7)
  for _ in range(100):
   for i,n in enumerate(LEGS):
    q=[rng.uniform(-.2,.2),rng.uniform(-1,-.3),rng.uniform(.7,1.7)]
    e.data.qpos[7+3*i:10+3*i]=q
    mujoco.mj_forward(e.model,e.data)
    tip=e.data.site_xpos[mujoco.mj_name2id(e.model,mujoco.mjtObj.mjOBJ_SITE,n+'_tip')]
    hip=np.array([(.068,-.068,-.068,.068)[i],(.038,.038,-.038,-.038)[i],.109])
    x,d,l=fk(q); np.testing.assert_allclose(tip-hip,[x,l,-d],atol=1e-12)
 def test_stand(self):
  e=DogEnv(horizon=500);e.reset(seed=1)
  for _ in range(500):o,r,d,t,info=e.step(np.zeros(12));self.assertFalse(d)
  self.assertTrue(t);self.assertGreaterEqual(info['contacts'],4);self.assertLess(info['joint_max_error'],.06)
 def test_determinism(self):
  e=DogEnv('walk');runs=[]
  for _ in range(2):
   o,_=e.reset(seed=12,options={'randomize':True});run=[]
   for i in range(100):
    o,r,d,t,info=e.step(np.zeros(12));run.append(o)
   runs.append(run)
  np.testing.assert_array_equal(runs[0],runs[1])
 def test_bad_action_and_reset(self):
  e=DogEnv(horizon=1);e.reset(seed=1)
  with self.assertRaises(ValueError):e.step(np.full(12,np.nan))
  e.step(np.zeros(12))
  with self.assertRaises(RuntimeError):e.step(np.zeros(12))
 def test_torque_bound(self):
  e=DogEnv();e.reset(seed=1)
  for i in range(50):
   e.step(np.ones(12));self.assertLessEqual(np.abs(e.data.actuator_force).max(),.1500001)
if __name__=='__main__':unittest.main()
