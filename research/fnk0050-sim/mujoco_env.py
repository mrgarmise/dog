"""CPU-only Gymnasium environment; no rendering and no hardware side effects."""
from pathlib import Path
import math
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces
from core import *
class DogEnv(gym.Env):
    metadata={'render_modes':[]}
    def __init__(self, mode='stand', horizon=500):
        self.model=mujoco.MjModel.from_xml_path(str(Path(__file__).parent/'models/dog.xml'))
        self.data=mujoco.MjData(self.model); self.mode=mode; self.horizon=horizon
        self.action_space=spaces.Box(-1,1,(12,),dtype=np.float32)
        self.observation_space=spaces.Box(-np.inf,np.inf,(20,),dtype=np.float32)
        self.mass0=self.model.body_mass.copy(); self.inertia0=self.model.body_inertia.copy(); self.friction0=self.model.geom_friction.copy()
        self.steps=0; self.params={}; self.q=gait(0,stride=0,lift=0)
    def obs(self):
        phase=2*math.pi*self.steps*DT*self.params.get('hz',.5)
        return np.array(list(self.data.sensordata[:3]/9.81)+list(self.data.sensordata[3:6])+self.q+[math.sin(phase),math.cos(phase)],dtype=np.float32)
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed); options=options or {}; self.steps=0; self.return_=0; self.params=options.get('params',{})
        randomized=options.get('randomize',False)
        scale=float(self.np_random.uniform(.8,1.2)) if randomized else 1.
        self.model.body_mass[:]=self.mass0*scale; self.model.body_inertia[:]=self.inertia0*scale
        self.model.geom_friction[:]=self.friction0
        if randomized: self.model.geom_friction[:,0]=self.np_random.uniform(.4,1.)
        mujoco.mj_setConst(self.model,self.data); mujoco.mj_resetData(self.model,self.data)
        self.rate=float(self.np_random.uniform(1.5,2.5)) if randomized else 2.
        self.delay=int(self.np_random.integers(0,3)) if randomized else 0
        self.q=gait(0,stride=0,lift=0)
        self.data.qpos[:3]=[0,0,.109]; self.data.qpos[3:7]=[1,0,0,0]; self.data.qpos[7:]=self.q
        self.data.ctrl[:]=self.q; self.queue=[self.q[:] for _ in range(self.delay)]
        mujoco.mj_forward(self.model,self.data)
        self.previous_pos=self.data.qpos[:3].copy(); self.start_pos=self.previous_pos.copy()
        self.ended=False
        return self.obs(),{'calibrated':False}
    def baseline(self):
        t=self.steps*DT
        if self.mode=='stand': return gait(t,stride=0,lift=0,**self.params)
        if self.mode=='squat': return gait(t,stride=0,lift=0,height=.097+.005*math.cos(t*math.pi))
        if self.mode=='step': return gait(t,stride=0)
        return gait(t,turn=.3 if self.mode=='turn' else 0,**self.params)
    def step(self,action):
        if self.ended: raise RuntimeError('reset required after episode ends')
        a=np.asarray(action,dtype=float)
        if a.shape!=(12,) or not np.isfinite(a).all() or np.max(np.abs(a))>1: raise ValueError('12 actions in [-1,1] required')
        target=[q+.03*x for q,x in zip(self.baseline(),a)]; servo_degrees(target)
        self.queue.append(target); delayed=self.queue.pop(0); previous=self.q[:]; self.q=slew(self.q,delayed,self.rate)
        self.data.ctrl[:]=self.q
        substeps=round(DT/self.model.opt.timestep)
        mujoco.mj_step(self.model,self.data,nstep=substeps)
        self.steps+=1
        pos=self.data.qpos[:3]; R=self.data.xmat[1].reshape(3,3)
        roll=math.atan2(R[2,1],R[2,2]); pitch=math.asin(np.clip(-R[2,0],-1,1)); yaw=math.atan2(R[1,0],R[0,0])
        velocity=float((pos[0]-self.previous_pos[0])/DT); self.previous_pos=pos.copy()
        reward=DT*(1+min(.15,max(-.15,velocity))*5-2*(roll*roll+pitch*pitch))-.1*sum((x-y)**2 for x,y in zip(self.q,previous))
        done=bool(pos[2]<.055 or max(abs(roll),abs(pitch))>.7 or not np.isfinite(self.obs()).all())
        if done: reward-=5
        truncated=self.steps>=self.horizon; self.ended=done or truncated; self.return_+=reward
        info={'position':pos.tolist(),'rpy':[roll,pitch,yaw],'distance_x':float(pos[0]-self.start_pos[0]),
        'joint_max_error':float(np.max(np.abs(self.data.qpos[7:]-self.q))), 'contacts':self.data.ncon,'calibrated':False}
        return self.obs(),float(reward),done,truncated,info
