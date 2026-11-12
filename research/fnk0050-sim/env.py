"""Gymnasium interface hosted inside a Webots Supervisor Python controller."""
import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from core import *

class DogEnv(gym.Env):
    metadata={'render_modes':[]}
    def __init__(self, supervisor, mode='stand', horizon=500):
        self.s=supervisor; self.node=self.s.getSelf(); self.mode=mode; self.horizon=horizon
        self.action_space=spaces.Box(-1,1,(12,),dtype=np.float32)
        # IMU specific force / g, gyro rad/s, last commanded angles, sin/cos phase
        self.observation_space=spaces.Box(-np.inf,np.inf,(20,),dtype=np.float32)
        self.motors=[self.s.getDevice(n+'_'+j) for n in LEGS for j in ('roll','hip','knee')]
        self.sensors=[self.s.getDevice(n+'_'+j+'_sensor') for n in LEGS for j in ('roll','hip','knee')]
        self.acc=self.s.getDevice('accel'); self.gyro=self.s.getDevice('gyro'); self.imu=self.s.getDevice('imu')
        for device in self.sensors+[self.acc,self.gyro,self.imu]: device.enable(20)
        self.initial=self.node.getField('translation').getSFVec3f()
        self.params={}; self.delay=0
        self.node.saveState("initial")
    def obs(self):
        phase=2*math.pi*self.steps*DT*self.params.get('hz',.5)
        return np.asarray([v/9.81 for v in self.acc.getValues()]+list(self.gyro.getValues())+self.q+[math.sin(phase),math.cos(phase)],dtype=np.float32)
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed); options=options or {}; self.steps=0; self.return_=0
        self.params=options.get('params',{})
        self.node.loadState('initial')
        self.node.getField('translation').setSFVec3f(self.initial)
        self.node.getField('rotation').setSFRotation([0,0,1,0])
        self.q=gait(0,stride=0,lift=0)
        for i,m in enumerate(self.motors): m.setPosition(self.q[i])
        self.node.resetPhysics()
        self.rate=float(self.np_random.uniform(1.5,2.5)) if options.get('randomize') else 2.
        self.delay=int(self.np_random.integers(0,3)) if options.get('randomize') else 0
        self.queue=[self.q[:] for _ in range(self.delay)]
        # Randomized body mass and foot-ground friction; hypotheses, not identified values.
        physics=self.node.getField('physics').getSFNode()
        physics.getField('mass').setSFFloat(float(self.np_random.uniform(.32,.48)) if options.get('randomize') else .40)
        world=self.s.getRoot().getField('children').getMFNode(0)
        contact=world.getField('contactProperties').getMFNode(0)
        contact.getField('coulombFriction').setMFFloat(0,float(self.np_random.uniform(.4,1.)) if options.get('randomize') else .7)
        if self.s.step(20)==-1: raise RuntimeError('Webots stopped')
        self.previous_pos=self.node.getPosition(); self.start_pos=self.previous_pos[:]
        return self.obs(), {'calibrated':False}
    def baseline(self):
        t=self.steps*DT
        if self.mode=='stand': return gait(t,stride=0,lift=0,**self.params)
        if self.mode=='squat': return gait(t,stride=0,lift=0,height=.097+.005*math.cos(t*math.pi))
        if self.mode=='step': return gait(t,stride=0)
        return gait(t,turn=.3 if self.mode=='turn' else 0,**self.params)
    def step(self, action):
        action=np.asarray(action,dtype=float)
        if action.shape!=(12,) or not np.isfinite(action).all() or np.max(np.abs(action))>1: raise ValueError('action must be 12 finite values in [-1,1]')
        target=[q+.03*a for q,a in zip(self.baseline(),action)]
        servo_degrees(target)
        self.queue.append(target); delayed=self.queue.pop(0)
        previous=self.q[:]; self.q=slew(self.q,delayed,self.rate)
        for m,q in zip(self.motors,self.q): m.setPosition(q)
        if self.s.step(20)==-1: raise RuntimeError('Webots stopped')
        self.steps+=1
        pos=self.node.getPosition(); roll,pitch,yaw=self.imu.getRollPitchYaw()
        velocity=(pos[0]-self.previous_pos[0])/DT; self.previous_pos=pos
        effort=sum((a-b)**2 for a,b in zip(self.q,previous))
        reward=DT*(1+min(.15,max(-.15,velocity))*5-2*(roll*roll+pitch*pitch))-.1*effort
        terminated=bool(pos[2]<.055 or max(abs(roll),abs(pitch))>.7 or not np.isfinite(self.obs()).all())
        if terminated: reward-=5
        self.return_+=reward
        info={'position':pos,'rpy':[roll,pitch,yaw],'distance_x':pos[0]-self.start_pos[0],
              'joint_max_error':max(abs(s.getValue()-q) for s,q in zip(self.sensors,self.q)),
              'calibrated':False}
        return self.obs(),float(reward),terminated,self.steps>=self.horizon,info
