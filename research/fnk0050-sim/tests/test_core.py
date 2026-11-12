import math, random, unittest, sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core import *
class CoreTests(unittest.TestCase):
    def test_round_trip(self):
        rng=random.Random(42)
        for _ in range(1000):
            p=[rng.uniform(-.02,.02),rng.uniform(.085,.11),rng.uniform(-.015,.015)]
            self.assertLess(max(abs(a-b) for a,b in zip(p,fk(ik(*p)))),1e-12)
    def test_official_forward_equations(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'reference'))
        from official_kinematics import Official
        rng=random.Random(101)
        for _ in range(1000):
            r,b,c=rng.uniform(-.2,.2),rng.uniform(-1,-.2),rng.uniform(.6,1.8)
            expected=Official().angleToCoordinate(90-math.degrees(r),math.degrees(b),math.degrees(c))
            self.assertLess(max(abs(x-y/1000) for x,y in zip(fk([r,b,c]),expected)),1e-12)
    def test_channels_and_neutral(self):
        q=[0,-math.pi/4,math.pi/2]*4
        d=servo_degrees(q)
        self.assertEqual(set(d),set(range(2,14)))
        self.assertEqual([d[x] for x in (4,3,2,7,6,5,8,9,10,11,12,13)],[90,135,90,90,135,90,90,45,90,90,45,90])
    def test_gait_limits(self):
        for turn in (-.5,0,.5):
            for k in range(1000): servo_degrees(gait(k*.02,stride=.016,lift=.01,turn=turn))
    def test_reject(self):
        for p in [(0,1,0),(0,0,0),(float('nan'),.1,0)]:
            with self.assertRaises(ValueError): ik(*p)
        with self.assertRaises(ValueError): servo_degrees([0]*12)
        with self.assertRaises(ValueError): slew([0]*12,[float('nan')]*12)
    def test_slew(self):
        self.assertEqual(slew([0]*12,[1]*12),[.04]*12)
    def test_independent_homogeneous_chain(self):
        import numpy as np
        for r,b,c in ((.1,-.6,1.3),(-.15,-.8,1.5),(0,-.4,1)):
            rx=np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
            def ry(q): return np.array([[math.cos(q),0,-math.sin(q)],[0,1,0],[math.sin(q),0,math.cos(q)]])
            p=rx@(np.array([0,0,-L1])+ry(b)@(np.array([0,0,-L2])+ry(c)@np.array([0,0,-L3])))
            x,down,lat=fk([r,b,c]); np.testing.assert_allclose(p,[x,lat,-down],atol=1e-12)
if __name__=='__main__': unittest.main()
