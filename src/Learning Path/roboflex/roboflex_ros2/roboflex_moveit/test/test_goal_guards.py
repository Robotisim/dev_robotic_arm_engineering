"""Offline checks of interpolation and firmware gating; no hardware access."""
import importlib.util
from pathlib import Path
import time
import unittest

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

path=Path(__file__).resolve().parents[1]/'scripts/moveit_goal.py'
spec=importlib.util.spec_from_file_location('moveit_goal',path)
goal=importlib.util.module_from_spec(spec);spec.loader.exec_module(goal)

class GuardTests(unittest.TestCase):
    def trajectory(self, end=.01, end_time=1, velocities=None):
        points=[]
        for q,t,v in [(0.,0,velocities or [0.]*4),(end,end_time,velocities or [0.]*4)]:
            points.append(JointTrajectoryPoint(positions=[q]*4,velocities=v,accelerations=[0.]*4,
                                              time_from_start=Duration(sec=t)))
        return JointTrajectory(joint_names=goal.JOINTS,points=points)

    def test_quintic_zero_derivative_endpoints_and_peak(self):
        duration,samples=goal.trajectory_samples(self.trajectory())
        self.assertEqual(duration,1)
        for actual,expected in zip(samples[0][1],[0.]*4):self.assertAlmostEqual(actual,expected)
        for actual in samples[-1][1]:self.assertAlmostEqual(actual,.01)
        for actual in samples[-1][2]+samples[-1][3]:self.assertAlmostEqual(actual,0)
        # A zero-end-derivative quintic has peak velocity 1.875 * displacement / duration.
        self.assertAlmostEqual(max(s[2][0] for s in samples),.01875,delta=.0001)

    def test_intermediate_overshoot_is_detectable(self):
        _,samples=goal.trajectory_samples(self.trajectory(end=0.,velocities=[1.]*4))
        self.assertGreater(max(abs(s[1][0]) for s in samples),.05)

    def test_bad_time_and_nonfinite_rejected(self):
        for traj in [self.trajectory(end_time=0),self.trajectory(end=float('nan'))]:
            with self.assertRaises(ValueError):goal.trajectory_samples(traj)

    def fixture(self):
        class Dummy:pass
        d=Dummy();d.firmware_at=time.monotonic()
        d.firmware=dict(status_age_s=.2,armed=1,hold=0,driver_fault=0,wifi='connected',
                        rx_count=10,last_rx_ms_ago=100,current_deg=[2.,52.,37.,69.,90.],
                        target_deg=[2.,52.,37.,69.,90.])
        return d

    def test_firmware_rejects_hold_staleness_fault_and_disagreement(self):
        q=[.0349066,-.3839724,-.9250245,-.3665191,0.]
        goal.GoalClient.firmware_healthy(self.fixture(),q)
        for key,value in [('armed',0),('hold',1),('driver_fault',1),('status_age_s',4.),
                          ('wifi','disconnected'),('last_rx_ms_ago',1000),('rx_count',0),
                          ('current_deg',[90.]*5)]:
            d=self.fixture();d.firmware[key]=value
            with self.assertRaises(RuntimeError,msg=key):goal.GoalClient.firmware_healthy(d,q)

if __name__=='__main__':unittest.main()
