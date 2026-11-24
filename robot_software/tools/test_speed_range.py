#!/usr/bin/python3
"""Range timing must rescue rate-limited geometry without relaxing its floor."""
import math,unittest
import numpy as np
from scipy.interpolate import CubicSpline
from pinchy_motion.paths import CartesianCurve,timed_trace
from pinchy_motion.speed_range import BoundedSpeedLaw
from pinchy_bringup.control_contract import JOINTS

class Kinematics:
    def evaluate(self,q):
        t=np.eye(4);t[0,3]=q[0];j=np.zeros((3,6));j[0,0]=1.;return t,j,None

class RangeTests(unittest.TestCase):
    def problem(self):
        d=np.linspace(0,.1,401);q=np.zeros((len(d),6));q[:,0]=d;q[:,1]=.4*np.tanh((d-.05)/.008)
        a=np.eye(4);b=a.copy();b[0,3]=.1;curve=CartesianCurve([a,b])
        limits={n:dict(lower=-1.,upper=1.,velocity=math.radians(30),acceleration=math.radians(180)) for n in JOINTS}
        return curve,d,q,limits
    def time(self,minimum=.004,**kwargs):
        c,d,q,l=self.problem();return timed_trace(c,d,q,Kinematics(),l,'tool_axis',.018,.08,[False],enforce_tcp_speed=True,tcp_min_speed=minimum,**kwargs)
    def test_point_file_range_defaults_and_validation(self):
        from pinchy_motion.point_sets import normalize
        from pinchy_motion.nominal_paths import fast_cylinder_loop,bounded_cylinder_loop
        self.assertEqual(normalize(fast_cylinder_loop())['tcp_min_speed_m_s'],0.)
        self.assertEqual(normalize(bounded_cylinder_loop())['tcp_min_speed_m_s'],.004)
        for value in (-1.,True,float('nan'),.0001,.018,.02):
            data=bounded_cylinder_loop();data['tcp_min_speed_m_s']=value
            with self.subTest(value=value),self.assertRaises(ValueError):normalize(data)
        data=bounded_cylinder_loop();data['style']='visits'
        with self.assertRaisesRegex(ValueError,'continuous'):normalize(data)

    def test_range_passes_when_maximum_fixed_speed_fails(self):
        c,d,q,l=self.problem()
        with self.assertRaisesRegex(ValueError,'joint timing'):timed_trace(c,d,q,Kinematics(),l,'tool_axis',.018,.08,[False],enforce_tcp_speed=True)
        tr,r=self.time();self.assertGreaterEqual(r['modeled_cruise_min_m_s'],.004-1e-7);self.assertLessEqual(r['modeled_cruise_max_m_s'],.018+1e-7)
        self.assertGreater(r['modeled_cruise_max_m_s'],r['modeled_cruise_min_m_s']*2)
        self.assertAlmostEqual(r['transit_time_scale'],1.,places=8);self.assertTrue(r['speed_range']['enabled']);self.assertTrue(r['speed_range']['slowdown_intervals_m'])
        self.assertTrue(all(p.velocities[0]>0 for p in tr.joint_trajectory.points[1:-1]))
    def test_infeasible_floor_rejects_without_silent_reduction(self):
        with self.assertRaisesRegex(ValueError,'minimum|Minimum'):self.time(.014)
    def test_short_stroke_and_invalid_range_reject(self):
        spline=CubicSpline([0.,.001],np.zeros((2,6)))
        for lo,hi in ((0.,.01),(.01,.01),(.02,.01),(float('nan'),.01)):
            with self.assertRaises(ValueError):BoundedSpeedLaw(.1,lo,hi,.08,spline,[1.]*6,[1.]*6)
        with self.assertRaisesRegex(ValueError,'short'):BoundedSpeedLaw(.0002,.004,.018,.02,spline,[1.]*6,[1.]*6)
    def test_contact_ramp_permission_is_preserved(self):
        c,d,q,l=self.problem()
        with self.assertRaisesRegex(ValueError,'Contact includes'):timed_trace(c,d,q,Kinematics(),l,'tool_axis',.018,.08,[True],tcp_min_speed=.004)
        _,r=timed_trace(c,d,q,Kinematics(),l,'tool_axis',.018,.08,[True],tcp_min_speed=.004,allow_contact_ramps=True)
        self.assertTrue(r['contact_ramps_allowed']);self.assertAlmostEqual(r['modeled_contact_min_m_s'],0.)
        self.assertGreaterEqual(r['modeled_steady_contact_min_m_s'],.004-1e-7)
    def test_joint_position_bounds_and_stop_remain_authoritative(self):
        c,d,q,l=self.problem();q[:,2]=2.
        with self.assertRaisesRegex(ValueError,'bounds'):timed_trace(c,d,q,Kinematics(),l,'tool_axis',.018,.08,[False],tcp_min_speed=.004)
        with self.assertRaisesRegex(RuntimeError,'Stopped'):self.time(stop=lambda:True)
    def test_long_timing_candidates_have_bounded_solver_size_and_duration(self):
        d=np.linspace(0.,10.,8001);q=np.zeros((len(d),6));q[:,0]=d
        law=BoundedSpeedLaw(10.,.004,.2,.08,CubicSpline(d,q),[1.]*6,[2.]*6)
        self.assertLessEqual(len(law.squared.c),128);self.assertLess(law.duration,110.)
        with self.assertRaisesRegex(ValueError,'110-second'):BoundedSpeedLaw(50.,.004,.2,.08,CubicSpline(d,q),[1.]*6,[2.]*6)

    def test_smooth_law_and_acceleration_through_internal_slowdown(self):
        c,d,q,l=self.problem();law=BoundedSpeedLaw(.1,.004,.018,.08,CubicSpline(d,q),[math.radians(30)]*6,[math.radians(180)]*6)
        values=np.array([law.at(t) for t in np.linspace(0,law.duration,2001)])
        self.assertTrue(np.all(np.diff(values[:,0])>0));self.assertLessEqual(max(abs(values[:,2])),.08+1e-8)
        for s in np.linspace(0,.1,80):self.assertAlmostEqual(law.at(law.time_at(s))[0],s,places=7)
        for t in (law.ramp_time,law.duration-law.ramp_time):
            self.assertAlmostEqual(law.at(t)[2],0.,places=8);self.assertLess(abs(law.at(t-1e-6)[2]-law.at(t+1e-6)[2]),.0001)

if __name__=='__main__':unittest.main()
