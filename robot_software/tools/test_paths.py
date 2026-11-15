#!/usr/bin/python3
"""Geometric continuity, speed-law and actual joint-spline tracing checks."""
import unittest,sys
import numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))
from test_ik import calibrated_solver
from pinchy_motion.paths import CartesianCurve,SpeedLaw,sample_ik,select_trace,timed_trace,corner_indices,separated,TraceIncomplete
from pinchy_motion.geometry import seconds


class PathTests(unittest.TestCase):
    def branch_problem(self):
        # Five constrained coordinates, one free twist. At the far end the
        # near-zero branch dies; the planner must turn before reaching it.
        class Solver:
            lower=np.full(6,-1.);upper=np.full(6,1.)
            def system(self,q,target,mode):return np.zeros(5),np.eye(6)[:5]
            def search(self,target,seed,mode,valid,**kwargs):
                q=np.asarray(seed).copy();q[0]=target[0,3]
                if q[0]>.006 and q[5]<.15:return None
                return q.tolist() if valid(q) else None
        a=np.eye(4);b=a.copy();b[0,3]=.012
        return CartesianCurve([a,b]),Solver()

    def test_whole_stroke_search_turns_before_a_greedy_dead_end(self):
        curve,solver=self.branch_problem();start=np.zeros(6)
        with self.assertRaises(TraceIncomplete):sample_ik(curve,'tool_axis',start,solver,lambda q:True,lambda:False)
        s,q,result,report=select_trace(curve,'tool_axis',start,solver,lambda q:True,lambda:False,lambda s,q:'validated')
        self.assertEqual(report['method'],'whole_stroke_beam');self.assertEqual(result,'validated')
        np.testing.assert_array_equal(q[0],start)
        self.assertEqual(s[-1],curve.length);self.assertTrue(np.all(np.diff(s)>0))
        self.assertLessEqual(np.max(abs(np.diff(q,axis=0))),.15)
        self.assertTrue(np.all(q[s>.006,5]>=.15))
        self.assertTrue(np.any(q[s<=.006,5]>=.1))

    def test_complete_candidates_must_pass_timing_and_interpolation(self):
        curve,solver=self.branch_problem();seen=[]
        def reject(s,q):
            seen.append(q.copy());raise ValueError('timing/geometry/collision rejected')
        with self.assertRaisesRegex(TraceIncomplete,'whole-stroke alternatives also failed'):
            select_trace(curve,'tool_axis',np.zeros(6),solver,lambda q:True,lambda:False,reject)
        self.assertGreater(len(seen),0)
        self.assertTrue(all(q[-1,0]==curve.length for q in seen))

    def test_search_preserves_collision_rejection_and_exact_mode(self):
        curve,solver=self.branch_problem()
        for mode,valid in [('full',lambda q:True),('tool_axis',lambda q:q[5]<.09)]:
            with self.subTest(mode=mode),self.assertRaises(TraceIncomplete):
                select_trace(curve,mode,np.zeros(6),solver,valid,lambda:False,lambda s,q:self.fail('Invalid candidate accepted'))

    def test_stop_during_candidate_validation_does_not_retry(self):
        curve,solver=self.branch_problem();stopped=[False];calls=[]
        def finish(s,q):
            calls.append(1);stopped[0]=True;raise RuntimeError('Stopped')
        with self.assertRaisesRegex(RuntimeError,'stopped'):
            select_trace(curve,'tool_axis',np.zeros(6),solver,lambda q:True,lambda:stopped[0],finish)
        self.assertEqual(len(calls),1)

    def test_real_model_alternative_passes_shared_timed_validation(self):
        from pinchy_motion.geometry import model_limits
        from pinchy_bringup.control_contract import JOINTS
        root,solver=calibrated_solver();limits=model_limits(root)
        for n,l,h in zip(JOINTS,solver.lower,solver.upper):limits[n].update(lower=l,upper=h)
        start=np.array([.1,-.25,.5,-.25,.4,.4]);a=solver.evaluate(start)[0];b=a.copy();b[1,3]+=.02
        curve=CartesianCurve([a,b]);_,greedy=sample_ik(curve,'tool_axis',start,solver,lambda q:True,lambda:False)
        def finish(s,q):
            # Simulate rejecting a previously selected configuration. Require
            # another endpoint, then check real FK and JTC timing, not a stub.
            if np.max(abs(q[-1]-greedy[-1]))<.025:raise ValueError('Original configuration excluded')
            return timed_trace(curve,s,q,solver,limits,'tool_axis',.005,.05,[False])
        s,q,(trajectory,summary),report=select_trace(curve,'tool_axis',start,solver,lambda q:True,lambda:False,finish)
        self.assertEqual(report['method'],'whole_stroke_beam')
        self.assertGreater(report['complete_candidates'],1)
        np.testing.assert_array_equal(q[0],start)
        self.assertGreater(np.max(abs(q[-1]-greedy[-1])),.025)
        self.assertGreater(seconds(trajectory.joint_trajectory.points[-1]),0.)

    def test_speed_law_constant_plateau_and_smooth_off_surface_ramps(self):
        law=SpeedLaw(.1,.02,.05)
        self.assertEqual(law.at(0.),(0.,0.,0.));self.assertAlmostEqual(law.at(law.duration)[0],.1)
        self.assertAlmostEqual(law.at(law.duration/2)[1],.02)
        self.assertAlmostEqual(law.at(law.ramp_time)[2],0.)
        for s in np.linspace(0,.1,25):self.assertAlmostEqual(law.at(law.time_at(s))[0],s,places=9)

    def test_free_axis_curve_does_not_lock_pen_twist(self):
        _,solver=calibrated_solver();a=solver.evaluate([.1,-.25,.5,-.25,.4,.4])[0];b=a.copy();b[1,3]+=.02
        from scipy.spatial.transform import Rotation
        b[:3,:3]=b[:3,:3]@Rotation.from_euler('z',140,degrees=True).as_matrix()
        curve=CartesianCurve([a,b]);mid=curve.evaluate(.01,'tool_axis')
        np.testing.assert_allclose(mid[:3,2],a[:3,2],atol=1e-12)
        self.assertGreater(np.linalg.norm(curve.evaluate(.01,'full')[:3,0]-mid[:3,0]),.1)

    def test_contact_trace_has_nonzero_speed_through_interior_waypoints(self):
        _,solver=calibrated_solver();q=np.array([.1,-.25,.5,-.25,.4,.4]);a=solver.evaluate(q)[0]
        transforms=[]
        for y in (0.,.005,.015,.02):
            t=a.copy();t[1,3]+=y;transforms.append(t)
        curve=CartesianCurve(transforms);distances,positions=sample_ik(curve,'tool_axis',q,solver,lambda q:True,lambda:False)
        limits={n:dict(lower=l,upper=h,velocity=.5235987756,acceleration=3.141592654) for n,l,h in zip(__import__('pinchy_bringup.control_contract',fromlist=['JOINTS']).JOINTS,solver.lower,solver.upper)}
        trajectory,summary=timed_trace(curve,distances,positions,solver,limits,'tool_axis',.005,.05,[False,True,False])
        self.assertGreater(summary['modeled_contact_min_m_s'],.00475)
        self.assertLess(summary['modeled_contact_max_m_s'],.00525)
        self.assertAlmostEqual(summary['transit_time_scale'],1.,places=8)
        self.assertTrue(any(np.linalg.norm(p.velocities)>.001 for p in trajectory.joint_trajectory.points[1:-1]))
        with self.assertRaisesRegex(ValueError,'lead-in/out'):
            timed_trace(curve,distances,positions,solver,limits,'tool_axis',.005,.05,[True,True,True])

    def test_explicit_smooth_curve_preserves_waypoints_and_contact_speed(self):
        _,solver=calibrated_solver();q=np.array([.1,-.25,.5,-.25,.4,.4]);origin=solver.evaluate(q)[0];transforms=[]
        for x,y in [(0.,0.),(.005,.005),(.01,.015),(.015,.02)]:
            t=origin.copy();t[0,3]+=x;t[1,3]+=y;transforms.append(t)
        curve=CartesianCurve(transforms,'spline')
        for s,t in zip(curve.waypoint_distances,transforms):np.testing.assert_allclose(curve.evaluate(s,'tool_axis')[:3,3],t[:3,3],atol=1e-8)
        distances,positions=sample_ik(curve,'tool_axis',q,solver,lambda q:True,lambda:False)
        from pinchy_bringup.control_contract import JOINTS
        limits={n:dict(lower=l,upper=h,velocity=.5235987756,acceleration=3.141592654) for n,l,h in zip(JOINTS,solver.lower,solver.upper)}
        _,summary=timed_trace(curve,distances,positions,solver,limits,'tool_axis',.005,.05,[False,True,False])
        self.assertGreater(summary['modeled_contact_min_m_s'],.00475);self.assertLess(summary['modeled_contact_max_m_s'],.00525)

    def test_nearby_regular_grid_sample_does_not_flatten_ik_curve(self):
        from test_ik import calibrated_solver
        from pinchy_motion.ik import BoundedIK
        from pinchy_motion.geometry import model_limits
        root,_=calibrated_solver();limits=model_limits(root);solver=BoundedIK(root,limits)
        q=np.array([.09999378674138193,-.25000312209390624,.5000304693843767,-.25002878386615435,.39999136538131635,.39999104075434455])
        ideal=solver.evaluate([.1,-.25,.5,-.25,.4,.4])[0];transforms=[solver.evaluate(q)[0]]
        for distance in (.01,.03,.04):
            target=ideal.copy();target[1,3]+=distance;transforms.append(target)
        curve=CartesianCurve(transforms)
        distances,positions=sample_ik(curve,'tool_axis',q,solver,lambda q:True,lambda:False)
        self.assertGreater(np.diff(distances).min(),.00005)
        _,summary=timed_trace(curve,distances,positions,solver,limits,'tool_axis',.01,.05,[False,True,False])
        self.assertGreater(summary['modeled_contact_min_m_s'],.0095)
        self.assertLess(summary['modeled_contact_max_m_s'],.0105)

    def test_sample_merging_prevents_nanosecond_duplicate_intervals(self):
        self.assertEqual(len(separated([0.,.1,.1+1e-12,.2])),3)

    def test_polyline_corners_are_explicit_and_rotation_only_trace_rejected(self):
        a=np.eye(4);b=a.copy();b[0,3]=.01;c=b.copy();c[1,3]=.01
        self.assertEqual(corner_indices([a,b,c]),[1])
        with self.assertRaises(ValueError):CartesianCurve([a,a.copy()])


if __name__=='__main__':unittest.main(verbosity=2)
