#!/usr/bin/python3
"""Exact broad-arc regressions using saved bounds and default joint-rate caps.

Numerical only. Independent MoveIt collision/FK/approach evidence is recorded
by check_stroke_selection.py --moveit; these tests never access a motor port.
"""
import time
import unittest
import numpy as np

from test_ik import calibrated_solver
from pinchy_bringup.control_contract import JOINTS,DEFAULT_SPEED,DEFAULT_ACCELERATION
from pinchy_motion.feasibility import CylinderArc
from pinchy_motion.geometry import model_limits
from pinchy_motion.paths import sample_ik,select_trace,timed_trace,TraceIncomplete
from pinchy_motion.stroke_search import select_stroke


class WholeStrokeTests(unittest.TestCase):
    def problem(self,cx,z):
        root,solver=calibrated_solver();limits=model_limits(root)
        for name,l,h in zip(JOINTS,solver.lower,solver.upper):
            limits[name].update(lower=l,upper=h,velocity=DEFAULT_SPEED,acceleration=DEFAULT_ACCELERATION)
        curve=CylinderArc(cx,.04,z,60)
        start=solver.search(curve.evaluate(0.),np.zeros(6),'tool_axis',budget=5.)
        self.assertIsNotNone(start)
        began=time.monotonic()
        def stopped():return time.monotonic()-began>90
        def finish(d,p):
            original=curve.waypoint_distances
            curve.waypoint_distances=np.array([0.,.1*curve.length,.9*curve.length,curve.length])
            try:return timed_trace(curve,d,p,solver,limits,'tool_axis',.003,.02,[False,True,False])
            finally:curve.waypoint_distances=original
        return curve,solver,start,stopped,finish

    def check_complete(self,curve,solver,d,p,result):
        self.assertEqual(d[0],0.);self.assertEqual(d[-1],curve.length)
        self.assertTrue(np.all(np.diff(d)>0));self.assertLessEqual(np.max(abs(np.diff(p,axis=0))),.15)
        self.assertTrue(np.all(p>=solver.lower));self.assertTrue(np.all(p<=solver.upper))
        for s,q in zip(d,p):
            distance,angle=solver.errors(q,curve.evaluate(s),'tool_axis')
            self.assertLessEqual(distance,1e-6);self.assertLessEqual(angle,1e-5)
        trajectory,summary=result
        self.assertGreater(summary['modeled_contact_min_m_s'],.00285)
        self.assertLess(summary['modeled_contact_max_m_s'],.00315)
        self.assertAlmostEqual(summary['transit_time_scale'],1.,places=8)

    def test_529_percent_failure_selects_a_complete_alternative_start(self):
        curve,solver,start,stop,finish=self.problem(.4,.2)
        with self.assertRaises(TraceIncomplete):sample_ik(curve,'tool_axis',start,solver,lambda q:True,stop)
        d,p,result,report=select_stroke(curve,'tool_axis',start,solver,lambda q:True,stop,finish)
        self.check_complete(curve,solver,d,p,result)
        self.assertGreater(np.max(abs(p[0]-start)),.05)
        self.assertGreaterEqual(report['starting_configurations'],1)
        self.assertIn(report.get('seed_method',report['method']),('whole_stroke_start_search','whole_stroke_bidirectional'))
        self.assertTrue(report['rejections'])

    def test_wrist_bound_failure_is_fixed_without_changing_held_start(self):
        curve,solver,start,stop,finish=self.problem(.35,.15)
        d,p=sample_ik(curve,'tool_axis',start,solver,lambda q:True,stop)
        with self.assertRaisesRegex(ValueError,'bounds'):finish(d,p)
        d,p,result,report=select_trace(curve,'tool_axis',start,solver,lambda q:True,stop,finish)
        self.check_complete(curve,solver,d,p,result)
        np.testing.assert_array_equal(p[0],start)
        self.assertEqual(report['method'],'whole_stroke_optimization')


if __name__=='__main__':unittest.main(verbosity=2)
