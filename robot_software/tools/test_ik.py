#!/usr/bin/python3
"""Independent FK/Jacobian and known-reachable bounded-IK regression tests."""
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from pinchy_motion.ik import BoundedIK,SearchStopped
from pinchy_motion.geometry import forward_kinematics,model_limits
from pinchy_bringup.control_contract import JOINTS
ROOT=Path(__file__).resolve().parents[1]


def calibrated_solver():
    root=ET.parse(ROOT/'src/pinchy_description/urdf/pinchy_arm.urdf').getroot()
    limits=model_limits(root);profile=json.loads((ROOT/'calibration/arm.json').read_text())
    for n in JOINTS:limits[n].update(lower=profile['joints'][n]['min_radians'],upper=profile['joints'][n]['max_radians'])
    return root,BoundedIK(root,limits)


class IKTests(unittest.TestCase):
    def test_cached_chain_matches_existing_full_tree_fk(self):
        root,solver=calibrated_solver()
        for q in np.random.default_rng(51).uniform(solver.lower,solver.upper,(20,6)):
            np.testing.assert_allclose(solver.evaluate(q)[0],forward_kinematics(root,dict(zip(JOINTS,q))),atol=1e-12)

    def test_analytic_jacobian_matches_finite_difference_in_all_modes(self):
        _,solver=calibrated_solver();q=(solver.lower+solver.upper)/2
        target=solver.evaluate(q+.01)[0]
        for mode in ('full','tool_axis','position'):
            _,analytic=solver.system(q,target,mode)
            numeric=np.column_stack([(solver.system(q+np.eye(6)[i]*1e-6,target,mode)[0]-solver.system(q-np.eye(6)[i]*1e-6,target,mode)[0])/2e-6 for i in range(6)])
            np.testing.assert_allclose(analytic,numeric,atol=1e-8)

    def test_uncached_known_reachable_full_poses_including_joint_boundaries(self):
        root,solver=calibrated_solver();rng=np.random.default_rng(87)
        states=[np.zeros(6),solver.lower,solver.upper,*rng.uniform(solver.lower,solver.upper,(37,6))]
        for i,q in enumerate(states):
            # Targets come from separate full-tree FK, not the new search's FK.
            target=forward_kinematics(root,dict(zip(JOINTS,q)))
            _,fresh=calibrated_solver();answer=fresh.search(target,np.zeros(6),budget=5.)
            self.assertIsNotNone(answer,f'known reachable case {i}: {fresh.last_report}')
            distance,angle=fresh.errors(answer,target,'full')
            self.assertLessEqual(distance,.0001);self.assertLessEqual(angle,.001)

    def test_moveit_twist_candidates_preserve_axis_and_ignore_input_twist(self):
        from pinchy_motion.ik import free_twist_frames
        _,solver=calibrated_solver();target=solver.evaluate([.1,-.2,.4,-.2,.3,.5])[0]
        rotated=target.copy();rotated[:3,:3]=target[:3,:3]@Rotation.from_euler('z',123,degrees=True).as_matrix()
        reference=solver.evaluate(np.zeros(6))[0][:3,:3]
        first=list(free_twist_frames(target,reference));second=list(free_twist_frames(rotated,reference))
        self.assertEqual(len(first),24)
        for a,b in zip(first,second):
            np.testing.assert_allclose(a[:3,2],target[:3,2],atol=1e-12)
            np.testing.assert_allclose(a,b,atol=1e-12)
        self.assertGreater(np.linalg.norm(first[0][:3,0]-first[12][:3,0]),.1)

    def test_operator_second_example_succeeds_with_pen_axis_freedom(self):
        _,solver=calibrated_solver();target=np.eye(4);target[:3,3]=[.333508,.000154,.125493]
        target[:3,:3]=Rotation.from_euler('xyz',[-5.159,-62.390,-173.985],degrees=True).as_matrix()
        answer=solver.search(target,np.zeros(6),'tool_axis')
        self.assertIsNotNone(answer,solver.last_report)
        position,angle=solver.errors(answer,target,'tool_axis')
        self.assertLess(position,.0001);self.assertLess(angle,.001)
        # It really is an explicit relaxation, not a solver that ignores full RPY.
        self.assertGreater(solver.errors(answer,target,'full')[1],.001)

    def test_position_only_does_not_require_requested_orientation(self):
        _,solver=calibrated_solver();target=np.eye(4);target[:3,3]=[.395763,-.058652,.129172]
        target[:3,:3]=Rotation.from_euler('xyz',[-120.927,-86.75,-85.681],degrees=True).as_matrix()
        answer=solver.search(target,np.zeros(6),'position')
        self.assertIsNotNone(answer,solver.last_report);self.assertLess(solver.errors(answer,target,'position')[0],.0001)

    def test_collision_filter_cannot_be_bypassed_by_exact_fk_seed(self):
        _,solver=calibrated_solver();target=solver.evaluate(np.zeros(6))[0]
        self.assertIsNone(solver.search(target,np.zeros(6),valid=lambda q:False,budget=.2,attempts=3))
        self.assertGreater(solver.last_report['rejected_candidates'],0)

    def test_stop_and_budget_bound_the_search(self):
        _,solver=calibrated_solver();target=np.eye(4);target[:3,3]=[.9,.9,.9]
        with self.assertRaises(SearchStopped):solver.search(target,np.zeros(6),stop=lambda:True)
        self.assertIsNone(solver.search(target,np.zeros(6),budget=.01))
        self.assertEqual(solver.last_report['reason'],'time budget')

    def test_conditioning_uses_the_selected_task_dimension(self):
        _,solver=calibrated_solver()
        axis=solver.conditioning(np.zeros(6),'tool_axis');position=solver.conditioning(np.zeros(6),'position')
        self.assertEqual(axis['task_dimensions'],5);self.assertEqual(position['task_dimensions'],3)
        self.assertGreater(axis['smallest_singular_value'],.02)
        self.assertFalse(axis['near_singular'])

    def test_near_zero_has_poorer_conditioning_than_bent_wrist(self):
        _,solver=calibrated_solver()
        zero=solver.conditioning(np.zeros(6));bent=solver.conditioning([.1,-.3,.6,-.3,.6,.3])
        self.assertTrue(zero['near_singular']);self.assertGreater(bent['smallest_singular_value'],zero['smallest_singular_value'])


if __name__=='__main__':unittest.main(verbosity=2)
