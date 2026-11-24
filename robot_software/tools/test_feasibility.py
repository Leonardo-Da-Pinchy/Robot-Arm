#!/usr/bin/python3
"""Feasibility diagnostic contracts; no ROS launch or motor connection."""
import math
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from test_ik import calibrated_solver
from pinchy_bringup.control_contract import JOINTS
from pinchy_motion.feasibility import axis_frame,CylinderArc,posture_metrics,twist_family,speed_envelope

class FeasibilityTests(unittest.TestCase):
    def test_wrist_bound_never_rejects_fk_generated_reachable_poses(self):
        from pinchy_motion.feasibility import wrist_reach_bound
        _,solver=calibrated_solver()
        for q in np.random.default_rng(121).uniform(solver.lower,solver.upper,(100,6)):
            result=wrist_reach_bound(solver,solver.evaluate(q)[0])
            self.assertFalse(result['certainly_outside_wrist_reach'])
        self.assertAlmostEqual(result['axial_tool_offset_m'],.149,places=9)
        self.assertAlmostEqual(result['perpendicular_tool_offset_m'],.0188,places=9)

    def test_current_cylinder_far_side_exceeds_even_unrestricted_link_reach(self):
        from pinchy_motion.feasibility import wrist_reach_bound,axis_frame
        from pinchy_motion.nominal_paths import surface_pose
        _,solver=calibrated_solver()
        for angle in (75.,90.):
            p,d=surface_pose('cylinder',[.4,.1,.22],.04*np.radians(angle),0.)
            result=wrist_reach_bound(solver,axis_frame(p,d))
            self.assertTrue(result['certainly_outside_wrist_reach']);self.assertGreater(result['excess_m'],.01)

    def test_direction_frame_is_right_handed_and_normalized(self):
        for d in ([1,0,0],[0,0,1],[0,0,-3],[1,2,3]):
            frame=axis_frame([.3,0,.2],d);np.testing.assert_allclose(frame[:3,:3].T@frame[:3,:3],np.eye(3),atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(frame[:3,:3]),1.)
            np.testing.assert_allclose(frame[:3,2],np.array(d)/np.linalg.norm(d))
        with self.assertRaises(ValueError):axis_frame([0,0,0],[0,0,0])

    def test_cylinder_front_is_near_side_and_pen_points_inward(self):
        curve=CylinderArc(.35,.04,.15,90)
        first,front,last=[curve.evaluate(s) for s in (0,curve.length/2,curve.length)]
        np.testing.assert_allclose(front[:3,3],[.31,0,.15],atol=1e-12)
        np.testing.assert_allclose(front[:3,2],[1,0,0],atol=1e-12)
        for t in (first,front,last):
            outward=t[:3,3]-[.35,0,.15]
            np.testing.assert_allclose(outward/.04,-t[:3,2],atol=1e-12)

    def test_arc_parameter_is_true_surface_arc_length(self):
        curve=CylinderArc(.35,.04,.15,60)
        for distance in np.linspace(.001,curve.length-.001,9):
            derivative=(curve.evaluate(distance+1e-6)[:3,3]-curve.evaluate(distance-1e-6)[:3,3])/2e-6
            self.assertAlmostEqual(np.linalg.norm(derivative),1.,places=7)

    def test_fixed_task_twist_family_preserves_task_and_bounds(self):
        _,solver=calibrated_solver();q=np.array([.1,-.25,.5,-.25,.4,.4]);target=solver.evaluate(q)[0]
        lower,upper=solver.lower.copy(),solver.upper.copy()
        family=twist_family(solver,target,q,steps=5)
        self.assertGreater(len(family),2)
        for candidate in family:
            distance,angle=solver.errors(candidate,target,'tool_axis')
            self.assertLess(distance,1e-6);self.assertLess(angle,1e-5)
            self.assertTrue(np.all(np.array(candidate)>=lower));self.assertTrue(np.all(np.array(candidate)<=upper))
        np.testing.assert_array_equal(solver.lower,lower);np.testing.assert_array_equal(solver.upper,upper)

    def test_alternative_twist_has_real_joint_reserve(self):
        _,solver=calibrated_solver();q=np.array([.1,-.25,.5,-.25,solver.upper[4]-.01,.4])
        family=twist_family(solver,solver.evaluate(q)[0],q,steps=10)
        before=posture_metrics(solver,q)['nearest_limit_deg']
        after=max(posture_metrics(solver,p)['nearest_limit_deg'] for p in family)
        self.assertGreater(after,before+1.)

    def test_trajectory_reserve_includes_between_waypoint_extrema(self):
        from pinchy_motion.geometry import trajectory_reserve
        from moveit_msgs.msg import RobotTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        trajectory=RobotTrajectory();trajectory.joint_trajectory.joint_names=JOINTS
        first=JointTrajectoryPoint(positions=[0.]*6,velocities=[4.,0.,0.,0.,0.,0.])
        last=JointTrajectoryPoint(positions=[0.]*6,velocities=[-4.,0.,0.,0.,0.,0.]);last.time_from_start.sec=1
        trajectory.joint_trajectory.points=[first,last]
        limits={n:dict(lower=-1.2,upper=1.2) for n in JOINTS}
        reserve=trajectory_reserve(trajectory,limits)
        self.assertEqual(reserve['joint'],JOINTS[0]);self.assertAlmostEqual(reserve['margin_deg'],math.degrees(.2),places=8)

    def test_incomplete_trace_exposes_exact_target_for_independent_check(self):
        from pinchy_motion.paths import sample_ik,TraceIncomplete
        class NoSolution:
            def search(self,*args,**kwargs):return None
        curve=CylinderArc(.35,.04,.15,1.)
        with self.assertRaises(TraceIncomplete) as caught:
            sample_ik(curve,'tool_axis',[0.]*6,NoSolution(),lambda q:True,lambda:False)
        failure=caught.exception
        self.assertEqual(failure.last_distance,0.);self.assertGreater(failure.distance,0.)
        np.testing.assert_allclose(failure.target,curve.evaluate(failure.distance))
        self.assertIn('No partial plan retained',str(failure))

    def test_runtime_trace_diagnostic_distinguishes_other_configuration(self):
        from types import SimpleNamespace
        from unittest.mock import Mock,patch
        from geometry_msgs.msg import PoseStamped
        from pinchy_motion.paths import TraceIncomplete
        from pinchy_motion.server import MotionServer
        from pinchy_motion.geometry import model_limits
        root,solver=calibrated_solver();limits=model_limits(root)
        for i,name in enumerate(JOINTS):limits[name].update(lower=solver.lower[i],upper=solver.upper[i])
        target=solver.evaluate([.1,-.25,.5,-.25,.4,.4])[0]
        failure=TraceIncomplete('incomplete trace',.005,.004,[0.]*6,target)
        fake=SimpleNamespace(ik_solver=solver,root=root,limits=limits,stop_requested=False,model_changed=False,
            request_id='test',planning_step=Mock(),position_diagnostic_valid=lambda q:True,get_logger=lambda:Mock())
        pose=PoseStamped();pose.pose.position.x=.3;pose.pose.position.z=.2;pose.pose.orientation.w=1.
        with patch('pinchy_motion.server.select_trace',side_effect=failure):
            with self.assertRaisesRegex(RuntimeError,'DOES have collision-checked IK in another configuration'):
                MotionServer.trace_paths(fake,[0.]*6,[pose],'tool_axis','polyline',[False],.01,.05,1.,1.)

    def test_pen_up_pose_routing_retries_another_endpoint_configuration(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from pinchy_motion.server import MotionServer
        answers=iter([[.1]*6,[.2]*6]);excluded=[]
        def solve(target,start,mode,exclude):
            excluded.append([list(q) for q in exclude]);return next(answers)
        fake=SimpleNamespace(stop_requested=False,model_changed=False,solve_ik=solve,
            pose_from_joints=lambda q:q,plan_transit=Mock(side_effect=[RuntimeError('first branch blocked'),'checked route']))
        goal,pose,trajectory=MotionServer.route_to_pose(fake,'target',[0.]*6,'tool_axis',.5,.5)
        self.assertEqual(goal,[.2]*6);self.assertEqual(trajectory,'checked route')
        self.assertEqual(excluded,[[],[[.1]*6]])

    def test_pen_up_pose_routing_does_not_retry_after_stop(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from pinchy_motion.server import MotionServer
        fake=SimpleNamespace(stop_requested=False,model_changed=False,solve_ik=Mock(return_value=[.1]*6),pose_from_joints=lambda q:q)
        def stop(*args):fake.stop_requested=True;raise RuntimeError('Stopped')
        fake.plan_transit=stop
        with self.assertRaisesRegex(RuntimeError,'Stopped'):MotionServer.route_to_pose(fake,'target',[0.]*6,'tool_axis',.5,.5)
        self.assertEqual(fake.solve_ik.call_count,1)

    def test_sampled_speed_capacity_matches_linear_joint_path(self):
        distances=np.linspace(0,.04,20);q=np.zeros((20,6));q[:,0]=2*distances
        limits={n:dict(velocity=.5,acceleration=3.) for n in JOINTS}
        result=speed_envelope(distances,q,limits,.1,.05)
        self.assertAlmostEqual(result['sampled_constant_speed_ceiling_m_s'],.25,places=8)
        self.assertTrue(result['requested_speed_within_sampled_envelope'])
        self.assertAlmostEqual(result['worst_joint_velocity_rad_s'],.2)
        self.assertFalse(speed_envelope(distances,q,limits,.3,.05)['requested_speed_within_sampled_envelope'])

if __name__=='__main__':unittest.main(verbosity=2)
