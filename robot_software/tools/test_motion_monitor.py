#!/usr/bin/python3
"""Recorder joins independent timestamps without altering control."""
import importlib.util,json,math,time,unittest
from pathlib import Path
from types import SimpleNamespace as NS,MethodType
from unittest.mock import Mock
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
spec=importlib.util.spec_from_file_location('motion_monitor',Path(__file__).resolve().parents[1]/'src/pinchy_bringup/scripts/motion_monitor.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class RecorderTests(unittest.TestCase):
    def recorder(self):
        r=NS(applied=None,servo_goals=None,safety='READY',writer=Mock())
        for name in ('target_snapshot','aligned_snapshot','sample'):setattr(r,name,MethodType(getattr(module.MotionMonitor,name),r))
        return r
    def state(self):
        m=JointTrajectoryControllerState();m.joint_names=['a','b'];m.output.positions=[.2,.3];m.feedback.positions=[.1,.1];m.reference.positions=[.21,.31];m.reference.velocities=[.01,.02];m.speed_scaling_factor=1.;return m
    def test_join_keeps_reference_output_applied_readback_feedback_separate(self):
        r=self.recorder();a=JointState(name=['b','a'],position=[.28,.18]);a.header.stamp.sec=3;r.target_snapshot('applied',a)
        b=JointState(name=['a','b'],position=[.17,.27]);b.header.stamp.sec=2;r.target_snapshot('servo_goals',b)
        r.sample(self.state());v=json.loads(r.writer.info.call_args.args[0])
        self.assertEqual(v['reference_positions'],[.21,.31]);self.assertEqual(v['commanded'],[.2,.3]);self.assertEqual(v['measured'],[.1,.1])
        self.assertEqual(v['adapter_applied']['positions'],[.18,.28]);self.assertEqual(v['servo_goals']['positions'],[.17,.27])
        self.assertEqual(v['adapter_applied']['stamp_ns'],3_000_000_000);self.assertEqual(v['servo_goals']['stamp_ns'],2_000_000_000)
        self.assertGreaterEqual(v['servo_goals']['age_s'],0);self.assertEqual(v['safety'],'READY')
    def test_missing_observation_remains_explicitly_unknown(self):
        r=self.recorder();r.sample(self.state());v=json.loads(r.writer.info.call_args.args[0]);self.assertIsNone(v['adapter_applied']);self.assertIsNone(v['servo_goals'])
    def test_invalid_and_reordered_observations_cannot_replace_newer_values(self):
        r=self.recorder();m=JointState(name=['a'],position=[.2]);m.header.stamp.sec=3;r.target_snapshot('applied',m)
        m.header.stamp.sec=2;m.position=[.7];r.target_snapshot('applied',m);self.assertEqual(r.applied['values']['a'],.2)
        m.header.stamp.sec=4;m.position=[float('nan')];r.target_snapshot('applied',m);self.assertEqual(r.applied['stamp_ns'],3_000_000_000)
        self.assertIsNone(r.aligned_snapshot('applied',['a','b'],time.monotonic()))

if __name__=='__main__':unittest.main()
