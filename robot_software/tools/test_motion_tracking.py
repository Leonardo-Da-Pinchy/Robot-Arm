#!/usr/bin/python3
import unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))
from analyze_motion_tracking import boundary,analyze
from pinchy_bringup.control_contract import JOINTS

class BoundaryTests(unittest.TestCase):
    def test_missing_layer_is_unknown_not_blame_on_motors(self):
        self.assertEqual(boundary({}, {},10.),'adapter_applied_unobserved_or_stale')
        self.assertEqual(boundary(dict(adapter_applied=1.),dict(adapter_applied=10.),10.),'servo_goals_unobserved_or_stale')
    def test_distinguishes_driver_target_goal_register_and_feedback_layers(self):
        a=dict(adapter_applied=1.,servo_goals=1.)
        self.assertEqual(boundary(a,dict(adapter_applied=0.,servo_goals=0.),10.),'adapter_targets_not_advancing')
        self.assertEqual(boundary(a,dict(adapter_applied=10.,servo_goals=0.),10.),'servo_goal_readback_not_advancing')
        self.assertEqual(boundary(a,dict(adapter_applied=10.,servo_goals=10.),10.),'feedback_not_following_advancing_servo_goals')
    def test_full_recording_analysis_localizes_frozen_feedback_and_rejects_stale_readback(self):
        model=(Path(__file__).resolve().parents[1]/'src/pinchy_description/urdf/pinchy_arm.urdf').read_text()
        rows=[]
        for i in range(31):
            stamp=1_000_000_000+i*20_000_000;q=[.01*i,0.,0.,0.,0.,0.]
            rows.append(dict(stamp_ns=stamp,joints=JOINTS,commanded=q,measured=[0.]*6,scaling=1.,safety='READY',
                adapter_applied=dict(positions=q,stamp_ns=stamp,age_s=0.),servo_goals=dict(positions=q,stamp_ns=stamp,age_s=0.)))
        events=analyze(rows,model);self.assertTrue(events)
        self.assertTrue(all(e['boundary']=='feedback_not_following_advancing_servo_goals' for e in events))
        for r in rows:r['servo_goals']['age_s']=1.
        self.assertTrue(all(e['boundary']=='servo_goals_unobserved_or_stale' for e in analyze(rows,model)))

    def test_explicit_hold_not_classified_as_tracking_failure(self):
        self.assertEqual(boundary({},dict(scaling=0.),10.),'intentional_pause_or_safety_hold')
        self.assertEqual(boundary({},dict(safety='HOLD: example'),10.),'intentional_pause_or_safety_hold')

if __name__=='__main__':unittest.main()
