#!/usr/bin/python3
"""Focused separation of measured settling margin from strict motion bounds."""
import unittest
from types import SimpleNamespace
from pinchy_motion.server import MotionServer
from pinchy_bringup.control_contract import JOINTS,FEEDBACK_BOUND_TOLERANCE


class FeedbackBoundaryTests(unittest.TestCase):
    def fixture(self,collision=False):
        calls=[]
        model=SimpleNamespace(limits={n:dict(lower=0.,upper=1.) for n in JOINTS},validity=object(),
            robot_state=lambda q:q,call=lambda client,request:calls.append(request) or SimpleNamespace(valid=not collision,contacts=[]))
        return model,calls

    def test_one_tick_negative_measurement_is_checked_for_collisions_without_clamping(self):
        model,calls=self.fixture();q=[0.]*6;q[4]=-FEEDBACK_BOUND_TOLERANCE/10
        MotionServer.valid(model,q,measured=True)
        self.assertEqual(calls[-1].robot_state,q)
        with self.assertRaises(ValueError):MotionServer.valid(model,q)

    def test_more_than_feedback_margin_is_rejected(self):
        model,calls=self.fixture();q=[0.]*6;q[4]=-FEEDBACK_BOUND_TOLERANCE-1e-5
        with self.assertRaises(ValueError):MotionServer.valid(model,q,measured=True)
        self.assertFalse(calls)

    def test_measured_allowance_never_bypasses_collision_failure(self):
        model,calls=self.fixture(collision=True);q=[0.]*6;q[4]=-FEEDBACK_BOUND_TOLERANCE/10
        with self.assertRaises(RuntimeError):MotionServer.valid(model,q,measured=True)
        self.assertTrue(calls)


if __name__=='__main__':unittest.main(verbosity=2)
