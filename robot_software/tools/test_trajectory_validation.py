#!/usr/bin/python3
"""Batched collision transport retains every sample and fails closed."""
import unittest
from concurrent.futures import Future
from types import MethodType,SimpleNamespace
from pinchy_motion.server import MotionServer
from pinchy_bringup.control_contract import JOINTS


class ValidationTests(unittest.TestCase):
    def server(self,invalid=None,pending=False):
        server=SimpleNamespace(stop_requested=False,model_changed=False,
            limits={n:dict(lower=-1.,upper=1.) for n in JOINTS})
        for name in ('check_bounds','robot_state','validate_states'):
            setattr(server,name,MethodType(getattr(MotionServer,name),server))
        server.check_validity_response=MotionServer.check_validity_response
        futures=[];seen=[]
        def call(request):
            q=request.robot_state.joint_state.position;seen.append(q[0]);future=Future();futures.append(future)
            if pending:server.stop_requested=True
            else:future.set_result(SimpleNamespace(valid=q[0]!=invalid,contacts=[]))
            return future
        server.validity=SimpleNamespace(call_async=call)
        return server,seen,futures

    def test_every_state_including_last_partial_batch_is_checked(self):
        server,seen,_=self.server();server.validate_states(([i/100.]*6 for i in range(79)))
        self.assertEqual(seen,[i/100. for i in range(79)])

    def test_collision_in_middle_rejects_before_next_batch(self):
        server,seen,_=self.server(invalid=.4)
        with self.assertRaisesRegex(RuntimeError,'Collision'):server.validate_states(([i/100.]*6 for i in range(79)))
        self.assertEqual(len(seen),44)

    def test_bounds_do_not_inherit_feedback_allowance(self):
        server,seen,_=self.server()
        with self.assertRaisesRegex(ValueError,'bounds'):server.validate_states([[1.00001]*6])
        self.assertFalse(seen)

    def test_stop_cancels_pending_service_requests(self):
        server,_,futures=self.server(pending=True)
        with self.assertRaisesRegex(RuntimeError,'Stopped'):server.validate_states([[0.]*6]*64)
        self.assertTrue(futures);self.assertTrue(all(f.cancelled() for f in futures))


if __name__=='__main__':unittest.main()
