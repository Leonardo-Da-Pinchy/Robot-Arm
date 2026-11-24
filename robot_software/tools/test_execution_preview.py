#!/usr/bin/python3
"""Preview reuse is exact and execution still fails closed on fresh checks."""
import copy
import threading
import time
import unittest
from concurrent.futures import Future
from types import MethodType,SimpleNamespace as NS
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from pinchy_motion.server import MotionServer
from pinchy_bringup.control_contract import JOINTS


class PreviewTests(unittest.TestCase):
    def fixture(self):
        s=NS(model='immutable model',model_changed=False,stop_requested=False)
        for name in ('validation_digest','validate_execution_preview','execute'):
            setattr(s,name,MethodType(getattr(MotionServer,name),s))
        tr=RobotTrajectory();tr.joint_trajectory.joint_names=list(JOINTS)
        tr.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.]*6,velocities=[0.]*6)]
        record=dict(trajectory=tr,model=s.model,scene='scene',start=[0.]*6,created=time.monotonic(),
                    target_revision=2,target=NS(pose=None))
        record['validation_digest']=s.validation_digest(tr,'scene')
        return s,record

    def test_exact_copy_reuses_validation(self):
        s,r=self.fixture();s.validate_execution_preview(copy.deepcopy(r),'scene')

    def test_changes_to_any_trajectory_or_environment_reject(self):
        mutations=[lambda s,r:r['trajectory'].joint_trajectory.points[0].positions.__setitem__(1,.01),
                   lambda s,r:r['trajectory'].joint_trajectory.points[0].velocities.__setitem__(1,.01),
                   lambda s,r:setattr(r['trajectory'].joint_trajectory.points[0].time_from_start,'nanosec',1),
                   lambda s,r:r['trajectory'].joint_trajectory.joint_names.reverse(),
                   lambda s,r:setattr(s,'model','another model'),
                   lambda s,r:r.pop('validation_digest')]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                s,r=self.fixture();mutate(s,r)
                with self.assertRaisesRegex(RuntimeError,'preview'):s.validate_execution_preview(r,'scene')
        s,r=self.fixture()
        with self.assertRaisesRegex(RuntimeError,'preview'):s.validate_execution_preview(r,'changed scene')

    def gateway(self):
        s,r=self.fixture();events=[]
        s.operation=threading.Lock();s.plans={'token':r};s.hardware=False;s.actual=dict.fromkeys(JOINTS,0.)
        s.target_editor=NS(revision=2);s.hold=NS(commanded=[0.]*6,safety='READY');s.scene_digest=lambda:'scene'
        s.check_ready=lambda:events.append('ready');s.valid=lambda *a,**kw:events.append('actual collision check')
        s.validate_states=lambda _:self.fail('Preview collision checks repeated')
        s.wait_hold=lambda **kw:events.append('powered hold')
        s.get_logger=lambda:NS(info=lambda _:None)
        def send(goal):
            events.append('dispatch');future=Future()
            future.set_result(NS(accepted=False));return future
        s.controller=NS(send_goal_async=send)
        h=NS(request=NS(plan_id='token'),is_cancel_requested=False,
             publish_feedback=lambda _:None,abort=lambda:events.append('abort'),canceled=lambda:events.append('canceled'))
        return s,r,h,events

    def test_unchanged_preview_checks_actual_then_dispatches_without_rechecking_path(self):
        s,_,h,events=self.gateway();s.execute(h)
        self.assertIn('actual collision check',events);self.assertIn('dispatch',events)
        self.assertLess(events.index('actual collision check'),events.index('dispatch'))
        self.assertFalse(s.plans)

    def test_fresh_failures_block_dispatch(self):
        changes=[lambda s,r,h:setattr(s.target_editor,'revision',3),
                 lambda s,r,h:setattr(s,'model','changed'),
                 lambda s,r,h:setattr(s,'scene_digest',lambda:'changed'),
                 lambda s,r,h:s.hold.commanded.__setitem__(0,.02),
                 lambda s,r,h:s.actual.__setitem__(JOINTS[0],.06),
                 lambda s,r,h:r.update(created=0),
                 lambda s,r,h:r.pop('validation_digest')]
        for change in changes:
            with self.subTest(change=change):
                s,r,h,events=self.gateway();change(s,r,h);s.execute(h)
                self.assertNotIn('dispatch',events);self.assertIn('abort',events)

    def test_stop_cancel_or_model_change_during_preflight_blocks_dispatch(self):
        for kind in ('stop','cancel','model'):
            with self.subTest(kind=kind):
                s,_,h,events=self.gateway()
                def check(*a,**kw):
                    if kind=='stop':s.stop_requested=True
                    elif kind=='model':s.model_changed=True
                    else:h.is_cancel_requested=True
                s.valid=check;s.execute(h)
                self.assertNotIn('dispatch',events)
                if kind in ('stop','cancel'):self.assertIn('powered hold',events)

    def test_second_scene_and_start_checks_still_block_dispatch(self):
        for kind in ('scene','command','actual','safety','late safety'):
            with self.subTest(kind=kind):
                s,_,h,events=self.gateway();count=0
                def ready():
                    nonlocal count
                    count+=1
                    if count!=(3 if kind=='late safety' else 2):return
                    if kind=='scene':s.scene_digest=lambda:'changed'
                    elif kind=='command':s.hold.commanded[0]=.02
                    elif kind=='actual':s.actual[JOINTS[0]]=.06
                    else:raise RuntimeError('Safety hold')
                s.check_ready=ready;s.execute(h);self.assertNotIn('dispatch',events)


if __name__=='__main__':unittest.main()
