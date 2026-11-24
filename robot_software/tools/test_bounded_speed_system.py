#!/usr/bin/python3
"""Bounded-speed gateway on isolated mocks only; never a physical execute client."""
import json,unittest
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from pinchy_motion_interfaces.srv import PlanPath
from test_cartesian import CartesianFixture,JOINTS

class BoundedGateway(CartesianFixture,unittest.TestCase):
    def request(self):
        q=[.1,-.25,.5,-.25,.4,.4]
        start=self.plan(q);self.assertTrue(self.execute(start).result.success)
        a=self.target(q);b=PoseStamped();b.header.frame_id='base_link';b.pose=a.pose
        # ROS messages need independent copies before changing an endpoint.
        import copy
        b=copy.deepcopy(a);b.pose.position.y+=.025
        return PlanPath.Request(waypoints=[a,b],orientation_mode='tool_axis',style='spline_axis',
            tcp_speed=.006,tcp_min_speed=.002,tcp_acceleration=.02,contact_segments=[False,False],
            enforce_tcp_speed=False,velocity_scaling=.25,acceleration_scaling=.4)
    def test_01_range_preview_execute_and_report(self):
        req=self.request();p=self.service(PlanPath,'/pinchy_motion/plan_path',req,seconds=90)
        self.assertTrue(p.success,p.message);self.assertGreaterEqual(p.modeled_cruise_min_speed,.002-1e-7);self.assertLessEqual(p.modeled_cruise_max_speed,.006+1e-7)
        report=json.loads(p.speed_range_report_json);self.assertTrue(report[0]['enabled']);self.assertGreaterEqual(report[0]['modeled_mean_m_s'],.002)
        self.assertTrue(self.execute(p).result.success)
    def test_02_invalid_bands_never_produce_a_plan_or_motion(self):
        req=self.request();before=dict(self.state)
        for value in (.006,.007,-.001,float('nan'),.0005):
            req.tcp_min_speed=value;p=self.service(PlanPath,'/pinchy_motion/plan_path',req)
            self.assertFalse(p.success);self.assertFalse(p.plan_id)
        req.tcp_min_speed=.002;req.style='visits';p=self.service(PlanPath,'/pinchy_motion/plan_path',req)
        self.assertFalse(p.success);self.assertIn('continuous',p.message)
        for n in JOINTS:self.assertAlmostEqual(before[n],self.state[n],delta=.002)
    def test_03_scene_change_rejects_checked_range_preview(self):
        req=self.request();p=self.service(PlanPath,'/pinchy_motion/plan_path',req,seconds=90);self.assertTrue(p.success,p.message)
        self.add_box('range_scene_change',[.8,.8,.8])
        try:
            result=self.execute(p);self.assertFalse(result.result.success);self.assertIn('Scene changed',result.result.message)
        finally:self.add_box('range_scene_change',[0.,0.,0.],remove=True)
    def test_04_cancel_discards_range_and_holds(self):
        req=self.request();p=self.service(PlanPath,'/pinchy_motion/plan_path',req,seconds=90);self.assertTrue(p.success,p.message)
        before=dict(self.state);handle=self.execute(p,wait=False);self.wait_for_motion(before)
        self.assertTrue(self.wait(handle.cancel_goal_async(),5).goals_canceling)
        result=self.wait(handle.get_result_async(),12);self.assertEqual(result.status,5);self.assertFalse(result.result.success)

def load_tests(loader,tests,pattern):
    # The shared fixture also contains legacy tests with their own ordered state
    # assumptions. Keep this independent suite restricted to its four cases.
    names=('test_01_range_preview_execute_and_report','test_02_invalid_bands_never_produce_a_plan_or_motion','test_03_scene_change_rejects_checked_range_preview','test_04_cancel_discards_range_and_holds')
    return unittest.TestSuite(BoundedGateway(name) for name in names)

if __name__=='__main__':unittest.main()
