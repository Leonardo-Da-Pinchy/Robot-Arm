#!/usr/bin/python3
"""Focused goal-direction UI regression; isolated ROS, no motor connection."""
import math
import unittest
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.context import Context
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtCore import Qt
from pinchy_motion.panel import PosePanel
from pinchy_motion.geometry import goal_difference

CURRENT=[.4745,-.0017,.2554,-95.21,32.38,-92.76]
GOAL=[.471461,-.003679,.239280,-130.083,-86.586,4.025]

class GoalDirectionTests(unittest.TestCase):
    def test_pen_up_queue_default_and_stale_failure_invalidation(self):
        from types import SimpleNamespace
        app=QApplication.instance() or QApplication([])
        context=Context();rclpy.init(context=context,domain_id=221)
        node=rclpy.create_node('pose_queue_default_test',context=context);window=PosePanel(node)
        try:
            window.timer.stop();self.assertEqual(window.path_style.currentData(),'visits')
            window.planned(SimpleNamespace(result=lambda:SimpleNamespace(success=False,plan_id='',message='old trace failure')))
            self.assertIn('old trace failure',window.notice.text())
            window.inputs[0].setValue(.35)
            self.assertNotIn('old trace failure',window.notice.text())
            self.assertIn('Inputs changed',window.notice.text());self.assertFalse(window.plan_id)
            self.assertFalse(window.execute_button.isEnabled())
            window.add_waypoint();window.queue.item(0,6).setCheckState(Qt.Checked)
            window.inputs[0].setValue(.370924)
            self.assertEqual(float(window.queue.item(0,0).text()),.35)
            window.update_waypoint();self.assertEqual(float(window.queue.item(0,0).text()),.370924)
            self.assertEqual(window.queue.item(0,6).checkState(),Qt.Checked)
            window.busy=True;window.inputs[0].setValue(.4);window.update_waypoint();window.clear_waypoints()
            self.assertEqual(window.queue.rowCount(),1);self.assertEqual(float(window.queue.item(0,0).text()),.370924)
        finally:
            window.target_timer.stop();window.executor.shutdown();window.close();app.processEvents();node.destroy_node();context.shutdown()

    def test_cylinder_file_retains_required_speed_and_visits_clear_it(self):
        from pinchy_motion.nominal_paths import dry_run_loop,fast_cylinder_loop
        app=QApplication.instance() or QApplication([])
        context=Context();rclpy.init(context=context,domain_id=221)
        node=rclpy.create_node('continuous_speed_file_test',context=context);window=PosePanel(node)
        try:
            window.timer.stop();window.apply_point_set(dry_run_loop())
            self.assertEqual(window.queue.rowCount(),41);self.assertTrue(window.require_speed.isChecked())
            self.assertTrue(window.stroke_approach.isChecked());self.assertEqual(window.tcp_speed.value(),3.)
            self.assertTrue(window.queue_document()['enforce_tcp_speed'])
            window.path_style.setCurrentIndex(window.path_style.findData('visits'))
            self.assertFalse(window.require_speed.isChecked())
            window.apply_point_set(fast_cylinder_loop())
            self.assertEqual(window.queue.rowCount(),61);self.assertEqual(window.tcp_speed.value(),7.)
            self.assertEqual(window.path_style.currentData(),'spline_axis')
            self.assertTrue(window.require_speed.isChecked());self.assertTrue(window.queue_document()['enforce_tcp_speed'])
            self.assertEqual(window.queue_document()['style'],'spline_axis')
        finally:
            window.target_timer.stop();window.queue_timer.stop();window.executor.shutdown();window.close();app.processEvents();node.destroy_node();context.shutdown()

    def test_range_load_roundtrip_request_and_visits_reset(self):
        from pinchy_motion.nominal_paths import bounded_cylinder_loop
        app=QApplication.instance() or QApplication([])
        context=Context();rclpy.init(context=context,domain_id=221)
        node=rclpy.create_node('bounded_speed_panel_test',context=context);window=PosePanel(node)
        try:
            window.timer.stop();self.assertFalse(window.speed_range.isChecked())
            window.apply_point_set(bounded_cylinder_loop())
            self.assertTrue(window.speed_range.isChecked());self.assertEqual(window.tcp_min_speed.value(),4.)
            self.assertEqual(window.tcp_speed.value(),18.);self.assertIn('max',window.tcp_speed.suffix())
            self.assertEqual(window.queue_document()['tcp_min_speed_m_s'],.004)
            window.status_data={'ready':True,'max_speed_deg_s':90.,'max_acceleration_deg_s2':360.};window.path_planner=Mock();window.execution=Mock()
            window.plan_queued();request=window.path_planner.call_async.call_args.args[0]
            self.assertAlmostEqual(request.tcp_min_speed,.004);self.assertAlmostEqual(request.tcp_speed,.018)
            window.execution.send_goal_async.assert_not_called()
            window.busy=False;window.path_style.setCurrentIndex(window.path_style.findData('visits'))
            self.assertFalse(window.speed_range.isChecked());self.assertEqual(window.queue_document()['tcp_min_speed_m_s'],0.)
        finally:
            window.busy=False;window.target_timer.stop();window.queue_timer.stop();window.executor.shutdown();window.close();app.processEvents();node.destroy_node();context.shutdown()

    def test_screenshot_is_nearby_in_xyz_but_large_direction_change(self):
        distance,angle=goal_difference(CURRENT,GOAL)
        self.assertAlmostEqual(distance*1000,16.52,delta=.1)
        self.assertAlmostEqual(math.degrees(angle),53.90148,places=4)

    def test_difference_ignores_rotation_about_pen_axis(self):
        current=Rotation.from_euler('xyz',CURRENT[3:],degrees=True)
        twist=(current*Rotation.from_euler('z',90,degrees=True)).as_euler('xyz',degrees=True)
        distance,angle=goal_difference(CURRENT,[*CURRENT[:3],*twist])
        self.assertEqual(distance,0.);self.assertLess(angle,1e-7)

    def test_copy_direction_preserves_xyz_updates_marker_input_and_never_plans(self):
        app=QApplication.instance() or QApplication([])
        context=Context();rclpy.init(context=context,domain_id=221)
        node=rclpy.create_node('pose_direction_ui_test',context=context)
        window=PosePanel(node)
        try:
            self.assertEqual([window.speed.value(),window.acceleration.value(),window.tcp_speed.value(),window.tcp_acceleration.value()],[30.,180.,20.,50.])
            self.assertEqual([window.speed.maximum(),window.acceleration.maximum(),window.tcp_speed.maximum(),window.tcp_acceleration.maximum()],[92.14,398.5,400.,2000.])
            window.timer.stop();window.status_data={'current_pose_xyz_rpy_deg':CURRENT}
            for box,value in zip(window.inputs,GOAL):box.setValue(value)
            window.planner=Mock();window.execution=Mock();window.target_pub=Mock()
            window.update_current_display()
            self.assertIn('53.9°',window.current.text())
            xyz=[box.value() for box in window.inputs[:3]]
            window.copy_current_direction();window.target_timer.stop();window.publish_target()
            self.assertEqual([box.value() for box in window.inputs[:3]],xyz)
            np.testing.assert_allclose([box.value() for box in window.inputs[3:]],CURRENT[3:],atol=.001)
            self.assertIn('0.0°',window.current.text())
            window.target_pub.publish.assert_called_once()
            window.planner.call_async.assert_not_called();window.execution.send_goal_async.assert_not_called()
            window.orientation_mode.setCurrentIndex(2)
            self.assertNotIn('direction differs',window.current.text())
            window.status_data['joint_limits_deg']={'wrist_roll':{'current':9.8,'minimum':-10.,'maximum':10.}}
            window.update_current_display();self.assertIn('Low joint travel reserve: wrist_roll 0.2°',window.current.text())
            window.show_limits();self.assertEqual(window.limit_table.columnCount(),5)
            self.assertEqual(window.limit_table.item(0,4).text(),'+0.20')
        finally:
            window.target_timer.stop();window.executor.shutdown();window.close();app.processEvents();node.destroy_node();context.shutdown()

if __name__=='__main__':unittest.main(verbosity=2)
