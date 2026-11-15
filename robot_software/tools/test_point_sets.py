#!/usr/bin/python3
"""Reusable point-set files and staging-only panel behavior; no motor access."""
import copy
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.context import Context
from python_qt_binding.QtWidgets import QApplication
from ament_index_python.packages import get_package_share_directory
from pinchy_motion import point_sets
from pinchy_motion.panel import PosePanel

EXAMPLES=Path(get_package_share_directory('pinchy_motion'))/'examples/point_sets'


class PointSetTests(unittest.TestCase):
    def sample(self):return point_sets.read(EXAMPLES/'01_fixed_tcp_tilts.json')

    def test_roundtrip_preserves_geometry_and_settings_without_execution_token(self):
        value=self.sample();value['plan_id']='must not survive save'
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'points.json';point_sets.write(path,value);loaded=point_sets.read(path)
            self.assertNotIn('plan_id',loaded);self.assertEqual(loaded['style'],'fixed_tcp')
            self.assertEqual(loaded['joint_speed_deg_s'],5.)
            for first,last in zip(value['waypoints'],loaded['waypoints']):
                np.testing.assert_allclose(first['xyz'],last['xyz'],atol=1e-12)
                np.testing.assert_allclose(Rotation.from_euler('xyz',first['rpy_deg'],degrees=True).as_matrix(),
                    Rotation.from_euler('xyz',last['rpy_deg'],degrees=True).as_matrix(),atol=1e-12)

    def test_invalid_file_cannot_replace_a_saved_set(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'points.json';point_sets.write(path,self.sample());original=path.read_bytes()
            invalid=self.sample();invalid['waypoints'][1]['xyz'][0]+=1
            with self.assertRaises(ValueError):point_sets.write(path,invalid)
            self.assertEqual(path.read_bytes(),original)

    def test_invalid_geometry_modes_units_and_settings_are_rejected(self):
        for mutate in [lambda d:d.update(units={'position':'mm','orientation':'deg'}),
                lambda d:d.update(frame='camera'),lambda d:d.update(orientation_mode='tool_axis'),
                lambda d:d.update(joint_speed_deg_s=float('nan')),lambda d:d.update(velocity_scaling=.1),
                lambda d:d['waypoints'][0].update(contact=True),lambda d:d.update(schema_version=99)]:
            value=self.sample();mutate(value)
            with self.subTest(value=value),self.assertRaises(ValueError):point_sets.normalize(value)

    def test_captured_queue_contains_the_seven_original_points(self):
        value=point_sets.read(EXAMPLES/'03_captured_session_points.json')
        self.assertEqual(len(value['waypoints']),7);self.assertEqual(value['style'],'visits')
        self.assertEqual(value['waypoints'][0]['xyz'],[.405899,.049812,.246919])
        self.assertEqual(value['waypoints'][-1]['xyz'],[.455855,-.00124,.247397])


class PointSetPanelTests(unittest.TestCase):
    def setUp(self):
        self.app=QApplication.instance() or QApplication([]);self.context=Context();rclpy.init(context=self.context,domain_id=220)
        self.node=rclpy.create_node('saved_point_set_panel_test',context=self.context);self.window=PosePanel(self.node)
        self.window.timer.stop();self.window.planner=Mock();self.window.path_planner=Mock();self.window.execution=Mock()

    def tearDown(self):
        self.window.close();self.window.executor.shutdown();self.app.processEvents();self.node.destroy_node();self.context.shutdown()

    def test_load_roundtrip_requires_new_preview_and_never_executes(self):
        w=self.window;value=point_sets.read(EXAMPLES/'01_fixed_tcp_tilts.json');w.plan_id='old token'
        w.apply_point_set(value)
        self.assertFalse(w.plan_id);self.assertEqual(w.queue.rowCount(),7)
        self.assertEqual(w.orientation_mode.currentData(),'full');self.assertEqual(w.path_style.currentData(),'fixed_tcp')
        self.assertEqual(w.speed.value(),5.)
        w.planner.call_async.assert_not_called();w.path_planner.call_async.assert_not_called();w.execution.send_goal_async.assert_not_called()
        saved=w.queue_document()
        for first,last in zip(value['waypoints'],saved['waypoints']):np.testing.assert_allclose(first['xyz'],last['xyz'],atol=1e-12)
        original=copy.deepcopy(saved);bad=copy.deepcopy(value);bad['waypoints'][-1]['xyz'][0]=float('nan')
        with self.assertRaises(ValueError):w.apply_point_set(bad)
        self.assertEqual(w.queue_document(),original)

    def test_capture_uses_current_feedback_not_goal_fields_and_publishes_snapshot(self):
        w=self.window;actual=[.35,.02,.25,10.,20.,30.]
        w.status_data={'current_pose_xyz_rpy_deg':actual};w.last_status=time.monotonic()
        w.inputs[0].setValue(.5);w.capture_current_waypoint();self.assertEqual(w.queue.rowCount(),1)
        self.assertEqual(float(w.queue.item(0,0).text()),.35)
        w.queue_pub=Mock();w.publish_queue_snapshot();snapshot=json.loads(w.queue_pub.publish.call_args.args[0].data)
        self.assertTrue(snapshot['queue_valid']);self.assertEqual(snapshot['waypoints'][0]['xyz'],actual[:3])
        w.execution.send_goal_async.assert_not_called();w.path_planner.call_async.assert_not_called()


if __name__=='__main__':unittest.main(verbosity=2)
