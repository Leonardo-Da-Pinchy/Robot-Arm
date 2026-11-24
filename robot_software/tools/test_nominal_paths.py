#!/usr/bin/python3
"""Known-surface dry runs: dimensions, normals, stand-off and strict speed intent."""
import math
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pinchy_motion.nominal_paths import dry_run_loop
from pinchy_motion.point_sets import normalize


class NominalPathTests(unittest.TestCase):
    def test_cylinder_loop_spans_two_inches_and_preserves_normal_and_gap(self):
        data=normalize(dry_run_loop());points=data['waypoints'];xyz=np.array([p['xyz'] for p in points]);center=np.array(data['source']['center_m'])
        angle=np.arctan2(xyz[:,1]-center[1],center[0]-xyz[:,0]);u=angle*.04
        self.assertAlmostEqual(np.ptp(u),.0508,places=10);self.assertAlmostEqual(np.ptp(xyz[:,2]),.0508,places=10)
        np.testing.assert_allclose(np.linalg.norm(xyz[:,:2]-center[:2],axis=1),.045,atol=1e-12)
        for p,theta in zip(points,angle):
            direction=Rotation.from_euler('xyz',p['rpy_deg'],degrees=True).as_matrix()[:,2]
            np.testing.assert_allclose(direction,[math.cos(theta),-math.sin(theta),0.],atol=1e-12)
        self.assertTrue(data['enforce_tcp_speed']);self.assertTrue(data['plan_pen_up_approach']);self.assertEqual(data['style'],'spline')
        self.assertFalse(any(p['contact'] for p in points));self.assertEqual(data['tcp_speed_m_s'],.003)

    def test_fast_cylinder_is_larger_faster_and_keeps_the_nominal_object(self):
        from pinchy_motion.nominal_paths import fast_cylinder_loop
        from pinchy_motion.point_sets import read
        data=normalize(fast_cylinder_loop());center=np.array(data['source']['center_m'])
        xyz=np.array([p['xyz'] for p in data['waypoints']]);u=.04*np.arctan2(xyz[:,1]-center[1],center[0]-xyz[:,0])
        self.assertAlmostEqual(np.ptp(u),.0762,places=10);self.assertAlmostEqual(np.ptp(xyz[:,2]),.0762,places=10)
        self.assertAlmostEqual(float((u.max()+u.min())/2),-.04*np.pi/12,places=10)
        self.assertGreater(data['tcp_speed_m_s'],2*dry_run_loop()['tcp_speed_m_s'])
        self.assertEqual(data['source']['nominal_radius_m'],.04);self.assertEqual(data['style'],'spline_axis')
        self.assertTrue(data['enforce_tcp_speed']);self.assertFalse(any(p['contact'] for p in data['waypoints']))
        np.testing.assert_allclose(np.linalg.norm(xyz[:,:2]-center[:2],axis=1),.045,atol=1e-12)
        root=Path(__file__).resolve().parents[1]/'src/pinchy_motion/examples/point_sets'
        saved=read(root/'07_cylinder_continuous_3in_fast.json')
        self.assertEqual(data['source'],saved['source']);self.assertEqual(data['style'],saved['style'])
        for first,last in zip(data['waypoints'],saved['waypoints']):
            np.testing.assert_allclose(first['xyz'],last['xyz'],atol=1e-14,rtol=0.)
            np.testing.assert_allclose(Rotation.from_euler('xyz',first['rpy_deg'],degrees=True).as_matrix(),Rotation.from_euler('xyz',last['rpy_deg'],degrees=True).as_matrix(),atol=1e-14,rtol=0.)

    def test_packaged_files_match_the_qualified_generator_settings_and_poses(self):
        from pinchy_motion.point_sets import read
        root=Path(__file__).resolve().parents[1]/'src/pinchy_motion/examples/point_sets'
        for kind,name in [('cylinder','04_cylinder_continuous_2in.json'),('plane','05_plane_continuous_2in.json'),('quarter_pipe','06_quarter_pipe_continuous_2in.json')]:
            expected=normalize(dry_run_loop(kind));saved=read(root/name)
            for key,value in expected.items():
                if key not in ('notes','waypoints'):self.assertEqual(saved[key],value,(name,key))
            self.assertEqual(len(saved['waypoints']),len(expected['waypoints']))
            for first,last in zip(saved['waypoints'],expected['waypoints']):
                np.testing.assert_allclose(first['xyz'],last['xyz'],atol=1e-14,rtol=0.)
                np.testing.assert_allclose(Rotation.from_euler('xyz',first['rpy_deg'],degrees=True).as_matrix(),
                    Rotation.from_euler('xyz',last['rpy_deg'],degrees=True).as_matrix(),atol=1e-14,rtol=0.)
                self.assertEqual(first['contact'],last['contact'])

    def test_concave_pipe_normal_points_into_wall_not_outside(self):
        data=normalize(dry_run_loop('quarter_pipe'));center=np.array(data['source']['center_m'])
        for p in data['waypoints']:
            radial=np.array(p['xyz'])-center;radial[2]=0
            self.assertAlmostEqual(np.linalg.norm(radial),.095,places=12)
            direction=Rotation.from_euler('xyz',p['rpy_deg'],degrees=True).as_matrix()[:,2]
            np.testing.assert_allclose(direction,radial/.095,atol=1e-12)

    def test_boundary_cases_cover_both_edges_and_height_without_weakening_motion(self):
        from pinchy_motion.nominal_paths import coverage_strokes
        for kind in ('cylinder','plane','quarter_pipe'):
            cases=coverage_strokes(kind);self.assertEqual(len(cases),3)
            for case in cases:
                data=normalize(case);self.assertTrue(data['enforce_tcp_speed']);self.assertTrue(data['plan_pen_up_approach'])
                self.assertEqual(data['tcp_speed_m_s'],.003);self.assertFalse(any(p['contact'] for p in data['waypoints']))
                self.assertEqual(len(data['waypoints']),31)
            low,vertical,high=[np.array([p['xyz'] for p in c['waypoints']]) for c in cases]
            np.testing.assert_allclose(low[:,2],.175);np.testing.assert_allclose(high[:,2],.265)
            self.assertAlmostEqual(np.ptp(vertical[:,2]),.09)
            np.testing.assert_allclose(low[:,:2],high[::-1,:2],atol=1e-14)
            if kind=='cylinder':
                self.assertAlmostEqual(np.degrees(cases[0]['source']['surface_u_interval_m'][0]/.04),-75.)
                self.assertAlmostEqual(np.degrees(cases[0]['source']['surface_u_interval_m'][1]/.04),45.)

    def test_surface_measurement_recovers_generated_normal_and_air_gap(self):
        from pinchy_motion.nominal_paths import surface_pose,surface_measurement
        for kind in ('cylinder','plane','quarter_pipe'):
            for u in (-.0254,0.,.0254):
                p,normal=surface_pose(kind,[.4,.1,.22],u,-.03,.005)
                measured,gap=surface_measurement(kind,[.4,.1,.22],p)
                np.testing.assert_allclose(measured,normal,atol=1e-12);self.assertAlmostEqual(gap,.005,places=12)
        with self.assertRaises(ValueError):surface_measurement('cylinder',[0,0,0],[0,0,0])

    def test_plane_is_upright_and_normal_is_constant(self):
        data=normalize(dry_run_loop('plane'))
        for p in data['waypoints']:
            self.assertAlmostEqual(p['xyz'][0],.355)
            np.testing.assert_allclose(Rotation.from_euler('xyz',p['rpy_deg'],degrees=True).as_matrix()[:,2],[1,0,0],atol=1e-12)


if __name__=='__main__':unittest.main(verbosity=2)
