"""Regression checks against supplied CAD 4 kinematics and SolidWorks readbacks."""
import hashlib
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

PACKAGE = Path(__file__).resolve().parents[1] / 'src/pinchy_description'
CAD = PACKAGE / 'cad/cad4'


def transform(origin):
    result = np.eye(4)
    result[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    result[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return result


def fk(root, positions):
    frames = {'base_link': np.eye(4)}
    joints = list(root.findall('joint'))
    while joints:
        progressed = False
        for joint in list(joints):
            parent = joint.find('parent').get('link')
            if parent not in frames:
                continue
            local = transform(joint.find('origin'))
            if joint.get('type') == 'revolute':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(axis * positions.get(joint.get('name'), 0)).as_matrix()
                local = local @ rotation
            frames[joint.find('child').get('link')] = frames[parent] @ local
            joints.remove(joint); progressed = True
        if not progressed:
            raise ValueError('Disconnected or cyclic model')
    return frames


class ModelTests(unittest.TestCase):
    def setUp(self):
        self.source = ET.parse(CAD / 'iteration_2_cad_4_arm.urdf').getroot()
        self.arm = ET.parse(PACKAGE / 'urdf/pinchy_arm.urdf').getroot()

    def test_all_supplied_joint_transforms_names_and_ranges_preserved(self):
        self.assertEqual(len(self.arm.findall("joint[@type='revolute']")), 6)
        for source in self.source.findall('joint'):
            joint = self.arm.find(f"joint[@name='{source.get('name')}']")
            self.assertIsNotNone(joint)
            for tag in ['parent', 'child', 'origin', 'axis']:
                if source.find(tag) is not None:
                    self.assertEqual(joint.find(tag).attrib, source.find(tag).attrib)
            if source.get('type') == 'revolute':
                for bound in ['lower', 'upper']:
                    self.assertEqual(joint.find('limit').get(bound), source.find('limit').get(bound))
                self.assertAlmostEqual(float(joint.find('limit').get('velocity')), math.radians(30), places=12)

    def test_nominal_zero_tip_matches_supplied_validation(self):
        expected = json.loads((CAD / 'transform_validation.json').read_text())['base_link_to_pen_tip_at_all_zero_m']
        np.testing.assert_allclose(fk(self.arm, {})['pen_tip'], expected, atol=1e-13, rtol=0)
        tip = self.arm.find("joint[@name='pen_tip_fixed']/origin")
        self.assertEqual(tip.get('xyz'), '0 -0.149 0.0188')
        self.assertEqual(self.arm.find("joint[@name='tool0_joint']/parent").get('link'), 'pen_holder_link')

    def test_tip_matches_solidworks_zero_45_and_100_degrees(self):
        configurations = json.loads((CAD / 'source/cad4_holder_measurements.json').read_text())['configurations']
        for name in ['Q6_0_Set', 'Q6_45_Set', 'Q6_100_Set']:
            measurement = configurations[name]
            frames = fk(self.arm, {'pen_pitch': np.deg2rad(measurement['q6_deg'])})
            actual = (np.linalg.inv(frames['wrist_roll_link']) @ frames['pen_tip'])[:3, 3] * 1000
            np.testing.assert_allclose(actual, measurement['pen_tip_mm'], atol=0.001, rtol=0)

    def test_active_tcp_matches_visual_endpoint_and_points_outward(self):
        profile = json.loads((PACKAGE / 'config/active_tool.json').read_text())
        self.assertEqual(profile['tcp_frame'], 'pointer_tip')
        self.assertFalse(profile['physical_tip_verified'])
        visual = self.arm.find("link[@name='pen_holder_link']/visual[@name='pen_holder_link_sharpie_nib_visual']")
        cylinder = visual.find('geometry/cylinder')
        geometry_pose = transform(visual.find('origin'))
        endpoint = geometry_pose @ np.array([0, 0, float(cylinder.get('length'))/2, 1])
        frames = fk(self.arm, {})
        local_tcp = np.linalg.inv(frames['pen_holder_link']) @ frames['pointer_tip']
        np.testing.assert_allclose(local_tcp[:3, 3], endpoint[:3], atol=1e-13, rtol=0)
        np.testing.assert_allclose(local_tcp[:3, 2], geometry_pose[:3, 2], atol=1e-13, rtol=0)
        self.assertIsNotNone(self.arm.find("link[@name='pointer_tip']/visual[@name='active_tcp_marker']"))
        self.assertIsNone(self.arm.find("link[@name='pointer_tip']/collision"))

    def test_active_tcp_fk_and_original_links_at_107_poses(self):
        names = [j.get('name') for j in self.source.findall("joint[@type='revolute']")]
        bounds = np.array([[float(self.source.find(f"joint[@name='{name}']/limit").get(bound))
                            for bound in ('lower', 'upper')] for name in names])
        poses = [np.zeros(6), bounds[:, 0], bounds[:, 1]]
        for angle in (0, 45, 90, 100):
            pose = np.zeros(6); pose[-1] = math.radians(angle); poses.append(pose)
        poses.extend(np.random.default_rng(7).uniform(bounds[:, 0], bounds[:, 1], (100, 6)))
        for pose in poses:
            positions = dict(zip(names, pose))
            frames, original = fk(self.arm, positions), fk(self.source, positions)
            for link in original:
                np.testing.assert_allclose(frames[link], original[link], atol=1e-13, rtol=0)
            np.testing.assert_allclose(frames['pointer_tip'], original['pen_tip'], atol=1e-13, rtol=0)
            np.testing.assert_allclose(frames['tool0'], frames['pen_holder_link'], atol=1e-13, rtol=0)

    def test_mesh_paths_units_and_no_original_claw(self):
        for mesh in self.arm.findall('.//mesh'):
            path = mesh.get('filename').removeprefix('package://pinchy_description/')
            self.assertTrue((PACKAGE / path).is_file(), path)
            self.assertNotIn('moving_jaw', path)
            self.assertNotIn('wrist_roll_follower_so101_v1.stl', path)
            expected_scale = '0.001 0.001 0.001' if '/ee_v1/' in path else '1 1 1'
            self.assertEqual(mesh.get('scale'), expected_scale)

    def test_cad_source_hashes(self):
        manifest = json.loads((CAD / 'source/provenance.json').read_text())
        for filename, record in manifest.items():
            self.assertEqual(hashlib.sha256((CAD / 'source' / filename).read_bytes()).hexdigest(), record['sha256'])

    def test_ee_inertias_and_collision_proxies_are_present(self):
        for link, mass in [('wrist_roll_link', 0.07934), ('pen_holder_link', 0.05233)]:
            element = self.arm.find(f"link[@name='{link}']")
            self.assertAlmostEqual(float(element.find('inertial/mass').get('value')), mass)
            self.assertTrue(element.findall('visual'))
            self.assertTrue(element.findall('collision'))
        self.assertTrue(self.arm.findall("link[@name='pen_holder_link']/collision/geometry/sphere"))

    def test_pen_pitch_cad_soft_limits(self):
        limits = self.arm.find("joint[@name='pen_pitch']/limit")
        self.assertAlmostEqual(np.rad2deg(float(limits.get('lower'))), -3, places=3)
        self.assertAlmostEqual(np.rad2deg(float(limits.get('upper'))), 100, places=3)
        self.assertIsNone(self.arm.find("joint[@name='tool_rotation']"))


if __name__ == '__main__':
    unittest.main()
