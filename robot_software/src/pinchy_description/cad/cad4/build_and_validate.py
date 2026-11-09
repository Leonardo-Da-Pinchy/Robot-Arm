"""Build a transform-only URDF from the CAD 4 model and check it independently."""
from pathlib import Path
import csv
import hashlib
import json
import math
import shutil
import warnings
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from yourdfpy import URDF

ROOT = Path(__file__).resolve().parents[1]
CAD = Path('C:/Users/theso/Documents/LDP_SO101_Arm')
OUT = ROOT / 'outputs/iteration_2_cad_4_urdf'
if (Path(__file__).resolve().parent / 'source').is_dir():
    OUT = Path(__file__).resolve().parent
OUT.mkdir(parents=True, exist_ok=True)
SOURCE = OUT / 'source'
SOURCE.mkdir(exist_ok=True)
files = {
    'cad4_model.xml': 'analysis/joint6/so101_joint6.xml',
    'cad4_parameters.json': 'analysis/joint6/joint6_params.json',
    'cad4_holder_measurements.json': 'cad/joint6_pen/checks/holder_asm_report.json',
    'cad4_assembly_measurements.json': 'cad/joint6_pen/checks/arm_asm_report.json',
    'base_registration.json': 'cad/checks/pose_and_axes.json',
    'cad_joint_axes.json': 'cad/checks/urdf_axes_zero_pose.json',
}
manifest = {}
for name, relative in files.items():
    if not (SOURCE / name).exists():
        shutil.copy2(CAD / relative, SOURCE / name)
    manifest[name] = {'source': relative, 'sha256': hashlib.sha256((SOURCE / name).read_bytes()).hexdigest()}
(SOURCE / 'provenance.json').write_text(json.dumps(manifest, indent=2) + '\n')

source = ET.parse(SOURCE / 'cad4_model.xml').getroot()
bodies = list(source.find('worldbody').iter('body'))
names = {
    'base': 'base_link', 'shoulder': 'shoulder_link',
    'upper_arm': 'upper_arm_link', 'lower_arm': 'forearm_link',
    'wrist': 'wrist_pitch_link', 'gripper': 'wrist_roll_link',
    'pen_holder': 'pen_holder_link',
}

def numbers(text):
    return np.array([float(value) for value in text.split()])

def formatted(values):
    return ' '.join(format(float(value), '.15g') for value in values)

robot = ET.Element('robot', name='iteration_2_cad_4_arm')
robot.append(ET.Comment(' CAD 4 NoClaw, 2026-10-05 review fixes. Metres and radians. Kinematics only. '))
robot.append(ET.Comment(' Six revolute joints; pen_tip fixed at 5 mm sleeve compression. See README.txt. '))
robot.append(ET.Comment(' Effort and velocity are zero placeholders: this file does not specify motor control limits. '))
for body in bodies:
    ET.SubElement(robot, 'link', name=names[body.get('name')])
ET.SubElement(robot, 'link', name='pen_tip')
rows = []
for parent in bodies:
    for body in parent.findall('body'):
        source_joint = body.find('joint')
        assert source_joint.get('type') == 'hinge'
        assert np.allclose(numbers(source_joint.get('pos', '0 0 0')), 0)
        assert float(source_joint.get('ref', '0')) == 0
        name = source_joint.get('name')
        if name == 'gripper':
            name = 'pen_pitch'
        quaternion = numbers(body.get('quat', '1 0 0 0'))
        rotation = Rotation.from_quat(quaternion[[1, 2, 3, 0]])
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', UserWarning)
            rpy = rotation.as_euler('xyz')
        xyz = numbers(body.get('pos', '0 0 0'))
        axis = numbers(source_joint.get('axis'))
        lower, upper = numbers(source_joint.get('range'))
        joint = ET.SubElement(robot, 'joint', name=name, type='revolute')
        ET.SubElement(joint, 'parent', link=names[parent.get('name')])
        ET.SubElement(joint, 'child', link=names[body.get('name')])
        ET.SubElement(joint, 'origin', xyz=formatted(xyz), rpy=formatted(rpy))
        ET.SubElement(joint, 'axis', xyz=formatted(axis))
        ET.SubElement(joint, 'limit', lower=formatted([lower]), upper=formatted([upper]), effort='0', velocity='0')
        rows.append([name, names[parent.get('name')], names[body.get('name')], *xyz, *rpy, *axis, lower, upper])

tip = bodies[-1].find("site[@name='pen_tip']")
assert tip.get('pos') == '0 -0.149 0.0188'
assert tip.get('zaxis') == '0 -1 0'
joint = ET.SubElement(robot, 'joint', name='pen_tip_fixed', type='fixed')
ET.SubElement(joint, 'parent', link='pen_holder_link')
ET.SubElement(joint, 'child', link='pen_tip')
ET.SubElement(joint, 'origin', xyz=tip.get('pos'), rpy=formatted([math.pi / 2, 0, 0]))
ET.indent(robot, space='  ')
path = OUT / 'iteration_2_cad_4_arm.urdf'
ET.ElementTree(robot).write(path, encoding='utf-8', xml_declaration=True)
with (OUT / 'joint_transforms.csv').open('w', newline='') as handle:
    writer = csv.writer(handle)
    writer.writerow(['joint', 'parent', 'child', 'x_m', 'y_m', 'z_m', 'roll_rad', 'pitch_rad', 'yaw_rad', 'axis_x', 'axis_y', 'axis_z', 'lower_rad', 'upper_rad'])
    writer.writerows(rows)
    writer.writerow(['pen_tip_fixed', 'pen_holder_link', 'pen_tip', 0, -0.149, 0.0188, math.pi / 2, 0, 0, '', '', '', '', ''])

# MuJoCo computes the original model's FK; meshes are irrelevant to transforms.
# Strip geometry only in memory so validation needs no external mesh files.
source.remove(source.find('asset'))
for body in bodies:
    for geom in body.findall('geom'):
        body.remove(geom)
reference = mujoco.MjModel.from_xml_string(ET.tostring(source, encoding='unicode'))
data = mujoco.MjData(reference)
urdf = URDF.load(str(path), build_scene_graph=True, load_meshes=False)
assert urdf.base_link == 'base_link'
assert urdf.actuated_joint_names == [row[0] for row in rows]
assert len(urdf.robot.links) == 8 and len(urdf.robot.joints) == 7
assert reference.nq == 6
limits = np.array([row[-2:] for row in rows])
poses = [np.zeros(6), limits[:, 0], limits[:, 1]]
for angle in [0, 45, 90, 100]:
    q = np.zeros(6)
    q[5] = math.radians(angle)
    poses.append(q)
poses.extend(np.random.default_rng(4).uniform(limits[:, 0], limits[:, 1], (100, 6)))
max_position = 0.0
max_rotation = 0.0
for q in poses:
    data.qpos[:] = q
    mujoco.mj_forward(reference, data)
    urdf.update_cfg(dict(zip(urdf.actuated_joint_names, q)))
    for body_name, link in names.items():
        expected = data.body(body_name)
        actual = urdf.get_transform(link)
        max_position = max(max_position, float(np.linalg.norm(actual[:3, 3] - expected.xpos)))
        max_rotation = max(max_rotation, float(np.max(np.abs(actual[:3, :3] - expected.xmat.reshape(3, 3)))))
    expected = data.site('pen_tip')
    actual = urdf.get_transform('pen_tip')
    max_position = max(max_position, float(np.linalg.norm(actual[:3, 3] - expected.xpos)))
    max_rotation = max(max_rotation, float(np.max(np.abs(actual[:3, :3] - expected.xmat.reshape(3, 3)))))
assert max_position < 1e-10, max_position
assert max_rotation < 1e-10, max_rotation

# Independent CAD readback: pen tips measured in the wrist-roll frame by SolidWorks.
measurements = json.loads((SOURCE / 'cad4_holder_measurements.json').read_text())
cad_errors = {}
for config in ['Q6_0_Set', 'Q6_45_Set', 'Q6_100_Set']:
    measurement = measurements['configurations'][config]
    q = np.zeros(6)
    q[5] = math.radians(measurement['q6_deg'])
    urdf.update_cfg(dict(zip(urdf.actuated_joint_names, q)))
    actual = urdf.get_transform('pen_tip', 'wrist_roll_link')[:3, 3] * 1000
    error = float(np.linalg.norm(actual - measurement['pen_tip_mm']))
    assert error < 0.001, (config, error)
    cad_errors[config] = error

urdf.update_cfg(np.zeros(6))
registration = json.loads((SOURCE / 'base_registration.json').read_text())
assembly_from_base = np.array(registration['T_asm_link_step_pose']['base_link'])
assembly_from_base[:3, 3] /= 1000
axes = json.loads((SOURCE / 'cad_joint_axes.json').read_text())
axis_errors = {}
for row, axis in zip(rows, axes):
    assert axis['joint'] == row[0] or (axis['joint'] == 'gripper' and row[0] == 'pen_pitch')
    transform = assembly_from_base @ urdf.get_transform(row[2])
    position_error = float(np.linalg.norm(transform[:3, 3] * 1000 - axis['point_mm']))
    direction_error = float(np.linalg.norm(transform[:3, :3] @ np.array(row[9:12]) - axis['dir']))
    assert position_error < 0.02, (row[0], position_error)
    assert direction_error < 0.0001, (row[0], direction_error)
    axis_errors[row[0]] = {'position_error_mm': position_error, 'axis_vector_error': direction_error}
assert len(axis_errors) == 6
report = {
    'status': 'PASS', 'revolute_joints': urdf.actuated_joint_names,
    'fixed_tip_frame': 'pen_tip', 'poses_checked_against_mujoco': len(poses),
    'maximum_position_difference_m': max_position,
    'maximum_rotation_matrix_element_difference': max_rotation,
    'solidworks_tip_readback_error_mm': cad_errors,
    'cad_zero_pose_axis_errors': axis_errors,
    'base_link_to_pen_tip_at_all_zero_m': urdf.get_transform('pen_tip').tolist(),
    'assembly_from_base_link_m': assembly_from_base.tolist(),
    'registration_note': 'Existing CAD mesh registration; rotation and translation rounded in source. Not a physical calibration.',
    'versions': {'mujoco': mujoco.__version__, 'numpy': np.__version__},
}
(OUT / 'transform_validation.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
