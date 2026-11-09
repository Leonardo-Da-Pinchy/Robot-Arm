#!/usr/bin/env python3
"""Compare the runtime URDF with the supplied MuJoCo source at 107 poses.

Optional dependencies are listed in cad/cad4/validation_requirements.txt.
No robot, ROS graph, physical serial port, or source export is modified.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from yourdfpy import URDF


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1] / 'src/pinchy_description'
    cad = package / 'cad/cad4'
    source = ET.parse(cad / 'source/cad4_model.xml').getroot()
    source.remove(source.find('asset'))
    for body in source.findall('.//body'):
        for geom in body.findall('geom'):
            body.remove(geom)
    reference = mujoco.MjModel.from_xml_string(ET.tostring(source, encoding='unicode'))
    data = mujoco.MjData(reference)
    arm = URDF.load(str(package / 'urdf/pinchy_arm.urdf'), load_meshes=False,
                    load_collision_meshes=False)
    names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'pen_pitch']
    assert arm.actuated_joint_names == names and reference.nq == 6
    body_links = dict(base='base_link', shoulder='shoulder_link', upper_arm='upper_arm_link',
                      lower_arm='forearm_link', wrist='wrist_pitch_link', gripper='wrist_roll_link',
                      pen_holder='pen_holder_link')
    root = ET.parse(package / 'urdf/pinchy_arm.urdf').getroot()
    limits = np.array([[float(root.find(f"joint[@name='{name}']/limit").get(bound))
                       for bound in ['lower', 'upper']] for name in names])
    poses = [np.zeros(6), limits[:, 0], limits[:, 1]]
    for degrees in [0, 45, 90, 100]:
        pose = np.zeros(6); pose[5] = math.radians(degrees); poses.append(pose)
    poses.extend(np.random.default_rng(4).uniform(limits[:, 0], limits[:, 1], (100, 6)))
    max_position = max_rotation = 0.0
    for pose in poses:
        data.qpos[:] = pose; mujoco.mj_forward(reference, data)
        arm.update_cfg(dict(zip(names, pose)))
        for body, link in body_links.items():
            actual = arm.get_transform(link)
            expected = data.body(body)
            max_position = max(max_position, float(np.linalg.norm(actual[:3, 3] - expected.xpos)))
            max_rotation = max(max_rotation, float(np.max(np.abs(actual[:3, :3] - expected.xmat.reshape(3, 3)))))
        tip = arm.get_transform('pen_tip'); expected = data.site('pen_tip')
        max_position = max(max_position, float(np.linalg.norm(tip[:3, 3] - expected.xpos)))
        max_rotation = max(max_rotation, float(np.max(np.abs(tip[:3, :3] - expected.xmat.reshape(3, 3)))))
    assert max_position < 1e-10 and max_rotation < 1e-10, (max_position, max_rotation)
    cad_errors = {}
    readbacks = json.loads((cad / 'source/cad4_holder_measurements.json').read_text())['configurations']
    for config in ['Q6_0_Set', 'Q6_45_Set', 'Q6_100_Set']:
        arm.update_cfg(dict(zip(names, np.array([0.0]*5 + [math.radians(readbacks[config]['q6_deg'])]))))
        tip = arm.get_transform('pen_tip', 'wrist_roll_link')[:3, 3] * 1000
        error = float(np.linalg.norm(tip - readbacks[config]['pen_tip_mm']))
        assert error < 0.001, (config, error)
        cad_errors[config] = error
    arm.update_cfg(dict(zip(names, np.zeros(6))))
    report = dict(status='PASS', poses_checked_against_mujoco=len(poses),
                  maximum_position_difference_m=max_position,
                  maximum_rotation_matrix_element_difference=max_rotation,
                  solidworks_tip_readback_error_mm=cad_errors,
                  base_link_to_pen_tip_at_all_zero_m=arm.get_transform('pen_tip').tolist(),
                  spring_compression_mm=5, self_collision_checking=False,
                  versions=dict(mujoco=mujoco.__version__, numpy=np.__version__))
    text = json.dumps(report, indent=2)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True); args.output.write_text(text)
    print(text)


if __name__ == '__main__':
    main()
