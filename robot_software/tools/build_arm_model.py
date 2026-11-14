#!/usr/bin/env python3
"""Merge supplied CAD 4 kinematics with its provenance-matched MJCF geometry.

Source exports stay unchanged. Runtime velocity is a software setting, not a
motor rating. Effort remains unspecified. Collision geometry is not avoidance.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1] / 'src/pinchy_description'
BODY_LINKS = dict(base='base_link', shoulder='shoulder_link', upper_arm='upper_arm_link',
                 lower_arm='forearm_link', wrist='wrist_pitch_link', gripper='wrist_roll_link',
                 pen_holder='pen_holder_link')
SOFTWARE_VELOCITY = math.radians(30)  # Software ceiling; physical qualification remains separate.


def values(text):
    return [float(v) for v in text.split()]


def numbers(vector):
    return ' '.join(format(v, '.15g') for v in vector)


def quaternion_rpy(text='1 0 0 0'):
    w, x, y, z = values(text)
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = [v/norm for v in (w, x, y, z)]
    r00, r10, r20 = 1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)
    r11, r12 = 1-2*(x*x+z*z), 2*(y*z-w*x)
    r21, r22 = 2*(y*z+w*x), 1-2*(x*x+y*y)
    cp = math.hypot(r00, r10)
    if cp < 1e-12:
        return [math.atan2(-r12, r11), math.atan2(-r20, cp), 0]
    return [math.atan2(r21, r22), math.atan2(-r20, cp), math.atan2(r10, r00)]


def add_shape(link, tag, name, position, rpy, shape, rgba=None, **dimensions):
    item = ET.SubElement(link, tag, name=name)
    ET.SubElement(item, 'origin', xyz=numbers(position), rpy=numbers(rpy))
    geometry = ET.SubElement(item, 'geometry')
    ET.SubElement(geometry, shape, **{key: str(value) for key, value in dimensions.items()})
    if tag == 'visual':
        material = ET.SubElement(item, 'material', name=f'{name}_color')
        ET.SubElement(material, 'color', rgba=rgba or '0.2 0.55 0.95 1')


def add_geometry(link, geom, index, materials, assets):
    tag = geom.get('class')
    if tag not in ('visual', 'collision'):
        return
    name = f"{link.get('name')}_{geom.get('name') or geom.get('mesh') or index}_{tag}"
    shape = geom.get('type')
    position = values(geom.get('pos', '0 0 0'))
    rpy = quaternion_rpy(geom.get('quat', '1 0 0 0'))
    rgba = geom.get('rgba') or materials.get(geom.get('material'))
    if shape == 'mesh':
        if geom.get('mesh') == 'wrist_roll_follower_so101_noclaw':
            path, scale = 'meshes/cad4/wrist_roll_follower_so101_noclaw.stl', '1 1 1'
        else:
            path, scale = 'meshes/' + assets[geom.get('mesh')], '1 1 1'
        if not (PACKAGE / path).is_file():
            raise FileNotFoundError(path)
        add_shape(link, tag, name, position, rpy, 'mesh', rgba,
                  filename=f'package://pinchy_description/{path}', scale=scale)
    elif shape == 'box':
        add_shape(link, tag, name, position, rpy, 'box', rgba,
                  size=numbers([2*v for v in values(geom.get('size'))]))
    elif shape in ('cylinder', 'capsule'):
        radius = values(geom.get('size'))[0]
        endpoints = values(geom.get('fromto')); start, end = endpoints[:3], endpoints[3:]
        direction = [b-a for a, b in zip(start, end)]
        length = math.sqrt(sum(v*v for v in direction))
        position = [(a+b)/2 for a, b in zip(start, end)]
        rpy = [0, math.acos(direction[2]/length), math.atan2(direction[1], direction[0])]
        add_shape(link, tag, name, position, rpy, 'cylinder', rgba, radius=radius, length=length)
        if shape == 'capsule':  # URDF capsule equivalent: cylinder + spherical ends.
            for suffix, point in [('start', start), ('end', end)]:
                add_shape(link, tag, name+'_'+suffix, point, [0, 0, 0], 'sphere', rgba, radius=radius)
    else:
        raise ValueError(f'Unsupported source geometry: {shape}')


def build():
    cad = PACKAGE / 'cad/cad4'
    provenance = json.loads((cad / 'source/provenance.json').read_text())
    for filename, record in provenance.items():
        if hashlib.sha256((cad / 'source' / filename).read_bytes()).hexdigest() != record['sha256']:
            raise ValueError(f'CAD source hash mismatch: {filename}')
    root = ET.parse(cad / 'iteration_2_cad_4_arm.urdf').getroot()
    root.set('name', 'pinchy_arm')
    for limit in root.findall("joint[@type='revolute']/limit"):
        limit.set('velocity', str(SOFTWARE_VELOCITY))
    mjcf = ET.parse(cad / 'source/cad4_model.xml').getroot()
    materials = {m.get('name'): m.get('rgba') for m in mjcf.findall('./asset/material')}
    assets = {m.get('name', Path(m.get('file')).stem): m.get('file') for m in mjcf.findall('./asset/mesh')}
    for body in mjcf.findall('.//body'):
        link = root.find(f"link[@name='{BODY_LINKS[body.get('name')]}']")
        inertial = body.find('inertial')
        if inertial is not None:
            item = ET.SubElement(link, 'inertial')
            ET.SubElement(item, 'origin', xyz=inertial.get('pos'),
                          rpy=numbers(quaternion_rpy(inertial.get('quat', '1 0 0 0'))))
            ET.SubElement(item, 'mass', value=inertial.get('mass'))
            coefficients = values(inertial.get('fullinertia'))
            ET.SubElement(item, 'inertia', **dict(zip(
                ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'], map(str, coefficients))))
        for index, geom in enumerate(body.findall('geom')):
            add_geometry(link, geom, index, materials, assets)
            if geom.get('class') == 'visual' and (geom.get('type') == 'box' or body.get('name') == 'base'):
                collision = deepcopy(geom); collision.set('class', 'collision')
                add_geometry(link, collision, index, materials, assets)
    # Explicit active TCP, separate from the CAD pen reference and motor origin.
    profile = json.loads((PACKAGE / 'config/active_tool.json').read_text())
    attachment = profile['attachment']
    if root.find(f"link[@name='{attachment['parent_frame']}']") is None:
        raise ValueError('Active tool parent is missing from model')
    for key in ('xyz_m', 'rpy_rad'):
        vector = attachment[key]
        if len(vector) != 3 or any(not math.isfinite(value) for value in vector):
            raise ValueError('Active tool requires finite three-component transforms')
    tcp = ET.SubElement(root, 'link', name=profile['tcp_frame'])
    add_shape(tcp, 'visual', 'active_tcp_marker', [0, 0, 0], [0, 0, 0],
              'sphere', '0 1 1 1', radius='0.002')
    fixed = ET.SubElement(root, 'joint', name='active_tcp_fixed', type='fixed')
    ET.SubElement(fixed, 'parent', link=attachment['parent_frame'])
    ET.SubElement(fixed, 'child', link=profile['tcp_frame'])
    ET.SubElement(fixed, 'origin', xyz=numbers(attachment['xyz_m']), rpy=numbers(attachment['rpy_rad']))
    ET.SubElement(root, 'link', name='tool0')
    tool = ET.SubElement(root, 'joint', name='tool0_joint', type='fixed')
    ET.SubElement(tool, 'parent', link='pen_holder_link')
    ET.SubElement(tool, 'child', link='tool0')
    ET.SubElement(tool, 'origin', xyz='0 0 0', rpy='0 0 0')
    ET.indent(root, space='  ')
    destination = PACKAGE / 'urdf/pinchy_arm.urdf'
    destination.write_text('<?xml version="1.0"?>\n'
        '<!-- Generated by tools/build_arm_model.py from supplied CAD 4 URDF and MJCF.\n'
        '     Pen tip assumes nominal 5 mm sleeve compression. Velocity 30 deg/s is a\n'
        '     software setting; effort 0 remains unspecified. Geometry alone does not\n'
        '     provide collision avoidance. tool0 remains the sixth-axis origin.\n'
        '     pointer_tip is the active RViz endpoint proxy; physical tip is unverified. -->\n'
        + ET.tostring(root, encoding='unicode') + '\n')
    print(destination)


if __name__ == '__main__':
    build()
