#!/usr/bin/python3
"""Nominal scene solids: validation, conservative mesh and closed topology."""
from collections import Counter
import copy
import math
from pathlib import Path
import unittest
import numpy as np
import yaml
from shape_msgs.msg import SolidPrimitive
from pinchy_motion.scene_geometry import scene_objects,quarter_pipe_mesh


class SceneGeometryTests(unittest.TestCase):
    def test_legacy_floor_is_unchanged(self):
        root=Path(__file__).resolve().parents[1]
        data=yaml.safe_load((root/'src/pinchy_moveit_config/config/scene.yaml').read_text())
        obj=scene_objects(data)[0]
        self.assertEqual(obj.id,'example_floor');self.assertEqual(obj.primitives[0].type,SolidPrimitive.BOX)
        self.assertEqual(list(obj.primitives[0].dimensions),[1.2,1.2,.04]);self.assertEqual(obj.primitive_poses[0].position.z,-.04)

    def test_cylinder_dimension_order_and_orientation(self):
        obj=scene_objects({'objects':[dict(id='cylinder',frame='base_link',type='cylinder',center=[.4,.1,.22],radius=.04,height=.10,rpy_deg=[90.,0.,0.])]})[0]
        self.assertEqual(list(obj.primitives[0].dimensions),[.10,.04])
        self.assertAlmostEqual(obj.primitive_poses[0].orientation.x,math.sqrt(.5))

    def test_quarter_pipe_is_closed_and_has_an_open_cavity(self):
        mesh=quarter_pipe_mesh(.1,.1,.004);vertices=np.array([[p.x,p.y,p.z] for p in mesh.vertices]);edges=Counter();volume=0.
        for triangle in mesh.triangles:
            a,b,c=list(triangle.vertex_indices)
            for first,last in ((a,b),(b,c),(c,a)):edges[tuple(sorted((first,last)))]+=1
            volume+=vertices[a]@np.cross(vertices[b],vertices[c])/6
        self.assertTrue(all(count==2 for count in edges.values()));self.assertGreater(volume,0.)
        self.assertTrue(np.all(vertices[:,0]>.07));self.assertGreater(np.min(np.linalg.norm(vertices[:,:2],axis=1)),.099999)
        self.assertLess(np.max(vertices[:,2])-.1/2,1e-12)
        # Conservative inner chords shrink cavity, outer chords enclose the wall.
        for index in range(48):
            inner=(vertices[4*index,:2]+vertices[4*(index+1),:2])/2
            outer=(vertices[4*index+1,:2]+vertices[4*(index+1)+1,:2])/2
            self.assertLess(np.linalg.norm(inner),.1);self.assertGreaterEqual(np.linalg.norm(outer),.104-1e-12)

    def test_proposed_supports_preserve_objects_floor_and_under_object_contact(self):
        root=Path(__file__).resolve().parents[1]/'src/pinchy_moveit_config/config'
        for kind in ('cylinder','plane','quarter_pipe'):
            original=yaml.safe_load((root/f'nominal_{kind}_dry_run.yaml').read_text())
            proposed=yaml.safe_load((root/f'nominal_{kind}_pedestal_dry_run.yaml').read_text())
            self.assertEqual(original['objects'][:-1],proposed['objects'][:-1])
            obj=scene_objects(proposed)[-1];support=proposed['objects'][-1]
            self.assertEqual(obj.id,'nominal_fixture')
            height=support.get('height',support.get('size',[0,0,0])[2])
            self.assertAlmostEqual(support['center'][2]+height/2,.17)
            if kind=='plane':self.assertGreaterEqual(support['center'][0]-support['size'][0]/2,.36)
            elif kind=='cylinder':self.assertLessEqual(support['radius'],.04)
            else:
                self.assertLessEqual(support['inner_radius'],.1)
                self.assertGreaterEqual(support['inner_radius']+support['wall_thickness'],.104)

    def test_invalid_and_duplicate_objects_are_rejected(self):
        base={'objects':[dict(id='part',frame='base_link',center=[0.,0.,0.],type='cylinder',radius=.04,height=.1)]}
        for change in [lambda d:d['objects'][0].update(radius=-1),lambda d:d['objects'][0].update(center=[float('nan'),0.,0.]),
                lambda d:d['objects'][0].update(type='unknown'),lambda d:d['objects'].append(copy.deepcopy(d['objects'][0]))]:
            data=copy.deepcopy(base);change(data)
            with self.assertRaises(ValueError):scene_objects(data)


if __name__=='__main__':unittest.main(verbosity=2)
