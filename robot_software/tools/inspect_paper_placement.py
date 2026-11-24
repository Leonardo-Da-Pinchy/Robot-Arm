#!/usr/bin/python3
"""Read-only CAD datum check for paper against the +X front of the base feet."""
import hashlib,json,struct
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def inspect():
    root=Path(__file__).resolve().parents[1]
    description=root/'src/pinchy_description'
    model=description/'urdf/pinchy_arm.urdf';base=ET.parse(model).find("link[@name='base_link']")
    bounds=[];hashes={str(model.relative_to(root)):hashlib.sha256(model.read_bytes()).hexdigest()}
    for item in base.findall('collision'):
        mesh=item.find('geometry/mesh')
        if mesh is None:raise ValueError('Recheck non-mesh base geometry before choosing a paper datum')
        path=description/mesh.get('filename').split('package://pinchy_description/')[1]
        raw=path.read_bytes();count=struct.unpack_from('<I',raw,80)[0]
        if len(raw)!=84+50*count:raise ValueError('Expected binary STL: '+str(path))
        dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
        vertices=np.ndarray((count,),dtype=dtype,buffer=raw,offset=84)['vertices'].reshape(-1,3).astype(float)
        vertices*=np.array(list(map(float,mesh.get('scale','1 1 1').split())))
        origin=item.find('origin');rotation=Rotation.from_euler('xyz',list(map(float,origin.get('rpy','0 0 0').split())))
        vertices=rotation.apply(vertices)+np.array(list(map(float,origin.get('xyz','0 0 0').split())))
        bounds.append(dict(mesh=path.name,minimum_m=vertices.min(0).tolist(),maximum_m=vertices.max(0).tolist()))
        hashes[str(path.relative_to(root))]=hashlib.sha256(raw).hexdigest()
    feet=next(b for b in bounds if b['mesh']=='base_so101_v2.stl')
    front=max(b['maximum_m'][0] for b in bounds)
    if abs(front-feet['maximum_m'][0])>1e-8:raise ValueError('Another base part projects ahead of the assumed feet')
    table=feet['minimum_m'][2]
    return dict(frame='base_link',assumption='Paper near edge touches current CAD front feet; 11-inch length along +X, centered at Y=0. Table height inferred from CAD underside, not measured.',
        front_foot_x_m=front,center_from_front_foot_m=.1397,center_m=[front+.1397,0.,table],
        table_surface_z_m=table,dry_run_tcp_z_m=table+.005,paper_size_xy_m=[.2794,.2159],
        base_mesh_bounds=bounds,input_sha256=hashes,physical_measurement=False,motor_commands_sent=False)


if __name__=='__main__':print(json.dumps(inspect(),indent=2))
