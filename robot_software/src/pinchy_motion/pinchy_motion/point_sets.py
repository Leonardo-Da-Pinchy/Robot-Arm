"""Portable pose queues, without plan tokens, execution or motor I/O."""
import json
import math
import os
from pathlib import Path
import tempfile
import warnings

import numpy as np
from scipy.spatial.transform import Rotation
from pinchy_bringup.control_contract import MAX_SPEED,MAX_ACCELERATION,MAX_TCP_SPEED,MAX_TCP_ACCELERATION


def number(value,label,low=None,high=None):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise ValueError(label+' must be a finite number')
    if low is not None and value<low or high is not None and value>high:raise ValueError(label+' is outside supported bounds')
    return float(value)


def vector(value,label):
    if not isinstance(value,(list,tuple)) or len(value)!=3:raise ValueError(label+' needs three values')
    return [number(v,label) for v in value]


def normalize(data,bounds=None,allow_empty=False):
    """Validate the entire file before changing a queue; never clamp its values."""
    if not isinstance(data,dict):raise ValueError('Point set must be a JSON object')
    if isinstance(data.get('schema_version',1),bool) or data.get('schema_version',1)!=1:raise ValueError('Unsupported point-set schema version')
    if any(key in data for key in ('velocity_scaling','acceleration_scaling')):
        raise ValueError('Panel point sets use absolute joint_speed_deg_s and joint_acceleration_deg_s2, not scaling fractions')
    if data.get('frame','base_link')!='base_link':raise ValueError('Panel point sets require base_link coordinates')
    if data.get('tcp_frame','pointer_tip')!='pointer_tip':raise ValueError('Point set uses a different TCP frame')
    if data.get('units',{'position':'m','orientation':'deg'})!={'position':'m','orientation':'deg'}:
        raise ValueError('Point-set units must be metres and degrees')
    mode=data.get('orientation_mode','tool_axis');style=data.get('style','visits')
    if mode not in ('full','tool_axis','position'):raise ValueError('Unknown orientation mode')
    if style not in ('visits','polyline','spline','fixed_tcp'):raise ValueError('Unknown point-set path style')
    points=data.get('waypoints')
    if not isinstance(points,list) or not (0 if allow_empty else 1)<=len(points)<=80:raise ValueError('Point set needs 1..80 waypoints')
    result=dict(schema_version=1,name=str(data.get('name','Point set')),frame='base_link',tcp_frame='pointer_tip',
        units={'position':'m','orientation':'deg'},orientation_mode=mode,style=style,waypoints=[])
    for field,default in [('allow_contact_ramps',False),('plan_pen_up_approach',False)]:
        value=data.get(field,default)
        if not isinstance(value,bool):raise ValueError(field+' must be true or false')
        result[field]=value
    for field,default,low,high in [
        ('joint_speed_deg_s',30.,.1,math.degrees(MAX_SPEED)),
        ('joint_acceleration_deg_s2',180.,.1,math.degrees(MAX_ACCELERATION)),
        ('tcp_speed_m_s',.02,.001,MAX_TCP_SPEED),('tcp_acceleration_m_s2',.05,.001,MAX_TCP_ACCELERATION)]:
        result[field]=number(data.get(field,default),field,low,high)
    for index,point in enumerate(points):
        if not isinstance(point,dict):raise ValueError('Each waypoint must be an object')
        if point.get('frame','base_link')!='base_link':raise ValueError('Waypoint frame must be base_link')
        xyz=vector(point.get('xyz'),'Waypoint XYZ')
        if bounds and any(not lo<=v<=hi for v,lo,hi in zip(xyz,bounds['minimum_m'],bounds['maximum_m'])):
            raise ValueError(f'Waypoint {index+1} exceeds numerical XYZ input bounds')
        if 'direction' in point:
            if mode!='tool_axis':raise ValueError('Direction vectors require pen-direction mode')
            direction=np.asarray(vector(point['direction'],'Direction'));norm=np.linalg.norm(direction)
            if norm<1e-9:raise ValueError('Direction must be nonzero')
            direction/=norm;axis=np.cross([0.,0.,1.],direction);length=np.linalg.norm(axis)
            axis=axis/length if length>1e-8 else np.array([1.,0.,0.])
            rotation=Rotation.from_rotvec(axis*math.acos(float(np.clip(direction[2],-1,1))))
        else:
            if 'rpy_deg' not in point and mode!='position':raise ValueError('Directed waypoint needs RPY or a direction')
            rotation=Rotation.from_euler('xyz',vector(point.get('rpy_deg',[0.,0.,0.]),'Waypoint RPY'),degrees=True)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore',UserWarning)
            rpy=rotation.as_euler('xyz',degrees=True).tolist()
        contact=point.get('contact',False)
        if not isinstance(contact,bool):raise ValueError('Contact label must be true or false')
        result['waypoints'].append(dict(xyz=xyz,rpy_deg=rpy,contact=contact))
    if style=='fixed_tcp':
        if mode!='full':raise ValueError('Fixed-TCP rotation requires Exact RPY')
        if any(p['contact'] for p in result['waypoints']):raise ValueError('Fixed-TCP rotation is a pen-up test')
        if result['plan_pen_up_approach']:raise ValueError('Fixed-TCP rotation includes its own approach')
        if points and any(np.linalg.norm(np.asarray(p['xyz'])-result['waypoints'][0]['xyz'])>1e-6 for p in result['waypoints']):
            raise ValueError('Fixed-TCP rotation waypoints must share one XYZ')
    for field in ('notes','source'):
        if field in data:result[field]=data[field]
    json.dumps(result,allow_nan=False)
    return result


def read(path,bounds=None):
    path=Path(path)
    if path.stat().st_size>1_000_000:raise ValueError('Point-set file exceeds 1 MB')
    return normalize(json.loads(path.read_text()),bounds)


def write(path,data,bounds=None):
    value=normalize(data,bounds);path=Path(path)
    # A file never contains an executable plan token. Loading always replans.
    temporary=None
    try:
        with tempfile.NamedTemporaryFile('w',dir=path.parent,prefix='.'+path.name+'.',delete=False) as stream:
            temporary=stream.name;stream.write(json.dumps(value,indent=2,allow_nan=False)+'\n')
        os.replace(temporary,path)
    finally:
        if temporary and os.path.exists(temporary):os.unlink(temporary)
    return value
