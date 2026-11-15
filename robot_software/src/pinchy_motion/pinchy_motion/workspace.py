"""One definition for input bounds and explicitly approximate reach visualization."""
import itertools
import json
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from .geometry import forward_kinematics
from pinchy_bringup.control_contract import JOINTS


def configuration():
    return json.loads((Path(get_package_share_directory('pinchy_motion'))/'config/workspace.json').read_text())


def inside(position,settings):
    return all(a<=v<=b for a,v,b in zip(settings['minimum_m'],position,settings['maximum_m']))


def markers(root,limits,settings):
    wire=Marker();wire.header.frame_id=settings['frame'];wire.ns='workspace';wire.id=0
    wire.type=Marker.LINE_LIST;wire.action=Marker.ADD;wire.pose.orientation.w=1.
    wire.scale.x=.003;wire.color.r=1.;wire.color.g=.8;wire.color.a=.65
    corners=list(itertools.product(*zip(settings['minimum_m'],settings['maximum_m'])))
    for a,b in itertools.combinations(corners,2):
        if sum(x!=y for x,y in zip(a,b))==1:
            wire.points.extend(Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in (a,b))
    samples=Marker();samples.header=wire.header;samples.ns='workspace';samples.id=2
    samples.type=Marker.POINTS;samples.action=Marker.ADD;samples.pose.orientation.w=1.
    samples.scale.x=samples.scale.y=.004;samples.color.g=.8;samples.color.b=1.;samples.color.a=.35
    bounds=np.array([[limits[name]['lower'],limits[name]['upper']] for name in JOINTS])
    states=np.random.default_rng(22).uniform(bounds[:,0],bounds[:,1],(settings['sample_count'],6))
    for q in states:
        position=forward_kinematics(root,dict(zip(JOINTS,q)))[:3,3]
        samples.points.append(Point(x=float(position[0]),y=float(position[1]),z=float(position[2])))
    # Remove labels from earlier revisions if this display is reloaded.
    deletions=[]
    for marker_id in (1,3):
        deletion=Marker();deletion.header=wire.header;deletion.ns='workspace'
        deletion.id=marker_id;deletion.action=Marker.DELETE;deletions.append(deletion)
    return MarkerArray(markers=[wire,samples,*deletions])


def working_region_marker(settings):
    """Designated front cube, separate from numerical input limits and obstacles."""
    region=settings['design_working_region']
    marker=Marker();marker.header.frame_id=settings['frame'];marker.ns='working_region';marker.id=0
    marker.type=Marker.LINE_LIST;marker.action=Marker.ADD;marker.pose.orientation.w=1.
    marker.scale.x=.003;marker.color.r=.2;marker.color.g=.55;marker.color.b=1.;marker.color.a=1.
    corners=list(itertools.product(*zip(region['minimum_m'],region['maximum_m'])))
    for a,b in itertools.combinations(corners,2):
        if sum(x!=y for x,y in zip(a,b))==1:
            marker.points.extend(Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in (a,b))
    return MarkerArray(markers=[marker])
