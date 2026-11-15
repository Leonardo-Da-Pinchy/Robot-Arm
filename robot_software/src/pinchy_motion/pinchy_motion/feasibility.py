"""Read-only task/branch diagnostics using the shared live-model IK.

Finite searches are witnesses, not impossibility certificates. No servo I/O,
planning-scene edits, controller goals or operational calibration changes.
"""
import math
import numpy as np
from .ik import BoundedIK
from pinchy_bringup.control_contract import JOINTS


def axis_frame(position,direction):
    d=np.asarray(direction,dtype=float);p=np.asarray(position,dtype=float)
    if d.shape!=(3,) or p.shape!=(3,) or not np.isfinite(np.r_[d,p]).all() or np.linalg.norm(d)<1e-9:
        raise ValueError('Finite XYZ and nonzero direction required')
    d=d/np.linalg.norm(d);reference=np.array([0.,0.,1.]) if abs(d[2])<.9 else np.array([1.,0.,0.])
    x=reference-d*(reference@d);x/=np.linalg.norm(x)
    result=np.eye(4);result[:3,3]=p;result[:3,:3]=np.column_stack([x,np.cross(d,x),d]);return result


def posture_metrics(solver,q):
    q=np.asarray(q);margins=np.minimum(q-solver.lower,solver.upper-q)
    i=int(np.argmin(margins));conditioning=solver.conditioning(q,'tool_axis')
    return dict(conditioning=conditioning,nearest_limit_joint=JOINTS[i],nearest_limit_deg=float(np.degrees(margins[i])),
                joint_margins_deg=dict(zip(JOINTS,np.degrees(margins).tolist())))


class CylinderArc:
    """Exact horizontal arc, front facing base -X; +Z tool axis points inward.

    Used only as an explicitly hypothetical analysis input, not active geometry.
    """
    def __init__(self,center_x,radius,height,half_angle_deg=60.,center_y=0.,start_angle_deg=None,end_angle_deg=None):
        if not all(math.isfinite(v) for v in (center_x,center_y,radius,height,half_angle_deg)) or radius<=0 or not 0<half_angle_deg<=90:
            raise ValueError('Positive radius and arc half-angle in (0,90] required')
        self.cx,self.cy,self.radius,self.height=center_x,center_y,radius,height
        self.half_angle=math.radians(half_angle_deg)
        start=-half_angle_deg if start_angle_deg is None else start_angle_deg
        end=half_angle_deg if end_angle_deg is None else end_angle_deg
        if not (math.isfinite(start) and math.isfinite(end) and -90<=start<end<=90):raise ValueError('Arc interval must lie within front [-90,+90] degrees')
        self.start_angle=math.radians(start);self.length=radius*math.radians(end-start)
        # This also makes the shared continuation sampler follow exact normals.
        self.waypoint_distances=np.array([0.,self.length])
        self.transforms=[self.evaluate(0.,'tool_axis'),self.evaluate(self.length,'tool_axis')]
    def evaluate(self,distance,mode='tool_axis'):
        a=float(distance)/self.radius+self.start_angle
        return axis_frame([self.cx-self.radius*math.cos(a),self.cy+self.radius*math.sin(a),self.height],
                          [math.cos(a),-math.sin(a),0.])


def twist_family(solver,target,start,valid=lambda q:True,steps=10,step=.12):
    """Explore the free-task nullspace while keeping XYZ and pen axis fixed.

    Different returned joints are different configurations of the SAME task,
    not motion commands. Connecting/repositioning still requires path planning.
    """
    result=[np.asarray(start,dtype=float).tolist()]
    for sign in (-1,1):
        q=np.asarray(start,dtype=float);previous=None
        for _ in range(steps):
            _,jac=solver.system(q,target,'tool_axis')
            null=np.linalg.svd(jac,full_matrices=True)[2][-1]
            if previous is not None and null@previous<0:null=-null
            elif previous is None:null*=sign
            seed=np.clip(q+step*null,solver.lower+1e-9,solver.upper-1e-9)
            candidate=solver.search(target,seed,'tool_axis',valid=valid,budget=.3,attempts=1,anchor_weight=.001,
                                    position_tolerance=1e-6,angle_tolerance=1e-5)
            if candidate is None or np.linalg.norm(np.asarray(candidate)-q)<.01:break
            q=np.asarray(candidate);previous=null
            if all(np.linalg.norm(q-old)>.03 for old in result):result.append(q.tolist())
    return result


def speed_envelope(distances,positions,limits,speed,acceleration):
    """Necessary sampled q(s) rate limits, not a retimed/controller certificate.

    Uses finite differences, so the report also needs the shared polynomial
    validation before any assertion that constant speed actually works.
    """
    q=np.asarray(positions);s=np.asarray(distances)
    if len(q)<4 or np.any(np.diff(s)<=0):raise ValueError('At least four increasing path samples required')
    dq=np.gradient(q,s,axis=0,edge_order=2);ddq=np.gradient(dq,s,axis=0,edge_order=2)
    v=np.array([limits[n]['velocity'] for n in JOINTS]);a=np.array([limits[n]['acceleration'] for n in JOINTS])
    vel_capacity=np.min(v/np.maximum(np.abs(dq),1e-12),axis=1)
    accel_capacity=np.sqrt(np.min(a/np.maximum(np.abs(ddq),1e-12),axis=1))
    capacity=np.minimum(vel_capacity,accel_capacity)
    return dict(sampled_constant_speed_ceiling_m_s=float(capacity.min()),
        requested_speed_m_s=speed,requested_speed_within_sampled_envelope=bool(speed<=capacity.min()),
        # q''v² during constant speed; q'a from ramping must ALSO be checked.
        worst_joint_velocity_rad_s=float(np.max(np.abs(dq)*speed)),
        worst_constant_speed_joint_acceleration_rad_s2=float(np.max(np.abs(ddq)*speed*speed)),
        requested_tcp_acceleration_m_s2=acceleration)
