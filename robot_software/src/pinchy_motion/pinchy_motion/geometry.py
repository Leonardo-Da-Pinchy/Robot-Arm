"""Pose conventions and validation of the exact joint interpolation sent to JTC."""
import copy
import math
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from pinchy_bringup.control_contract import JOINTS, MAX_SPEED, MAX_ACCELERATION, model_limits


def quaternion_from_rpy(rpy, degrees=True):
    if not all(math.isfinite(x) for x in rpy): raise ValueError('RPY must be finite')
    return Rotation.from_euler('xyz', rpy, degrees=degrees).as_quat().tolist()


def goal_difference(current_xyz_rpy_deg, goal_xyz_rpy_deg):
    """Position distance and tool-axis angle; independent of free pen twist."""
    current=np.asarray(current_xyz_rpy_deg,dtype=float);goal=np.asarray(goal_xyz_rpy_deg,dtype=float)
    if current.shape!=(6,) or goal.shape!=(6,) or not np.isfinite(np.r_[current,goal]).all():
        raise ValueError('Expected two finite XYZ/RPY poses')
    first=Rotation.from_euler('xyz',current[3:],degrees=True).as_matrix()[:,2]
    last=Rotation.from_euler('xyz',goal[3:],degrees=True).as_matrix()[:,2]
    return float(np.linalg.norm(current[:3]-goal[:3])),float(np.arccos(np.clip(first@last,-1.,1.)))


def pose_error(actual, target):
    ap, tp = actual.position, target.position
    distance = math.dist([ap.x,ap.y,ap.z], [tp.x,tp.y,tp.z])
    aq, tq = actual.orientation, target.orientation
    rotation = Rotation.from_quat([aq.x,aq.y,aq.z,aq.w]).inv()*Rotation.from_quat([tq.x,tq.y,tq.z,tq.w])
    return distance, float(rotation.magnitude())


def seconds(point):
    return point.time_from_start.sec+point.time_from_start.nanosec*1e-9


def set_seconds(point, value):
    point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(value*1e9), 1_000_000_000)


def segment_coefficients(a, b):
    """Match JTC position-only interface spline interpolation (linear/cubic/quintic)."""
    duration = seconds(b)-seconds(a)
    if duration <= 0: raise ValueError('Trajectory times must increase')
    p, q = np.array(a.positions), np.array(b.positions)
    coefficients = np.zeros((6,6)); coefficients[:,0] = p
    if len(a.velocities) == 6 and len(b.velocities) == 6:
        v, w = np.array(a.velocities), np.array(b.velocities)
        coefficients[:,1] = v
        if len(a.accelerations) == 6 and len(b.accelerations) == 6:
            x, y = np.array(a.accelerations), np.array(b.accelerations)
            coefficients[:,2] = x/2
            matrix = [[duration**3,duration**4,duration**5],
                      [3*duration**2,4*duration**3,5*duration**4],
                      [6*duration,12*duration**2,20*duration**3]]
            residual = np.array([q-p-v*duration-x*duration**2/2,w-v-x*duration,y-x])
            coefficients[:,3:] = np.linalg.solve(matrix,residual).T
        else:
            coefficients[:,2] = (3*(q-p)/duration-2*v-w)/duration
            coefficients[:,3] = (2*(p-q)/duration+v+w)/duration**2
    else: coefficients[:,1] = (q-p)/duration
    return duration, coefficients


def evaluate(coefficients, times, derivative=0):
    arrays = [np.polynomial.polynomial.polyder(row, m=derivative) for row in coefficients]
    return np.array([np.polynomial.polynomial.polyval(times,row) for row in arrays]).T


def extrema(coefficients, duration, derivative):
    result=[]
    for row in coefficients:
        function = np.polynomial.polynomial.polyder(row,m=derivative)
        roots = np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(function))
        times = [0.0,duration]+[float(t.real) for t in roots if abs(t.imag)<1e-8 and 0<t.real<duration]
        result.append(np.polynomial.polynomial.polyval(times,function))
    return result


def prepare_trajectory(trajectory, limits, velocity_scaling=1.0, acceleration_scaling=1.0, rescale=True):
    """Reorder, validate spline extrema, and uniformly slow without changing geometry."""
    result = copy.deepcopy(trajectory); joint = result.joint_trajectory
    if len(joint.joint_names)!=6 or set(joint.joint_names)!=set(JOINTS): raise ValueError('Plan must contain exactly the six arm joints')
    indices = [joint.joint_names.index(name) for name in JOINTS]; joint.joint_names = JOINTS
    if not 2 <= len(joint.points) <= 3000: raise ValueError('Plan requires 2..3000 timed points')
    for point in joint.points:
        for field in ('positions','velocities','accelerations'):
            values = list(getattr(point,field))
            if values and (len(values)!=6 or not all(math.isfinite(v) for v in values)):
                raise ValueError('Nonfinite/incomplete trajectory data')
            if field=='positions' and not values: raise ValueError('Missing joint positions')
            setattr(point,field,[values[i] for i in indices] if values else [])
    for field in ('velocities','accelerations'):
        if len({len(getattr(point,field)) for point in joint.points}) != 1:
            raise ValueError('Derivative fields must be consistent across the trajectory')
    if joint.points[0].accelerations and not joint.points[0].velocities:
        raise ValueError('Acceleration fields require velocity fields')
    if seconds(joint.points[0]) < 0 or seconds(joint.points[-1]) > 120:
        raise ValueError('Invalid or excessive trajectory duration')
    factor = 1.05 if rescale else 1.0
    limiter='none'
    for a,b in zip(joint.points,joint.points[1:]):
        duration, coefficients = segment_coefficients(a,b)
        position = extrema(coefficients,duration,0)
        velocity = extrema(coefficients,duration,1)
        acceleration = extrema(coefficients,duration,2)
        for i,name in enumerate(JOINTS):
            limit=limits[name]
            if min(position[i]) < limit['lower']-1e-8 or max(position[i]) > limit['upper']+1e-8:
                raise ValueError('Interpolated trajectory exceeds '+name+' bounds')
            for candidate,kind in [(1.05*max(abs(velocity[i]))/(limit['velocity']*velocity_scaling),'velocity'),
                    (1.05*math.sqrt(max(abs(acceleration[i]))/(limit['acceleration']*acceleration_scaling)),'acceleration')]:
                if candidate>factor:factor=candidate;limiter=f'{name} {kind} at {seconds(a):.6f}s (interval {duration:.6f}s)'
    if not rescale and factor>1.+1e-6:
        raise ValueError(f'Requested contact speed exceeds joint timing limits; reduce speed/acceleration (required time scale at least {factor:.2f}; {limiter})')
    for point in joint.points:
        set_seconds(point, seconds(point)*factor)
        if point.velocities: point.velocities=[v/factor for v in point.velocities]
        if point.accelerations: point.accelerations=[a/factor**2 for a in point.accelerations]
    if seconds(joint.points[-1])>120: raise ValueError('Timed trajectory exceeds 120 seconds')
    return result


def trajectory_reserve(trajectory,limits):
    """Smallest signed distance to a joint bound over actual JTC polynomials."""
    reserve=math.inf;joint_name=None
    for first,last in zip(trajectory.joint_trajectory.points,trajectory.joint_trajectory.points[1:]):
        duration,coefficients=segment_coefficients(first,last)
        for name,values in zip(JOINTS,extrema(coefficients,duration,0)):
            margin=min(min(values)-limits[name]['lower'],limits[name]['upper']-max(values))
            if margin<reserve:reserve=margin;joint_name=name
    if joint_name is None:raise ValueError('Trajectory reserve requires two timed points')
    return dict(joint=joint_name,margin_deg=math.degrees(reserve))


def sampled_positions(trajectory, dt=0.02, joint_step=0.008):
    """Dense JTC-spline states for collision validation, including each endpoint."""
    points=trajectory.joint_trajectory.points
    count=0
    yield list(points[0].positions)
    for a,b in zip(points,points[1:]):
        duration,coefficients=segment_coefficients(a,b)
        speed=max(max(abs(v)) for v in extrema(coefficients,duration,1))
        divisions=max(1,math.ceil(duration/dt),math.ceil(speed*duration/joint_step))
        count+=divisions
        if count>10000: raise ValueError('Trajectory validation sample limit exceeded')
        for row in evaluate(coefficients,np.linspace(0,duration,divisions+1)[1:]): yield row.tolist()


def forward_kinematics(root, positions, tip='pointer_tip'):
    """URDF FK for displayed encoder pose; MoveIt independently checks planned FK."""
    frames={'base_link':np.eye(4)}; pending=list(root.findall('joint'))
    while pending:
        progressed=False
        for joint in list(pending):
            parent=joint.find('parent').get('link')
            if parent not in frames: continue
            origin=joint.find('origin'); transform=np.eye(4)
            transform[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            transform[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            if joint.get('type') in ('revolute','continuous'):
                rotation=np.eye(4); axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
                rotation[:3,:3]=Rotation.from_rotvec(axis*positions[joint.get('name')]).as_matrix()
                transform=transform@rotation
            frames[joint.find('child').get('link')]=frames[parent]@transform
            pending.remove(joint); progressed=True
        if not progressed: raise ValueError('Disconnected model')
    return frames[tip]


def validate_cartesian_shape(trajectory, kinematics, start, end):
    """Check the actual JTC spline samples against the requested LERP/SLERP.

    Transit tolerance: 1 mm position / 0.01 rad orientation. This is not a
    contact-path or constant-speed certification, nor a continuous proof.
    """
    origin=kinematics.evaluate(start)[0];target=kinematics.evaluate(end)[0]
    vector=target[:3,3]-origin[:3,3];length2=vector@vector
    rotation=Rotation.from_matrix(origin[:3,:3].T@target[:3,:3]).as_rotvec()
    angle2=rotation@rotation;previous=0.
    for q in sampled_positions(trajectory):
        actual=kinematics.evaluate(q)[0]
        current_rotation=Rotation.from_matrix(origin[:3,:3].T@actual[:3,:3]).as_rotvec()
        # Estimate progress from the whole pose, scaled by the same geometry
        # tolerances used below. Micrometre FK drift must not dominate a
        # rotation-only motion merely because its translation is nonzero.
        denominator=length2/.001**2+angle2/.01**2
        fraction=float(((actual[:3,3]-origin[:3,3])@vector/.001**2+current_rotation@rotation/.01**2)/denominator) if denominator>1e-12 else 1.
        clamped=np.clip(fraction,0.,1.)
        distance=np.linalg.norm(actual[:3,3]-(origin[:3,3]+vector*clamped))
        expected=origin[:3,:3]@Rotation.from_rotvec(rotation*clamped).as_matrix()
        error=Rotation.from_matrix(expected.T@actual[:3,:3]).magnitude()
        if distance>.001 or error>.01 or fraction<previous-.01:
            raise ValueError(f'Timed path deviates from LERP/SLERP transit: {distance*1000:.2f} mm / {math.degrees(error):.2f} deg; no plan retained')
        previous=max(previous,fraction)
