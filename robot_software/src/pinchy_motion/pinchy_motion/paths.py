"""Continuous Cartesian path sampling and explicit TCP-speed timing.

No motor I/O. All generated states and actual JTC interpolation are checked by
our gateway/MoveIt before a preview token can exist.
"""
import math
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation,Slerp
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from pinchy_bringup.control_contract import JOINTS
from .geometry import prepare_trajectory,segment_coefficients,seconds,set_seconds


def separated(values,tolerance=1e-8):
    result=[]
    for value in sorted(values):
        if not result or value-result[-1]>tolerance:result.append(float(value))
    return np.array(result)


def pose_matrix(pose):
    p,q=pose.position,pose.orientation
    t=np.eye(4);t[:3,3]=[p.x,p.y,p.z];t[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();return t


class CartesianCurve:
    def __init__(self,transforms,style='polyline'):
        self.transforms=transforms;self.style=style
        positions=np.array([t[:3,3] for t in transforms]);lengths=np.linalg.norm(np.diff(positions,axis=0),axis=1)
        if np.any(lengths<1e-5):raise ValueError('Ordered traces need distinct XYZ points; use pose transit for rotation-only moves')
        self.knots=np.r_[0.,np.cumsum(lengths)]
        self.positions=positions;self.rotations=Slerp(self.knots,Rotation.from_matrix(np.array([t[:3,:3] for t in transforms])))
        self.spline=CubicSpline(self.knots,positions,axis=0,bc_type='natural') if style in ('spline','spline_axis') else None
        # Explicit mode: a smooth unit direction curve removes the angular-rate
        # jumps of per-segment S2 interpolation. Existing spline stays unchanged.
        self.axis_spline=CubicSpline(self.knots,np.array([t[:3,2] for t in transforms]),axis=0) if style=='spline_axis' else None
        dense=np.unique(np.r_[np.linspace(0,self.knots[-1],max(100,int(self.knots[-1]/.0005)+1)),self.knots])
        p=self.spline(dense) if self.spline is not None else np.column_stack([np.interp(dense,self.knots,positions[:,i]) for i in range(3)])
        self.arc_parameters=dense;self.arc_distances=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
        self.length=float(self.arc_distances[-1]);self.waypoint_distances=np.interp(self.knots,dense,self.arc_distances)

    def evaluate(self,distance,mode):
        parameter=float(np.interp(distance,self.arc_distances,self.arc_parameters));result=np.eye(4)
        result[:3,3]=self.spline(parameter) if self.spline is not None else [np.interp(parameter,self.knots,self.positions[:,i]) for i in range(3)]
        if mode=='full':result[:3,:3]=self.rotations(parameter).as_matrix()
        elif mode=='tool_axis' and self.axis_spline is not None:
            axis=self.axis_spline(parameter);norm=np.linalg.norm(axis)
            if norm<1e-6:raise ValueError('Smooth pen direction is undefined between these points; add intermediate normals')
            axis=axis/norm;reference=np.eye(3)[int(np.argmin(abs(axis)))]
            x=np.cross(reference,axis);x=x/np.linalg.norm(x)
            result[:3,:3]=np.column_stack((x,np.cross(axis,x),axis))
        else:
            # SLERP the pointing direction on S2, not a preselected pen twist.
            index=min(len(self.knots)-2,max(0,int(np.searchsorted(self.knots,parameter,side='right')-1)))
            fraction=(parameter-self.knots[index])/(self.knots[index+1]-self.knots[index])
            first,last=self.transforms[index][:3,2],self.transforms[index+1][:3,2]
            cross=np.cross(first,last);norm=np.linalg.norm(cross);angle=math.acos(float(np.clip(first@last,-1,1)))
            if norm<1e-8:
                axis=self.transforms[index][:3,0] if angle>1. else np.array([1.,0.,0.])
            else:axis=cross/norm
            result[:3,:3]=Rotation.from_rotvec(axis*angle*fraction).as_matrix()@self.transforms[index][:3,:3]
        return result


class SpeedLaw:
    """C2 position law: cosine acceleration ramps and constant-speed plateau."""
    def __init__(self,length,speed,acceleration):
        if not all(math.isfinite(x) and x>0 for x in (length,speed,acceleration)):raise ValueError('Positive finite path length, TCP speed and TCP acceleration required')
        self.length=length;self.requested=speed
        self.speed=min(speed,math.sqrt(2*acceleration*length/math.pi))
        self.ramp_time=math.pi*self.speed/(2*acceleration);self.ramp_distance=.5*self.speed*self.ramp_time
        self.plateau_time=max(0.,(length-2*self.ramp_distance)/self.speed)
        self.duration=2*self.ramp_time+self.plateau_time
    def at(self,t):
        t=float(np.clip(t,0,self.duration));T=self.ramp_time;v=self.speed
        if t<T:
            a=math.pi*t/T
            return v/2*(t-T/math.pi*math.sin(a)),v/2*(1-math.cos(a)),v*math.pi/(2*T)*math.sin(a)
        if t<=T+self.plateau_time:return self.ramp_distance+v*(t-T),v,0.
        remaining=self.duration-t;a=math.pi*remaining/T
        return self.length-v/2*(remaining-T/math.pi*math.sin(a)),v/2*(1-math.cos(a)),-v*math.pi/(2*T)*math.sin(a)
    def time_at(self,distance):
        if distance<=0:return 0.
        if distance>=self.length:return self.duration
        return brentq(lambda t:self.at(t)[0]-distance,0.,self.duration,xtol=1e-11)


class TraceIncomplete(RuntimeError):
    """Carries the exact failed target for independent diagnostic searches."""
    def __init__(self,message,distance,last_distance,previous_positions,target):
        super().__init__(message)
        self.distance=float(distance);self.last_distance=float(last_distance)
        self.previous_positions=list(previous_positions);self.target=np.asarray(target).copy()


def trace_knots(curve,step):
    fixed=curve.waypoint_distances
    grid=np.linspace(0,curve.length,max(2,math.ceil(curve.length/step)+1))
    # Preserve the requested waypoint knots. A regular sample micrometres away
    # can satisfy the IK tolerance with the identical previous q, creating an
    # artificial flat interval and extreme derivative in the joint spline.
    grid=[value for value in grid if min(abs(fixed-value))>=.0001]
    return separated(np.r_[fixed,grid])


def sample_ik(curve,mode,start,solver,valid,stop,step=.002,progress=None):
    """Adaptive continuation; never turn a partial trace into an executable path."""
    knots=trace_knots(curve,step)
    positions=[list(start)];distances=[0.]
    def advance(distance,depth=0):
        if stop():raise RuntimeError('Cartesian path stopped')
        previous=np.array(positions[-1]);target=curve.evaluate(distance,mode)
        # Try local continuation first. Global jumps are not valid tracing.
        found=solver.search(target,previous,mode,valid=lambda q:max(abs(np.array(q)-previous))<=.15 and valid(q),stop=stop,budget=.35,attempts=4,anchor_weight=.001 if mode!='full' else 0.,position_tolerance=1e-6,angle_tolerance=1e-5)
        if found is not None and max(abs(np.array(found)-previous))<=.15:
            distances.append(distance);positions.append(found);return
        if depth<5 and distance-distances[-1]>.0001:
            middle=(distance+distances[-1])/2;advance(middle,depth+1);advance(distance,depth+1);return
        raise TraceIncomplete(f'Cartesian trace incomplete at {100*distances[-1]/curve.length:.1f}%: no continuous collision-free IK step found within saved limits. A different twist/approach configuration may be needed; Visit poses permits pen-up repositioning, not contact tracing. No partial plan retained',distance,distances[-1],positions[-1],target)
    if progress:progress(0,len(knots)-1)
    for index,distance in enumerate(knots[1:],1):
        advance(float(distance))
        if progress:progress(index,len(knots)-1)
    return np.array(distances),np.array(positions)


def select_trace(curve,mode,start,solver,valid,stop,finish,step=.002,width=4,progress=None):
    """Select a complete connected trace, including timing/interpolation checks.

    Keep the inexpensive continuation when it passes. Otherwise retain several
    histories through the whole stroke, exploring local task-nullspace choices.
    This is a finite beam search, not a completeness or reachability proof.
    The held start is immutable; no same-task jump or pen-up motion is inserted.
    ``finish`` must validate the actual timed trajectory before accepting it.
    """
    def check():
        if stop():raise RuntimeError('Cartesian path stopped')
    check()
    try:
        distances,positions=sample_ik(curve,mode,start,solver,valid,stop,step,
            progress=(lambda done,total:progress('Solving continuous Cartesian IK',done,total,'path samples')) if progress else None)
        result=finish(distances,positions)
        check()
        return distances,positions,result,dict(method='local_continuation',complete_candidates=1)
    except (ValueError,RuntimeError) as error:
        check()  # cancellation/model changes are never treated as branch failures
        original=error

    # A complete branch may fail timing or overshoot between IK samples. Move
    # its configurations jointly before exploring more local branches.
    if 'positions' in locals() and hasattr(solver,'errors'):
        from .stroke_search import smooth_trace
        try:
            optimized_distances,optimized_positions,report=smooth_trace(curve,mode,distances,positions,solver,valid,stop,progress=progress)
            result=finish(optimized_distances,optimized_positions)
            check()
            return optimized_distances,optimized_positions,result,report
        except (ValueError,RuntimeError):check()

    # Score entire histories for travel, derivative changes and worst reserve.
    # Retain endpoint diversity so a cheap nearest branch cannot evict every
    # configuration that provides headroom later in the stroke.
    lower,upper=solver.lower,solver.upper
    search_report=dict(method='whole_stroke_beam',beam_width=width,complete_candidates=0,rejections=[])
    frontier=[(0.,[np.asarray(start,dtype=float)])];distances=[0.]
    def rank(history):
        values=np.asarray(history);delta=np.diff(values,axis=0)
        reserve=np.min(np.minimum(values-lower,upper-values),axis=1)
        return float(np.sum(delta**2)+2*np.sum(np.diff(delta,axis=0)**2)+.0001*np.mean(1/(np.maximum(0.,reserve)+.01)))
    def advance(distance,depth=0):
        nonlocal frontier
        check();target=curve.evaluate(distance,mode);children=[]
        for _,history in frontier:
            previous=history[-1]
            _,jacobian=solver.system(previous,target,mode)
            _,_,vectors=np.linalg.svd(jacobian,full_matrices=True)
            dimension={'full':6,'tool_axis':5,'position':3}[mode]
            seeds=[previous]
            for axis in vectors[dimension:]:
                seeds.extend([np.clip(previous+sign*.10*axis,lower,upper) for sign in (1.,-1.)])
            for seed in seeds:
                check()
                found=solver.search(target,seed,mode,
                    valid=lambda q:max(abs(np.asarray(q)-previous))<=.15 and valid(q),
                    stop=stop,budget=.12,attempts=1,anchor_weight=.001 if mode!='full' else 0.,
                    position_tolerance=1e-6,angle_tolerance=1e-5)
                if found is None or max(abs(np.asarray(found)-previous))>.15:continue
                path=[*history,np.asarray(found)]
                children.append((rank(path),path))
        if not children:
            if depth<5 and distance-distances[-1]>.0001:
                middle=(distance+distances[-1])/2
                advance(middle,depth+1);advance(distance,depth+1);return
            raise TraceIncomplete('Bounded whole-stroke search found no continuous collision-free extension; no partial plan retained',
                distance,distances[-1],frontier[0][1][-1],target)
        selected=[]
        for candidate in sorted(children,key=lambda item:item[0]):
            if all(np.max(abs(candidate[1][-1]-old[1][-1]))>.025 for old in selected):selected.append(candidate)
            if len(selected)>=width:break
        frontier=selected;distances.append(float(distance))

    try:
        knots=trace_knots(curve,step)
        if progress:progress('Searching continuous branch alternatives',0,len(knots)-1,'path samples')
        for index,distance in enumerate(knots[1:],1):
            advance(float(distance))
            if progress:progress('Searching continuous branch alternatives',index,len(knots)-1,'path samples')
        for _,history in frontier:
            check()
            search_report['complete_candidates']+=1
            try:result=finish(np.asarray(distances),np.asarray(history))
            except (ValueError,RuntimeError) as error:
                check();search_report['rejections'].append(str(error));continue
            check()
            return np.asarray(distances),np.asarray(history),result,search_report
    except TraceIncomplete as error:
        search_report['extension_failure_distance_m']=error.distance
    original.selection_report=search_report
    detail=(' Last complete candidate rejection: '+search_report['rejections'][-1]) if search_report['rejections'] else ''
    original.args=(str(original)+' Bounded whole-stroke alternatives also failed; this finite search does not prove the stroke unreachable.'+detail+' No partial plan retained',)
    raise original


def timed_trace(curve,distances,positions,solver,limits,mode,speed,acceleration,contacts,velocity_scale=1.,acceleration_scale=1.,allow_contact_ramps=False,enforce_tcp_speed=False,progress=None,tcp_min_speed=0.,stop=None):
    if not math.isfinite(tcp_min_speed) or tcp_min_speed<0:raise ValueError('Minimum TCP speed must be finite and nonnegative')
    spline=CubicSpline(distances,positions,axis=0)
    ranged=tcp_min_speed>0
    if ranged:
        from .speed_range import BoundedSpeedLaw
        if progress:progress('Selecting smooth TCP speed within range')
        law=BoundedSpeedLaw(curve.length,tcp_min_speed,speed,acceleration,spline,
            [limits[n]['velocity']*velocity_scale for n in JOINTS],
            [limits[n]['acceleration']*acceleration_scale for n in JOINTS],stop,joint_names=JOINTS)
    else:law=SpeedLaw(curve.length,speed,acceleration)
    if not ranged and enforce_tcp_speed and (law.speed<speed-1e-9 or law.plateau_time<=1e-6):
        raise ValueError('Trace is too short to reach the requested TCP cruise speed; no plan retained')
    if law.duration>110:raise ValueError('Path would exceed 110 seconds; split it or choose a feasible higher speed')
    contact_intervals=[(curve.waypoint_distances[i],curve.waypoint_distances[i+1]) for i,c in enumerate(contacts) if c]
    if not allow_contact_ramps and contact_intervals and any(a<law.ramp_distance-1e-8 or b>curve.length-law.ramp_distance+1e-8 for a,b in contact_intervals):
        raise ValueError('Contact includes acceleration/braking: add pen-up lead-in/out or reduce the requested contact speed')
    # C2 continuity prevents acceleration jumps at IK knots from being
    # amplified by short-interval JTC quintics. Check bounds and geometry below.
    # Zero joint curvature at a Cartesian endpoint is generally inconsistent
    # with the requested surface curve. Not-a-knot preserves the fitted local
    # curvature instead of biasing endpoint TCP speed during contact ramps.
    times=separated(np.r_[np.arange(0,law.duration,.02),[law.duration,law.ramp_time,law.ramp_time+law.plateau_time],
                            [law.time_at(s) for s in curve.waypoint_distances]],.001)
    times[-1]=law.duration
    if len(times)>2900:raise ValueError('Trace has too many timed samples; split the path')
    trajectory=RobotTrajectory();trajectory.joint_trajectory.joint_names=JOINTS
    for t in times:
        t=round(float(t)*1e9)/1e9
        s,v,a=law.at(t)
        point=JointTrajectoryPoint(positions=spline(s).tolist(),velocities=(spline(s,1)*v).tolist(),
            accelerations=(spline(s,2)*v*v+spline(s,1)*a).tolist())
        set_seconds(point,float(t));trajectory.joint_trajectory.points.append(point)
    # A contact request must not be silently slowed to satisfy joint limits.
    trajectory=prepare_trajectory(trajectory,limits,velocity_scale,acceleration_scale,rescale=not (bool(contact_intervals) or enforce_tcp_speed or ranged),
        progress=(lambda done,total:progress('Checking joint bounds and rates',done,total,'intervals')) if progress else None)
    factor=seconds(trajectory.joint_trajectory.points[-1])/law.duration
    contact_speeds=[];steady_contact_speeds=[];cruise_speeds=[];cruise_samples=[];peak=0.;ramp_error=0.;law_error=0.;range_peak_acceleration=0.
    total=len(trajectory.joint_trajectory.points)-1
    if progress:progress('Checking TCP speed and surface normal',0,total,'intervals')
    for index,(first,last) in enumerate(zip(trajectory.joint_trajectory.points,trajectory.joint_trajectory.points[1:]),1):
        if progress and (index%10==0 or index==total):progress('Checking TCP speed and surface normal',index-1,total,'intervals')
        if stop and stop():raise RuntimeError('Stopped during TCP timing validation')
        duration,coeff=segment_coefficients(first,last)
        for local in np.linspace(0,duration,max(2,math.ceil(duration/(.01 if ranged else .02))+1)):
            q=coeff@np.array([local**i for i in range(6)])
            qdot=coeff[:,1:]@np.array([i*local**(i-1) for i in range(1,6)])
            absolute=seconds(first)+local;s,_,_=law.at(absolute/factor);expected=curve.evaluate(s,mode)
            actual,jp,_=solver.evaluate(q);distance=np.linalg.norm(actual[:3,3]-expected[:3,3])
            angle=Rotation.from_matrix(expected[:3,:3].T@actual[:3,:3]).magnitude() if mode=='full' else math.acos(float(np.clip(actual[:3,2]@expected[:3,2],-1,1))) if mode=='tool_axis' else 0.
            if distance>.001 or angle>.01:raise ValueError(f'Timed trace leaves requested geometry ({distance*1000:.2f} mm / {math.degrees(angle):.2f} deg); refine waypoints or choose another route')
            linear=float(np.linalg.norm(jp@qdot));peak=max(peak,linear)
            if ranged and linear>1e-7:
                qddot=coeff[:,2:]@np.array([i*(i-1)*local**(i-2) for i in range(2,6)])
                eps=1e-5
                jdot=(solver.evaluate(q+qdot*eps)[1]-solver.evaluate(q-qdot*eps)[1])/(2*eps)
                tangential=abs(float((jp@qdot)@(jp@qddot+jdot@qdot))/linear)
                range_peak_acceleration=max(range_peak_acceleration,tangential)
                if tangential>acceleration*1.05+1e-7:raise ValueError('Controller interpolation exceeds requested TCP range acceleration; no plan retained')
            expected_speed=law.at(absolute/factor)[1]/factor
            law_error=max(law_error,abs(linear-expected_speed))
            if law.plateau_time>1e-6 and law.ramp_distance-1e-9<=s<=curve.length-law.ramp_distance+1e-9:
                cruise_speeds.append(linear)
                if ranged:cruise_samples.append((absolute,linear))
            if any(a-1e-9<=s<=b+1e-9 for a,b in contact_intervals):
                contact_speeds.append(linear)
                ramp_error=max(ramp_error,abs(linear-expected_speed))
                if law.plateau_time>1e-6 and law.ramp_distance-1e-9<=s<=curve.length-law.ramp_distance+1e-9:steady_contact_speeds.append(linear)
    if progress:progress('Checking TCP speed and surface normal',total,total,'intervals')
    checked_speeds=steady_contact_speeds if allow_contact_ramps else contact_speeds
    if not ranged and checked_speeds and max(abs(v-speed) for v in checked_speeds)>speed*.05:
        raise ValueError('Commanded contact TCP speed varies more than the 5% planning tolerance; no plan retained')
    if not ranged and contact_speeds and allow_contact_ramps and ramp_error>speed*.05:
        raise ValueError('Contact ramp TCP speed differs from its smooth speed law by more than 5%; no plan retained')
    if not ranged and enforce_tcp_speed and (not cruise_speeds or max(abs(v-speed) for v in cruise_speeds)>speed*.05 or law_error>speed*.05):
        raise ValueError('Requested TCP speed varies more than the 5% planning tolerance; no plan retained')
    if ranged and (not cruise_speeds or min(cruise_speeds)<tcp_min_speed-1e-7 or max(cruise_speeds)>speed+1e-7):
        raise ValueError('Actual controller TCP speed leaves the explicitly requested range; minimum/maximum were not relaxed')
    range_report=dict(enabled=ranged,minimum_m_s=tcp_min_speed,maximum_m_s=speed,length_m=curve.length)
    if ranged:
        measured=np.array(cruise_samples);_,unique=np.unique(measured[:,0],return_index=True);measured=measured[np.sort(unique)]
        if len(measured)<2 or measured[-1,0]<=measured[0,0]:raise ValueError('TCP range interior too short to validate its speed band; no plan retained')
        modeled_mean=float(np.trapz(measured[:,1],measured[:,0])/(measured[-1,0]-measured[0,0]))
        range_report['modeled_mean_m_s']=modeled_mean
    if ranged:range_report.update(timing_solver_seconds=law.solver_seconds,planned_mean_m_s=(curve.length-2*law.ramp_distance)/law.plateau_time,
        slowdown_distance_fraction=law.slowdown_distance_fraction,slowdown_intervals_m=law.slowdown_intervals,
        modeled_peak_tangential_acceleration_m_s2=range_peak_acceleration)
    return trajectory,dict(requested_tcp_speed_m_s=speed,modeled_peak_tcp_speed_m_s=peak,
        modeled_contact_min_m_s=min(contact_speeds) if contact_speeds else 0.,modeled_contact_max_m_s=max(contact_speeds) if contact_speeds else 0.,
        modeled_steady_contact_min_m_s=min(steady_contact_speeds) if steady_contact_speeds else 0.,
        modeled_steady_contact_max_m_s=max(steady_contact_speeds) if steady_contact_speeds else 0.,
        contact_ramps_allowed=allow_contact_ramps,contact_has_constant_speed_plateau=bool(not ranged and steady_contact_speeds and law.plateau_time>1e-6),
        ramp_only=bool(contact_intervals and law.plateau_time<=1e-6),
        enforced_tcp_speed=enforce_tcp_speed or ranged,speed_range=range_report,modeled_cruise_min_m_s=min(cruise_speeds) if cruise_speeds else 0.,
        modeled_cruise_max_m_s=max(cruise_speeds) if cruise_speeds else 0.,
        trace_start_s=0.,trace_end_s=law.duration*factor,cruise_start_s=law.ramp_time*factor,
        cruise_end_s=(law.ramp_time+law.plateau_time)*factor,
        transit_time_scale=factor,physical_speed_qualified=False)


def corner_indices(transforms):
    result=[]
    for i in range(1,len(transforms)-1):
        a=transforms[i][:3,3]-transforms[i-1][:3,3];b=transforms[i+1][:3,3]-transforms[i][:3,3]
        if np.linalg.norm(a)<1e-5 or np.linalg.norm(b)<1e-5:continue
        if a@b/(np.linalg.norm(a)*np.linalg.norm(b))<1.-1e-6:result.append(i)
    return result


def concatenate(trajectories):
    result=RobotTrajectory();result.joint_trajectory.joint_names=JOINTS;offset=0.
    for trajectory in trajectories:
        for i,point in enumerate(trajectory.joint_trajectory.points):
            if result.joint_trajectory.points and i==0:continue
            import copy
            point=copy.deepcopy(point);set_seconds(point,seconds(point)+offset);result.joint_trajectory.points.append(point)
        offset=seconds(result.joint_trajectory.points[-1])
    return result
