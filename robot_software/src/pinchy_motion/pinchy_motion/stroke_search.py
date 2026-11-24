"""Whole-stroke joint optimization; candidates only, never motor commands.

The caller must validate exact task residuals, timed JTC interpolation, collisions
and any pen-up route before accepting a result. No geometry or rate is relaxed.
"""
import copy
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from scipy.sparse import block_diag,diags,eye,kron,vstack


def smooth_trace(curve,mode,distances,positions,solver,valid,stop,fixed_start=True,progress=None):
    """Jointly fit a redundant trajectory with smoothness and travel reserve.

    A complete IK history supplies the branch. Optimization can move *earlier*
    configurations to avoid a later bound; independent exact projection and the
    caller's full timed validation determine acceptance, not optimizer cost.
    """
    from .paths import trace_knots
    def check():
        if stop():raise RuntimeError('Cartesian path stopped')
    check()
    # Optimize a uniform coarse grid, then reproject *all* requested knots
    # below. Near-duplicate waypoint/grid knots otherwise make the sparse fit
    # unnecessarily stiff and slow without adding a geometric constraint.
    samples=np.linspace(0.,curve.length,max(3,int(np.ceil(curve.length/.004))+1))
    initial=CubicSpline(distances,positions,axis=0)(samples)
    targets=[curve.evaluate(s,mode) for s in samples];count=len(samples)
    if count<3:raise ValueError('Whole-stroke optimization needs three path samples')
    first=np.asarray(positions[0]);offset=6 if fixed_start else 0
    intervals=np.diff(samples);spacing=float(np.mean(intervals))
    difference=kron(diags([-spacing/intervals,spacing/intervals],[0,1],shape=(count-1,count)),eye(6)).tocsr()
    # Scale derivatives by their actual path distances, independently of the
    # number of coarse optimizer samples.
    left,right=intervals[:-1],intervals[1:]
    curvature=kron(diags([2*spacing**2/(left*(left+right)),-2*spacing**2/(left*right),
        2*spacing**2/(right*(left+right))],[0,1,2],shape=(count-2,count)),eye(6)).tocsr()
    midpoint=(solver.lower+solver.upper)/2
    margin=np.minimum(.02,(solver.upper-solver.lower)*.05)
    lower=np.tile(solver.lower+margin,count)[offset:];upper=np.tile(solver.upper-margin,count)[offset:]
    def unpack(x):
        return np.vstack([first,x.reshape(-1,6)]) if fixed_start else x.reshape(-1,6)
    evaluations=0
    def residual(x):
        nonlocal evaluations
        evaluations+=1
        if progress and (evaluations==1 or evaluations%5==0):progress('Optimizing whole-stroke configurations',evaluations,None,'optimizer evaluations (100 maximum)')
        check();values=unpack(x);flat=values.ravel()
        return np.r_[np.concatenate([solver.system(q,t,mode)[0]*500 for q,t in zip(values,targets)]),
            curvature@flat*.4,difference@flat*.02,(values-midpoint).ravel()*.02]
    def jacobian(x):
        check();values=unpack(x)
        matrix=vstack([block_diag([solver.system(q,t,mode)[1]*500 for q,t in zip(values,targets)]),
            curvature*.4,difference*.02,eye(count*6)*.02]).tocsr()
        return matrix[:,offset:]
    fit=least_squares(residual,np.clip(initial.ravel()[offset:],lower+1e-9,upper-1e-9),jac=jacobian,
        bounds=(lower,upper),max_nfev=100,ftol=1e-9,gtol=1e-9,xtol=1e-9)
    # Restore exact task constraints after the soft optimization. Do not use
    # soft residuals or optimizer success as permission to execute.
    seeds=CubicSpline(samples,unpack(fit.x),axis=0)
    output_distances=[];output=[]
    def project(distance,depth=0):
        check();target=curve.evaluate(distance,mode)
        q=first.tolist() if fixed_start and distance==0. else solver.search(target,seeds(distance),mode,
            valid=valid,stop=stop,budget=.25,attempts=1,position_tolerance=1e-7,angle_tolerance=1e-6)
        if q is None:raise ValueError('Whole-stroke optimizer could not project every sample to the requested task')
        distance_error,angle_error=solver.errors(q,target,mode)
        if distance_error>1e-6 or angle_error>1e-5 or not valid(q):raise ValueError('Whole-stroke candidate failed exact task/state verification')
        if output and max(abs(np.asarray(q)-output[-1]))>.15:
            if depth<5 and distance-output_distances[-1]>.0001:
                project((distance+output_distances[-1])/2,depth+1);project(distance,depth+1);return
            raise ValueError('Whole-stroke candidate has a joint jump')
        output.append(np.asarray(q));output_distances.append(float(distance))
    # Dense reprojection controls nonlinear FK/speed error between joint knots,
    # particularly when free twist changes quickly near a sensitive posture.
    knots=trace_knots(curve,.0005)
    if progress:progress('Refining stroke IK',0,len(knots),'samples')
    for index,distance in enumerate(knots,1):
        project(float(distance))
        if progress:progress('Refining stroke IK',index,len(knots),'samples')
    return np.asarray(output_distances),np.asarray(output),dict(method='whole_stroke_optimization',iterations=fit.nfev,fixed_start=fixed_start)


class CurveSection:
    """Follow an unchanged curve forward/backward from an interior witness."""
    def __init__(self,curve,start,end):
        self.curve=curve;self.start=start;self.sign=1. if end>start else -1.;self.length=abs(end-start)
        self.waypoint_distances=np.unique(np.r_[0.,self.length,
            [abs(s-start) for s in curve.waypoint_distances if min(start,end)<s<max(start,end)]])
    def evaluate(self,distance,mode):return self.curve.evaluate(self.start+self.sign*distance,mode)


def bidirectional_candidates(curve,mode,failure,solver,valid,stop,attempts=4):
    """An interior IK witness is usable only after connecting *both* sides.

    This can escape greedy start choices at a wrist-limit bottleneck without
    inserting a joint jump or moving off the requested stroke geometry.
    """
    from .paths import sample_ik,TraceIncomplete
    pivot=failure.distance
    if pivot<=1e-6 or pivot>=curve.length-1e-6:return
    query=copy.copy(solver);query.cache=[];seen=[]
    for attempt in range(attempts):
        if stop():raise RuntimeError('Cartesian path stopped')
        query.global_seeds=np.roll(solver.global_seeds,-8*attempt,axis=0)
        witness=query.search(failure.target,np.zeros(6),mode,
            valid=lambda q:all(np.linalg.norm(np.asarray(q)-old)>.25 for old in seen) and valid(q),
            stop=stop,budget=2.,attempts=64,position_tolerance=1e-6,angle_tolerance=1e-5)
        if witness is None:continue
        seen.append(np.asarray(witness))
        try:
            dl,ql=sample_ik(CurveSection(curve,pivot,0.),mode,witness,solver,valid,stop)
            dr,qr=sample_ik(CurveSection(curve,pivot,curve.length),mode,witness,solver,valid,stop)
        except TraceIncomplete:continue
        yield np.r_[pivot-dl[::-1],pivot+dr[1:]],np.vstack([ql[::-1],qr[1:]])


def select_stroke(curve,mode,reference,solver,valid,stop,finish,attempts=8,progress=None):
    """Search starting configurations *before* committing a pen-up approach.

    ``finish`` must validate both the complete timed stroke and a collision-
    planned route from the real held state to its selected first configuration.
    It is never valid to insert a same-task jump into the drawing itself.
    """
    from .paths import sample_ik
    starts=[];rejections=[];target=curve.evaluate(0.,mode);phase='starting IK'
    # Continuation fills the ordinary cache with many states on one branch.
    # Do not let those consume every seed/time slot of the next start search.
    candidate_solver=copy.copy(solver);candidate_solver.cache=[]
    last_progress=None;attempt=0
    def detail(label,done,total,unit):
        if progress:progress(dict(phase=label,completed=done,total=total,unit=unit,configuration_attempt=attempt+1,configuration_limit=attempts))
    def check():
        nonlocal last_progress
        marker=(phase,len(starts),len(rejections))
        if progress is not None and marker!=last_progress:
            progress(dict(phase=phase,configuration_attempt=attempt+1,configuration_limit=attempts,starting_configurations=len(starts),rejected_candidates=len(rejections),last_rejection=rejections[-1] if rejections else ''))
            last_progress=marker
        try:
            if stop():raise RuntimeError('Cartesian path stopped')
        except RuntimeError as error:
            error.selection_report=dict(method='whole_stroke_start_search',starting_configurations=len(starts),rejections=rejections,phase=phase)
            raise
    def accept(distances,positions,seed_method):
        nonlocal phase
        try:
            phase='complete trajectory and approach validation';check();result=finish(distances,positions);check()
            return distances,positions,result,dict(method=seed_method,starting_configurations=len(starts),rejections=rejections)
        except (ValueError,RuntimeError) as error:check();rejections.append(str(error))
        try:
            phase='whole-stroke optimization'
            check()
            distances,positions,report=smooth_trace(curve,mode,distances,positions,solver,valid,stop,fixed_start=False,progress=detail)
            phase='optimized trajectory and approach validation';check();result=finish(distances,positions);check()
            report.update(starting_configurations=len(starts),rejections=rejections,seed_method=seed_method)
            return distances,positions,result,report
        except (ValueError,RuntimeError) as error:check();rejections.append(str(error))
        return None
    for attempt in range(attempts):
        phase='starting IK';check()
        # Excluded candidates cannot be immediately returned from the IK cache.
        seed=np.asarray(reference)
        if starts and mode!='full':
            jacobian=solver.system(starts[0],target,mode)[1]
            vectors=np.linalg.svd(jacobian,full_matrices=True)[2]
            dimension=5 if mode=='tool_axis' else 3
            axis=vectors[dimension+(attempt-1)//2%(6-dimension)]
            seed=np.clip(starts[0]+(1 if attempt%2 else -1)*.35*((attempt+1)//2)*axis,solver.lower,solver.upper)
        candidate_solver.global_seeds=np.roll(solver.global_seeds,-attempt*8,axis=0)
        start=candidate_solver.search(target,seed,mode,
            valid=lambda q:all(np.linalg.norm(np.asarray(q)-old)>.25 for old in starts) and valid(q),
            stop=stop,budget=5. if not starts else 2.,attempts=64,position_tolerance=1e-6,angle_tolerance=1e-5)
        if start is None:continue
        starts.append(np.asarray(start))
        try:
            phase='continuous IK'
            check()
            distances,positions=sample_ik(curve,mode,start,solver,valid,stop,progress=lambda done,total:detail('Solving continuous stroke IK',done,total,'path samples'))
        except (ValueError,RuntimeError) as error:
            check();rejections.append(str(error))
            if attempt==0 and hasattr(error,'target'):
                phase='bidirectional bottleneck search'
                check()
                for distances,positions in bidirectional_candidates(curve,mode,error,solver,valid,stop):
                    accepted=accept(distances,positions,'whole_stroke_bidirectional')
                    if accepted is not None:return accepted
            continue
        accepted=accept(distances,positions,'whole_stroke_start_search')
        if accepted is not None:return accepted
    error=RuntimeError('No complete stroke and pen-up approach passed bounded configuration/timing/collision search. '
        'No partial plan retained; this is not proof of impossibility. Last rejection: '+(rejections[-1] if rejections else 'no alternative start IK found'))
    error.selection_report=dict(method='whole_stroke_start_search',starting_configurations=len(starts),rejections=rejections)
    raise error
