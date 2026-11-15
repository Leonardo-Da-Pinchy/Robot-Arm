"""Whole-stroke joint optimization; candidates only, never motor commands.

The caller must validate exact task residuals, timed JTC interpolation, collisions
and any pen-up route before accepting a result. No geometry or rate is relaxed.
"""
import copy
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from scipy.sparse import block_diag,diags,eye,kron,vstack


def smooth_trace(curve,mode,distances,positions,solver,valid,stop,fixed_start=True):
    """Jointly fit a redundant trajectory with smoothness and travel reserve.

    A complete IK history supplies the branch. Optimization can move *earlier*
    configurations to avoid a later bound; independent exact projection and the
    caller's full timed validation determine acceptance, not optimizer cost.
    """
    from .paths import trace_knots
    def check():
        if stop():raise RuntimeError('Cartesian path stopped')
    check()
    samples=trace_knots(curve,.004)
    initial=CubicSpline(distances,positions,axis=0)(samples)
    targets=[curve.evaluate(s,mode) for s in samples];count=len(samples)
    if count<3:raise ValueError('Whole-stroke optimization needs three path samples')
    first=np.asarray(positions[0]);offset=6 if fixed_start else 0
    intervals=np.diff(samples);spacing=float(np.mean(intervals))
    difference=kron(diags([-spacing/intervals,spacing/intervals],[0,1],shape=(count-1,count)),eye(6)).tocsr()
    # Requested waypoint knots make the sampling nonuniform. Penalize actual
    # derivatives, not equal joint increments over unequal path distances.
    left,right=intervals[:-1],intervals[1:]
    curvature=kron(diags([2*spacing**2/(left*(left+right)),-2*spacing**2/(left*right),
        2*spacing**2/(right*(left+right))],[0,1,2],shape=(count-2,count)),eye(6)).tocsr()
    midpoint=(solver.lower+solver.upper)/2
    margin=np.minimum(.02,(solver.upper-solver.lower)*.05)
    lower=np.tile(solver.lower+margin,count)[offset:];upper=np.tile(solver.upper-margin,count)[offset:]
    def unpack(x):
        return np.vstack([first,x.reshape(-1,6)]) if fixed_start else x.reshape(-1,6)
    def residual(x):
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
    for distance in trace_knots(curve,.0005):project(float(distance))
    return np.asarray(output_distances),np.asarray(output),dict(method='whole_stroke_optimization',iterations=fit.nfev,fixed_start=fixed_start)


def select_stroke(curve,mode,reference,solver,valid,stop,finish,attempts=8):
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
    def check():
        try:
            if stop():raise RuntimeError('Cartesian path stopped')
        except RuntimeError as error:
            error.selection_report=dict(method='whole_stroke_start_search',starting_configurations=len(starts),rejections=rejections,phase=phase)
            raise
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
            distances,positions=sample_ik(curve,mode,start,solver,valid,stop)
        except (ValueError,RuntimeError) as error:
            check();rejections.append(str(error));continue
        try:
            phase='complete trajectory and approach validation'
            result=finish(distances,positions)
            check()
            return distances,positions,result,dict(method='whole_stroke_start_search',starting_configurations=len(starts),rejections=rejections)
        except (ValueError,RuntimeError) as error:
            check();rejections.append(str(error))
        try:
            phase='whole-stroke optimization'
            distances,positions,report=smooth_trace(curve,mode,distances,positions,solver,valid,stop,fixed_start=False)
            phase='optimized trajectory and approach validation'
            result=finish(distances,positions)
            check();report.update(starting_configurations=len(starts),rejections=rejections)
            return distances,positions,result,report
        except (ValueError,RuntimeError) as error:
            check();rejections.append(str(error))
    error=RuntimeError('No complete stroke and pen-up approach passed bounded configuration/timing/collision search. '
        'No partial plan retained; this is not proof of impossibility. Last rejection: '+(rejections[-1] if rejections else 'no alternative start IK found'))
    error.selection_report=dict(method='whole_stroke_start_search',starting_configurations=len(starts),rejections=rejections)
    raise error
