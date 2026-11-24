#!/usr/bin/python3
"""Finite cylinder placement study plus necessary reach bounds. No ROS nodes/I/O.

Endpoint witnesses are not collision checks, complete strokes or coverage. The
model/TCP/limits remain unchanged. Unresolved searches are not impossibility
proofs; an exceeded necessary link-reach bound is a geometric exclusion.
"""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
from pinchy_motion.ik import BoundedIK
from pinchy_motion.geometry import model_limits
from pinchy_motion.feasibility import axis_frame,wrist_reach_bound
from pinchy_motion.nominal_paths import surface_pose
from pinchy_bringup.control_contract import JOINTS
ROOT=Path(__file__).resolve().parents[1]



def joint_placement_search(model,limits,attempts):
    """Optimize one shared placement and three independently constrained poses.

    Candidate fits only: no collision, motion or global impossibility claim.
    Joint bounds remain the saved bounds throughout every optimization.
    """
    from scipy.optimize import least_squares
    from scipy.stats import qmc
    solver=BoundedIK(model,limits);angles=(-90.,0.,90.);count=3
    lower=np.r_[[.15,-.15,.15],np.tile(solver.lower,count)]
    upper=np.r_[[.45,.15,.35],np.tile(solver.upper,count)]
    rows=[]
    for attempt,row in enumerate(qmc.Halton(d=21,scramble=True,seed=43).random(attempts)):
        seed=lower+row*(upper-lower)
        if attempt<6:seed[:3]=[.25+(attempt%3)*.05,0.,.25+(attempt//3)*.05]
        def systems(x):
            return [solver.system(x[3+6*i:9+6*i],axis_frame(*surface_pose('cylinder',x[:3],.04*np.radians(angle),0.)),'tool_axis') for i,angle in enumerate(angles)]
        def residual(x):return np.concatenate([r for r,j in systems(x)])
        def jacobian(x):
            output=np.zeros((18,21))
            for i,(_,jac) in enumerate(systems(x)):
                output[i*6:i*6+3,:3]=-np.eye(3)*10.
                output[i*6:i*6+6,3+i*6:9+i*6]=jac
            return output
        fit=least_squares(residual,np.clip(seed,lower+1e-8,upper-1e-8),jac=jacobian,bounds=(lower,upper),
            max_nfev=180,ftol=1e-10,xtol=1e-10,gtol=1e-10)
        errors=[solver.errors(fit.x[3+6*i:9+6*i],axis_frame(*surface_pose('cylinder',fit.x[:3],.04*np.radians(angle),0.)),'tool_axis') for i,angle in enumerate(angles)]
        row=dict(attempt=attempt,center_m=fit.x[:3].tolist(),positions_rad=fit.x[3:].reshape(-1,6).tolist(),
            errors_position_m_angle_rad=errors,all_three_found=all(p<1e-6 and a<1e-5 for p,a in errors),cost=float(fit.cost),iterations=fit.nfev)
        rows.append(row)
        if attempt%8==0:print('Joint placement/configuration search',attempt,'of',attempts,flush=True)
        if row['all_three_found']:break
    return dict(attempted=len(rows),successful=sum(r['all_three_found'] for r in rows),
        scope='Fixed upright cylinder/-X front; center XYZ optimized jointly with three pen-normal IK configurations. Saved limits, no collisions/continuity proof; finite local optimizations, not a global infeasibility proof.',
        center_bounds_m=[[.15,-.15,.15],[.45,.15,.35]],seed=43,max_nfev=180,results=rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--budget',type=float,default=.8)
    parser.add_argument('--joint-search',type=int,default=0,help='Also jointly fit placement and three endpoint configurations with this many starts')
    parser.add_argument('--skip-grid',action='store_true')
    args=parser.parse_args()
    if not 0<=args.joint_search<=256:parser.error('Joint-search starts must be 0..256')
    if args.skip_grid and not args.joint_search:parser.error('Skip-grid requires a joint search')
    if not math.isfinite(args.budget) or not .1<=args.budget<=10:parser.error('Budget must be 0.1..10 seconds')
    model=(ROOT/'src/pinchy_description/urdf/pinchy_arm.urdf').read_text();profile=json.loads((ROOT/'calibration/arm.json').read_text());limits=model_limits(model)
    for name in JOINTS:limits[name].update(lower=profile['joints'][name]['min_radians'],upper=profile['joints'][name]['max_radians'])
    centers=[(x,0.,z) for x in (.2,.25,.3,.35,.4,.45) for z in (.15,.2,.25,.3,.35)]
    centers +=[(x,y,z) for x in (.2,.25,.3) for y in (.1,.15,.2) for z in (.2,.25,.3)]
    centers +=[(.4,.1,.22)]
    if args.skip_grid:centers=[]
    report=dict(physical_motors_moved=False,collision_checked=False,complete_paths_checked=False,endpoint_budget_s=args.budget,
        model_sha256=hashlib.sha256(model.encode()).hexdigest(),profile_sha256=hashlib.sha256((ROOT/'calibration/arm.json').read_bytes()).hexdigest(),
        scope='Fixed vertical cylinder, fixed -X front reference; only -90/0/+90 endpoints. Finite alternative placement search, no global placement infeasibility claim.',placements=[])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    for center in centers:
        row=dict(center_m=list(center),samples=[]);report['placements'].append(row)
        for angle in (-90.,0.,90.):
            solver=BoundedIK(model,limits);p,d=surface_pose('cylinder',center,.04*np.radians(angle),0.)
            target=axis_frame(p,d);bound=wrist_reach_bound(solver,target);q=None
            if not bound['certainly_outside_wrist_reach']:
                q=solver.search(target,np.zeros(6),'tool_axis',budget=args.budget,position_tolerance=1e-6,angle_tolerance=1e-5)
            row['samples'].append(dict(angle_deg=angle,necessary_reach_bound=bound,ik_found=q is not None,positions_rad=q,
                search=solver.last_report if not bound['certainly_outside_wrist_reach'] else None))
        row['endpoint_witness_count']=sum(s['ik_found'] for s in row['samples'])
        print(center,row['endpoint_witness_count'],'/3',flush=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
    if args.joint_search:report['joint_placement_search']=joint_placement_search(model,limits,args.joint_search)
    report['all_three_endpoint_witness_placements']=sum(r['endpoint_witness_count']==3 for r in report['placements'])
    args.output.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
