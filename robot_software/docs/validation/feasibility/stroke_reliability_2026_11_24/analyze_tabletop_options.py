"""Offline finite multi-start study only; no ROS, production edits or motor I/O."""
import json,time,hashlib
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from pinchy_motion.ik import BoundedIK
from pinchy_motion.geometry import model_limits
from pinchy_bringup.control_contract import JOINTS
root=Path('/home/leo/Robot-Arm/robot_software');xml=(root/'src/pinchy_description/urdf/pinchy_arm.urdf').read_text();profile=json.loads((root/'calibration/arm.json').read_text());limits=model_limits(xml)
for n in JOINTS:limits[n].update(lower=profile['joints'][n]['min_radians'],upper=profile['joints'][n]['max_radians'])
ik=BoundedIK(xml,limits);n=np.array([0.,0.,-1.]);rng=np.random.default_rng(126);lo,hi=ik.lower,ik.upper
seeds=[(lo+hi)/2,*[lo+s*(hi-lo) for s in ik.global_seeds[:8]]]
points=[('center',0,0),('circle_far',.05,0),('circle_near',-.05,0),('circle_left',0,-.05),('circle_right',0,.05),('square_near_left',-.05,-.05),('square_far_left',.05,-.05),('square_far_right',.05,.05),('square_near_right',-.05,.05)]
report=dict(profile_sha256=hashlib.sha256((root/'calibration/arm.json').read_bytes()).hexdigest(),model_sha256=hashlib.sha256(xml.encode()).hexdigest(),collision_checked=False,physical_motors_moved=False,search='9 finite multistarts per point, SLSQP maximizes down-axis alignment subject to exact XYZ and saved bounds; local results are witnesses, not global minima',cases=[])
for elevation in [0.,.0381,.0508]:
 warm=[]
 for label,dx,dy in points:
  began=time.monotonic();p=np.array([.2043352923245813+dx,dy,.0026+elevation]);best=None;valid=0
  def state(q):
   t,j,a=ik.evaluate(q);return t,j,np.cross(a.T,t[:3,2]).T
  def objective(q):t,j,a=state(q);return 1.-n@t[:3,2]
  def derivative(q):t,j,a=state(q);return -n@a
  constraints=[dict(type='eq',fun=lambda q:(ik.evaluate(q)[0][:3,3]-p)*10,jac=lambda q:ik.evaluate(q)[1]*10)]
  for seed in [*warm[-1:],*seeds]:
   fit=minimize(objective,np.clip(seed,lo+1e-9,hi-1e-9),jac=derivative,bounds=list(zip(lo,hi)),constraints=constraints,method='SLSQP',options=dict(maxiter=150,ftol=1e-11))
   t,_,_=ik.evaluate(fit.x);error=np.linalg.norm(t[:3,3]-p);angle=float(np.degrees(np.arccos(np.clip(n@t[:3,2],-1,1))))
   if error<1e-6:
    valid+=1
    if best is None or angle<best[0]:best=(angle,fit.x.copy(),error)
  if best:
   angle,q,error=best;warm.append(q);margins=np.minimum(q-lo,hi-q)
   row=dict(height_added_mm=elevation*1000,target=label,xyz_m=p.tolist(),best_tilt_deg=angle,position_error_m=error,within_5deg=angle<=5.000001,strict_witness=angle<=.001,valid_fits=valid,seconds=time.monotonic()-began,q_deg=np.degrees(q).tolist(),nearest_bounds=[dict(joint=JOINTS[i],reserve_deg=float(np.degrees(margins[i])),at='lower' if q[i]-lo[i]<hi[i]-q[i] else 'upper') for i in np.argsort(margins)[:3]])
  else:row=dict(height_added_mm=elevation*1000,target=label,xyz_m=p.tolist(),best_tilt_deg=None,valid_fits=0,seconds=time.monotonic()-began)
  report['cases'].append(row);print(json.dumps(row),flush=True);Path('/tmp/tabletop-options-study.json').write_text(json.dumps(report,indent=2)+'\n')
