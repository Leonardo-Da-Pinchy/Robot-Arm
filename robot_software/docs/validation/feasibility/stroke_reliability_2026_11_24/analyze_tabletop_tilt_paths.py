"""Offline candidate comparison only; not an installed normal-tolerance feature."""
import json,time,numpy as np
from pathlib import Path
from pinchy_motion.ik import BoundedIK
from pinchy_motion.geometry import model_limits
from pinchy_motion.feasibility import axis_frame
from pinchy_motion.nominal_paths import paper_plane_loop,paper_square_strokes
from pinchy_motion.paths import CartesianCurve,sample_ik,timed_trace
from pinchy_bringup.control_contract import JOINTS
root=Path('/home/leo/Robot-Arm/robot_software');xml=(root/'src/pinchy_description/urdf/pinchy_arm.urdf').read_text();profile=json.loads((root/'calibration/arm.json').read_text());limits=model_limits(xml)
for n in JOINTS:limits[n].update(lower=profile['joints'][n]['min_radians'],upper=profile['joints'][n]['max_radians'],velocity=np.deg2rad(30),acceleration=np.deg2rad(180))
cases=[paper_plane_loop(),*paper_square_strokes()[::2]];report=[]
for tilt_deg,azimuth_deg in [(5,0),(5,180),(5,90),(5,270),(2.5,0),(2.5,180)]:
 angle,phi=np.radians([tilt_deg,azimuth_deg]);normal=[np.sin(angle)*np.cos(phi),np.sin(angle)*np.sin(phi),-np.cos(angle)];group=[]
 for index,data in enumerate(cases):
  solver=BoundedIK(xml,limits);curve=CartesianCurve([axis_frame(p['xyz'],normal) for p in data['waypoints']],data['style']);began=time.monotonic();row=dict(tilt_deg=tilt_deg,azimuth_deg=azimuth_deg,case='circle' if index==0 else f'square_side_{index}',collision_checked=False,physical_motors_moved=False)
  def stopped():return time.monotonic()-began>30
  start=solver.search(curve.evaluate(0,'tool_axis'),np.zeros(6),'tool_axis',budget=1.5,attempts=48)
  try:
   if start is None:raise ValueError('No start IK in finite search')
   d,q=sample_ik(curve,'tool_axis',start,solver,lambda q:True,stopped)
   row.update(geometric_path=True,states=len(d),minimum_joint_reserve_deg=float(np.degrees(np.min(np.minimum(q-solver.lower,solver.upper-q)))))
   np.savez(f'/tmp/tabletop-tilt-{tilt_deg}-{azimuth_deg}-{index}.npz',d=d,q=q)
   tr,summary=timed_trace(curve,d,q,solver,limits,'tool_axis',.018,.08,[False]*(len(data['waypoints'])-1),tcp_min_speed=.004,enforce_tcp_speed=True,stop=stopped)
   row.update(timing_passed=True,summary=summary)
  except Exception as error:row.update(timing_passed=False,error=str(error))
  row['seconds']=time.monotonic()-began;group.append(row);report.append(row);print(json.dumps(row),flush=True);Path('/tmp/tabletop-tilt-path-study.json').write_text(json.dumps(report,indent=2)+'\n')
  if index==0 and not row.get('geometric_path'):break
 if len(group)==5 and all(r.get('timing_passed') for r in group):break
