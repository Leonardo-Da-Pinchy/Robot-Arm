import numpy as np,json,time
from pinchy_motion.point_sets import read
from pinchy_motion.paths import CartesianCurve,timed_trace
from pinchy_motion.feasibility import axis_frame
from scipy.spatial.transform import Rotation
from pinchy_motion.geometry import model_limits
from pinchy_motion.ik import BoundedIK
from pinchy_bringup.control_contract import JOINTS
from pathlib import Path
root=Path('src/pinchy_description/urdf/pinchy_arm.urdf').read_text();lim=model_limits(root);p=json.load(open('calibration/arm.json'))
for n in JOINTS:lim[n].update(lower=p['joints'][n]['min_radians'],upper=p['joints'][n]['max_radians'],velocity=np.deg2rad(30),acceleration=np.deg2rad(180))
solver=BoundedIK(root,lim);data=read('src/pinchy_motion/examples/point_sets/07_cylinder_continuous_3in_fast.json');curve=CartesianCurve([axis_frame(p['xyz'],Rotation.from_euler('xyz',p['rpy_deg'],degrees=True).as_matrix()[:,2]) for p in data['waypoints']],data['style']);q=np.load('docs/validation/engineering/operating_envelope_2026_11_22/geometric_branch.npz');began=time.monotonic()
rows=[]
for label,speed,acc,minimum in [('fixed',.007,.08,0.),('range',.018,.08,.004)]:
 began=time.monotonic();cpu=time.process_time()
 tr,report=timed_trace(curve,q['d'],q['q'],solver,lim,'tool_axis',speed,acc,[False]*60,enforce_tcp_speed=True,tcp_min_speed=minimum)
 rows.append(dict(mode=label,wall_seconds=time.monotonic()-began,cpu_seconds=time.process_time()-cpu,points=len(tr.joint_trajectory.points),duration_s=report['trace_end_s'],range_report=report['speed_range']))
 print(rows[-1],flush=True)
json.dump(rows,open('/tmp/range_timing_benchmark_reproduced.json','w'),indent=2)
