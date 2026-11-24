exec(open('/tmp/analyze_tabletop_tilt_paths.py').read().split('for tilt_deg,azimuth_deg in ')[0])
data=cases[3];rows=[]
for tilt_deg,azimuth_deg in [(5,90),(5,45),(5,135),(5,315),(5,225),(5,0),(2.5,90)]:
 angle,phi=np.radians([tilt_deg,azimuth_deg]);normal=[np.sin(angle)*np.cos(phi),np.sin(angle)*np.sin(phi),-np.cos(angle)]
 solver=BoundedIK(xml,limits);curve=CartesianCurve([axis_frame(p['xyz'],normal) for p in data['waypoints']],data['style']);seen=[]
 for attempt in range(3):
  began=time.monotonic();row=dict(tilt_deg=tilt_deg,azimuth_deg=azimuth_deg,case='square_side_3',attempt=attempt,collision_checked=False,physical_motors_moved=False)
  start=solver.search(curve.evaluate(0,'tool_axis'),np.zeros(6),'tool_axis',valid=lambda q:all(np.linalg.norm(np.array(q)-v)>.15 for v in seen),budget=1.5,attempts=64)
  try:
   if start is None:raise ValueError('No alternative start IK')
   seen.append(np.array(start));d,q=sample_ik(curve,'tool_axis',start,solver,lambda q:True,lambda:time.monotonic()-began>20)
   row.update(geometric_path=True,minimum_joint_reserve_deg=float(np.degrees(np.min(np.minimum(q-solver.lower,solver.upper-q)))))
   tr,s=timed_trace(curve,d,q,solver,limits,'tool_axis',.018,.08,[False]*(len(data['waypoints'])-1),tcp_min_speed=.004,enforce_tcp_speed=True)
   row.update(timing_passed=True,summary=s);np.savez('/tmp/tabletop-square-side3-cone-witness.npz',d=d,q=q,normal=normal)
  except Exception as error:row.update(timing_passed=False,error=str(error))
  row['seconds']=time.monotonic()-began;rows.append(row);print(json.dumps(row),flush=True);Path('/tmp/tabletop-square-side3-cone-study.json').write_text(json.dumps(rows,indent=2)+'\n')
  if row.get('timing_passed'):break
 if rows[-1].get('timing_passed'):break
