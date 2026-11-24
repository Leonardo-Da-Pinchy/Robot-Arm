exec(open('/tmp/analyze_tabletop_tilt_paths.py').read().split('for tilt_deg,azimuth_deg in ')[0])
import xml.etree.ElementTree as ET
rows=[];data=cases[0]
for elevation,length in [(0.,.144),(0.,.139),(.0381,.109),(.0508,.089)]:
 model=ET.fromstring(xml);model.find("joint[@name='pen_tip_fixed']/origin").set('xyz',f'0 {-length} 0.0188');solver=BoundedIK(model,limits)
 ts=[]
 for p in data['waypoints']:
  xyz=np.array(p['xyz']);xyz[2]+=elevation;ts.append(axis_frame(xyz,[0,0,-1]))
 curve=CartesianCurve(ts,data['style']);began=time.monotonic();row=dict(height_added_mm=elevation*1000,axial_tip_offset_mm=length*1000,tilt_deg=0,collision_checked=False,physical_motors_moved=False)
 start=solver.search(curve.evaluate(0,'tool_axis'),np.zeros(6),'tool_axis',budget=2,attempts=64)
 try:
  if start is None:raise ValueError('No start IK')
  d,q=sample_ik(curve,'tool_axis',start,solver,lambda q:True,lambda:time.monotonic()-began>30)
  row.update(geometric_path=True,minimum_joint_reserve_deg=float(np.degrees(np.min(np.minimum(q-solver.lower,solver.upper-q)))))
  tr,s=timed_trace(curve,d,q,solver,limits,'tool_axis',.018,.08,[False]*(len(ts)-1),tcp_min_speed=.004,enforce_tcp_speed=True)
  row.update(timing_passed=True,summary=s)
 except Exception as error:row.update(timing_passed=False,error=str(error))
 row['seconds']=time.monotonic()-began;rows.append(row);print(json.dumps(row),flush=True);Path('/tmp/tabletop-tool-height-study.json').write_text(json.dumps(rows,indent=2)+'\n')
