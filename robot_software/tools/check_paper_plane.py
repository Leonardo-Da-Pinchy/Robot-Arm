#!/usr/bin/python3
"""Preview exact tabletop examples on an isolated saved-limit mock; never execute."""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np
from pinchy_motion.point_sets import read
from pinchy_motion.geometry import model_limits
from pinchy_motion.ik import BoundedIK
from pinchy_motion.feasibility import axis_frame
from pinchy_bringup.control_contract import JOINTS
from analyze_motion_feasibility import MockChecks,ROOT
from inspect_paper_placement import inspect
from qualify_nominal_paths import call_plan,request_for


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=Path('/tmp/tabletop-paper-preflight'))
    args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    scene=ROOT/'src/pinchy_moveit_config/config/tabletop_paper_dry_run.yaml'
    checks=MockChecks(args.output_dir,ROOT/'calibration/arm.json',scene=scene)
    report=dict(placement=inspect(),physical_motors_moved=False,execution_client_created=False,cases=[])
    try:
        limits=model_limits(checks.models[-1]);report['zero_collision_free']=checks.valid(np.zeros(6))
        report['input_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'calibration/arm.json',scene)}
        report['loaded_model_sha256']=hashlib.sha256(checks.models[-1].encode()).hexdigest()
        files=[ROOT/'src/pinchy_motion/examples/point_sets/09_tabletop_circle_100mm.json',*sorted((ROOT/'src/pinchy_motion/examples/point_sets').glob('10_tabletop_square_*a_side.json'))]
        for path in files:
            data=read(path);print('Preflight:',path.name,flush=True);began=time.monotonic()
            response=call_plan(checks,request_for(data,limits))
            item=dict(file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),success=response.success,message=response.message,
                elapsed_s=time.monotonic()-began,plan_retained=bool(response.plan_id),executed=False)
            report['cases'].append(item);print(json.dumps(item),flush=True)
            (args.output_dir/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        # Separate saved-vs-CAD center witness: diagnostic only, no commands and
        # no claim that one witness guarantees a full outline or safe new bounds.
        target=axis_frame([*report['placement']['center_m'][:2],report['placement']['dry_run_tcp_z_m']],[0.,0.,-1.])
        witnesses={}
        for label,xml,lim in [('saved',checks.models[-1],limits),('CAD',(ROOT/'src/pinchy_description/urdf/pinchy_arm.urdf').read_text(),None)]:
            if lim is None:lim=model_limits(xml)
            solver=BoundedIK(xml,lim);q=solver.search(target,np.zeros(6),'tool_axis',budget=5.,attempts=64)
            witnesses[label]=dict(solved=q is not None,diagnostic=solver.last_report,q_deg=np.degrees(q).tolist() if q is not None else None,collision_checked=False)
            if q is not None and label=='CAD':
                witnesses[label]['outside_saved_joints']=[n for n,v in zip(JOINTS,q) if not limits[n]['lower']<=v<=limits[n]['upper']]
        report['center_ik_witnesses']=witnesses
        checks.rclpy.spin_once(checks.node,timeout_sec=.1)
        report['status_tcp_speed']={k:checks.status.get(k) for k in ('current_tcp_speed_m_s','commanded_tcp_speed_m_s','tcp_speed_source')}
    finally:
        (args.output_dir/'report.json').write_text(json.dumps(report,indent=2)+'\n');checks.close()


if __name__=='__main__':main()
