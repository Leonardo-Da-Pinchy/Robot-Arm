#!/usr/bin/python3
"""Compare local and whole-stroke selection on the recorded hypothetical arcs.

No action client or serial connection. Optional MoveIt uses an isolated mock;
ramps remain on the hypothetical surface, so this never creates a contact plan.
"""
import argparse
import hashlib
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from analyze_motion_feasibility import MockChecks,ROOT
from pinchy_motion.feasibility import CylinderArc
from pinchy_motion.geometry import model_limits,sampled_positions
from pinchy_motion.ik import BoundedIK
from pinchy_motion.paths import sample_ik,select_trace,timed_trace


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--moveit',action='store_true')
    parser.add_argument('--budget',type=float,default=45.)
    parser.add_argument('--output-dir',type=Path,default=Path('/tmp/stroke-selection'))
    args=parser.parse_args()
    if not 1<=args.budget<=120:parser.error('budget must be 1..120 seconds per search')
    folder=args.output_dir;folder.mkdir(parents=True,exist_ok=True)
    checks=None
    report=dict(motor_commands_sent=False,collision_checked=args.moveit,
        scope='Hypothetical geometry, unmeasured TCP; on-arc numerical ramps, not approved contact or pen-up motion. Finite search, not coverage qualification.',
        profile_sha256=hashlib.sha256((ROOT/'calibration/arm.json').read_bytes()).hexdigest(),cases=[])
    try:
        if args.moveit:checks=MockChecks(folder,ROOT/'calibration/arm.json')
        root=ET.parse(ROOT/'src/pinchy_description/urdf/pinchy_arm.urdf').getroot()
        model=checks.models[-1] if checks else root
        limits=model_limits(model)
        # The offline URDF is nominal; use the same saved bounds as live bringup.
        from pinchy_bringup.control_contract import JOINTS
        profile=json.loads((ROOT/'calibration/arm.json').read_text())
        for name in JOINTS:limits[name].update(lower=profile['joints'][name]['min_radians'],upper=profile['joints'][name]['max_radians'])
        valid=checks.valid if checks else lambda q:True
        for name,cx,z,start,end,speed,acceleration in [
            ('continuation_failure',.4,.2,-60,60,.01,.05),
            ('broad_timing',.35,.15,-60,60,.01,.05),
            ('narrow_timing',.35,.15,-45,-10,.005,.02)]:
            curve=CylinderArc(cx,.04,z,60,start_angle_deg=start,end_angle_deg=end)
            fresh=BoundedIK(model,limits)
            q=fresh.search(curve.evaluate(0),np.zeros(6),'tool_axis',valid=valid,budget=5.)
            row=dict(name=name,center_x_m=cx,height_m=z,radius_m=.04,angle_interval_deg=[start,end],start_q=q,methods={})
            report['cases'].append(row)
            if q is None:row['status']='start_unresolved';continue
            for method in ('local','whole_stroke'):
                fresh=BoundedIK(model,limits);began=time.monotonic();rejections=[]
                def stopped():return time.monotonic()-began>args.budget
                def finish(d,p):
                    original=curve.waypoint_distances
                    curve.waypoint_distances=np.array([0.,.1*curve.length,.9*curve.length,curve.length])
                    try:
                        trajectory,summary=timed_trace(curve,d,p,fresh,limits,'tool_axis',speed,acceleration,[False,True,False])
                        if stopped():raise RuntimeError('Search time budget exceeded')
                        if checks:
                            for candidate in sampled_positions(trajectory):
                                if stopped():raise RuntimeError('Search time budget exceeded')
                                if not valid(candidate):raise ValueError('Timed interpolation collides')
                            for candidate in p:checks.verify_fk(fresh,candidate)
                        return summary
                    except (ValueError,RuntimeError) as error:
                        rejections.append(str(error));raise
                    finally:curve.waypoint_distances=original
                outcome={}
                try:
                    if method=='local':
                        d,p=sample_ik(curve,'tool_axis',q,fresh,valid,stopped)
                        summary=finish(d,p);selection=dict(method='local_continuation')
                    else:d,p,summary,selection=select_trace(curve,'tool_axis',q,fresh,valid,stopped,finish)
                    outcome.update(success=True,selection=selection,timing=summary,distances_m=d.tolist(),positions=p.tolist(),
                        minimum_reserve_deg=float(np.degrees(np.minimum(p-fresh.lower,fresh.upper-p).min())))
                except (ValueError,RuntimeError) as error:
                    outcome.update(success=False,reason=str(error))
                    if hasattr(error,'selection_report'):outcome['selection']=error.selection_report
                    if hasattr(error,'target'):outcome.update(failed_distance_m=error.distance,failed_target=error.target.tolist())
                outcome.update(elapsed_s=time.monotonic()-began,complete_candidate_rejections=rejections)
                row['methods'][method]=outcome
                (folder/'report.json').write_text(json.dumps(report,indent=2)+'\n')
                print(name,method,outcome['success'],outcome.get('reason',outcome.get('selection')),flush=True)
    finally:
        if checks:checks.close()
        (folder/'report.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
