#!/usr/bin/python3
"""Whole-path boundary checks on isolated nominal mocks; never executes a goal."""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np
from analyze_motion_feasibility import MockChecks,ROOT
from qualify_nominal_paths import call_plan,request_for,trajectory_metrics
from pinchy_motion.nominal_paths import coverage_strokes
from pinchy_motion import point_sets
from pinchy_motion.ik import BoundedIK
from pinchy_motion.geometry import model_limits,sampled_positions,seconds
from pinchy_motion.paths import CartesianCurve,sample_ik,timed_trace
from pinchy_bringup.control_contract import DEFAULT_SPEED,DEFAULT_ACCELERATION


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--surfaces',nargs='+',choices=['cylinder','plane','quarter_pipe'],default=['cylinder','plane','quarter_pipe'])
    parser.add_argument('--cases',nargs='+',choices=['lower_horizontal','center_vertical','upper_horizontal'],default=['lower_horizontal','center_vertical','upper_horizontal'])
    parser.add_argument('--output-dir',type=Path,required=True);args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    report=dict(physical_motors_moved=False,execute_clients_created=False,scope='Proposed pedestal fixtures; complete boundary/height witnesses, not arbitrary path or physical coverage',surfaces=[])
    def save():(args.output_dir/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for kind in args.surfaces:
        folder=args.output_dir/kind;folder.mkdir(exist_ok=True);scene=ROOT/f'src/pinchy_moveit_config/config/nominal_{kind}_pedestal_dry_run.yaml'
        item=dict(surface=kind,scene_file=str(scene),scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),cases=[]);report['surfaces'].append(item);checks=None
        try:
            checks=MockChecks(folder,ROOT/'calibration/arm.json',scene=scene)
            limits=model_limits(checks.models[-1]);solver=BoundedIK(checks.models[-1],limits)
            item.update(model_sha256=hashlib.sha256(checks.models[-1].encode()).hexdigest(),profile_sha256=hashlib.sha256((ROOT/'calibration/arm.json').read_bytes()).hexdigest(),zero_valid=checks.valid(np.zeros(6)))
            for raw in coverage_strokes(kind):
                name=raw['source']['case']
                if name not in args.cases:continue
                data=point_sets.normalize(raw);point_sets.write(folder/f'{name}.json',data)
                case=dict(case=name,point_set=data);item['cases'].append(case);began=time.monotonic()
                print('PATH',kind,name,flush=True)
                try:
                    response=call_plan(checks,request_for(data,limits));case.update(success=response.success,message=response.message,planning_elapsed_s=time.monotonic()-began)
                    if not response.success:print('REJECT',response.message,flush=True);save();continue
                    metrics=trajectory_metrics(response,solver,data);metrics.pop('samples');case['metrics']=metrics
                    for q in list(sampled_positions(response.trajectory))[::25]:checks.verify_fk(solver,q)
                    case['independent_moveit_fk']=True
                    last=np.array(response.trajectory.joint_trajectory.points[-1].positions);tip=solver.evaluate(last)[0];lift=tip.copy();lift[:3,3]-=tip[:3,2]*.010
                    curve=CartesianCurve([tip,lift],'polyline');d,q=sample_ik(curve,'tool_axis',last,solver,checks.valid,lambda:False)
                    lift_limits={n:dict(v,velocity=min(v['velocity'],DEFAULT_SPEED),acceleration=min(v['acceleration'],DEFAULT_ACCELERATION)) for n,v in limits.items()}
                    trajectory,_=timed_trace(curve,d,q,solver,lift_limits,'tool_axis',.003,.02,[False],enforce_tcp_speed=True)
                    if not all(checks.valid(q) for q in sampled_positions(trajectory)):raise RuntimeError('Retract collision')
                    case['retract']=dict(success=True,distance_m=.010,duration_s=seconds(trajectory.joint_trajectory.points[-1]))
                    print('PASS',kind,name,flush=True)
                except (RuntimeError,ValueError) as error:case.update(success=False,error=str(error));print('ERROR',error,flush=True)
                save()
        finally:
            if checks:checks.close()
            save()
    return 0 if all(c.get('success') for item in report['surfaces'] for c in item['cases']) else 1


if __name__=='__main__':raise SystemExit(main())
