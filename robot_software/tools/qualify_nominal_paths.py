#!/usr/bin/python3
"""Nominal-surface preflight on isolated mocks; never creates an execute client."""
import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
import yaml
from geometry_msgs.msg import PoseStamped,Quaternion
from scipy.spatial.transform import Rotation
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from pinchy_motion_interfaces.srv import PlanPath
from pinchy_motion import point_sets
from pinchy_motion.nominal_paths import dry_run_loop,surface_pose,surface_measurement
from pinchy_motion.ik import BoundedIK
from pinchy_motion.geometry import model_limits,sampled_positions,segment_coefficients,seconds
from pinchy_bringup.control_contract import JOINTS,DEFAULT_SPEED,DEFAULT_ACCELERATION
from analyze_motion_feasibility import MockChecks,ROOT


def request_for(data,limits):
    request=PlanPath.Request(style=data['style'],orientation_mode=data['orientation_mode'],tcp_speed=data['tcp_speed_m_s'],tcp_min_speed=data.get('tcp_min_speed_m_s',0.),
        tcp_acceleration=data['tcp_acceleration_m_s2'],enforce_tcp_speed=data['enforce_tcp_speed'],plan_pen_up_approach=data['plan_pen_up_approach'],
        allow_contact_ramps=data['allow_contact_ramps'],velocity_scaling=min(1.,np.deg2rad(data.get('joint_speed_deg_s',30.))/max(v['velocity'] for v in limits.values())),
        acceleration_scaling=min(1.,np.deg2rad(data.get('joint_acceleration_deg_s2',180.))/min(v['acceleration'] for v in limits.values())))
    for point in data['waypoints']:
        pose=PoseStamped();pose.header.frame_id='base_link';pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=point['xyz']
        q=Rotation.from_euler('xyz',point['rpy_deg'],degrees=True).as_quat()
        pose.pose.orientation=Quaternion(x=float(q[0]),y=float(q[1]),z=float(q[2]),w=float(q[3]));request.waypoints.append(pose)
        request.contact_segments.append(point['contact'])
    return request


def call_plan(checks,request,timeout=480):
    client=checks.node.create_client(PlanPath,'/pinchy_motion/plan_path')
    try:
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('PlanPath unavailable')
        future=client.call_async(request);started=time.monotonic();next_update=started+15
        while not future.done() and time.monotonic()-started<timeout:
            checks.rclpy.spin_once(checks.node,timeout_sec=.1)
            if time.monotonic()>next_update:print('  planning',round(time.monotonic()-started),'seconds',flush=True);next_update+=15
        if not future.done():raise RuntimeError('Nominal preflight request timed out')
        return future.result()
    finally:checks.node.destroy_client(client)


def trajectory_metrics(response,solver,data):
    samples=[];center=np.array(data['source']['center_m']);kind=data['source']['surface']
    for first,last in zip(response.trajectory.joint_trajectory.points,response.trajectory.joint_trajectory.points[1:]):
        duration,coeff=segment_coefficients(first,last)
        for local in np.linspace(0,duration,max(2,int(np.ceil(duration/.02))+1)):
            t=seconds(first)+local
            if not response.trace_start_time-1e-8<=t<=response.trace_end_time+1e-8:continue
            q=coeff@np.array([local**i for i in range(6)])
            dq=coeff[:,1:]@np.array([i*local**(i-1) for i in range(1,6)])
            transform,jac,_=solver.evaluate(q);p=transform[:3,3]
            expected,gap=surface_measurement(kind,center,p)
            angle=float(np.arccos(np.clip(expected@transform[:3,2],-1,1)))
            samples.append([t,*p,float(np.linalg.norm(jac@dq)),angle,gap])
    values=np.array(samples);steady=values[(values[:,0]>=response.cruise_start_time+.02)&(values[:,0]<=response.cruise_end_time-.02)]
    if not len(steady):raise RuntimeError('No steady-speed samples in accepted trace')
    speed=data['tcp_speed_m_s']
    minimum=data.get('tcp_min_speed_m_s',0.)
    if minimum:
        if steady[:,4].min()<minimum-1e-7 or steady[:,4].max()>speed+1e-7:raise RuntimeError('Independent TCP range assay failed')
    elif np.max(abs(steady[:,4]-speed))>speed*.05:raise RuntimeError('Independent speed assay failed')
    if np.max(abs(values[:,6]-data['source']['stand_off_m']))>.001:raise RuntimeError('Independent surface stand-off assay failed')
    if np.max(values[:,5])>.01:raise RuntimeError('Independent cylinder/plane normal assay failed')
    return dict(trace_interval_s=[response.trace_start_time,response.trace_end_time],cruise_interval_s=[response.cruise_start_time,response.cruise_end_time],
        sampled_cruise_min_m_s=float(steady[:,4].min()),sampled_cruise_max_m_s=float(steady[:,4].max()),
        min_cruise_speed_ratio=float(steady[:,4].min()/speed),max_normal_error_deg=float(np.degrees(values[:,5].max())),
        minimum_tcp_stand_off_m=float(values[:,6].min()),maximum_tcp_stand_off_m=float(values[:,6].max()),
        xyz_span_m=np.ptp(values[:,1:4],axis=0).tolist(),samples=values.tolist())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--surfaces',nargs='+',choices=['cylinder','plane','quarter_pipe'],default=['cylinder','plane','quarter_pipe'])
    parser.add_argument('--output-dir',type=Path,default=Path('/tmp/nominal-path-qualification'))
    parser.add_argument('--support',choices=['original','pedestal'],default='original',help='Explicitly choose an assumed support; never changes the live scene')
    parser.add_argument('--skip-grid',action='store_true')
    parser.add_argument('--grid-only',action='store_true',help='Endpoint witnesses only; no complete-path claim')
    parser.add_argument('--full-grid',action='store_true',help='Sample full nominal angular/width extent, 5 mm inside top/bottom rims')
    parser.add_argument('--endpoint-budget',type=float,default=.8,help='Seconds per independent endpoint IK search')
    args=parser.parse_args()
    if not np.isfinite(args.endpoint_budget) or not .1<=args.endpoint_budget<=10:parser.error('Endpoint budget must be 0.1..10 seconds')
    if args.grid_only and args.skip_grid:parser.error('Grid-only and skip-grid cannot be combined')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    report=dict(motor_commands_sent=False,physical_qualification=False,endpoint_search_budget_s=args.endpoint_budget,scope='Hypothetical placements and fixture assumptions, current proxy TCP and saved bounds. Complete-path witnesses are not full-area coverage.',cases=[])
    for kind in args.surfaces:
        folder=args.output_dir/kind;folder.mkdir(exist_ok=True);suffix='_pedestal' if args.support=='pedestal' else ''
        scene=ROOT/f'src/pinchy_moveit_config/config/nominal_{kind}{suffix}_dry_run.yaml'
        item=dict(surface=kind,support=args.support,scene_file=str(scene),scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest());report['cases'].append(item)
        checks=None
        try:
            print('SURFACE',kind,flush=True);checks=MockChecks(folder,ROOT/'calibration/arm.json',scene=scene)
            limits=model_limits(checks.models[-1]);solver=BoundedIK(checks.models[-1],limits)
            item['profile_sha256']=hashlib.sha256((ROOT/'calibration/arm.json').read_bytes()).hexdigest()
            item['zero_collision_free']=checks.valid(np.zeros(6));data=point_sets.normalize(dry_run_loop(kind))
            item['point_set']=data
            scene_client=checks.node.create_client(GetPlanningScene,'/get_planning_scene')
            if not scene_client.wait_for_service(timeout_sec=5):raise RuntimeError('Scene inspection unavailable')
            req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            world=checks.call(scene_client,req).scene.world
            item['scene_objects']=[dict(id=obj.id,primitives=len(obj.primitives),meshes=len(obj.meshes)) for obj in world.collision_objects]
            checks.node.destroy_client(scene_client)
            if not args.grid_only:
                began=time.monotonic();response=call_plan(checks,request_for(data,limits));item.update(success=response.success,message=response.message,planning_elapsed_s=time.monotonic()-began)
                print('RESULT',kind,response.success,response.message,flush=True)
                if response.success:
                    metrics=trajectory_metrics(response,solver,data);item['metrics']=metrics
                    for q in list(sampled_positions(response.trajectory))[::20]:checks.verify_fk(solver,q)
                    item['independent_fk_checked']=True;item['trajectory_duration_s']=response.duration
                    # A separately checked 10 mm pen-up lift from the exact accepted
                    # end configuration. This is evidence, not an automatic move.
                    from pinchy_motion.paths import CartesianCurve,sample_ik,timed_trace
                    from pinchy_bringup.control_contract import JOINTS
                    last=np.array(response.trajectory.joint_trajectory.points[-1].positions)
                    tip=solver.evaluate(last)[0];lift=tip.copy();lift[:3,3]-=tip[:3,2]*.010
                    lift_curve=CartesianCurve([tip,lift],'polyline')
                    try:
                        ld,lq=sample_ik(lift_curve,'tool_axis',last,solver,checks.valid,lambda:False)
                        lift_limits={n:dict(v,velocity=min(v['velocity'],DEFAULT_SPEED),acceleration=min(v['acceleration'],DEFAULT_ACCELERATION)) for n,v in limits.items()}
                        retract,retract_metrics=timed_trace(lift_curve,ld,lq,solver,lift_limits,'tool_axis',.003,.02,[False],enforce_tcp_speed=True)
                        if not all(checks.valid(q) for q in sampled_positions(retract)):raise RuntimeError('Retract collision')
                        item['retract']=dict(success=True,distance_m=.010,duration_s=seconds(retract.joint_trajectory.points[-1]),executed=False)
                    except (RuntimeError,ValueError) as error:item['retract']=dict(success=False,error=str(error),executed=False)
                    (folder/'trace_samples.csv').write_text('t,x,y,z,tcp_speed,normal_error_rad,stand_off\n'+'\n'.join(','.join(map(str,row)) for row in metrics['samples'])+'\n')
                    del metrics['samples']
            if not args.skip_grid:
                grid=[]
                half_width=({'cylinder':.04*np.pi/2,'plane':.05,'quarter_pipe':.10*np.pi/4}[kind] if args.full_grid else .0254)
                half_height=.045 if args.full_grid else .0254
                columns=13 if args.full_grid and kind=='cylinder' else 9 if args.full_grid else 7
                item['grid_scope']='Full nominal angular/width extent, 5 mm inside top/bottom rims; endpoint IK only' if args.full_grid else 'Two-inch stroke bounding rectangle; endpoint IK only'
                for u in np.linspace(-half_width,half_width,columns):
                    for v in np.linspace(-half_height,half_height,5):
                        xyz,direction=surface_pose(kind,data['source']['center_m'],float(u),float(v),.005)
                        from pinchy_motion.feasibility import axis_frame
                        fresh=BoundedIK(checks.models[-1],limits);target=axis_frame(xyz,direction);checks.rejections.clear()
                        q=fresh.search(target,np.zeros(6),'tool_axis',valid=checks.valid,budget=args.endpoint_budget,attempts=64)
                        grid.append(dict(u_m=float(u),v_m=float(v),ik_collision_valid=q is not None,positions_rad=q,
                            failed_search=fresh.last_report if q is None else None,collision_rejections=checks.rejections[-3:],
                            reserve_deg=float(np.degrees(np.minimum(np.asarray(q)-fresh.lower,fresh.upper-np.asarray(q)).min())) if q is not None else None))
                if args.grid_only:item.update(success=True,message='Endpoint study completed; no whole-stroke plan requested')
                item['sampled_grid']=grid;item['sampled_valid_count']=sum(p['ik_collision_valid'] for p in grid)
                print('GRID',kind,item['sampled_valid_count'],'/',len(grid),flush=True)
        except Exception as error:
            item.update(success=False,error=str(error));print('ERROR',kind,error,flush=True)
        finally:
            if checks:checks.close()
            (args.output_dir/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if all(c.get('success') for c in report['cases']) else 1


if __name__=='__main__':raise SystemExit(main())
