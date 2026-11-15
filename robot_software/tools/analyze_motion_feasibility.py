#!/usr/bin/python3
"""Explore nearby poses, free-twist reserve and example arcs. NEVER commands motors.

--moveit launches its OWN calibrated mock on an isolated domain and checks the
shared collision geometry/example floor, plus independent FK. It never connects
to existing physical bringup. No execution service/action client exists here.
"""
import argparse,hashlib,json,math,os,signal,subprocess,tempfile,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from pinchy_bringup.control_contract import JOINTS,DEFAULT_SPEED,DEFAULT_ACCELERATION
from pinchy_motion.geometry import model_limits
from pinchy_motion.ik import BoundedIK
from pinchy_motion.paths import sample_ik,timed_trace
from pinchy_motion.feasibility import axis_frame,CylinderArc,posture_metrics,twist_family,speed_envelope
ROOT=Path(__file__).resolve().parents[1]


class MockChecks:
    def __init__(self,folder,profile):
        import rclpy
        from rclpy.qos import QoSProfile,DurabilityPolicy
        from std_msgs.msg import String
        from moveit_msgs.srv import GetPositionFK,GetStateValidity
        self.rclpy=rclpy;self.process=None;self.node=None;self.models=[];self.states={};self.rejections=[];self.calls=0
        self.domain=200+os.getpid()%25;env=dict(os.environ,ROS_DOMAIN_ID=str(self.domain),ROS_LOG_DIR=str(folder/'ros_logs'))
        self.log=(folder/'mock_launch.log').open('w')
        self.process=subprocess.Popen(['ros2','launch','pinchy_moveit_config','mock_motion.launch.py','rviz:=false','panel:=false',f'calibration:={profile}'],env=env,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init(domain_id=self.domain);self.node=rclpy.create_node('read_only_motion_feasibility')
        self.node.create_subscription(String,'/robot_description',lambda m:self.models.append(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.validity=self.node.create_client(GetStateValidity,'/check_state_validity');self.fk=self.node.create_client(GetPositionFK,'/compute_fk')
        deadline=time.monotonic()+45
        while time.monotonic()<deadline and not(self.models and self.validity.service_is_ready() and self.fk.service_is_ready()):
            if self.process.poll() is not None:raise RuntimeError('Isolated mock failed: '+str(folder))
            rclpy.spin_once(self.node,timeout_sec=.1)
        if not(self.models and self.validity.service_is_ready()):raise RuntimeError('Mock readiness timeout')
    def call(self,client,request):
        f=client.call_async(request);self.rclpy.spin_until_future_complete(self.node,f,timeout_sec=8)
        if not f.done():raise RuntimeError('Mock diagnostic service timed out')
        return f.result()
    def valid(self,q):
        from moveit_msgs.srv import GetStateValidity
        from sensor_msgs.msg import JointState
        key=tuple(np.round(q,8))
        if key not in self.states:
            req=GetStateValidity.Request();req.group_name='arm';req.robot_state.joint_state=JointState(name=JOINTS,position=list(map(float,q)))
            result=self.call(self.validity,req);self.calls+=1
            pairs=sorted(set('/'.join(sorted([c.contact_body_1,c.contact_body_2])) for c in result.contacts))
            self.states[key]=result.valid
            if not result.valid:self.rejections.append(pairs or ['MoveIt invalid state without contact pair'])
        return self.states[key]
    def verify_fk(self,solver,q):
        from moveit_msgs.srv import GetPositionFK
        from sensor_msgs.msg import JointState
        from pinchy_motion.paths import pose_matrix
        req=GetPositionFK.Request();req.header.frame_id='base_link';req.fk_link_names=['pointer_tip'];req.robot_state.joint_state=JointState(name=JOINTS,position=list(map(float,q)))
        result=self.call(self.fk,req)
        if result.error_code.val!=1:raise RuntimeError('Independent MoveIt FK unavailable')
        if not np.allclose(solver.evaluate(q)[0],pose_matrix(result.pose_stamped[0].pose),atol=1e-7):raise RuntimeError('Diagnostic and MoveIt FK disagree')
    def close(self):
        if self.node is not None:self.node.destroy_node()
        if self.rclpy.ok():self.rclpy.shutdown()
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try:self.process.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(self.process.pid,signal.SIGKILL);self.process.wait(timeout=5)
        self.log.close()


def neighbourhood(solver,q,valid):
    target=solver.evaluate(q)[0];trials=[]
    for axis in range(3):
        for sign in (-1,1):
            t=target.copy();t[axis,3]+=.002*sign;trials.append((f'{"XYZ"[axis]} {sign*2:+} mm',t))
    for axis in range(2):
        for sign in (-1,1):
            t=target.copy();t[:3,:3]=Rotation.from_rotvec(target[:3,axis]*math.radians(sign*5)).as_matrix()@target[:3,:3]
            trials.append((f'direction tangent {axis} {sign*5:+} deg',t))
    result=[]
    for name,t in trials:
        local=solver.search(t,q,'tool_axis',valid=lambda candidate: max(abs(np.asarray(candidate)-q))<=.15 and valid(candidate),budget=.35,attempts=1,anchor_weight=.001)
        local_report=solver.last_report.copy();global_q=None
        if local is None:global_q=solver.search(t,np.zeros(6),'tool_axis',valid=valid,budget=.8,attempts=20)
        result.append(dict(change=name,local_success=local is not None,alternate_solution_found=global_q is not None,
            local_report=local_report,solution=local or global_q))
    return result


def arc_check(root,limits,cx,z,half,speed,valid,checks,start_angle=None,end_angle=None,tcp_acceleration=.05):
    solver=BoundedIK(root,limits);curve=CylinderArc(cx,.04,z,half,start_angle_deg=start_angle,end_angle_deg=end_angle)
    result=dict(center_x_m=cx,center_y_m=0.,surface_height_m=z,radius_m=.04,half_angle_deg=half,angle_interval_deg=[-half if start_angle is None else start_angle,half if end_angle is None else end_angle],requested_speed_m_s=speed,tcp_acceleration_m_s2=tcp_acceleration,assumed_geometry=True,collision_checked=checks is not None)
    q=solver.search(curve.evaluate(0.),np.zeros(6),'tool_axis',valid=valid,budget=5.)
    if q is None:return dict(result,status='start_pose_unresolved',report=solver.last_report)
    if checks:checks.verify_fk(solver,q)
    started=time.monotonic()
    try:
        distances,positions=sample_ik(curve,'tool_axis',q,solver,valid,lambda:time.monotonic()-started>55)
        if checks:
            for candidate in positions:checks.verify_fk(solver,candidate)
        result.update(status='continuous_ik_found',samples=len(positions),start_q=q,end_q=positions[-1].tolist(),
            minimum_joint_margin_deg=min(posture_metrics(solver,q)['nearest_limit_deg'] for q in positions),
            worst_task_condition_number=max(solver.conditioning(q,'tool_axis')['condition_number'] for q in positions),
            sampled_speed_envelope=speed_envelope(distances,positions,limits,speed,tcp_acceleration))
        # A NON-EXECUTABLE numerical timing assay. Ramps along this arc are not
        # physical pen-up geometry; only the middle 80% tests steady timing.
        curve.waypoint_distances=np.array([0.,.1*curve.length,.9*curve.length,curve.length])
        try:
            trajectory,timing=timed_trace(curve,distances,positions,solver,limits,'tool_axis',speed,tcp_acceleration,[False,True,False])
            if checks:
                from pinchy_motion.geometry import sampled_positions
                if not all(valid(q) for q in sampled_positions(trajectory)):raise RuntimeError('Timed interpolation collides')
            result['timing_assay']=dict(success=True,**timing)
        except (ValueError,RuntimeError) as error:result['timing_assay']=dict(success=False,reason=str(error))
        result['timing_assay']['scope']='Interior 80% only; analysis ramps lie on the arc, not approved pen-up approaches. No executable plan created.'
    except RuntimeError as error:
        result.update(status='continuation_unresolved',reason=str(error),last_report=solver.last_report)
        if hasattr(error,'target'):
            alternate=BoundedIK(root,limits)
            witness=alternate.search(error.target,np.zeros(6),'tool_axis',valid=valid,budget=5.)
            result['failed_target_diagnostic']=dict(distance_m=error.distance,last_distance_m=error.last_distance,
                previous_positions=error.previous_positions,target_matrix=error.target.tolist(),
                broader_ik_witness=witness,report=alternate.last_report,
                interpretation='Broader IK exists but local continuation failed; not proof of a connected drawing path' if witness is not None else 'No broader IK found; not an impossibility proof')
    return result


def plots(report,folder):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,4.6))
    rows=report.get('kinematic_cylinder_sweep',[])
    if rows:
        matrix=np.array([[float(s['success']) for s in row['samples']] for row in rows])
        axes[0].imshow(matrix,aspect='auto',extent=[-97.5,97.5,len(rows)-.5,-.5],vmin=0,vmax=1,cmap='RdYlGn')
        axes[0].set_yticks(range(len(rows)),[f'x={r["cx"]:.2f}, z={r["z"]:.2f}' for r in rows],fontsize=8)
        axes[0].set_xlabel('Cylinder front angle (deg)');axes[0].set_title('Sampled IK only; green = witness found\nred = unresolved, NOT an impossibility proof')
    else:axes[0].axis('off')
    cases=report['neighbourhoods'];x=np.arange(len(cases));axes[1].bar(x-.18,[sum(t['local_success'] for t in c.get('original_trials',[])) for c in cases],.36,label='Original configuration')
    axes[1].bar(x+.18,[sum(t['local_success'] for t in c.get('reserve_trials',[])) for c in cases],.36,label='Same pose, alternative twist')
    axes[1].set_xticks(x,[c['name'] for c in cases],rotation=15,ha='right');axes[1].set_ylim(0,10.8)
    axes[1].set_ylabel('Nearby targets solved locally / 10');axes[1].legend(fontsize=8);axes[1].set_title('±2 mm XYZ and ±5° pen direction\nLocal = ≤0.15 rad joint change')
    fig.suptitle('Current model / saved bounds — exploratory software feasibility, no motor motion'+(' (IK only, no collisions)' if report['collision_scope'].startswith('NOT') else ''));fig.tight_layout();fig.savefig(folder/'feasibility.png',dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--global-endpoint',action='store_true',help='Also independently search one failed side-normal example with differential evolution')
    parser.add_argument('--moveit',action='store_true',help='Validate with own isolated mock; never attach to real robot')
    parser.add_argument('--joint-deg',nargs=6,type=float,metavar=('J1','J2','J3','J4','J5','J6'),help='Also diagnose this saved-range joint configuration; degrees, base to tip')
    parser.add_argument('--neighbourhood-only',action='store_true',help='Skip assumed cylinder arcs and hypothetical range checks')
    parser.add_argument('--sweep',action='store_true',help='Also sample 15 assumed cylinder placements; takes several minutes')
    parser.add_argument('--reuse-sweep',type=Path,help='Reuse this tool study sweep JSON for plotting; hash must match current model/profile')
    parser.add_argument('--output-dir',type=Path,default=Path('/tmp')/('pinchy-feasibility-'+str(os.getpid())))
    parser.add_argument('--tcp-speed',type=float,default=.01,help='Timing assay m/s; default 0.01, physical drawing speed unqualified')
    args=parser.parse_args()
    if not math.isfinite(args.tcp_speed) or args.tcp_speed<=0:parser.error('Positive finite TCP speed required')
    folder=args.output_dir.resolve();folder.mkdir(parents=True,exist_ok=True);profile=ROOT/'calibration/arm.json';model=ROOT/'src/pinchy_description/urdf/pinchy_arm.urdf'
    checks=None;report={}
    try:
        if args.moveit:
            checks=MockChecks.__new__(MockChecks);checks.__init__(folder,profile)
        root=ET.fromstring(checks.models[-1]) if checks else ET.parse(model).getroot();limits=model_limits(root);saved=json.loads(profile.read_text())
        for n in JOINTS:limits[n].update(lower=saved['joints'][n]['min_radians'],upper=saved['joints'][n]['max_radians'],velocity=min(DEFAULT_SPEED,limits[n]['velocity']),acceleration=min(DEFAULT_ACCELERATION,limits[n]['acceleration']))
        solver=BoundedIK(root,limits);valid=checks.valid if checks else lambda q:True
        report=dict(model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),profile_sha256=hashlib.sha256(profile.read_bytes()).hexdigest(),motor_commands_sent=False,
            collision_scope='MoveIt shared model and example floor; no object/fixture collision geometry' if checks else 'NOT collision checked',
            task='XYZ + inward pen +z; free twist',joint_rate_caps_deg=[30,180],tcp_speed_m_s=args.tcp_speed,
            limitations=['No final cylinder, fixture, TCP, tabletop or physical duty measurements.','Finite IK searches and sampled paths are not proofs of impossibility or full-region coverage.','No physical contact trajectory is generated; arc timing assays are mathematical steady-speed checks.'],neighbourhoods=[],arcs=[])
        def save(): (folder/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        if args.reuse_sweep:
            previous=json.loads(args.reuse_sweep.read_text())
            if any(previous[k]!=report[k] for k in ('model_sha256','profile_sha256')):raise ValueError('Sweep input model/profile changed')
            report['kinematic_cylinder_sweep']=previous['kinematic_cylinder_sweep']
        if args.sweep:
            rows=[]
            for cx in (.20,.25,.30,.35,.40):
                for z in (.10,.15,.20):
                    fresh=BoundedIK(root,limits);q=np.zeros(6);samples=[]
                    for deg in range(-90,91,15):
                        curve=CylinderArc(cx,.04,z,90);target=curve.evaluate(.04*math.radians(deg+90))
                        found=fresh.search(target,q,'tool_axis',budget=.7,attempts=32)
                        samples.append(dict(angle_deg=deg,success=found is not None,q=found,report=fresh.last_report.copy()))
                        if found is not None:q=found
                    rows.append(dict(cx=cx,z=z,radius=.04,successes=sum(s['success'] for s in samples),total=len(samples),samples=samples))
                    report['kinematic_cylinder_sweep']=rows;save();print('IK-only sweep',cx,z,rows[-1]['successes'],'/13',flush=True)
        # A deterministic random search finds a poorly conditioned current-model
        # configuration; it is NOT assumed collision free until checked below.
        rng=np.random.default_rng(71);candidates=[]
        for q in rng.uniform(solver.lower,solver.upper,(1500,6)):
            p=solver.evaluate(q)[0][:3,3]
            if .1<p[0]<.5 and abs(p[1])<.2 and .03<p[2]<.4:candidates.append(q)
        candidates.sort(key=lambda q:solver.conditioning(q,'tool_axis')['smallest_singular_value'])
        singular=next((q for q in candidates[:30] if valid(q)),None)
        cases=[('zero',np.zeros(6)),('bent wrist',np.array([.1,-.25,.5,-.25,.4,.4])),('wrist near limit',np.array([.1,-.25,.5,-.25,solver.upper[4]-.01,.4]))]
        if singular is not None:cases.append(('poor conditioning',singular))
        if args.joint_deg is not None:
            requested=np.radians(args.joint_deg)
            if not np.isfinite(requested).all() or np.any(requested<solver.lower) or np.any(requested>solver.upper):raise ValueError('Requested posture outside saved bounds')
            cases.append(('requested posture',requested))
        for name,q in cases:
            case=dict(name=name,q=q.tolist(),metrics=posture_metrics(solver,q),valid_start=bool(valid(q)),collision_checked=checks is not None)
            if case['valid_start']:
                if checks:checks.verify_fk(solver,q)
                family=twist_family(solver,solver.evaluate(q)[0],q,valid)
                # Preference is diagnostic, not a path-planning command.
                reserve=max(family,key=lambda q: min(posture_metrics(solver,q)['nearest_limit_deg'],15)-2*math.log10(max(1,solver.conditioning(q,'tool_axis')['condition_number'])))
                case.update(original_trials=neighbourhood(solver,q,valid),twist_configurations_found=len(family),reserve_q=reserve,reserve_metrics=posture_metrics(solver,reserve),reserve_trials=neighbourhood(solver,np.array(reserve),valid))
            report['neighbourhoods'].append(case);save();print('Neighbourhood',name,case.get('twist_configurations_found'),flush=True)
        for cx,z,half in ([] if args.neighbourhood_only else [(.35,.15,60),(.35,.15,90),(.25,.15,60),(.40,.20,60)]):
            if checks:checks.rejections.clear()
            result=arc_check(root,limits,cx,z,half,args.tcp_speed,valid,checks)
            if checks:result['observed_collision_pairs']=checks.rejections[-8:]
            report['arcs'].append(result);save();print('Arc',cx,z,half,result['status'],flush=True)
        if not args.neighbourhood_only:
            narrow=arc_check(root,limits,.35,.15,60,min(args.tcp_speed,.005),valid,checks,start_angle=-45,end_angle=-10,tcp_acceleration=.02)
            narrow['case']='Narrower off-center 35-degree segment, explicit 5 mm/s and 20 mm/s² timing assay'
            report['arcs'].append(narrow);save();print('Narrow off-center arc',narrow['status'],narrow.get('timing_assay'),flush=True)
        # Explicitly NON-EXECUTABLE range diagnostic; never alter real limits.
        report['hypothetical_joint6_range_diagnostics']=[]
        for degrees in (() if args.neighbourhood_only else (-90,90)):
            diagnostic=BoundedIK(root,limits);diagnostic.upper[5]=math.pi
            target=CylinderArc(.35,.04,.15,90).evaluate(.04*math.radians(degrees+90))
            found=diagnostic.search(target,np.zeros(6),'tool_axis',budget=5.)
            report['hypothetical_joint6_range_diagnostics'].append(dict(angle_deg=degrees,solution_deg=np.degrees(found).tolist() if found is not None else None,
                requires_joint6_deg=float(np.degrees(found[5])) if found is not None else None,executable=False,collision_checked=False,scope='Hypothetical 180-deg joint6 upper bound; exceeds CAD and saved range; never an authorized limit change.'))
        if args.global_endpoint:
            from scipy.optimize import differential_evolution
            independent=BoundedIK(root,limits);target=CylinderArc(.35,.04,.15,90).evaluate(0.)
            def objective(q):
                residual,_=independent.system(q,target,'tool_axis');return float(residual@residual)
            fit=differential_evolution(objective,list(zip(independent.lower,independent.upper)),popsize=10,maxiter=350,tol=1e-9,seed=91,polish=True)
            distance,angle=independent.errors(fit.x,target,'tool_axis')
            report['independent_endpoint_search']=dict(method='Finite differential evolution; not an impossibility proof',angle_deg=-90,
                position_error_mm=distance*1000,direction_error_deg=float(np.degrees(angle)),q_deg=np.degrees(fit.x).tolist(),objective=float(fit.fun),collision_checked=False)
        if checks:report.update(isolated_domain=checks.domain,collision_service_checks=checks.calls)
        save();plots(report,folder);print('Report:',folder/'report.json',flush=True)
    finally:
        if checks:checks.close()

if __name__=='__main__':main()
