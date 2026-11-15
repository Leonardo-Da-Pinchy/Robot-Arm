#!/usr/bin/python3
"""Reproducible known-reachable IK benchmark; launches isolated mock only.

Uses the saved physical profile for bounds, never its serial device. Targets
come from MoveIt FK and collision-valid witnesses; the solver never gets that
witness as a seed. Plan services preview only; no execution action is called.
"""
import argparse,contextlib,json,math,os,signal,subprocess,tempfile,time,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped,Quaternion
from moveit_msgs.srv import GetPositionIK,GetPositionFK,GetStateValidity
from pinchy_bringup.control_contract import JOINTS
from pinchy_motion.geometry import model_limits,pose_error
from pinchy_motion.ik import BoundedIK
from pinchy_motion.workspace import configuration,inside
from pinchy_motion_interfaces.srv import PlanPose
ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--count',type=int,default=64);parser.add_argument('--stress',action='store_true',help='Bias witnesses toward near-straight elbow/wrist and saved boundaries')
    parser.add_argument('--output',type=Path,default=Path('/tmp/pinchy-ik-benchmark.json'));args=parser.parse_args()
    domain=210+os.getpid()%10;os.environ['ROS_DOMAIN_ID']=str(domain)
    folder=Path(tempfile.mkdtemp(prefix='pinchy-ik-benchmark-'));log=(folder/'launch.log').open('w')
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_LOG_DIR=str(folder/'ros_logs'))
    process=subprocess.Popen(['ros2','launch','pinchy_moveit_config','mock_motion.launch.py','rviz:=false','panel:=false',
        'calibration:='+str(ROOT/'calibration/arm.json')],stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
    rclpy.init(domain_id=domain);node=rclpy.create_node('known_reachable_ik_benchmark');models=[];status=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String,'/robot_description',lambda m:models.append(m.data),qos)
    node.create_subscription(String,'/pinchy_motion/status',lambda m:status.append(json.loads(m.data)),qos)
    clients={name:node.create_client(kind,'/'+name) for name,kind in [('compute_ik',GetPositionIK),('compute_fk',GetPositionFK),('check_state_validity',GetStateValidity),('pinchy_motion/plan_pose',PlanPose)]}
    def call(name,request,timeout=15):
        f=clients[name].call_async(request);rclpy.spin_until_future_complete(node,f,timeout_sec=timeout)
        if not f.done():raise RuntimeError('Benchmark service timeout: '+name)
        return f.result()
    try:
        end=time.monotonic()+45
        while time.monotonic()<end and not (models and status and status[-1].get('ready')):
            rclpy.spin_once(node,timeout_sec=.1)
            if process.poll() is not None:raise RuntimeError('Mock launch failed: '+str(folder))
        if not status or not status[-1].get('ready'):raise RuntimeError('Mock readiness timeout: '+str(folder))
        limits=model_limits(models[-1]);solver=BoundedIK(models[-1],limits);region=configuration()['design_working_region']
        def valid(q):
            if any(not limits[n]['lower']-1e-9<=v<=limits[n]['upper']+1e-9 for n,v in zip(JOINTS,q)):return False
            req=GetStateValidity.Request();req.group_name='arm';req.robot_state.joint_state=JointState(name=JOINTS,position=list(map(float,q)))
            return call('check_state_validity',req).valid
        rows=[];rng=np.random.default_rng(93)
        witnesses=rng.uniform(solver.lower,solver.upper,(3000,6))
        if args.stress:
            witnesses[:,4]=solver.lower[4]+1e-5
            witnesses[::2,2]=np.clip(1e-5,solver.lower[2],solver.upper[2])
            witnesses[1::3,5]=np.clip(0.,solver.lower[5],solver.upper[5])
        for witness in witnesses:
            if len(rows)>=args.count:break
            matrix=solver.evaluate(witness)[0]
            if len(rows)<args.count//2 and not inside(matrix[:3,3],region):continue
            if not valid(witness):continue
            fk=GetPositionFK.Request();fk.header.frame_id='base_link';fk.fk_link_names=['pointer_tip']
            fk.robot_state.joint_state=JointState(name=JOINTS,position=witness.tolist())
            target=call('compute_fk',fk).pose_stamped[0]
            p=target.pose.position;q=target.pose.orientation
            matrix[:3,3]=[p.x,p.y,p.z];matrix[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            native=False;native_started=time.monotonic()
            for seed in [np.zeros(6),(solver.lower+solver.upper)/2]:
                request=GetPositionIK.Request();request.ik_request.group_name='arm';request.ik_request.ik_link_name='pointer_tip'
                request.ik_request.pose_stamped=target;request.ik_request.robot_state.joint_state=JointState(name=JOINTS,position=seed.tolist())
                request.ik_request.avoid_collisions=True;request.ik_request.timeout.nanosec=150_000_000
                result=call('compute_ik',request)
                if result.error_code.val==1:
                    positions=dict(zip(result.solution.joint_state.name,result.solution.joint_state.position));candidate=[positions[n] for n in JOINTS]
                    error=solver.errors(candidate,matrix,'full')
                    if valid(candidate) and error[0]<=.0001 and error[1]<=.001:native=True;break
            native_time=time.monotonic()-native_started
            fresh=BoundedIK(models[-1],limits);started=time.monotonic();candidate=fresh.search(matrix,np.zeros(6),valid=valid)
            rows.append(dict(index=len(rows),inside_design_region=inside(matrix[:3,3],region),native_fastpath=native,native_time_s=native_time,
                bounded_success=candidate is not None,bounded_time_s=time.monotonic()-started,witness_joint_radians=witness.tolist(),report=fresh.last_report))
            print(len(rows),'native',native,'bounded',candidate is not None,flush=True)
        examples=[]
        for xyz,rpy,mode in [([.395763,-.058652,.129172],[-120.927,-86.75,-85.681],'full'),([.333508,.000154,.125493],[-5.159,-62.39,-173.985],'full'),([.395763,-.058652,.129172],[-120.927,-86.75,-85.681],'position'),([.333508,.000154,.125493],[-5.159,-62.39,-173.985],'tool_axis')]:
            target=PoseStamped();target.header.frame_id='base_link';target.pose.position.x,target.pose.position.y,target.pose.position.z=xyz
            q=Rotation.from_euler('xyz',rpy,degrees=True).as_quat();target.pose.orientation=Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
            plan=call('pinchy_motion/plan_pose',PlanPose.Request(target=target,orientation_mode=mode),90)
            examples.append(dict(xyz=xyz,rpy=rpy,mode=mode,success=plan.success,message=plan.message))
            print('operator example',mode,plan.success,plan.message,flush=True)
        report=dict(count=len(rows),stress=args.stress,domain=domain,artifacts=str(folder),native_fastpath_successes=sum(r['native_fastpath'] for r in rows),
            bounded_successes=sum(r['bounded_success'] for r in rows),known_reachable=rows,operator_examples=examples,
            zero_conditioning=solver.conditioning(np.zeros(6)),bent_conditioning=solver.conditioning([.1,-.3,.6,-.3,.6,.3]),
            method='Fresh searches from zero/midpoint; valid target witnesses generated in calibrated bounds and checked by MoveIt; target poses independently returned by MoveIt FK. Native is the fast phase, not an exhaustive proof. No physical port or execute action.',motor_commands_sent=False)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        return 0 if len(rows)==args.count and all(r['bounded_success'] for r in rows) and all(e['success'] for e in examples if e['mode']!='full') else 1
    finally:
        node.destroy_node();rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        log.close()

if __name__=='__main__':raise SystemExit(main())
