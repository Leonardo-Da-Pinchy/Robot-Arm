"""XYZ/metres + roll/pitch/yaw pose commands; plan and execute are separate."""
import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Quaternion
from pinchy_motion_interfaces.srv import PlanPose,PlanPath
from pinchy_motion_interfaces.action import ExecutePlan
from .geometry import quaternion_from_rpy


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('status',help='Show mode, readiness and current encoder/model TCP pose')
    queue=sub.add_parser('queue',help='Read the current panel point queue without planning or moving')
    queue.add_argument('--out',type=Path,help='Save the published queue as a reusable point-set JSON')
    plan=sub.add_parser('plan',help='Solve full-pose IK, collision-check and preview; never moves')
    plan.add_argument('--xyz',nargs=3,type=float,required=True,metavar=('X','Y','Z'),help='Metres in --frame')
    plan.add_argument('--rpy',nargs=3,type=float,metavar=('ROLL','PITCH','YAW'),help='Degrees unless --radians; Rz(yaw) Ry(pitch) Rx(roll)')
    plan.add_argument('--orientation',choices=['full','tool_axis','position'],default='tool_axis',help='Exact RPY, pen +z with free twist, or XYZ only. Default: tool_axis.')
    plan.add_argument('--path',choices=['joint','cartesian'],default='joint',help='OMPL free-space transit, or straight TCP LERP + SLERP. Both preview first.')
    plan.add_argument('--frame',default='base_link'); plan.add_argument('--radians',action='store_true')
    plan.add_argument('--speed',type=float,help='Maximum joint deg/s (default: 30, up to loaded cap)')
    plan.add_argument('--acceleration',type=float,help='Maximum joint deg/s² (default: 180, up to loaded cap)')
    plan.add_argument('--out',type=Path,help='Write plan ID and summary JSON; execution still uses server-held plan')
    path_command=sub.add_parser('path',help='Preview one complete ordered path from a JSON file; never moves')
    path_command.add_argument('file',type=Path)
    path_command.add_argument('--out',type=Path)
    execute=sub.add_parser('execute',help='Execute a fresh server-held plan after preview')
    execute.add_argument('plan_id'); execute.add_argument('--hardware',action='store_true',help='Explicitly authorize the selected physical session')
    sub.add_parser('stop',help='Discard the current motion and establish powered hold')
    sub.add_parser('recover',help='Explicitly request recovery after correcting a hardware fault')
    args=parser.parse_args(remove_ros_args(sys.argv)[1:])
    if args.command=='plan' and args.rpy is None:
        if args.orientation!='position':parser.error('--rpy is required unless --orientation position is explicit')
        args.rpy=[0.,0.,0.]
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('pinchy_pose_command'); stopped=False
    def interrupt(signum,frame): raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt)
    def wait(future,timeout):
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        if not future.done(): raise RuntimeError('Request timed out')
        return future.result()
    def trigger(name):
        client=node.create_client(Trigger,name)
        if not client.wait_for_service(timeout_sec=5): raise RuntimeError('Motion server unavailable')
        response=wait(client.call_async(Trigger.Request()),12)
        node.destroy_client(client); return response
    try:
        if args.command=='queue':
            snapshots=[]
            node.create_subscription(String,'/pinchy_motion/queued_points',lambda m:snapshots.append(json.loads(m.data)),
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            deadline=time.monotonic()+8
            while not snapshots and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
            if not snapshots:raise RuntimeError('No published point queue; reopen an updated pose panel')
            value=snapshots[-1];print(json.dumps(value,indent=2))
            if not value.get('queue_valid',False):raise ValueError(value.get('error','Invalid point queue'))
            if args.out:
                from .point_sets import write
                write(args.out,value)
            return 0
        if args.command in ('stop','recover'):
            result=trigger('/pinchy_motion/'+args.command); print(result.message); return 0 if result.success else 1
        status=[]
        node.create_subscription(String,'/pinchy_motion/status',lambda msg:status.append(json.loads(msg.data)),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline=time.monotonic()+8
        while not status and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if not status: raise RuntimeError('No motion server status; start mock_motion or start_motion first')
        current=status[-1]
        if args.command=='status': print(json.dumps(current,indent=2)); return 0
        if not current['ready']: raise RuntimeError(current['reason'])
        if args.command=='path':
            data=json.loads(args.file.read_text())
            if 'schema_version' in data:
                from .point_sets import normalize
                data=normalize(data)
            mode=data.get('orientation_mode','tool_axis')
            for key,scale,maximum in [('joint_speed_deg_s','velocity_scaling',current.get('max_speed_deg_s',30.)),
                    ('joint_acceleration_deg_s2','acceleration_scaling',current.get('max_acceleration_deg_s2',180.))]:
                if key in data:
                    if scale in data:raise ValueError('Specify absolute '+key+' or '+scale+', not both')
                    value=data[key]
                    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0<value<=maximum:
                        raise ValueError(key+' exceeds the current session caps or is invalid')
                    data[scale]=value/maximum
            request=PlanPath.Request(orientation_mode=mode,style=data.get('style',''),
                tcp_speed=float(data.get('tcp_speed_m_s',.02)),tcp_acceleration=float(data.get('tcp_acceleration_m_s2',.05)),
                velocity_scaling=float(data.get('velocity_scaling',min(1.,30./current.get('max_speed_deg_s',30.)))),acceleration_scaling=float(data.get('acceleration_scaling',min(1.,180./current.get('max_acceleration_deg_s2',180.)))))
            for name in ('allow_contact_ramps','plan_pen_up_approach'):
                value=data.get(name,False)
                if not isinstance(value,bool):raise ValueError(name+' must be true or false')
                setattr(request,name,value)
            for point in data['waypoints']:
                pose=PoseStamped();pose.header.frame_id=point.get('frame',data.get('frame','base_link'))
                xyz=point['xyz']
                if len(xyz)!=3:raise ValueError('Each waypoint needs three XYZ values in metres')
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,xyz)
                if 'direction' in point:
                    if mode!='tool_axis':raise ValueError('direction vectors require tool_axis mode')
                    import numpy as np
                    from scipy.spatial.transform import Rotation
                    vector=np.array(point['direction'],dtype=float)
                    if vector.shape!=(3,) or not np.isfinite(vector).all() or np.linalg.norm(vector)<1e-8:raise ValueError('Invalid direction vector')
                    vector/=np.linalg.norm(vector);axis=np.cross([0.,0.,1.],vector);angle=math.acos(float(np.clip(vector[2],-1,1)))
                    axis=axis/np.linalg.norm(axis) if np.linalg.norm(axis)>1e-8 else np.array([1.,0.,0.])
                    q=Rotation.from_rotvec(axis*angle).as_quat()
                else:
                    if mode!='position' and 'rpy_deg' not in point:raise ValueError('A directed waypoint needs direction or rpy_deg')
                    q=quaternion_from_rpy(point.get('rpy_deg',[0.,0.,0.]))
                pose.pose.orientation=Quaternion(x=float(q[0]),y=float(q[1]),z=float(q[2]),w=float(q[3]))
                contact=point.get('contact',False)
                if not isinstance(contact,bool):raise ValueError('contact must be true or false')
                request.waypoints.append(pose);request.contact_segments.append(contact)
            client=node.create_client(PlanPath,'/pinchy_motion/plan_path')
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Ordered-path gateway unavailable')
            result=wait(client.call_async(request),150)
            summary=dict(success=result.success,message=result.message,plan_id=result.plan_id,duration_s=result.duration,
                modeled_contact_min_mm_s=result.modeled_contact_min_speed*1000,modeled_contact_max_mm_s=result.modeled_contact_max_speed*1000,hardware=current['hardware'])
            print(json.dumps(summary,indent=2))
            if result.success:
                print('Preview in RViz, then: ros2 run pinchy_motion pose execute '+result.plan_id+(' --hardware' if current['hardware'] else ''))
                if args.out:args.out.write_text(json.dumps(summary,indent=2)+'\n')
            return 0 if result.success else 1
        if args.command=='plan':
            target=PoseStamped(); target.header.frame_id=args.frame
            target.pose.position.x,target.pose.position.y,target.pose.position.z=args.xyz
            q=quaternion_from_rpy(args.rpy,degrees=not args.radians)
            target.pose.orientation=Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
            max_speed=current.get('max_speed_deg_s',30.); max_acceleration=current.get('max_acceleration_deg_s2',180.)
            speed=args.speed if args.speed is not None else min(30.,max_speed)
            acceleration=args.acceleration if args.acceleration is not None else min(180.,max_acceleration)
            if not all(math.isfinite(v) and 0<v<=cap+1e-8 for v,cap in [(speed,max_speed),(acceleration,max_acceleration)]):
                raise ValueError(f'Speed/acceleration must be positive and within loaded caps {max_speed:g} deg/s, {max_acceleration:g} deg/s²')
            client=node.create_client(PlanPose,'/pinchy_motion/plan_pose')
            if not client.wait_for_service(timeout_sec=5): raise RuntimeError('Planning gateway unavailable')
            result=wait(client.call_async(PlanPose.Request(target=target,orientation_mode=args.orientation,path_mode=args.path,
                velocity_scaling=min(1.,speed/max_speed),acceleration_scaling=min(1.,acceleration/max_acceleration))),90)
            summary=dict(success=result.success,message=result.message,plan_id=result.plan_id,duration_s=result.duration,
                frame=args.frame,xyz=args.xyz,rpy=args.rpy,rpy_units='radians' if args.radians else 'degrees',
                hardware=current['hardware'],orientation_mode=args.orientation,path_mode=args.path)
            print(json.dumps(summary,indent=2))
            if result.success:
                print('Preview in RViz, then: ros2 run pinchy_motion pose execute '+result.plan_id+(' --hardware' if current['hardware'] else ''))
                if args.out: args.out.write_text(json.dumps(summary,indent=2)+'\n')
            return 0 if result.success else 1
        if current['hardware'] and not args.hardware: raise RuntimeError('Physical execution requires --hardware')
        if not current['hardware'] and args.hardware: raise RuntimeError('--hardware supplied but server is MOCK')
        client=ActionClient(node,ExecutePlan,'/pinchy_motion/execute_plan')
        if not client.wait_for_server(timeout_sec=5): raise RuntimeError('Execution gateway unavailable')
        goal=wait(client.send_goal_async(ExecutePlan.Goal(plan_id=args.plan_id,hardware_confirmed=args.hardware)),5)
        if not goal.accepted: raise RuntimeError('Plan rejected: missing/expired/busy or wrong hardware authorization')
        result=wait(goal.get_result_async(),150)
        print(result.result.message)
        return 0 if result.result.success else 1
    except KeyboardInterrupt:
        if args.command=='execute':
            try: print(trigger('/pinchy_motion/stop').message)
            except Exception as error: print('Stop not confirmed: '+str(error),file=sys.stderr)
        return 130
    except Exception as error: print(str(error),file=sys.stderr); return 1
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
