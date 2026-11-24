"""Plan-only MoveIt plus a guarded execution gateway to the existing controller."""
from pinchy_bringup.control_contract import DEFAULT_SPEED,DEFAULT_ACCELERATION,MAX_TCP_SPEED,MAX_TCP_ACCELERATION
import copy
import hashlib
import math
import json
import math
from pathlib import Path
import signal
import threading
import time
import uuid
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import (RobotState, Constraints, JointConstraint, DisplayTrajectory,
                             PlanningScene, CollisionObject, PlanningSceneComponents)
from moveit_msgs.srv import (GetPositionIK, GetPositionFK, GetMotionPlan,
    GetStateValidity, GetPlanningScene, ApplyPlanningScene, GetCartesianPath)
from control_msgs.action import FollowJointTrajectory
from tf2_ros import Buffer, TransformListener
from pinchy_motion_interfaces.srv import PlanPose,PlanPath
from pinchy_motion_interfaces.action import ExecutePlan
from pinchy_bringup.control_contract import *
from pinchy_bringup.powered_hold import PoweredHold
from .target_editor import TargetEditor
from .paths import CartesianCurve,select_trace,timed_trace,corner_indices,concatenate,pose_matrix
from .ik import BoundedIK, MODES, POSITION_TOLERANCE, ANGLE_TOLERANCE, free_twist_frames
from .workspace import configuration as workspace_configuration, inside as inside_workspace, markers as workspace_markers, working_region_marker
from .geometry import (trajectory_reserve, model_limits, quaternion_from_rpy, pose_error, prepare_trajectory,
    sampled_positions, seconds, forward_kinematics, validate_cartesian_shape)


from .planning_progress import PlanningProgress


class MotionServer(Node):
    def __init__(self):
        super().__init__('pinchy_motion')
        self.declare_parameter('hardware',False); self.declare_parameter('scene_file','')
        self.hardware=self.get_parameter('hardware').value
        self.workspace=workspace_configuration();self.workspace_drawn=False
        self.group=ReentrantCallbackGroup(); self.operation=threading.Lock()
        self.ik_attempt_count=0;self.ik_solver=None;self.last_ik_report={}
        self.request_id='';self.last_ik=None; self.model=None; self.model_changed=False; self.limits={}; self.actual={}; self.actual_at=0.; self.plans={}
        self.scene_ready=False; self.scene_pending=False; self.scene_error='Waiting for MoveIt'
        self.stage='waiting'; self.active=False; self.stop_requested=False
        self.planning_progress=PlanningProgress()
        self.controller_goal=None; self.goal_dispatched=False; self.stop_future=None; self.stopping=False; self.active_id=''
        self.hold=PoweredHold(self,self.hardware,self.group)
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/robot_description',self.description,qos,callback_group=self.group)
        self.create_subscription(JointState,'/joint_states',self.joints,10,callback_group=self.group)
        self.status=self.create_publisher(String,'/pinchy_motion/status',qos)
        self.preview=self.create_publisher(DisplayTrajectory,'/display_planned_path',qos)
        self.workspace_pub=self.create_publisher(MarkerArray,'/pinchy_motion/workspace_markers',qos)
        self.working_region_pub=self.create_publisher(MarkerArray,'/pinchy_motion/working_region',qos)
        self.path_preview=self.create_publisher(MarkerArray,'/pinchy_motion/path_preview',qos)
        self.target_editor=TargetEditor(self,self.target_changed,lambda:not self.active)
        self.ik=self.create_client(GetPositionIK,'/compute_ik',callback_group=self.group)
        self.cartesian=self.create_client(GetCartesianPath,'/compute_cartesian_path',callback_group=self.group)
        self.fk=self.create_client(GetPositionFK,'/compute_fk',callback_group=self.group)
        self.planner=self.create_client(GetMotionPlan,'/plan_kinematic_path',callback_group=self.group)
        self.validity=self.create_client(GetStateValidity,'/check_state_validity',callback_group=self.group)
        self.scene=self.create_client(GetPlanningScene,'/get_planning_scene',callback_group=self.group)
        self.apply=self.create_client(ApplyPlanningScene,'/apply_planning_scene',callback_group=self.group)
        self.controller=ActionClient(self,FollowJointTrajectory,TRAJECTORY_ACTION,callback_group=self.group)
        self.create_service(PlanPose,'/pinchy_motion/plan_pose',self.plan,callback_group=self.group)
        self.create_service(PlanPath,'/pinchy_motion/plan_path',self.plan_path,callback_group=self.group)
        self.create_service(Trigger,'/pinchy_motion/stop',self.stop,callback_group=self.group)
        self.create_service(Trigger,'/pinchy_motion/recover',self.recover,callback_group=self.group)
        self.action=ActionServer(self,ExecutePlan,'/pinchy_motion/execute_plan',
            execute_callback=self.execute,goal_callback=self.goal,cancel_callback=self.cancel,
            callback_group=self.group)
        self.create_timer(0.3,self.update_status,callback_group=self.group)
        self.create_timer(0.5,self.initialize_scene,callback_group=self.group)

    def description(self,message):
        try:
            root=ET.fromstring(message.data)
            plugin=root.find('./ros2_control/hardware/plugin')
            expected='pinchy_hardware/FeetechSystem' if self.hardware else 'mock_components/GenericSystem'
            if plugin is None or plugin.text!=expected: raise ValueError('Live model/mode mismatch')
            if root.find("link[@name='pointer_tip']") is None: raise ValueError('Active pointer_tip missing')
            if self.model is not None and self.model!=message.data:
                self.model_changed=True; self.plans.clear()
                if self.active: self.begin_stop()
                return # MoveIt has an immutable model for this session.
            self.model=message.data; self.root=root; self.limits=model_limits(root)
            if self.ik_solver is None:self.ik_solver=BoundedIK(root,self.limits)
        except Exception as error: self.model=None; self.scene_error=str(error)

    def joints(self,message):
        values=dict(zip(message.name,message.position))
        if all(name in values and math.isfinite(values[name]) for name in JOINTS):
            self.actual={name:values[name] for name in JOINTS}; self.actual_at=time.monotonic()

    def robot_state(self,positions):
        state=RobotState(); state.joint_state=JointState(name=JOINTS,position=list(positions)); return state

    def call(self,client,request,timeout=8):
        if not client.service_is_ready(): raise RuntimeError('MoveIt service unavailable: '+client.srv_name)
        future=client.call_async(request); event=threading.Event()
        future.add_done_callback(lambda _:event.set())
        if not event.wait(timeout): future.cancel(); raise RuntimeError('Service timed out: '+client.srv_name)
        result=future.result()
        if result is None: raise RuntimeError('Empty service response: '+client.srv_name)
        return result

    def check_ready(self):
        if self.model_changed: raise RuntimeError('Live model changed; restart the planning layer to load it before planning again')
        if self.model is None: raise RuntimeError('Waiting for the matching live robot model')
        if not self.scene_ready: raise RuntimeError(self.scene_error)
        if time.monotonic()-self.actual_at>0.5 or time.monotonic()-self.hold.last_state>0.5:
            raise RuntimeError('Joint/controller feedback is stale')
        if self.hold.safety!='READY': raise RuntimeError('Safety status: '+self.hold.safety)
        if self.hold.phase!='idle' or self.hold.paused: raise RuntimeError('Controller is paused or establishing a hold')
        if not self.controller.server_is_ready(): raise RuntimeError('Trajectory controller unavailable')

    def initialize_scene(self):
        if self.scene_ready or self.scene_pending or not self.apply.service_is_ready(): return
        self.scene_pending=True
        try:
            path=self.get_parameter('scene_file').value
            definition=yaml.safe_load(Path(path).read_text()) if path else {'objects':[]}
            request=ApplyPlanningScene.Request(); request.scene.is_diff=True
            from .scene_geometry import scene_objects
            request.scene.world.collision_objects=scene_objects(definition)
            future=self.apply.call_async(request)
            def complete(future):
                try:
                    self.scene_ready=future.result().success
                    self.scene_error='' if self.scene_ready else 'Could not apply planning scene'
                except Exception as error: self.scene_error=str(error)
                self.scene_pending=False
            future.add_done_callback(complete)
        except Exception as error:
            self.scene_error=str(error); self.scene_pending=False

    def base_pose(self,stamped,transforms=None):
        values=[stamped.pose.position.x,stamped.pose.position.y,stamped.pose.position.z,
            stamped.pose.orientation.x,stamped.pose.orientation.y,stamped.pose.orientation.z,stamped.pose.orientation.w]
        if not all(math.isfinite(v) for v in values): raise ValueError('Pose must be finite')
        if abs(np.linalg.norm(values[3:])-1)>1e-3: raise ValueError('Orientation must be a normalized quaternion')
        frame=stamped.header.frame_id
        if not frame: raise ValueError('Pose requires a reference frame')
        target=copy.deepcopy(stamped)
        if frame!='base_link':
            if transforms is None:transform=self.tf.lookup_transform('base_link',frame,Time()).transform
            else:
                if frame not in transforms:transforms[frame]=self.tf.lookup_transform('base_link',frame,Time()).transform
                transform=transforms[frame]
            r=Rotation.from_quat([transform.rotation.x,transform.rotation.y,transform.rotation.z,transform.rotation.w])
            p=r.apply(values[:3])+[transform.translation.x,transform.translation.y,transform.translation.z]
            q=(r*Rotation.from_quat(values[3:])).as_quat()
            target.pose.position.x,target.pose.position.y,target.pose.position.z=map(float,p)
            target.pose.orientation=Quaternion(x=float(q[0]),y=float(q[1]),z=float(q[2]),w=float(q[3]))
        target.header.frame_id='base_link'; target.header.stamp=self.get_clock().now().to_msg()
        return target

    def check_bounds(self,positions,measured=False):
        if len(positions)!=6: raise ValueError('Expected six joint positions')
        margin=FEEDBACK_BOUND_TOLERANCE if measured else 0.
        for name,value in zip(JOINTS,positions):
            limit=self.limits[name]
            if not math.isfinite(value) or not limit['lower']-margin-1e-9<=value<=limit['upper']+margin+1e-9:
                kind='Measured feedback' if measured else 'Command/IK state'
                raise ValueError(f"{kind} exceeds calibrated bounds: {name}; value={value:.6f} rad, range=[{limit['lower']:.6f}, {limit['upper']:.6f}], feedback allowance={margin:.6f} rad")
    @staticmethod
    def check_validity_response(response):
        if not response.valid:
            pairs=sorted(set(f'{c.contact_body_1}/{c.contact_body_2}' for c in response.contacts))
            raise RuntimeError('Collision/invalid state: '+(', '.join(pairs) or 'joint or scene constraints'))

    def valid(self,positions,measured=False):
        self.check_bounds(positions,measured)
        request=GetStateValidity.Request();request.group_name='arm';request.robot_state=self.robot_state(positions)
        self.check_validity_response(self.call(self.validity,request))

    def planning_step(self,phase,completed=None,total=None,unit='',**context):
        self.planning_progress.update(phase,completed,total,unit,**context)

    def validate_states(self,states,stop=None):
        """Check every original sample, pipelining DDS requests in bounded batches.

        This changes transport scheduling, not sample density or constraints.
        Scene snapshots are still verified by the caller before acceptance/use.
        """
        from itertools import islice
        tracker=getattr(self,'planning_progress',None)
        if tracker is not None and tracker.snapshot().get('active'):
            states=list(states);total=len(states);completed=0
            phase=tracker.snapshot()['phase']
            tracker.update(phase,0,total,'collision checks')
        else:tracker=None
        states=iter(states)
        def check():
            if self.stop_requested or self.model_changed or (stop is not None and stop()):
                raise RuntimeError('Stopped during trajectory collision validation')
        while True:
            check();batch=list(islice(states,4))
            if not batch:return
            futures=[]
            try:
                for positions in batch:
                    self.check_bounds(positions)
                    request=GetStateValidity.Request(group_name='arm',robot_state=self.robot_state(positions))
                    future=self.validity.call_async(request);event=threading.Event()
                    future.add_done_callback(lambda _,event=event:event.set());futures.append((future,event))
                deadline=time.monotonic()+8
                for future,event in futures:
                    while not event.wait(.02):
                        check()
                        if time.monotonic()>deadline:raise RuntimeError('Trajectory collision service timed out')
                    check();self.check_validity_response(future.result())
                    if tracker is not None:
                        completed+=1
                        tracker.update(phase,completed,total,'collision checks')
            finally:
                for future,_ in futures:
                    if not future.done():future.cancel()

    def scene_digest(self):
        request=GetPlanningScene.Request()
        request.components.components=(PlanningSceneComponents.WORLD_OBJECT_GEOMETRY|PlanningSceneComponents.OCTOMAP|
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX|PlanningSceneComponents.TRANSFORMS|
            PlanningSceneComponents.LINK_PADDING_AND_SCALING|PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        scene=self.call(self.scene,request).scene
        def strip(value):
            if isinstance(value,dict): return {k:strip(v) for k,v in value.items() if k not in ('stamp','name','is_diff')}
            if isinstance(value,list): return [strip(v) for v in value]
            return value
        components=dict(world=message_to_ordereddict(scene.world),
            allowed=message_to_ordereddict(scene.allowed_collision_matrix),
            transforms=[message_to_ordereddict(x) for x in scene.fixed_frame_transforms],
            padding=[message_to_ordereddict(x) for x in scene.link_padding],
            scale=[message_to_ordereddict(x) for x in scene.link_scale],
            attached=[message_to_ordereddict(x) for x in scene.robot_state.attached_collision_objects])
        return hashlib.sha256(json.dumps(strip(components),sort_keys=True).encode()).hexdigest()

    def validation_digest(self,trajectory,scene):
        """Bind the exact checked message to this immutable model and scene.

        Only issued after full preview validation; never persisted or shared
        across plans. Execution still checks fresh feedback, start and scene.
        """
        digest=hashlib.sha256()
        # Canonical field values avoid nondeterministic CDR padding bytes.
        message=json.dumps(message_to_ordereddict(trajectory),sort_keys=True,separators=(',',':'),allow_nan=False).encode()
        for value in (self.model.encode(),scene.encode(),message):
            digest.update(len(value).to_bytes(8,'big'));digest.update(value)
        return digest.hexdigest()

    def validate_execution_preview(self,record,scene):
        if self.stop_requested or self.model_changed:
            raise RuntimeError('Stopped before execution')
        if record.get('validation_digest')!=self.validation_digest(record['trajectory'],scene):
            raise RuntimeError('Validated preview changed or missing; plan again')

    def solve_ik(self,target,start,mode='full',exclude=()):
        self.planning_step('Solving destination IK')
        q=target.pose.orientation;p=target.pose.position
        transform=np.eye(4);transform[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        transform[:3,3]=[p.x,p.y,p.z];rejections=[]
        def accept(candidate):
            if any(np.linalg.norm(np.array(candidate)-old)<.05 for old in exclude):return False
            distance,angle=self.ik_solver.errors(candidate,transform,mode)
            if distance>POSITION_TOLERANCE or angle>ANGLE_TOLERANCE:return False
            try:self.valid(candidate)
            except (RuntimeError,ValueError) as error:rejections.append(str(error));return False
            return True
        # Query MoveIt natively in BOTH exact and free-twist modes. The latter
        # searches full rotations about the requested pen axis without changing
        # XYZ or the axis itself. Continuous bounded 5-DOF search follows if
        # discretized twists miss a narrow feasible interval.
        native_queries=0
        if mode in ('full','tool_axis') and not exclude:
            seeds=[start]
            if self.last_ik is not None:seeds.append(self.last_ik)
            seeds.append(((self.ik_solver.lower+self.ik_solver.upper)/2).tolist())
            if mode=='full':queries=[(transform,seed) for seed in seeds]
            else:
                reference=self.ik_solver.evaluate(start)[0][:3,:3]
                queries=[(frame,seeds[i%len(seeds)]) for i,frame in enumerate(free_twist_frames(transform,reference))]
            for frame,seed in queries:
                if self.stop_requested:raise RuntimeError('Stopped during IK')
                request=GetPositionIK.Request();request.ik_request.group_name='arm'
                request.ik_request.ik_link_name='pointer_tip'
                variant=copy.deepcopy(target)
                quaternion=Rotation.from_matrix(frame[:3,:3]).as_quat()
                variant.pose.orientation=Quaternion(x=float(quaternion[0]),y=float(quaternion[1]),z=float(quaternion[2]),w=float(quaternion[3]))
                request.ik_request.pose_stamped=variant
                request.ik_request.robot_state=self.robot_state(seed);request.ik_request.avoid_collisions=True
                request.ik_request.timeout.nanosec=80_000_000 if mode=='tool_axis' else 150_000_000
                self.ik_attempt_count+=1;native_queries+=1
                solution=self.call(self.ik,request,timeout=4)
                if solution.error_code.val==1:
                    values=dict(zip(solution.solution.joint_state.name,solution.solution.joint_state.position))
                    candidate=[values[n] for n in JOINTS]
                    if accept(candidate):
                        self.last_ik=candidate;self.ik_solver.remember(candidate)
                        self.last_ik_report=dict(source='MoveIt KDL free-twist' if mode=='tool_axis' else 'MoveIt KDL',
                            mode=mode,native_queries=native_queries,conditioning=self.ik_solver.conditioning(candidate,mode))
                        return candidate
        candidate=self.ik_solver.search(transform,start,mode,valid=accept,
            stop=lambda:self.stop_requested or self.model_changed,budget=6.)
        self.ik_attempt_count+=self.ik_solver.last_report.get('attempts',0)
        self.last_ik_report=dict(source='bounded multi-start',native_queries=native_queries,**self.ik_solver.last_report)
        if candidate is not None:
            self.last_ik=candidate;self.last_ik_report['conditioning']=self.ik_solver.conditioning(candidate,mode)
            return candidate
        report=self.last_ik_report
        if mode=='tool_axis':
            if rejections:
                report['failure_kind']='candidate_collision_or_state_rejection'
                raise RuntimeError('IK solutions met XYZ and pen direction, but failed collision/state checks: '+'; '.join(sorted(set(rejections))[:3])+'. No motion sent.')
            review=self.range_review_only(transform,start)
            if review:
                report['range_review_only']=review
                details='; '.join(f"{item['joint']} {item['candidate_deg']:+.1f} deg vs saved [{item['minimum_deg']:+.1f}, {item['maximum_deg']:+.1f}]" for item in review)
                raise RuntimeError('MoveIt found no pen-direction solution within the saved ranges. A diagnostic model-range solution exists but exceeds: '+details+
                    '. It is NOT an executable plan. Physical range verification is required; no limits were changed and no motion sent.')

            # Diagnostic ONLY: never return this relaxed candidate or put it in
            # a plan. Explain whether the issue is XYZ or its requested axis.
            diagnostic=BoundedIK(self.root,self.limits)
            point=diagnostic.search(transform,start,'position',valid=lambda q:self.position_diagnostic_valid(q),
                stop=lambda:self.stop_requested or self.model_changed,budget=1.,attempts=16)
            report['xyz_without_direction_reachable']=point is not None
            if point is not None:
                start_distance,start_angle=self.ik_solver.errors(start,transform,'tool_axis')
                detail=(f" Best numerical residual: {report['best_position_error_mm']:.2f} mm / {report['best_orientation_error_deg']:.2f} deg." if 'best_position_error_mm' in report else '')
                raise RuntimeError('XYZ alone is reachable; no solution was found for XYZ plus the requested pen direction within saved limits. '
                    f'Goal is {start_distance*1000:.1f} mm from the held tip, but changes its direction by {math.degrees(start_angle):.1f} deg.'+detail+
                    ' This is not proof of impossibility. Use Copy current direction to deliberately keep your XYZ with the current pointing direction, or edit the goal direction. No motion sent; constraints were not relaxed.')
        if rejections:
            raise RuntimeError('IK candidates failed MoveIt collision/bounds checks: '+'; '.join(sorted(set(rejections))[:3]))
        detail=''
        if 'best_position_error_mm' in report:
            detail=(f" Best numerical residual: {report['best_position_error_mm']:.2f} mm / "
                f"{report['best_orientation_error_deg']:.2f} deg; nearby bounds: "+', '.join(report['limiting_joints'])+'.')
        constraint={'full':'exact XYZ + RPY','tool_axis':'XYZ + pen direction (free twist)','position':'XYZ (free orientation)'}[mode]
        raise RuntimeError('No valid IK found for '+constraint+' after native/bounded search.'+detail+
            ' This is not proof of impossibility. Check saved ranges, orientation and collisions; Exact RPY is never relaxed automatically.')

    def range_review_only(self,target,start):
        """Explain a range restriction; this candidate can NEVER become a plan."""
        nominal=copy.deepcopy(self.limits)
        for value in nominal.values():
            value['lower']=value['nominal_model_lower'];value['upper']=value['nominal_model_upper']
        if all(value['lower']==self.limits[n]['lower'] and value['upper']==self.limits[n]['upper'] for n,value in nominal.items()):return None
        diagnostic=BoundedIK(self.root,nominal)
        def collision_only(q):
            request=GetStateValidity.Request();request.group_name='arm';request.robot_state=self.robot_state(q)
            return self.call(self.validity,request).valid
        candidate=diagnostic.search(target,start,'tool_axis',valid=collision_only,
            stop=lambda:self.stop_requested or self.model_changed,budget=1.5,attempts=24)
        if candidate is None:return None
        review=[]
        for name,q in zip(JOINTS,candidate):
            limit=self.limits[name]
            if not limit['lower']-1e-9<=q<=limit['upper']+1e-9:
                review.append(dict(joint=name,candidate_deg=math.degrees(q),minimum_deg=math.degrees(limit['lower']),maximum_deg=math.degrees(limit['upper'])))
        return review or None

    def position_diagnostic_valid(self,positions):
        try:self.valid(positions);return True
        except (RuntimeError,ValueError):return False

    def pose_from_joints(self,positions):
        transform=forward_kinematics(self.root,dict(zip(JOINTS,positions)))
        pose=PoseStamped();pose.header.frame_id='base_link';pose.header.stamp=self.get_clock().now().to_msg()
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,transform[:3,3])
        q=Rotation.from_matrix(transform[:3,:3]).as_quat()
        pose.pose.orientation=Quaternion(x=float(q[0]),y=float(q[1]),z=float(q[2]),w=float(q[3]))
        return pose

    def plan_transit(self,start,goal_positions,resolved,velocity,acceleration,path_mode):
        self.planning_step('Planning Cartesian connection' if path_mode=='cartesian' else 'Planning collision-free pen-up connection')
        if path_mode=='cartesian':
            request=GetCartesianPath.Request();request.header.frame_id='base_link'
            request.start_state=self.robot_state(start);request.group_name='arm';request.link_name='pointer_tip'
            request.waypoints=[resolved.pose];request.max_step=.002
            request.jump_threshold=5.;request.revolute_jump_threshold=.2;request.avoid_collisions=True
            request.max_velocity_scaling_factor=velocity;request.max_acceleration_scaling_factor=acceleration
            result=self.call(self.cartesian,request,timeout=20)
            self.get_logger().info('PLAN_EVENT '+json.dumps(dict(event='cartesian_result',request_id=self.request_id,fraction=result.fraction,points=len(result.solution.joint_trajectory.points))))
            if result.error_code.val!=1 or result.fraction<1.-1e-9:
                raise RuntimeError(f'Straight TCP path incomplete ({result.fraction*100:.1f}%, code {result.error_code.val}); no partial path retained. Endpoint reachability does not guarantee a straight route. Try free-space transit.')
            return result.solution
        plan=GetMotionPlan.Request();motion=plan.motion_plan_request
        motion.group_name='arm';motion.pipeline_id='ompl';motion.planner_id='RRTConnectkConfigDefault'
        motion.start_state=self.robot_state(start);motion.allowed_planning_time=5.;motion.num_planning_attempts=3
        motion.max_velocity_scaling_factor=velocity;motion.max_acceleration_scaling_factor=acceleration
        motion.workspace_parameters.header.frame_id='base_link'
        for axis,value in zip('xyz',self.workspace['minimum_m']):setattr(motion.workspace_parameters.min_corner,axis,value)
        for axis,value in zip('xyz',self.workspace['maximum_m']):setattr(motion.workspace_parameters.max_corner,axis,value)
        constraint=Constraints(name='checked_ik_goal')
        constraint.joint_constraints=[JointConstraint(joint_name=name,position=value,
            tolerance_above=1e-5,tolerance_below=1e-5,weight=1.) for name,value in zip(JOINTS,goal_positions)]
        motion.goal_constraints=[constraint]
        result=self.call(self.planner,plan,timeout=15).motion_plan_response
        if result.error_code.val!=1:raise RuntimeError('Free-space route planning failed (code '+str(result.error_code.val)+'); endpoint IK succeeded')
        return result.trajectory

    def route_to_pose(self,target,start,mode,velocity,acceleration):
        """Share collision-aware endpoint/route retries across both pose UIs."""
        excluded=[];last_error=None
        for attempt in range(3):
            if self.stop_requested or self.model_changed:raise RuntimeError('Stopped during pose routing')
            try:goal=self.solve_ik(target,start,mode,exclude=excluded)
            except RuntimeError:
                if last_error is not None:raise RuntimeError('No checked route found to the valid pose configurations: '+str(last_error)) from last_error
                raise
            excluded.append(goal);resolved=self.pose_from_joints(goal)
            try:
                trajectory=self.plan_transit(start,goal,resolved,velocity,acceleration,'joint')
                return goal,resolved,trajectory
            except RuntimeError as error:
                if self.stop_requested or self.model_changed:raise
                last_error=error
        raise RuntimeError('No checked route found after three pose configurations: '+str(last_error)) from last_error

    def target_changed(self,revision):
        self.clear_path_preview();self.plans.clear()
        if not self.active:self.active_id=''
        if self.stage=='planned':self.stage='target edited; plan again'

    def clear_path_preview(self):
        marker=Marker();marker.action=Marker.DELETEALL
        self.path_preview.publish(MarkerArray(markers=[marker]))

    def show_path_preview(self,trajectory,targets):
        line=Marker();line.header.frame_id='base_link';line.ns='ordered_path';line.id=0
        line.type=Marker.LINE_STRIP;line.action=Marker.ADD;line.pose.orientation.w=1.
        line.scale.x=.003;line.color.r=1.;line.color.g=.2;line.color.b=.7;line.color.a=1.
        points=trajectory.joint_trajectory.points;stride=max(1,len(points)//600)
        chosen=list(points[::stride])
        if chosen[-1] is not points[-1]:chosen.append(points[-1])
        for point in chosen:
            p=self.ik_solver.evaluate(point.positions)[0][:3,3]
            line.points.append(Point(x=float(p[0]),y=float(p[1]),z=float(p[2])))
        dots=Marker();dots.header=line.header;dots.ns='ordered_path';dots.id=1;dots.type=Marker.SPHERE_LIST;dots.action=Marker.ADD;dots.pose.orientation.w=1.
        dots.scale.x=dots.scale.y=dots.scale.z=.007;dots.color.r=dots.color.g=dots.color.b=dots.color.a=1.
        dots.points=[p.pose.position for p in targets]
        self.path_preview.publish(MarkerArray(markers=[line,dots]))

    def publish_target(self,target):
        return self.target_editor.set_target(target,force=True)

    def plan(self,request,response):
        if not self.operation.acquire(False): response.message='Another planning/execution operation is active'; return response
        try:
            self.request_id=uuid.uuid4().hex
            self.planning_progress.begin(self.request_id,'pose')
            self.clear_path_preview()
            self.check_ready(); self.stage='planning'; self.plans.clear(); self.stop_requested=False
            mode=request.orientation_mode or 'tool_axis';path_mode=request.path_mode or 'joint'
            if mode not in MODES:raise ValueError('Invalid orientation mode')
            if path_mode not in ('joint','cartesian'):raise ValueError('Invalid path mode')
            velocity=request.velocity_scaling or min(1.,DEFAULT_SPEED/max(v['velocity'] for v in self.limits.values())); acceleration=request.acceleration_scaling or min(1.,DEFAULT_ACCELERATION/min(v['acceleration'] for v in self.limits.values()))
            if not all(math.isfinite(x) and 0<x<=1 for x in (velocity,acceleration)): raise ValueError('Scaling must be in (0, 1]')
            target=self.base_pose(request.target); target_revision=self.publish_target(target)
            q=target.pose.orientation;p=target.pose.position
            rpy=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz',degrees=True).tolist()
            self.get_logger().info('Pose request base_link XYZ='+str([p.x,p.y,p.z])+' RPY_deg='+str(rpy))
            start=list(self.hold.applied if self.hardware else self.hold.commanded)
            actual=[self.actual[name] for name in JOINTS]
            self.get_logger().info('PLAN_EVENT '+json.dumps(dict(event='request',request_id=self.request_id,orientation_mode=mode,path_mode=path_mode,xyz=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w],commanded_start=start,measured_start=actual,model_sha256=hashlib.sha256(self.model.encode()).hexdigest())))
            if max(abs(a-b) for a,b in zip(start,actual))>0.05: raise RuntimeError('Robot is too far from its held command; settle/support and inspect first')
            self.valid(actual,measured=True); self.valid(start)
            scene_hash=self.scene_digest()
            p=target.pose.position
            if not inside_workspace([p.x,p.y,p.z],self.workspace):
                raise RuntimeError('Pose is outside the configured numerical XYZ input bounds (not a reachable-workspace boundary)')
            if path_mode=='joint':
                goal_positions,resolved,trajectory=self.route_to_pose(target,start,mode,velocity,acceleration)
            else:
                goal_positions=self.solve_ik(target,start,mode);self.valid(goal_positions)
                resolved=self.pose_from_joints(goal_positions)
                trajectory=(self.trace_paths(start,[target],mode,'polyline',[False],.02,.05,velocity,acceleration)[0]
                    if path_mode=='cartesian' and mode!='full' else self.plan_transit(start,goal_positions,resolved,velocity,acceleration,path_mode))
            if len(trajectory.joint_trajectory.points)==1:
                point=copy.deepcopy(trajectory.joint_trajectory.points[0]); point.time_from_start.sec=1
                trajectory.joint_trajectory.points.append(point)
            trajectory=prepare_trajectory(trajectory,self.limits,velocity,acceleration)
            if path_mode=='cartesian' and mode=='full':validate_cartesian_shape(trajectory,self.ik_solver,start,goal_positions)
            if path_mode=='cartesian' and mode!='full':resolved=self.pose_from_joints(trajectory.joint_trajectory.points[-1].positions)
            first=trajectory.joint_trajectory.points[0]
            if max(abs(a-b) for a,b in zip(first.positions,start))>1e-4: raise RuntimeError('Planner start does not match held command')
            samples=list(sampled_positions(trajectory));self.planning_step('Checking pose trajectory collisions',0,len(samples),'collision checks')
            for index,positions in enumerate(samples,1):
                self.valid(positions);self.planning_step('Checking pose trajectory collisions',index,len(samples),'collision checks')
            self.planning_step('Verifying pose endpoint and scene')
            fk=GetPositionFK.Request(); fk.header.frame_id='base_link'; fk.fk_link_names=['pointer_tip']
            fk.robot_state=self.robot_state(trajectory.joint_trajectory.points[-1].positions)
            end=self.call(self.fk,fk)
            if end.error_code.val!=1: raise RuntimeError('Endpoint FK verification failed')
            position_error,rotation_error=pose_error(end.pose_stamped[0].pose,target.pose)
            if mode=='tool_axis':
                a=end.pose_stamped[0].pose.orientation;b=target.pose.orientation
                va=Rotation.from_quat([a.x,a.y,a.z,a.w]).as_matrix()[:,2]
                vb=Rotation.from_quat([b.x,b.y,b.z,b.w]).as_matrix()[:,2]
                rotation_error=float(np.arccos(np.clip(va@vb,-1,1)))
            if position_error>2*POSITION_TOLERANCE or (mode!='position' and rotation_error>2*ANGLE_TOLERANCE):
                raise RuntimeError('MoveIt FK endpoint does not satisfy the selected pose constraints')
            if self.scene_digest()!=scene_hash: raise RuntimeError('Planning scene changed while planning; plan again')
            self.check_ready()
            if self.stop_requested: raise RuntimeError('Stopped during planning; no plan retained')
            if max(abs(a-b) for a,b in zip(start,self.hold.commanded))>.01: raise RuntimeError('Robot command changed while planning; plan again')
            if self.target_editor.revision!=target_revision:raise RuntimeError('Target changed while planning; plan again')
            token=uuid.uuid4().hex
            self.plans[token]=dict(trajectory=trajectory,start=start,target=resolved,requested_target=target,orientation_mode=mode,path_mode=path_mode,scene=scene_hash,
                model=self.model,target_revision=target_revision,created=time.monotonic(),
                validation_digest=self.validation_digest(trajectory,scene_hash))
            self.active_id=token; self.stage='planned'
            display=DisplayTrajectory(model_id=ET.fromstring(self.model).get('name'),
                trajectory_start=self.robot_state(start),trajectory=[trajectory])
            self.preview.publish(display)
            response.success=True; response.plan_id=token; response.trajectory=trajectory
            response.duration=seconds(trajectory.joint_trajectory.points[-1]);response.resolved_target=resolved
            q=resolved.pose.orientation;angles=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz',degrees=True)
            response.message=('Plan ready; no motion sent. '+mode+' / '+path_mode+
                '; resolved RPY '+', '.join(f'{v:.2f}' for v in angles)+' deg.'+
                (' Near a kinematic singularity; speed may need limiting.' if self.last_ik_report.get('conditioning',{}).get('near_singular') else ''))
            reserve=trajectory_reserve(trajectory,self.limits)
            if reserve['margin_deg']<5.:response.message+=f" Low joint travel reserve: {reserve['joint']} {reserve['margin_deg']:.2f} deg along this plan; a later move may need another posture."
        except Exception as error:
            self.stage='rejected'; response.message=str(error); self.get_logger().warn(response.message)
            self.get_logger().info('PLAN_EVENT '+json.dumps(dict(event='rejected',request_id=self.request_id,message=response.message,ik_report=self.last_ik_report)))
        finally:
            self.planning_progress.end()
            if response.success:self.get_logger().info('PLAN_EVENT '+json.dumps(dict(event='planned',request_id=self.request_id,plan_id=response.plan_id,duration=response.duration,ik_report=self.last_ik_report,endpoint=list(response.trajectory.joint_trajectory.points[-1].positions))))
            self.operation.release()
        return response

    def trace_paths(self,start,targets,mode,style,contacts,speed,acceleration,velocity_scale,acceleration_scale,allow_contact_ramps=False,enforce_tcp_speed=False,tcp_min_speed=0.):
        transforms=[self.ik_solver.evaluate(start)[0],*[pose_matrix(p.pose) for p in targets]]
        resolved=[];deadline=time.monotonic()+45
        def stopped():
            if time.monotonic()>deadline:raise RuntimeError('Ordered trace planning exceeded its time budget; no partial plan retained')
            return self.stop_requested or self.model_changed
        # An initial copy-current-pose waypoint is a useful UI gesture, not a
        # zero-length tracing segment. Remove only identical constrained poses.
        while len(transforms)>1 and np.linalg.norm(transforms[1][:3,3]-transforms[0][:3,3])<1e-5:
            _,angle=self.ik_solver.errors(start,transforms[1],mode)
            if angle>.001:raise ValueError('Rotation-only trace: use a pose transit or Visit poses')
            transforms.pop(1);contacts=contacts[1:];resolved.append(self.pose_from_joints(start))
        if len(transforms)<2:raise ValueError('Path has no positional movement')
        corners=corner_indices(transforms) if style=='polyline' else []
        if corners and (enforce_tcp_speed or tcp_min_speed>0):raise ValueError('Constant-speed traces need smooth geometry; sharp corners require a separate stroke or explicit curve')
        if any(contacts[i-1] or contacts[i] for i in corners):
            raise ValueError('Sharp contact corner requires a pen lift or explicitly selected Smooth through points; no hidden rounding/slowdown')
        boundaries=[0,*corners,len(transforms)-1];trajectories=[];summaries=[];q=list(start)
        for leg_index,(first,last) in enumerate(zip(boundaries,boundaries[1:]),1):
            self.planning_step('Solving continuous Cartesian trace',leg_index=leg_index,leg_total=len(boundaries)-1)
            curve=CartesianCurve(transforms[first:last+1],style)
            rejected=[]
            def trace_valid(candidate):
                try:self.valid(candidate);return True
                except (RuntimeError,ValueError) as error:rejected.append(str(error));return False
            def finish(distances,positions):
                trajectory,summary=timed_trace(curve,distances,positions,self.ik_solver,self.limits,mode,speed,acceleration,
                    contacts[first:last],velocity_scale,acceleration_scale,allow_contact_ramps,enforce_tcp_speed,progress=self.planning_step,tcp_min_speed=tcp_min_speed,stop=stopped)
                self.planning_step('Checking trace collisions')
                self.validate_states(sampled_positions(trajectory),stopped)
                return trajectory,summary
            try:
                distances,positions,(trajectory,summary),selection=select_trace(curve,mode,q,self.ik_solver,trace_valid,stopped,finish,progress=self.planning_step)
                summary['configuration_selection']=selection
                self.get_logger().info('PLAN_EVENT '+json.dumps(dict(event='trace_configuration_selection',request_id=self.request_id,**selection)))
            except RuntimeError as error:
                if hasattr(error,'selection_report'):
                    self.get_logger().info('PLAN_EVENT '+json.dumps(dict(event='trace_configuration_rejected',request_id=self.request_id,**error.selection_report)))
                detail=(' Last state rejection: '+rejected[-1]) if rejected else (' Last IK diagnostic: '+str(self.ik_solver.last_report))
                if hasattr(error,'target') and not self.stop_requested and not self.model_changed:
                    diagnostic=BoundedIK(self.root,self.limits)
                    alternate=diagnostic.search(error.target,np.zeros(6),mode,valid=self.position_diagnostic_valid,
                        stop=lambda:self.stop_requested or self.model_changed,budget=1.2,attempts=24)
                    if alternate is not None:
                        jump=math.degrees(max(abs(np.asarray(alternate)-error.previous_positions)))
                        detail=(f' The failed intermediate pose DOES have collision-checked IK in another configuration (up to {jump:.1f} deg from the stalled configuration). '
                            'No continuous connection was found from this starting posture; this is not an unreachable-point result. Plan a different pen-up approach/posture before tracing.')
                    self.get_logger().info('PLAN_EVENT '+json.dumps(dict(event='trace_failure_diagnostic',request_id=self.request_id,
                        distance_m=error.distance,target_matrix=error.target.tolist(),broader_ik_found=alternate is not None,executable=False,report=diagnostic.last_report)))
                raise RuntimeError(str(error)+detail) from error
            for distance in curve.waypoint_distances[1:]:
                index=int(np.argmin(abs(distances-distance)));resolved.append(self.pose_from_joints(positions[index]))
            trajectories.append(trajectory);summaries.append(summary);q=trajectory.joint_trajectory.points[-1].positions
        return concatenate(trajectories),summaries,resolved

    def trace_with_approach(self,start,targets,mode,style,contacts,speed,acceleration,velocity_scale,acceleration_scale,allow_contact_ramps,enforce_tcp_speed=False,tcp_min_speed=0.):
        """Explicit pen-up route to a start selected for the entire stroke."""
        from .stroke_search import select_stroke
        if len(targets)<2 or contacts[0] or style=='visits':
            raise ValueError('Stroke setup requires two or more waypoints, a pen-up first segment and a trace style')
        transforms=[pose_matrix(p.pose) for p in targets]
        if style=='polyline' and corner_indices(transforms):
            raise ValueError('Stroke setup needs a smooth trace or collinear points; no automatic contact corner blending')
        curve=CartesianCurve(transforms,style);deadline=time.monotonic()+360;state_checks={}
        def stopped():
            if time.monotonic()>deadline:raise RuntimeError('Whole-stroke setup exceeded its time budget; no partial plan retained')
            return self.stop_requested or self.model_changed
        def valid(q):
            if stopped():raise RuntimeError('Whole-stroke planning stopped')
            key=tuple(map(float,q))
            if key not in state_checks:
                try:self.valid(q);state_checks[key]=True
                except (RuntimeError,ValueError):state_checks[key]=False
            return state_checks[key]
        def finish(distances,positions):
            def stage(name):
                self.planning_step(name)
                self.get_logger().info('PLAN_EVENT '+json.dumps(dict(event='stroke_validation_progress',request_id=self.request_id,phase=name)))
            stage('controller polynomial timing')
            stroke,summary=timed_trace(curve,distances,positions,self.ik_solver,self.limits,mode,speed,acceleration,
                contacts[1:],velocity_scale,acceleration_scale,allow_contact_ramps,enforce_tcp_speed,progress=self.planning_step,tcp_min_speed=tcp_min_speed,stop=stopped)
            stage('stroke collision samples')
            self.validate_states(sampled_positions(stroke),stopped)
            if stopped():raise RuntimeError('Whole-stroke planning stopped')
            stage('pen-up approach planning')
            approach=self.plan_transit(start,positions[0].tolist(),self.pose_from_joints(positions[0]),velocity_scale,acceleration_scale,'joint')
            if len(approach.joint_trajectory.points)==1:
                point=copy.deepcopy(approach.joint_trajectory.points[0]);point.time_from_start.sec=1
                approach.joint_trajectory.points.append(point)
            if max(abs(a-b) for a,b in zip(start,approach.joint_trajectory.points[0].positions))>1e-6:
                raise RuntimeError('Pen-up approach does not start at the held command')
            if max(abs(a-b) for a,b in zip(positions[0],approach.joint_trajectory.points[-1].positions))>1e-5:
                raise RuntimeError('Pen-up approach does not reach the selected stroke start')
            # Match the exact constrained start; any endpoint change and zero
            # derivatives are made BEFORE polynomial and collision validation.
            approach.joint_trajectory.points[-1].positions=positions[0].tolist()
            for endpoint in (approach.joint_trajectory.points[0],approach.joint_trajectory.points[-1]):
                endpoint.velocities=[0.]*6;endpoint.accelerations=[0.]*6
            approach=prepare_trajectory(approach,self.limits,velocity_scale,acceleration_scale)
            stage('pen-up approach collision samples')
            self.validate_states(sampled_positions(approach),stopped)
            offset=seconds(approach.joint_trajectory.points[-1])
            for field in ('trace_start_s','trace_end_s','cruise_start_s','cruise_end_s'):summary[field]+=offset
            return concatenate([approach,stroke]),summary
        def progress(state):
            self.planning_progress.update(state['phase'],state.get('completed'),state.get('total'),state.get('unit',''),
                **{k:state[k] for k in ('configuration_attempt','configuration_limit','starting_configurations','rejected_candidates') if k in state})
            if 'completed' in state:return
            self.get_logger().info('PLAN_EVENT '+json.dumps(dict(event='stroke_search_progress',request_id=self.request_id,**state)))
        distances,positions,(trajectory,summary),selection=select_stroke(curve,mode,start,self.ik_solver,valid,stopped,finish,progress=progress)
        summary['configuration_selection']=selection
        self.get_logger().info('PLAN_EVENT '+json.dumps(dict(event='stroke_start_selected',request_id=self.request_id,**selection)))
        resolved=[self.pose_from_joints(positions[int(np.argmin(abs(distances-distance)))]) for distance in curve.waypoint_distances]
        return trajectory,[summary],resolved

    def plan_path(self,request,response):
        if not self.operation.acquire(False):response.message='Another planning/execution operation is active';return response
        try:
            self.clear_path_preview()
            self.check_ready();self.plans.clear();self.stop_requested=False;self.stage='planning ordered path';self.request_id=uuid.uuid4().hex
            self.planning_progress.begin(self.request_id,'ordered path')
            if not 1<=len(request.waypoints)<=80:raise ValueError('Path requires 1..80 ordered waypoints')
            mode=request.orientation_mode or 'tool_axis'
            contacts=list(request.contact_segments) or [False]*len(request.waypoints)
            if len(contacts)!=len(request.waypoints):raise ValueError('Contact labels must match waypoint count')
            # A pen-up pose queue promises destinations, not an unrequested
            # straight Cartesian drawing trace from the held posture.
            style=request.style or ('polyline' if any(contacts) else 'visits')
            if mode not in MODES or style not in ('polyline','spline','spline_axis','visits','fixed_tcp'):raise ValueError('Invalid path style/orientation mode')
            if request.enforce_tcp_speed and style in ('visits','fixed_tcp'):
                raise ValueError('TCP cruise speed requires a continuous trace, not stopped pose visits or fixed-TCP rotation')
            if style=='fixed_tcp' and (mode!='full' or any(contacts)):
                raise ValueError('Fixed-TCP rotation requires Exact RPY and pen-up points')
            if request.plan_pen_up_approach and (style in ('visits','fixed_tcp') or len(request.waypoints)<2 or contacts[0]):
                raise ValueError('Stroke setup requires two or more trace waypoints and a pen-up first segment')
            if self.hardware and any(contacts):raise ValueError('Contact-tagged paths are mock-only until physical pen/contact qualification; pen-up paths remain available')
            velocity=request.velocity_scaling or min(1.,DEFAULT_SPEED/max(v['velocity'] for v in self.limits.values()));acceleration=request.acceleration_scaling or min(1.,DEFAULT_ACCELERATION/min(v['acceleration'] for v in self.limits.values()))
            if not all(math.isfinite(v) and 0<v<=1 for v in (velocity,acceleration)):raise ValueError('Scaling must be in (0,1]')
            speed=request.tcp_speed or .02;tcp_acceleration=request.tcp_acceleration or .05
            if not (math.isfinite(speed) and 0<speed<=MAX_TCP_SPEED and math.isfinite(tcp_acceleration) and 0<tcp_acceleration<=MAX_TCP_ACCELERATION):raise ValueError('TCP speed must be (0,0.4] m/s and acceleration (0,2] m/s²')
            minimum=request.tcp_min_speed
            if not math.isfinite(minimum) or minimum<0 or (minimum and minimum>=speed):raise ValueError('TCP range requires 0 < minimum < maximum; zero disables range mode')
            if minimum and minimum<.001:raise ValueError('Positive minimum TCP speed must be at least 1 mm/s')
            if minimum and style in ('visits','fixed_tcp'):raise ValueError('TCP speed range requires a continuous positional trace')
            transforms={}
            targets=[self.base_pose(p,transforms) for p in request.waypoints]
            for target in targets:
                p=target.pose.position
                if not inside_workspace([p.x,p.y,p.z],self.workspace):raise ValueError('Waypoint outside numerical XYZ input bounds')
            if style=='fixed_tcp':
                anchor=pose_matrix(targets[0].pose)[:3,3]
                if any(np.linalg.norm(pose_matrix(p.pose)[:3,3]-anchor)>1e-6 for p in targets):
                    raise ValueError('Fixed-TCP rotation waypoints must share one XYZ; the first leg is a pen-up approach')
            revision=self.publish_target(targets[-1]);start=list(self.hold.applied if self.hardware else self.hold.commanded)
            actual=[self.actual[n] for n in JOINTS]
            if max(abs(a-b) for a,b in zip(actual,start))>.05:raise RuntimeError('Measured start is too far from held command')
            self.valid(actual,measured=True);self.valid(start);scene=self.scene_digest()
            self.get_logger().info('PLAN_EVENT '+json.dumps(dict(event='path_request',request_id=self.request_id,style=style,mode=mode,start=start,waypoints=[dict(xyz=[p.pose.position.x,p.pose.position.y,p.pose.position.z],quaternion=[p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]) for p in targets],contacts=contacts,tcp_speed=speed,tcp_min_speed=minimum,tcp_acceleration=tcp_acceleration,velocity_scaling=velocity,acceleration_scaling=acceleration)))
            summaries=[]
            fixed_tcp_deviation=0.
            if style in ('visits','fixed_tcp'):
                if any(contacts):raise ValueError('Visit poses stops at each endpoint; it is not a contact trace')
                legs=[];q=start;resolved=[]
                for index,target in enumerate(targets):
                    self.planning_step('Planning ordered pose',waypoint_index=index+1,waypoint_total=len(targets))
                    if style=='fixed_tcp' and index>0:
                        qgoal=self.solve_ik(target,q,'full');pose=self.pose_from_joints(qgoal)
                        leg=self.plan_transit(q,qgoal,pose,velocity,acceleration,'cartesian')
                    else:qgoal,pose,leg=self.route_to_pose(target,q,mode,velocity,acceleration)
                    if len(leg.joint_trajectory.points)==1:
                        duplicate=copy.deepcopy(leg.joint_trajectory.points[0]);duplicate.time_from_start.sec=1
                        leg.joint_trajectory.points.append(duplicate)
                    if max(abs(a-b) for a,b in zip(q,leg.joint_trajectory.points[0].positions))>1e-6:raise RuntimeError('Visit leg start differs from previous endpoint')
                    leg.joint_trajectory.points[0].positions=list(q)
                    # Each visit explicitly stops. Equal zero derivatives on
                    # both sides preserve the validated polynomials when legs
                    # share a boundary point in the combined JTC trajectory.
                    for endpoint in (leg.joint_trajectory.points[0],leg.joint_trajectory.points[-1]):
                        endpoint.velocities=[0.]*6;endpoint.accelerations=[0.]*6
                    leg=prepare_trajectory(leg,self.limits,velocity,acceleration)
                    if style=='fixed_tcp' and index>0:
                        validate_cartesian_shape(leg,self.ik_solver,q,leg.joint_trajectory.points[-1].positions)
                        for candidate in sampled_positions(leg):
                            if self.stop_requested or self.model_changed:raise RuntimeError('Stopped during fixed-TCP validation')
                            deviation=float(np.linalg.norm(self.ik_solver.evaluate(candidate)[0][:3,3]-anchor))
                            fixed_tcp_deviation=max(fixed_tcp_deviation,deviation)
                            if deviation>.001:raise ValueError('Rotation trajectory moves the TCP by more than 1 mm; no plan retained')
                        distance,angle=self.ik_solver.errors(leg.joint_trajectory.points[-1].positions,pose_matrix(target.pose),'full')
                        if distance>.0002 or angle>.002:raise ValueError('Rotation endpoint fails the requested full pose')
                    legs.append(leg);q=leg.joint_trajectory.points[-1].positions;resolved.append(self.pose_from_joints(q))
                trajectory=concatenate(legs)
            else:
                if request.plan_pen_up_approach:
                    trajectory,summaries,resolved=self.trace_with_approach(start,targets,mode,style,contacts,speed,tcp_acceleration,velocity,acceleration,request.allow_contact_ramps,request.enforce_tcp_speed,minimum)
                else:
                    trajectory,summaries,resolved=self.trace_paths(start,targets,mode,style,contacts,speed,tcp_acceleration,velocity,acceleration,request.allow_contact_ramps,request.enforce_tcp_speed,minimum)
            trajectory=prepare_trajectory(trajectory,self.limits,velocity,acceleration,rescale=False,
                progress=lambda done,total:self.planning_step('Checking combined joint bounds and rates',done,total,'intervals'))
            if len(trajectory.joint_trajectory.points)>3000 or seconds(trajectory.joint_trajectory.points[-1])>120:raise ValueError('Combined path exceeds 3000 samples / 120 seconds; split the path')
            if max(abs(a-b) for a,b in zip(start,trajectory.joint_trajectory.points[0].positions))>1e-5:raise RuntimeError('Path start differs from held command')
            self.planning_step('Checking complete path collisions')
            self.validate_states(sampled_positions(trajectory))
            self.planning_step('Verifying scene, start and endpoint')
            self.check_ready()
            if self.scene_digest()!=scene or self.target_editor.revision!=revision:raise RuntimeError('Scene/target changed while planning; plan again')
            commanded=self.hold.applied if self.hardware else self.hold.commanded
            if max(abs(a-b) for a,b in zip(start,commanded))>.01:raise RuntimeError('Held start changed while planning')
            final=self.pose_from_joints(trajectory.joint_trajectory.points[-1].positions)
            verification=GetPositionFK.Request();verification.header.frame_id='base_link';verification.fk_link_names=['pointer_tip']
            verification.robot_state=self.robot_state(trajectory.joint_trajectory.points[-1].positions)
            verified=self.call(self.fk,verification)
            if verified.error_code.val!=1:raise RuntimeError('MoveIt endpoint FK unavailable')
            native_distance,native_angle=pose_error(final.pose,verified.pose_stamped[0].pose)
            if native_distance>1e-6 or native_angle>1e-5:raise RuntimeError('Live model and MoveIt FK disagree; restart matching planning model')
            if max(abs(self.actual[n]-v) for n,v in zip(JOINTS,start))>.05:raise RuntimeError('Measured start moved during path planning')
            distance,angle=pose_error(final.pose,targets[-1].pose)
            if mode=='tool_axis':
                a=pose_matrix(final.pose)[:3,2];b=pose_matrix(targets[-1].pose)[:3,2];angle=math.acos(float(np.clip(a@b,-1,1)))
            if distance>.0002 or (mode!='position' and angle>.002):raise RuntimeError('Path endpoint failed pose verification')
            resolved[-1]=final;token=uuid.uuid4().hex
            self.plans[token]=dict(trajectory=trajectory,start=start,target=final,requested_target=targets[-1],model=self.model,scene=scene,target_revision=revision,created=time.monotonic(),orientation_mode=mode,path_mode=style,expires_after=180.,validation_digest=self.validation_digest(trajectory,scene))
            self.active_id=token;self.stage='path planned'
            self.preview.publish(DisplayTrajectory(model_id=self.root.get('name'),trajectory_start=self.robot_state(start),trajectory=[trajectory]))
            self.show_path_preview(trajectory,targets)
            response.success=True;response.plan_id=token;response.trajectory=trajectory;response.duration=seconds(trajectory.joint_trajectory.points[-1]);response.resolved_target=final;response.resolved_waypoints=resolved
            mins=[s['modeled_contact_min_m_s'] for s in summaries if s['modeled_contact_max_m_s']>0];maxs=[s['modeled_contact_max_m_s'] for s in summaries if s['modeled_contact_max_m_s']>0]
            response.modeled_contact_min_speed=min(mins) if mins else 0.;response.modeled_contact_max_speed=max(maxs) if maxs else 0.
            cruises=[s for s in summaries if s.get('enforced_tcp_speed')]
            response.modeled_cruise_min_speed=min(s['modeled_cruise_min_m_s'] for s in cruises) if cruises else 0.
            response.modeled_cruise_max_speed=max(s['modeled_cruise_max_m_s'] for s in cruises) if cruises else 0.
            if len(summaries)==1:
                response.trace_start_time=summaries[0]['trace_start_s'];response.trace_end_time=summaries[0]['trace_end_s']
                response.cruise_start_time=summaries[0]['cruise_start_s'];response.cruise_end_time=summaries[0]['cruise_end_s']
            response.message=f'Ordered {style} path ready; {len(targets)} waypoints, no motion sent.'
            ranges=[s['speed_range'] for s in summaries if s.get('speed_range',{}).get('enabled')]
            response.speed_range_report_json=json.dumps(ranges)
            if ranges:
                response.message+=f' TCP range {1000*minimum:.2f}–{1000*speed:.2f} mm/s; modeled interior {1000*response.modeled_cruise_min_speed:.3f}–{1000*response.modeled_cruise_max_speed:.3f} mm/s. Smooth endpoint ramps are outside the interior band; no interior stops.'
                response.message+=' Timing means '+', '.join(f"{1000*r['modeled_mean_m_s']:.2f} mm/s" for r in ranges)+'.'
                slow=[f"{100*a/r['length_m']:.0f}–{100*b/r['length_m']:.0f}%" for r in ranges for a,b in r['slowdown_intervals_m']]
                if slow:response.message+=' Slower sections: '+', '.join(slow[:3])+('…' if len(slow)>3 else '')+'.'
            elif cruises:response.message+=f' Required TCP cruise {1000*speed:.2f} mm/s; modeled {1000*response.modeled_cruise_min_speed:.3f}–{1000*response.modeled_cruise_max_speed:.3f} mm/s, with no interior waypoint stops or automatic slowdown.'
            if style=='fixed_tcp':response.message+=f' Includes a pen-up approach to point 1; subsequent rotations have modeled TCP drift at most {fixed_tcp_deviation*1000:.3f} mm. Physical TCP accuracy is unmeasured.'
            if request.plan_pen_up_approach:response.message+=' Includes an explicitly requested pen-up route to a whole-stroke-selected starting configuration.'
            if request.allow_contact_ramps and any(contacts):
                response.message+=' Smooth contact endpoint ramps permitted; interior speed is checked.'
                if any(s.get('ramp_only') for s in summaries):response.message+=' Short stroke: ramp-only profile, requested cruise speed is not reached.'
            if mins:response.message+=f' Modeled contact speed {1000*min(mins):.2f}–{1000*max(maxs):.2f} mm/s; physical speed unqualified.'
            elif style=='fixed_tcp':response.message+=' Rotation timing uses the selected joint speed/acceleration caps.'
            elif not cruises:response.message+=' Pen-up timing; ramps and joint limits may lower TCP speed.'
            self.planning_step('Finalizing preview and joint travel reserve')
            reserve=trajectory_reserve(trajectory,self.limits)
            if reserve['margin_deg']<5.:response.message+=f" Low joint travel reserve: {reserve['joint']} {reserve['margin_deg']:.2f} deg along this plan; validate the next segment too."
            self.get_logger().info('PLAN_EVENT '+json.dumps(dict(event='path_planned',request_id=self.request_id,plan_id=token,duration=response.duration,points=len(trajectory.joint_trajectory.points),timing=summaries)))
        except Exception as error:
            self.stage='path rejected';response.message=str(error);self.get_logger().warn(response.message)
            self.get_logger().info('PLAN_EVENT '+json.dumps(dict(event='path_rejected',request_id=self.request_id,message=response.message,
                selection=getattr(error,'selection_report',{}))))
        finally:
            self.planning_progress.end();self.operation.release()
        return response

    def goal(self,request):
        record=self.plans.get(request.plan_id)
        if self.active or self.stopping or self.operation.locked() or record is None:return GoalResponse.REJECT
        if record['target_revision']!=self.target_editor.revision:return GoalResponse.REJECT
        if request.hardware_confirmed != self.hardware: return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel(self,handle):
        self.begin_stop(); return CancelResponse.ACCEPT

    def begin_stop(self,recover=False):
        self.clear_path_preview();self.stop_requested=True; self.plans.clear()
        self.stop_future=self.hold.request(recover=recover)
        return self.stop_future

    def wait_hold(self,recover=False,use_existing=False):
        future=self.stop_future if use_existing and self.stop_future is not None else self.begin_stop(recover); event=threading.Event(); future.add_done_callback(lambda _:event.set())
        if not event.wait(9): return False,'Powered hold response timed out; controller remains paused'
        return future.result()

    def stop(self,request,response):
        response.success,response.message=self.wait_hold(recover=self.hold.safety.startswith('HOLD:')); return response

    def recover(self,request,response):
        response.success,response.message=self.wait_hold(True); return response

    def estimated_pose(self):
        result=PoseStamped(); result.header.frame_id='base_link'; result.header.stamp=self.get_clock().now().to_msg()
        if self.model is None or len(self.actual)!=6: return result
        transform=forward_kinematics(self.root,self.actual)
        result.pose.position.x,result.pose.position.y,result.pose.position.z=map(float,transform[:3,3])
        q=Rotation.from_matrix(transform[:3,:3]).as_quat()
        result.pose.orientation=Quaternion(x=float(q[0]),y=float(q[1]),z=float(q[2]),w=float(q[3]))
        return result

    def execute(self,handle):
        response=ExecutePlan.Result();requested_at=time.monotonic()
        if not self.operation.acquire(False):
            response.message='Another operation is active'; handle.abort(); return response
        try:
            self.check_ready(); self.active=True; self.stop_requested=False; self.goal_dispatched=False; self.stop_future=None
            record=self.plans.pop(handle.request.plan_id,None); self.plans.clear()
            if record is None or time.monotonic()-record['created']>record.get('expires_after',60.): raise RuntimeError('Plan missing/expired; plan again')
            if record['target_revision']!=self.target_editor.revision:raise RuntimeError('Target changed; plan again')
            if record['model']!=self.model: raise RuntimeError('Model changed; plan again')
            scene=self.scene_digest()
            if scene!=record['scene']: raise RuntimeError('Scene changed; plan again')
            commanded=self.hold.applied if self.hardware else self.hold.commanded
            if max(abs(a-b) for a,b in zip(commanded,record['start']))>.01: raise RuntimeError('Held command moved since preview; plan again')
            if max(abs(self.actual[name]-value) for name,value in zip(JOINTS,record['start']))>.05:
                raise RuntimeError('Measured start moved since preview; settle and plan again')
            self.stage='validating execution'
            handle.publish_feedback(ExecutePlan.Feedback(stage='Checking current scene and start',progress=0.))
            self.valid([self.actual[name] for name in JOINTS],measured=True)
            self.validate_execution_preview(record,scene)
            self.check_ready()
            if self.scene_digest()!=record['scene']: raise RuntimeError('Scene changed during validation; plan again')
            commanded=self.hold.applied if self.hardware else self.hold.commanded
            if max(abs(a-b) for a,b in zip(commanded,record['start']))>.01:
                raise RuntimeError('Held command moved during validation; plan again')
            if max(abs(self.actual[name]-value) for name,value in zip(JOINTS,record['start']))>.05:
                raise RuntimeError('Measured start moved during validation; plan again')
            goal=FollowJointTrajectory.Goal(); goal.trajectory=copy.deepcopy(record['trajectory'].joint_trajectory)
            goal.trajectory.header.stamp.sec=0; goal.trajectory.header.stamp.nanosec=0
            # A slow scene service must not age out feedback/safety unnoticed.
            self.check_ready()
            if handle.is_cancel_requested or self.stop_requested or self.model_changed:
                raise RuntimeError('Stopped before controller dispatch')
            self.goal_dispatched=True
            event=threading.Event(); future=self.controller.send_goal_async(goal)
            self.get_logger().info('EXECUTION_DISPATCH '+json.dumps(dict(plan_id=handle.request.plan_id,
                preflight_s=time.monotonic()-requested_at,preview_validation_reused=True,points=len(goal.trajectory.points))))
            future.add_done_callback(lambda _:event.set())
            if not event.wait(5): raise RuntimeError('Controller goal acceptance timed out')
            self.controller_goal=future.result()
            if not self.controller_goal.accepted: raise RuntimeError('Controller rejected planned trajectory')
            result=self.controller_goal.get_result_async(); self.stage='executing'
            started=time.monotonic(); duration=seconds(goal.trajectory.points[-1]); scene_check=started
            while not result.done():
                if handle.is_cancel_requested or self.stop_requested:
                    self.wait_hold(use_existing=True); self.stage='stopped'; response.message='Stopped — old trajectory discarded; powered hold requested'
                    if handle.is_cancel_requested: handle.canceled()
                    else: handle.abort()
                    return response
                if self.hold.safety.startswith('HOLD:'): raise RuntimeError(self.hold.safety)
                if time.monotonic()-self.actual_at>.5 or time.monotonic()-self.hold.last_state>.5:
                    self.wait_hold(False); raise RuntimeError('Feedback stale during motion; powered hold requested')
                if time.monotonic()-started>duration+5:
                    self.wait_hold(False); raise RuntimeError('Execution timed out; powered hold requested')
                if time.monotonic()-scene_check>0.5:
                    if self.scene_digest()!=record['scene']:
                        self.wait_hold(False); raise RuntimeError('Planning scene changed; powered hold requested')
                    scene_check=time.monotonic()
                feedback=ExecutePlan.Feedback(stage='executing',progress=min(.99,(time.monotonic()-started)/max(duration,.01)))
                handle.publish_feedback(feedback); time.sleep(.04)
            if handle.is_cancel_requested or self.stop_requested:
                self.wait_hold(use_existing=True); self.stage='stopped'
                response.message='Stopped — old trajectory discarded; powered hold requested'
                if handle.is_cancel_requested: handle.canceled()
                else: handle.abort()
                return response
            outcome=result.result()
            if outcome.status!=4 or outcome.result.error_code!=0:
                raise RuntimeError('Controller stopped: '+outcome.result.error_string)
            if self.hold.safety.startswith('HOLD:'): raise RuntimeError(self.hold.safety)
            response.measured_pose=self.estimated_pose()
            response.position_error_m,response.orientation_error_rad=pose_error(response.measured_pose.pose,record['target'].pose)
            response.success=True
            response.message=('Controller completed; encoder/model TCP residual: '
                f'{response.position_error_m*1000:.2f} mm, {math.degrees(response.orientation_error_rad):.2f} deg. '
                'Independent physical tip accuracy is not measured.')
            handle.succeed(); self.stage='completed'
        except Exception as error:
            response.message=str(error); self.stage='stopped'
            if handle.is_cancel_requested:
                self.wait_hold(use_existing=True)
                handle.canceled()
                return response
            if self.stop_requested:
                self.wait_hold(use_existing=True)

            if self.goal_dispatched and not self.hold.safety.startswith('HOLD:'):
                self.wait_hold(use_existing=self.stop_requested)
            handle.abort()
        finally:
            self.active=False; self.controller_goal=None; self.goal_dispatched=False; self.operation.release()
        return response

    def update_status(self):
        try:
            self.check_ready(); ready=True; reason=''
        except Exception as error: ready=False; reason=str(error)
        pose=None
        try:
            value=self.estimated_pose().pose; q=value.orientation
            if self.model and self.actual and not self.model_changed:
                rpy=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz',degrees=True).tolist()
                pose=[value.position.x,value.position.y,value.position.z,*rpy]
        except Exception: pass
        message=dict(ready=ready,reason=reason,stage=self.stage,hardware=self.hardware,active=self.active,
            tcp_frame='pointer_tip',frame='base_link',current_pose_xyz_rpy_deg=pose,plan_id=self.active_id,
            motion_mode='pose_and_ordered_paths',contact_motion_supported=False,contact_timing_planning_supported=not self.hardware,
            safety=self.hold.safety,proxy_tcp=True,
            max_speed_deg_s=math.degrees(max(v['velocity'] for v in self.limits.values())) if self.limits else 30.,
            max_acceleration_deg_s2=math.degrees(min(v['acceleration'] for v in self.limits.values())) if self.limits else 180.,
            scene='Example floor; final base wings/fixture not yet modeled')
        message['planning_progress']=self.planning_progress.snapshot()
        message['ik_attempts']=self.ik_attempt_count
        message['ik_report']=self.last_ik_report
        if self.ik_solver is not None and self.actual:
            message['current_conditioning_modes']={mode:self.ik_solver.conditioning([self.actual[n] for n in JOINTS],mode) for mode in MODES}
            message['current_conditioning']=message['current_conditioning_modes']['tool_axis']
        message['target_revision']=self.target_editor.revision
        message['workspace']=self.workspace
        message['joint_limits_deg']={name:{'minimum':math.degrees(value['lower']),'maximum':math.degrees(value['upper']),
            'current':math.degrees(self.actual[name]) if name in self.actual else None} for name,value in self.limits.items()}
        if self.model and self.actual and not self.model_changed:
            self.target_editor.initialize(self.estimated_pose())
            if not self.workspace_drawn:
                self.workspace_drawn=True
                self.workspace_pub.publish(workspace_markers(self.root,self.limits,self.workspace))
                self.working_region_pub.publish(working_region_marker(self.workspace))
        self.status.publish(String(data=json.dumps(message)))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=MotionServer(); executor=MultiThreadedExecutor(num_threads=6); executor.add_node(node)
    stopping=threading.Event()
    def stop_signal(signum,frame):
        if not stopping.is_set():
            stopping.set()
            if node.active: node.begin_stop()
    signal.signal(signal.SIGINT,stop_signal); signal.signal(signal.SIGTERM,stop_signal)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=.1)
            if stopping.is_set() and not node.active and node.hold.phase=='idle': break
    finally:
        executor.shutdown(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 0
