#!/usr/bin/python3
"""MoveIt/FCL/KDL + real controller integration; mock or emulated PTY only."""
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
os.environ['QT_QPA_PLATFORM']='offscreen'
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionFK, ApplyPlanningScene
from moveit_msgs.msg import RobotState, CollisionObject
from shape_msgs.msg import SolidPrimitive
from pinchy_bringup.control_contract import JOINTS
from pinchy_motion_interfaces.srv import PlanPose,PlanPath
from pinchy_motion_interfaces.action import ExecutePlan
from pinchy_motion.geometry import (forward_kinematics,quaternion_from_rpy,model_limits,
    prepare_trajectory,segment_coefficients,extrema,sampled_positions,seconds)
from test_ros_control import SerialEmulator
import yaml
ROOT=Path(__file__).resolve().parents[1]


class GeometryTests(unittest.TestCase):
    def test_rpy_is_urdf_fixed_xyz_not_intrinsic_zyx(self):
        q=quaternion_from_rpy([20,30,40])
        r=Rotation.from_euler('xyz',[20,30,40],degrees=True)
        np.testing.assert_allclose(Rotation.from_quat(q).as_matrix(),r.as_matrix(),atol=1e-14)
        with self.assertRaises(ValueError):quaternion_from_rpy([0,math.nan,0])

    def test_exact_spline_extrema_retiming_and_bounds(self):
        from moveit_msgs.msg import RobotTrajectory
        model=(ROOT/'src/pinchy_description/urdf/pinchy_arm.urdf').read_text()
        limits=model_limits(model); trajectory=RobotTrajectory();trajectory.joint_trajectory.joint_names=JOINTS
        a=JointTrajectoryPoint(positions=[0.]*6,velocities=[0.]*6,accelerations=[0.]*6)
        b=copy.deepcopy(a);b.positions=[.1]*6;b.time_from_start.nanosec=100_000_000
        trajectory.joint_trajectory.points=[a,b]
        result=prepare_trajectory(trajectory,limits)
        duration,coefficients=segment_coefficients(*result.joint_trajectory.points)
        for i,name in enumerate(JOINTS):
            self.assertLessEqual(max(abs(extrema(coefficients,duration,1)[i])),limits[name]['velocity']+1e-10)
            self.assertLessEqual(max(abs(extrema(coefficients,duration,2)[i])),limits[name]['acceleration']+1e-10)
        self.assertGreater(len(list(sampled_positions(result))),10)
        bad=copy.deepcopy(trajectory);bad.joint_trajectory.points[-1].positions[5]=-1.
        with self.assertRaises(ValueError):prepare_trajectory(bad,limits)
        bad=copy.deepcopy(trajectory);bad.joint_trajectory.points[-1].positions[0]=math.nan
        with self.assertRaises(ValueError):prepare_trajectory(bad,limits)

    def test_rviz_keeps_the_shared_layout(self):
        spec=importlib.util.spec_from_file_location('planning_stack',ROOT/'src/pinchy_moveit_config/scripts/planning_stack.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        original=yaml.safe_load((ROOT/'src/pinchy_bringup/config/arm.rviz').read_text())
        configured=module.visualization_config()
        goal=next(d for d in configured['Visualization Manager']['Displays'] if d.get('Name')=='Draggable goal')
        self.assertEqual(goal['Interactive Markers Namespace'],'/pinchy_motion/goal')
        for key in ('Panels','Window Geometry'):
            self.assertEqual(configured[key],original[key])
        for key in ('Views','Global Options'):
            self.assertEqual(configured['Visualization Manager'][key],original['Visualization Manager'][key])
        self.assertEqual(configured['Visualization Manager']['Displays'][:len(original['Visualization Manager']['Displays'])],original['Visualization Manager']['Displays'])

    def test_working_region_bounds_match_visible_wireframe(self):
        from pinchy_motion.workspace import configuration,working_region_marker
        settings=configuration();region=settings['design_working_region']
        np.testing.assert_allclose(np.array(region['maximum_m'])-region['minimum_m'],[.4,.4,.4])
        marker=working_region_marker(settings).markers[0]
        self.assertEqual(marker.header.frame_id,'base_link');self.assertEqual(len(marker.points),24)
        coordinates=np.array([[p.x,p.y,p.z] for p in marker.points])
        np.testing.assert_allclose(coordinates.min(axis=0),[.1,-.2,0.])
        np.testing.assert_allclose(coordinates.max(axis=0),[.5,.2,.4])

    def test_default_collision_exclusions_only_adjacent_and_live_tool(self):
        root=ET.parse(ROOT/'src/pinchy_moveit_config/config/arm.srdf').getroot()
        self.assertEqual(root.find("group[@name='arm']/chain").get('tip_link'),'pointer_tip')
        self.assertEqual(len(root.findall('disable_collisions')),6)
        self.assertTrue(all(p.get('reason')=='Adjacent' for p in root.findall('disable_collisions')))


class HoldProtocolTests(unittest.TestCase):
    def test_late_recovery_response_cannot_resume_after_timeout(self):
        from pinchy_bringup.powered_hold import PoweredHold
        from rclpy.task import Future
        context=Context();rclpy.init(context=context,domain_id=222)
        node=rclpy.create_node('hold_timeout_test',context=context)
        hold=PoweredHold(node,True)
        class Publisher:
            def __init__(self):self.values=[]
            def publish(self,msg):self.values.append(msg.factor)
        publisher=Publisher();hold.scaling=publisher
        try:
            hold.request(recover=True);generation=hold.generation
            hold.phase='reset';hold.finish(False,'Timed out')
            late=Future();late.set_result(Trigger.Response(success=True,message='late'))
            hold.reset_done(late,generation)
            self.assertEqual(hold.phase,'idle')
            self.assertNotIn(1.,publisher.values)
            hold.request(recover=True)
            hold.reset_done(late,generation)
            self.assertEqual(hold.phase,'pause')
            self.assertNotIn(1.,publisher.values)
        finally:
            hold.client.destroy();node.destroy_node();context.shutdown()



class CartesianFixture:
    hardware=False
    @classmethod
    def setUpClass(cls):
        cls.folder=Path(tempfile.mkdtemp(prefix='pinchy-cartesian-tests-'))
        print(f'\n{cls.__name__} artifacts: {cls.folder}',flush=True)
        cls.domain=185+(os.getpid()%20)+int(cls.hardware)
        os.environ['ROS_DOMAIN_ID']=str(cls.domain)
        cls.context=Context();rclpy.init(context=cls.context,domain_id=cls.domain)
        cls.node=rclpy.create_node('cartesian_test',context=cls.context)
        cls.executor=SingleThreadedExecutor(context=cls.context);cls.executor.add_node(cls.node)
        cls.state={};cls.status={};cls.processes=[];cls.logs=[]
        cls.bus=SerialEmulator(cls.folder) if cls.hardware else None
        env=dict(os.environ,ROS_DOMAIN_ID=str(cls.domain),ROS_LOG_DIR=str(cls.folder/'ros_logs'))
        commands=[]
        if cls.hardware:
            config=yaml.safe_load((ROOT/'src/pinchy_bringup/config/servo_calibration.example.yaml').read_text())
            if os.environ.get('PINCHY_TEST_SAVED_LIMITS')=='1':
                config=json.loads((ROOT/'calibration/arm.json').read_text())
                for values in config['joints'].values():
                    values['zero_tick']=2048
                    ticks=[round(2048+values['direction']*a/(2*math.pi/4096)) for a in (values['min_radians'],values['max_radians'])]
                    values['min_tick'],values['max_tick']=min(ticks),max(ticks)
            config.update(calibration_confirmed=True,port=cls.bus.path,speed=500,acceleration=32,torque_limit=1000)
            for joint in config['joints'].values():joint['max_velocity']=math.radians(30)
            path=cls.folder/'emulated_calibration.yaml';path.write_text(yaml.safe_dump(config))
            commands.append(['ros2','launch','pinchy_bringup','hardware_control.launch.py',f'calibration:={path}','motion_enabled:=true'])
            commands.append(['ros2','launch','pinchy_moveit_config','planning.launch.py','hardware:=true','rviz:=false','panel:=false'])
        else:
            command=['ros2','launch','pinchy_moveit_config','mock_motion.launch.py','rviz:=false','panel:=false']
            if os.environ.get('PINCHY_TEST_CALIBRATED_MOCK')=='1':command.append(f'calibration:={ROOT / "calibration/arm.json"}')
            commands.append(command)
        for i,command in enumerate(commands):
            log=(cls.folder/f'launch-{i}.log').open('w');cls.logs.append(log)
            cls.processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True))
        def joints(message):cls.state=dict(zip(message.name,message.position))
        def status(message):cls.status=json.loads(message.data)
        cls.node.create_subscription(JointState,'/joint_states',joints,10)
        cls.node.create_subscription(String,'/pinchy_motion/status',status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.plan_client=cls.node.create_client(PlanPose,'/pinchy_motion/plan_pose')
        cls.execute_client=ActionClient(cls.node,ExecutePlan,'/pinchy_motion/execute_plan')
        deadline=time.monotonic()+50
        while time.monotonic()<deadline:
            cls.executor.spin_once(timeout_sec=.1)
            if cls.status.get('ready') and len(cls.state)==6 and cls.plan_client.service_is_ready():return
            if any(p.poll() is not None for p in cls.processes):break
        cls.tearDownClass();raise RuntimeError('Planning stack failed; inspect '+str(cls.folder))

    @classmethod
    def tearDownClass(cls):
        for process in reversed(cls.processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=14)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        for log in cls.logs:log.close()
        cls.executor.shutdown();cls.node.destroy_node();cls.context.shutdown()
        if cls.bus:cls.bus.close()

    def wait(self,future,seconds=60):
        self.executor.spin_until_future_complete(future,timeout_sec=seconds)
        self.assertTrue(future.done(),'Response timeout; '+str(self.folder));return future.result()

    def pump(self,seconds=.3):
        end=time.monotonic()+seconds
        while time.monotonic()<end:self.executor.spin_once(timeout_sec=.02)

    def service(self,kind,name,request,seconds=15):
        client=self.node.create_client(kind,name)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=8),name)
            return self.wait(client.call_async(request),seconds)
        finally:self.node.destroy_client(client)

    def target(self,q):
        request=GetPositionFK.Request();request.header.frame_id='base_link';request.fk_link_names=['pointer_tip']
        request.robot_state=RobotState(joint_state=JointState(name=JOINTS,position=q))
        result=self.service(GetPositionFK,'/compute_fk',request)
        self.assertEqual(result.error_code.val,1)
        return result.pose_stamped[0]

    def plan(self,q=None,scale=.5,target=None):
        request=PlanPose.Request(target=target if target is not None else self.target(q),
            velocity_scaling=scale,acceleration_scaling=scale,orientation_mode='full')
        result=self.wait(self.plan_client.call_async(request),90)
        self.assertTrue(result.success,result.message);return result

    def execute(self,plan,wait=True):
        handle=self.wait(self.execute_client.send_goal_async(ExecutePlan.Goal(plan_id=plan.plan_id,hardware_confirmed=self.hardware)),5)
        self.assertTrue(handle.accepted,'Plan goal rejected')
        return self.wait(handle.get_result_async(),max(40,plan.duration+20)) if wait else handle

    def wait_for_motion(self,before):
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            self.pump(.04)
            if self.status.get('active') and max(abs(self.state[name]-before[name]) for name in JOINTS)>.004:return
        self.fail('No measured joint movement before Stop/fault test')

    def add_box(self,identifier,center,size=.04,remove=False):
        request=ApplyPlanningScene.Request();request.scene.is_diff=True
        box=CollisionObject();box.header.frame_id='base_link';box.id=identifier
        box.operation=CollisionObject.REMOVE if remove else CollisionObject.ADD
        if not remove:
            box.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[size]*3)]
            pose=Pose();pose.orientation.w=1.;pose.position.x,pose.position.y,pose.position.z=center
            box.primitive_poses=[pose]
        request.scene.world.collision_objects=[box]
        self.assertTrue(self.service(ApplyPlanningScene,'/apply_planning_scene',request).success)
        self.pump(.1)

    def test_00_nonadjacent_self_collision_is_checked(self):
        from moveit_msgs.srv import GetStateValidity
        q=[.6544646696,-1.3436264282,1.3395256831,1.1876056153,-2.7280581707,.9210277628]
        request=GetStateValidity.Request(group_name='arm',robot_state=RobotState(joint_state=JointState(name=JOINTS,position=q)))
        result=self.service(GetStateValidity,'/check_state_validity',request)
        self.assertFalse(result.valid)
        self.assertTrue(any({c.contact_body_1,c.contact_body_2}=={'base_link','pen_holder_link'} for c in result.contacts))

    def test_01_plan_preview_execute_full_pose(self):
        before=dict(self.state)
        plan=self.plan([.05,-.08,.15,-.08,.05,.12])
        self.pump(.1)
        for name in JOINTS:self.assertAlmostEqual(before[name],self.state[name],delta=.002)
        result=self.execute(plan)
        self.assertTrue(result.result.success,result.result.message)
        self.assertLess(result.result.position_error_m,.003)
        self.assertLess(result.result.orientation_error_rad,.02)

    def test_01b_explicit_orientation_modes_and_invalid_modes(self):
        target=self.target([.1,-.2,.4,-.2,.3,.5]);before=dict(self.state)
        original=copy.deepcopy(target)
        q=target.pose.orientation
        rotation=Rotation.from_quat([q.x,q.y,q.z,q.w])*Rotation.from_euler('z',150,degrees=True)
        target.pose.orientation=Quaternion(x=float(rotation.as_quat()[0]),y=float(rotation.as_quat()[1]),z=float(rotation.as_quat()[2]),w=float(rotation.as_quat()[3]))
        for mode in ('tool_axis','position'):
            result=self.wait(self.plan_client.call_async(PlanPose.Request(target=target,orientation_mode=mode)),90)
            self.assertTrue(result.success,result.message)
            self.assertAlmostEqual(result.resolved_target.pose.position.x,target.pose.position.x,delta=.0002)
            if mode=='tool_axis':
                a=result.resolved_target.pose.orientation;b=original.pose.orientation
                np.testing.assert_allclose(Rotation.from_quat([a.x,a.y,a.z,a.w]).as_matrix()[:,2],
                    Rotation.from_quat([b.x,b.y,b.z,b.w]).as_matrix()[:,2],atol=.002)
        rejected=self.wait(self.plan_client.call_async(PlanPose.Request(target=target,orientation_mode='silent_relax')),5)
        self.assertFalse(rejected.success)
        self.pump(.1)
        for name in JOINTS:self.assertAlmostEqual(before[name],self.state[name],delta=.002)

    def test_01e_free_twist_known_reachable_directions_inside_working_region(self):
        from moveit_msgs.srv import GetStateValidity
        rng=np.random.default_rng(831);bounds=self.status['joint_limits_deg']
        lower=np.radians([bounds[n]['minimum'] for n in JOINTS]);upper=np.radians([bounds[n]['maximum'] for n in JOINTS])
        before=dict(self.state);records=[]
        for q in rng.uniform(lower,upper,(300,6)):
            target=self.target(q.tolist());p=target.pose.position
            if not (.1<=p.x<=.5 and -.2<=p.y<=.2 and 0<=p.z<=.4):continue
            valid=GetStateValidity.Request();valid.group_name='arm';valid.robot_state=self.robot_state_for_test(q.tolist())
            if not self.service(GetStateValidity,'/check_state_validity',valid).valid:continue
            r=target.pose.orientation;original=Rotation.from_quat([r.x,r.y,r.z,r.w])
            rotation=(original*Rotation.from_euler('z',float(rng.uniform(30,330)),degrees=True)).as_quat()
            target.pose.orientation=Quaternion(x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3])
            started=time.monotonic()
            # Empty mode verifies the intended default is free twist.
            result=self.wait(self.plan_client.call_async(PlanPose.Request(target=target)),90)
            records.append(dict(success=result.success,message=result.message,seconds=time.monotonic()-started))
            self.assertTrue(result.success,result.message);self.assertIn('tool_axis / joint',result.message)
            solved=result.resolved_target.pose.orientation
            np.testing.assert_allclose(Rotation.from_quat([solved.x,solved.y,solved.z,solved.w]).as_matrix()[:,2],original.as_matrix()[:,2],atol=.002)
            if len(records)==8:break
        (self.folder/'free_twist_witnesses.json').write_text(json.dumps(records,indent=2))
        self.assertEqual(len(records),8);self.pump(.35)
        self.assertGreater(self.status['ik_report'].get('native_queries',0),0)
        for n in JOINTS:self.assertAlmostEqual(before[n],self.state[n],delta=.002)

    def robot_state_for_test(self,q):
        return RobotState(joint_state=JointState(name=JOINTS,position=q))

    def test_01c_straight_lerp_slerp_path_preview_and_execute(self):
        interior=self.plan([.1,-.25,.5,-.25,.4,.4]);self.assertTrue(self.execute(interior).result.success)
        target=self.target([.12,-.23,.48,-.23,.42,.42]);before=dict(self.state)
        result=self.wait(self.plan_client.call_async(PlanPose.Request(target=target,path_mode='cartesian',orientation_mode='full')),90)
        self.assertTrue(result.success,result.message);self.pump(.1)
        for name in JOINTS:self.assertAlmostEqual(before[name],self.state[name],delta=.002)
        self.assertTrue(self.execute(result).result.success)

    def test_01d_blocked_straight_path_never_retains_partial_plan(self):
        start=self.plan([.1,-.25,.5,-.25,.4,.4]);self.assertTrue(self.execute(start).result.success)
        target=self.target([.45,-.25,.5,-.25,.4,.4]);origin=self.target([.1,-.25,.5,-.25,.4,.4])
        a,b=origin.pose.position,target.pose.position
        midpoint=[(a.x+b.x)/2,(a.y+b.y)/2,(a.z+b.z)/2]
        before=dict(self.state);self.add_box('blocked_line',midpoint,size=.015)
        try:
            result=self.wait(self.plan_client.call_async(PlanPose.Request(target=target,path_mode='cartesian',orientation_mode='full')),90)
            self.assertFalse(result.success);self.assertFalse(result.plan_id)
            self.assertTrue('incomplete' in result.message or 'collision' in result.message.lower(),result.message)
            self.pump(.1)
            for name in JOINTS:self.assertAlmostEqual(before[name],self.state[name],delta=.002)
        finally:self.add_box('blocked_line',[0.,0.,0.],remove=True)

    def test_01h_ordered_free_twist_path_and_contact_timing(self):
        interior=self.plan([.1,-.25,.5,-.25,.4,.4]);self.assertTrue(self.execute(interior).result.success)
        origin=self.target([.1,-.25,.5,-.25,.4,.4]);targets=[]
        for offset in (.01,.03,.04):
            p=copy.deepcopy(origin);p.pose.position.y+=offset;targets.append(p)
        request=PlanPath.Request(waypoints=targets,orientation_mode='tool_axis',style='polyline',tcp_speed=.01,tcp_acceleration=.05,
            contact_segments=[False,not self.hardware,False])
        before=dict(self.state);result=self.service(PlanPath,'/pinchy_motion/plan_path',request,seconds=120)
        self.assertTrue(result.success,result.message);self.assertEqual(len(result.resolved_waypoints),3)
        for target,solved in zip(targets,result.resolved_waypoints):self.assertAlmostEqual(target.pose.position.y,solved.pose.position.y,delta=.001)
        for name in JOINTS:self.assertAlmostEqual(before[name],self.state[name],delta=.002)
        if not self.hardware:
            self.assertGreater(result.modeled_contact_min_speed,.0095);self.assertLess(result.modeled_contact_max_speed,.0105)
        self.assertTrue(self.execute(result).result.success)
        # The same server-held plan mechanism handles whole paths; no partial
        # execution is permitted when contact timing or physical qualification fails.
        rejected=copy.deepcopy(request);rejected.contact_segments=[True,True,True]
        failure=self.service(PlanPath,'/pinchy_motion/plan_path',rejected,seconds=120)
        self.assertFalse(failure.success);self.assertFalse(failure.plan_id)

    def test_01i_queued_path_gui_and_cli(self):
        from python_qt_binding.QtWidgets import QApplication
        from rclpy.parameter import Parameter
        from pinchy_motion.panel import PosePanel
        app=QApplication.instance() or QApplication([])
        node=rclpy.create_node('queued_path_panel_test',context=self.context,parameter_overrides=[Parameter('hardware',value=self.hardware)])
        window=PosePanel(node);window.show()
        def until(predicate,timeout=90):
            end=time.monotonic()+timeout
            while not predicate() and time.monotonic()<end:app.processEvents();self.executor.spin_once(timeout_sec=0);time.sleep(.01)
            self.assertTrue(predicate(),window.notice.text())
        try:
            until(lambda:window.plan_button.isEnabled())
            window.copy_current();base=[box.value() for box in window.inputs]
            for delta in (.002,.004):
                window.inputs[1].setValue(base[1]+delta);window.add_waypoint()
            self.assertEqual(window.queue.rowCount(),2);window.plan_queued();until(lambda:not window.busy)
            self.assertTrue(window.plan_id,window.notice.text());self.assertIn('Ordered polyline',window.notice.text())
            window.grab().save(str(self.folder/'ordered-path-panel.png'))
            self.assertEqual(window.orientation_mode.currentData(),'tool_axis')
        finally:window.timer.stop();window.executor.shutdown();window.close();app.processEvents();node.destroy_node()
        target=self.target([self.state[n] for n in JOINTS]);p,q=target.pose.position,target.pose.orientation
        rpy=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz',degrees=True).tolist()
        file=self.folder/'ordered-path.json';file.write_text(json.dumps(dict(style='polyline',orientation_mode='tool_axis',tcp_speed_m_s=.01,waypoints=[dict(xyz=[p.x,p.y+.002,p.z],rpy_deg=rpy),dict(xyz=[p.x,p.y+.004,p.z],rpy_deg=rpy)])))
        process=subprocess.run(['ros2','run','pinchy_motion','pose','path',str(file)],capture_output=True,text=True,timeout=120)
        self.assertEqual(process.returncode,0,process.stdout+process.stderr)

    def test_01j_free_twist_straight_transit(self):
        interior=self.plan([.1,-.25,.5,-.25,.4,.4]);self.assertTrue(self.execute(interior).result.success)
        target=self.target([.1,-.25,.5,-.25,.4,.4]);target.pose.position.y+=.008
        result=self.service(PlanPose,'/pinchy_motion/plan_pose',PlanPose.Request(target=target,path_mode='cartesian',orientation_mode='tool_axis'),seconds=120)
        self.assertTrue(result.success,result.message)
        self.assertTrue(self.execute(result).result.success)

    def test_01k_spline_and_visit_path_previews(self):
        interior=self.plan([.1,-.25,.5,-.25,.4,.4]);self.assertTrue(self.execute(interior).result.success)
        origin=self.target([self.state[n] for n in JOINTS]);targets=[]
        for y,z in ((.005,.001),(.01,0.)):
            target=copy.deepcopy(origin);target.pose.position.y+=y;target.pose.position.z+=z;targets.append(target)
        before=dict(self.state)
        for style in ('spline','visits'):
            result=self.service(PlanPath,'/pinchy_motion/plan_path',PlanPath.Request(waypoints=targets,style=style,orientation_mode='tool_axis',tcp_speed=.01,tcp_acceleration=.05),seconds=120)
            self.assertTrue(result.success,result.message);self.assertEqual(len(result.resolved_waypoints),2)
            for name in JOINTS:self.assertAlmostEqual(before[name],self.state[name],delta=.002)

    def test_01m_operator_nearby_xyz_with_changed_direction(self):
        initial=self.plan([-.04833406335,.07856124796,-.26294063578,.19052735656,-2.09552105474,.05942761223])
        self.assertTrue(self.execute(initial).result.success)
        target=PoseStamped();target.header.frame_id='base_link'
        target.pose.position.x=.471461;target.pose.position.y=-.003679;target.pose.position.z=.239280
        quaternion=Rotation.from_euler('xyz',[-130.083,-86.586,4.025],degrees=True).as_quat()
        target.pose.orientation=Quaternion(x=float(quaternion[0]),y=float(quaternion[1]),z=float(quaternion[2]),w=float(quaternion[3]))
        before=dict(self.state)
        failed=self.service(PlanPose,'/pinchy_motion/plan_pose',PlanPose.Request(target=target,orientation_mode='tool_axis'),seconds=90)
        self.assertFalse(failed.success);self.assertFalse(failed.plan_id)
        self.assertIn('direction',failed.message)
        for name in JOINTS:self.assertAlmostEqual(before[name],self.state[name],delta=.002)
        # Deliberately copy the current direction while retaining exactly XYZ.
        current=self.target([self.state[name] for name in JOINTS]);target.pose.orientation=current.pose.orientation
        planned=self.service(PlanPose,'/pinchy_motion/plan_pose',PlanPose.Request(target=target,orientation_mode='tool_axis',path_mode='cartesian'),seconds=90)
        # These rounded coordinates are within micrometres of the starting
        # XYZ. A trace intentionally rejects duplicate points; that is not an
        # IK failure. Exercise an actual nearby translation separately.
        self.assertFalse(planned.success);self.assertIn('no positional movement',planned.message)
        target.pose.position.y+=.002
        planned=self.service(PlanPose,'/pinchy_motion/plan_pose',PlanPose.Request(target=target,orientation_mode='tool_axis',path_mode='cartesian'),seconds=90)
        self.assertTrue(planned.success,planned.message)
        for name in JOINTS:self.assertAlmostEqual(before[name],self.state[name],delta=.002)

    def test_02_unreachable_and_invalid_pose_never_moves(self):
        before=dict(self.state);target=PoseStamped();target.header.frame_id='base_link'
        target.pose.position.x=10.;target.pose.orientation.w=1.
        result=self.wait(self.plan_client.call_async(PlanPose.Request(target=target)),15)
        self.assertFalse(result.success);self.assertIn('numerical XYZ input bounds',result.message)
        target.pose.position.x=math.nan
        result=self.wait(self.plan_client.call_async(PlanPose.Request(target=target)),5)
        self.assertFalse(result.success);self.assertIn('finite',result.message)
        self.pump(.1)
        for name in JOINTS:self.assertAlmostEqual(before[name],self.state[name],delta=.002)

    def test_03_collision_rejected_without_movement(self):
        target=self.target([.12,-.1,.2,-.1,.12,.2]);p=target.pose.position
        before=dict(self.state)
        self.add_box('blocked_target',[p.x,p.y,p.z])
        try:
            result=self.wait(self.plan_client.call_async(PlanPose.Request(target=target)),15)
            self.assertFalse(result.success,result.message)
            for name in JOINTS:self.assertAlmostEqual(before[name],self.state[name],delta=.002)
        finally:self.add_box('blocked_target',[0.,0.,0.],remove=True)

    def test_04_scene_change_invalidates_preview(self):
        plan=self.plan([.08,-.08,.18,-.08,.08,.16]);before=dict(self.state)
        self.add_box('changed_scene',[.8,.8,.8])
        try:
            result=self.execute(plan)
            self.assertFalse(result.result.success)
            self.assertIn('Scene changed',result.result.message)
            for name in JOINTS:self.assertAlmostEqual(before[name],self.state[name],delta=.002)
        finally:self.add_box('changed_scene',[0.,0.,0.],remove=True)

    def test_05_stop_retains_powered_hold_and_allows_new_plan(self):
        plan=self.plan([.22,-.1,.25,-.12,.2,.35],scale=.12)
        before=dict(self.state);handle=self.execute(plan,wait=False);self.wait_for_motion(before)
        stopped=self.service(Trigger,'/pinchy_motion/stop',Trigger.Request())
        self.assertTrue(stopped.success,stopped.message)
        result=self.wait(handle.get_result_async(),10);self.assertFalse(result.result.success)
        self.pump(.2);held=dict(self.state);self.pump(.4)
        for name in JOINTS:self.assertAlmostEqual(held[name],self.state[name],delta=.003)
        plan=self.plan([.05,-.08,.15,-.08,.05,.12]);result=self.execute(plan)
        self.assertTrue(result.result.success,result.result.message)

    def test_06_action_cancel_is_canceled_and_holds(self):
        plan=self.plan([.2,-.1,.24,-.12,.18,.32],scale=.12)
        before=dict(self.state);handle=self.execute(plan,wait=False);self.wait_for_motion(before)
        response=self.wait(handle.cancel_goal_async(),5)
        self.assertTrue(response.goals_canceling)
        result=self.wait(handle.get_result_async(),12)
        self.assertEqual(result.status,5,result.result.message)
        self.pump(.2);held=dict(self.state);self.pump(.3)
        for name in JOINTS:self.assertAlmostEqual(held[name],self.state[name],delta=.003)

    def test_07_live_target_edits_and_drag_feedback_do_not_plan_or_move(self):
        from visualization_msgs.msg import InteractiveMarkerFeedback,MarkerArray
        regions=[]
        subscription=self.node.create_subscription(MarkerArray,'/pinchy_motion/working_region',lambda m:regions.append(m),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        try:
            self.pump(.4);self.assertTrue(regions)
            self.assertEqual(len(regions[-1].markers[0].points),24)
        finally:self.node.destroy_subscription(subscription)
        target=self.target([.11,-.11,.21,-.11,.11,.26])
        before=dict(self.state);self.pump(.4);attempts=self.status['ik_attempts']
        edits=[];sub=self.node.create_subscription(PoseStamped,'/pinchy_motion/target_pose',lambda m:edits.append(m),10)
        publisher=self.node.create_publisher(PoseStamped,'/pinchy_motion/target_input',10)
        feedback=self.node.create_publisher(InteractiveMarkerFeedback,'/pinchy_motion/goal/feedback',10)
        try:
            self.pump(.2);publisher.publish(target);self.pump(.5)
            self.assertTrue(edits)
            self.assertAlmostEqual(edits[-1].pose.position.y,target.pose.position.y,places=6)
            update=InteractiveMarkerFeedback();update.header.frame_id='base_link';update.marker_name='pose_goal'
            update.event_type=InteractiveMarkerFeedback.POSE_UPDATE;update.pose=copy.deepcopy(target.pose)
            update.pose.position.y+=.01
            feedback.publish(update);self.pump(.6)
            self.assertAlmostEqual(edits[-1].pose.position.y,update.pose.position.y,places=6)
            invalid=copy.deepcopy(update);invalid.pose.position.y=2.
            feedback.publish(invalid);self.pump(.3)
            self.assertAlmostEqual(edits[-1].pose.position.y,update.pose.position.y,places=6)
            self.assertEqual(self.status['ik_attempts'],attempts)
            for name in JOINTS:self.assertAlmostEqual(before[name],self.state[name],delta=.002)
        finally:
            self.node.destroy_subscription(sub);self.node.destroy_publisher(publisher);self.node.destroy_publisher(feedback)

    def test_07_editing_target_invalidates_old_preview(self):
        plan=self.plan([.08,-.08,.18,-.08,.08,.16]);target=self.target([.1,-.1,.2,-.1,.1,.25])
        publisher=self.node.create_publisher(PoseStamped,'/pinchy_motion/target_input',10)
        try:
            self.pump(.1);publisher.publish(target);self.pump(.4)
            handle=self.wait(self.execute_client.send_goal_async(ExecutePlan.Goal(plan_id=plan.plan_id,hardware_confirmed=self.hardware)),5)
            self.assertFalse(handle.accepted)
        finally:self.node.destroy_publisher(publisher)

    def test_07_pose_panel_plan_preview_execute(self):
        from python_qt_binding.QtWidgets import QApplication
        from rclpy.parameter import Parameter
        from pinchy_motion.panel import PosePanel
        app=QApplication.instance() or QApplication([])
        node=rclpy.create_node('pose_panel_test',context=self.context,
            parameter_overrides=[Parameter('hardware',value=self.hardware)])
        window=PosePanel(node);window.show()
        def until(predicate,timeout=30):
            end=time.monotonic()+timeout
            while not predicate() and time.monotonic()<end:
                app.processEvents();self.executor.spin_once(timeout_sec=0);time.sleep(.01)
            self.assertTrue(predicate(),window.notice.text())
        try:
            until(lambda:window.plan_button.isEnabled())
            window.show_limits();app.processEvents()
            self.assertEqual(window.limit_table.rowCount(),6)
            self.assertIsNotNone(window.limit_table.item(0,2))
            window.limits_window.grab().save(str(self.folder/'joint-limits.png'))
            window.limits_window.hide()
            edited=self.target([.09,-.09,.19,-.09,.09,.24])
            publisher=self.node.create_publisher(PoseStamped,'/pinchy_motion/target_input',10)
            try:
                self.pump(.2);publisher.publish(edited)
                until(lambda:abs(window.inputs[1].value()-edited.pose.position.y)<1e-6)
            finally:self.node.destroy_publisher(publisher)
            target=self.target([.1,-.1,.2,-.1,.1,.25]);p,q=target.pose.position,target.pose.orientation
            rpy=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz',degrees=True)
            for control,value in zip(window.inputs,[p.x,p.y,p.z,*rpy]):control.setValue(float(value))
            self.assertFalse(window.execute_button.isEnabled())
            window.plan();until(lambda:not window.busy)
            self.assertTrue(window.plan_id,window.notice.text())
            window.grab().save(str(self.folder/'pose-panel.png'))
            window.execute();until(lambda:not window.busy)
            self.assertIn('Controller completed',window.notice.text())
            self.assertEqual(window.speed.value(),30.)
            self.assertEqual(window.acceleration.value(),180.)
            self.assertEqual(window.orientation_mode.currentData(),'tool_axis')
            self.assertFalse(window.straight_path.isChecked())
            window.orientation_mode.setCurrentIndex(1);window.straight_path.setChecked(True)
            window.copy_current();app.processEvents();window.plan();until(lambda:not window.busy)
            self.assertTrue(window.plan_id,window.notice.text())
            self.assertIn('tool_axis / cartesian',window.notice.text())
            window.grab().save(str(self.folder/'pose-panel-options.png'))
        finally:
            window.timer.stop();window.executor.shutdown();window.close();app.processEvents();node.destroy_node()

    def test_08_cli_plan_then_execute(self):
        target=self.target([.06,-.09,.16,-.08,.06,.15]);p,q=target.pose.position,target.pose.orientation
        rpy=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz',degrees=True)
        command=['ros2','run','pinchy_motion','pose','plan','--xyz',str(p.x),str(p.y),str(p.z),
                 '--rpy',*[str(float(v)) for v in rpy],'--speed','30','--acceleration','180']
        planned=subprocess.run(command,capture_output=True,text=True,timeout=75)
        self.assertEqual(planned.returncode,0,planned.stdout+planned.stderr)
        summary,_=json.JSONDecoder().raw_decode(planned.stdout[planned.stdout.index('{'):])
        command=['ros2','run','pinchy_motion','pose','execute',summary['plan_id']]
        if self.hardware:command.append('--hardware')
        executed=subprocess.run(command,capture_output=True,text=True,timeout=40)
        self.assertEqual(executed.returncode,0,executed.stdout+executed.stderr)
        position_preview=subprocess.run(['ros2','run','pinchy_motion','pose','plan','--xyz',str(p.x),str(p.y),str(p.z),
            '--orientation','position','--path','cartesian'],capture_output=True,text=True,timeout=90)
        self.assertEqual(position_preview.returncode,0,position_preview.stdout+position_preview.stderr)
        position_summary,_=json.JSONDecoder().raw_decode(position_preview.stdout[position_preview.stdout.index('{'):])
        self.assertEqual(position_summary['orientation_mode'],'position');self.assertEqual(position_summary['path_mode'],'cartesian')
        self.assertIn('Controller completed',executed.stdout)


    def test_99_model_change_blocks_stale_planning_without_motor_motion(self):
        from rcl_interfaces.srv import GetParameters
        parameters=self.service(GetParameters,'/move_group/get_parameters',GetParameters.Request(names=['robot_description']))
        root=ET.fromstring(parameters.values[0].string_value);root.set('name','changed_live_model')
        publisher=self.node.create_publisher(String,'/robot_description',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        before=dict(self.state)
        try:
            publisher.publish(String(data=ET.tostring(root,encoding='unicode')))
            self.pump(.6)
            self.assertFalse(self.status['ready'])
            self.assertIn('Live model changed',self.status['reason'])
            request=PlanPose.Request(target=self.target([.05,-.08,.15,-.08,.05,.12]))
            result=self.wait(self.plan_client.call_async(request),5)
            self.assertFalse(result.success);self.assertIn('restart the planning layer',result.message)
            for name in JOINTS:self.assertAlmostEqual(before[name],self.state[name],delta=.002)
        finally:self.node.destroy_publisher(publisher)



class MockCartesian(CartesianFixture,unittest.TestCase):
    def test_01f_reported_direction_with_nominal_cad_ranges(self):
        target=PoseStamped();target.header.frame_id='base_link'
        target.pose.position.x,target.pose.position.y,target.pose.position.z=.329988,.124285,.155302
        q=Rotation.from_euler('xyz',[-113.168,-86.871,-108.553],degrees=True).as_quat()
        target.pose.orientation=Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
        response=self.wait(self.plan_client.call_async(PlanPose.Request(target=target,orientation_mode='tool_axis')),90)
        self.assertTrue(response.success,response.message)
        a=response.resolved_target.pose.orientation
        np.testing.assert_allclose(Rotation.from_quat([a.x,a.y,a.z,a.w]).as_matrix()[:,2],
            Rotation.from_quat(q).as_matrix()[:,2],atol=.002)
        final=response.trajectory.joint_trajectory.points[-1].positions
        (self.folder/'reported_direction_cad_solution.json').write_text(json.dumps(dict(message=response.message,q_rad=list(final),q_deg=np.degrees(final).tolist()),indent=2))



class HardwareCartesian(CartesianFixture,unittest.TestCase):
    hardware=True
    def test_00_feedback_below_zero_with_zero_hold_can_plan_and_execute_xyz(self):
        if self.status['joint_limits_deg']['wrist_roll']['minimum']!=0:
            self.skipTest('Run with PINCHY_TEST_SAVED_LIMITS=1 to reproduce the physical zero-boundary case')
        self.bus.feedback_offset(5,-1)
        try:
            self.pump(.3);self.assertLess(self.state['wrist_roll'],0.)
            target=PoseStamped();target.header.frame_id='base_link'
            target.pose.position.x,target.pose.position.y,target.pose.position.z=.404189,-.000513,.245081
            rotation=Rotation.from_euler('xyz',[-110.137,-86.936,-69.792],degrees=True).as_quat()
            target.pose.orientation=Quaternion(x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3])
            plan=self.wait(self.plan_client.call_async(PlanPose.Request(target=target,orientation_mode='position')),90)
            self.assertTrue(plan.success,plan.message);self.assertIn('position / joint',plan.message)
            index=JOINTS.index('wrist_roll')
            self.assertTrue(all(point.positions[index]>=-1e-9 for point in plan.trajectory.joint_trajectory.points))
            result=self.execute(plan);self.assertTrue(result.result.success,result.result.message)
            self.assertEqual(self.status.get('safety'),'READY')
        finally:self.bus.feedback_offset(5,0)

    def test_01f_reported_direction_with_saved_calibration(self):
        if self.status['joint_limits_deg']['wrist_roll']['minimum']!=0:
            self.skipTest('Set PINCHY_TEST_SAVED_LIMITS=1 for the saved-profile comparison')
        target=PoseStamped();target.header.frame_id='base_link'
        target.pose.position.x,target.pose.position.y,target.pose.position.z=.329988,.124285,.155302
        q=Rotation.from_euler('xyz',[-113.168,-86.871,-108.553],degrees=True).as_quat()
        target.pose.orientation=Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
        before=dict(self.state)
        response=self.wait(self.plan_client.call_async(PlanPose.Request(target=target,orientation_mode='tool_axis')),90)
        # This profile is narrower than CAD. Record the actual outcome; if a
        # better algorithm recovers a safe solution this is welcome, not a failure.
        (self.folder/'reported_direction_saved_result.json').write_text(json.dumps(dict(success=response.success,message=response.message),indent=2))
        if not response.success:
            self.assertFalse(response.plan_id)
            self.assertTrue('pen direction' in response.message or 'pen-direction' in response.message,response.message)
            self.pump(.4)
            if self.status.get('ik_report',{}).get('range_review_only'):
                self.assertIn('NOT an executable plan',response.message)
                self.assertTrue(any(item['candidate_deg']<item['minimum_deg'] or item['candidate_deg']>item['maximum_deg'] for item in self.status['ik_report']['range_review_only']))
        self.pump(.2)
        for n in JOINTS:self.assertAlmostEqual(before[n],self.state[n],delta=.002)

    def test_06_temperature_hold_recovery_preserves_power(self):
        plan=self.plan([.2,-.1,.25,-.12,.2,.3],scale=.12)
        before=dict(self.state);handle=self.execute(plan,wait=False);self.wait_for_motion(before)
        self.bus.fault(3,63,70)
        try:
            result=self.wait(handle.get_result_async(),10)
            self.assertFalse(result.result.success)
            self.pump(.3);self.assertTrue(self.status['safety'].startswith('HOLD:'))
            self.bus.fault(3,63,33);self.pump(.4)
            recovered=self.service(Trigger,'/pinchy_motion/stop',Trigger.Request()) # Same explicit recovery button behavior as sliders.
            self.assertTrue(recovered.success,recovered.message)
            self.pump(.3);self.assertEqual(self.status['safety'],'READY')
            plan=self.plan([.05,-.08,.15,-.08,.05,.12]);result=self.execute(plan)
            self.assertTrue(result.result.success,result.result.message)
        finally:self.bus.fault(3,63,33)


if __name__=='__main__':unittest.main(verbosity=2)
