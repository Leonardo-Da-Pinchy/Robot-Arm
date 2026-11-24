#!/usr/bin/python3
"""Execute nominal surface point sets on an isolated mock or PTY emulator only.

Never attaches to existing bringup or accepts a serial port. Commanded controller
reference speed qualifies software timing; encoder feedback is not physical
metrology. Artifacts include the full controller trace and independent assays.
"""
import argparse,copy,hashlib,json,os,time,unittest
from pathlib import Path
import numpy as np
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
from rclpy.qos import QoSProfile,DurabilityPolicy
from pinchy_motion.ik import BoundedIK
from pinchy_motion.geometry import model_limits,seconds
from pinchy_motion.nominal_paths import surface_measurement
from pinchy_motion import point_sets
from pinchy_motion_interfaces.srv import PlanPath
from pinchy_bringup.control_contract import JOINTS,MAX_SPEED
from test_cartesian import CartesianFixture,ROOT
from qualify_nominal_paths import request_for,trajectory_metrics


class Runner(CartesianFixture,unittest.TestCase):
    surface="cylinder"
    point_file=None
    retract=False
    def qualify(self):
        models=[];records=[]
        sub=self.node.create_subscription(String,'/robot_description',lambda m:models.append(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        def state(message):
            if len(message.reference.positions)!=6 or len(message.reference.velocities)!=6:return
            indices=[message.joint_names.index(n) for n in JOINTS]
            records.append(dict(wall=time.monotonic(),t=seconds(message.reference),
                reference=[message.reference.positions[i] for i in indices],
                velocity=[message.reference.velocities[i] for i in indices],
                feedback=[message.feedback.positions[i] for i in indices]))
        controller=self.node.create_subscription(JointTrajectoryControllerState,'/arm_controller/controller_state',state,50)
        try:
            self.pump(.5);self.assertTrue(models,'No shared robot model')
            limits=model_limits(models[-1]);solver=BoundedIK(models[-1],limits)
            point_file=self.point_file or ROOT/'src/pinchy_motion/examples/point_sets'/dict(cylinder='04_cylinder_continuous_2in.json',plane='05_plane_continuous_2in.json',quarter_pipe='06_quarter_pipe_continuous_2in.json')[self.surface]
            data=point_sets.read(point_file)
            self.assertEqual(data['source']['surface'],self.surface)
            print('Planning complete',self.surface,'+ approach',flush=True);began=time.monotonic()
            plan=self.service(PlanPath,'/pinchy_motion/plan_path',request_for(data,limits),seconds=480)
            self.assertTrue(plan.success,plan.message)
            metrics=trajectory_metrics(plan,solver,data);del metrics['samples']
            print('Plan ready after',round(time.monotonic()-began),'s; trajectory',round(plan.duration,2),'s',flush=True)
            records.clear();execute_requested=time.monotonic();handle=self.execute(plan,wait=False);future=handle.get_result_async();deadline=time.monotonic()+plan.duration+180
            last_update=0
            while not future.done() and time.monotonic()<deadline:
                self.pump(.1)
                if time.monotonic()-last_update>15:
                    print('Execution:',self.status.get('stage'),'samples',len(records),flush=True);last_update=time.monotonic()
            self.assertTrue(future.done(),'Execution timeout')
            result=future.result();self.assertTrue(result.result.success,result.result.message)
            # time_from_start belongs to the controller's actual sampled reference.
            # Remove pre-dispatch/final holds, whose reference clock resets to zero.
            cruise=[r for r in records if plan.cruise_start_time+.1<r['t']<plan.cruise_end_time-.1]
            self.assertGreater(len(cruise),500,'Insufficient live controller cruise samples')
            speeds=[];angles=[];gaps=[]
            center=np.array(data['source']['center_m'])
            for row in cruise:
                transform,jp,_=solver.evaluate(row['reference']);p=transform[:3,3]
                speeds.append(float(np.linalg.norm(jp@row['velocity'])))
                expected,gap=surface_measurement(self.surface,center,p)
                angles.append(float(np.arccos(np.clip(transform[:3,2]@expected,-1,1))))
                gaps.append(gap)
            speed=data['tcp_speed_m_s'];minimum=data.get('tcp_min_speed_m_s',0.)
            if minimum:
                self.assertGreaterEqual(min(speeds),minimum-1e-7);self.assertLessEqual(max(speeds),speed+1e-7)
            else:self.assertLess(max(abs(np.array(speeds)-speed)),speed*.05)
            peak_tangential_acceleration=None
            if minimum:
                reference_times=np.array([r['t'] for r in cruise]);dt=np.diff(reference_times);valid_dt=dt>.005
                peak_tangential_acceleration=float(np.max(abs(np.diff(speeds)[valid_dt]/dt[valid_dt])))
                self.assertLessEqual(peak_tangential_acceleration,data['tcp_acceleration_m_s2']*1.05+1e-6)
            self.assertLess(max(angles),.01);self.assertLess(max(abs(np.array(gaps)-data['source']['stand_off_m'])),.001)
            # Every one of the 39 interior input knots must be traversed at cruise.
            from pinchy_motion.paths import CartesianCurve,SpeedLaw
            from pinchy_motion.feasibility import axis_frame
            from scipy.spatial.transform import Rotation
            curve=CartesianCurve([axis_frame(p['xyz'],Rotation.from_euler('xyz',p['rpy_deg'],degrees=True).as_matrix()[:,2]) for p in data['waypoints']],data['style'])
            law=SpeedLaw(curve.length,speed,data['tcp_acceleration_m_s2']);times=np.array([r['t'] for r in cruise])
            waypoint_speeds=[]
            cruise_points=np.array([solver.evaluate(r['reference'])[0][:3,3] for r in cruise]) if minimum else None
            for distance in curve.waypoint_distances[1:-1]:
                if minimum:
                    target=curve.evaluate(distance,'tool_axis')[:3,3]
                    points=cruise_points
                    index=int(np.argmin(np.linalg.norm(points-target,axis=1)))
                    self.assertLess(np.linalg.norm(points[index]-target),.001,'Missing live traversal of interior waypoint')
                else:
                    target=plan.trace_start_time+law.time_at(distance);index=int(np.argmin(abs(times-target)))
                    self.assertLess(abs(times[index]-target),.15,'Missing live sample at an interior waypoint')
                waypoint_speeds.append(speeds[index])
            # Feedback covers the complete air loop; reported as encoder/model
            # geometry only, with no assertion of physical speed or nib accuracy.
            measured=np.array([solver.evaluate(r['feedback'])[0][:3,3] for r in cruise])
            expected_span=np.ptp(np.array([p['xyz'] for p in data['waypoints']]),axis=0)
            for axis in (1,2):
                if expected_span[axis]>.001:self.assertGreater(float(np.ptp(measured[:,axis])),expected_span[axis]*.85)
            self.node.destroy_subscription(controller);controller=None
            retract_report=None
            if self.retract:
                from scipy.spatial.transform import Rotation
                end=solver.evaluate(plan.trajectory.joint_trajectory.points[-1].positions)[0]
                lift_point=end[:3,3]-end[:3,2]*.010
                lift=copy.deepcopy(data);lift.update(style='polyline',plan_pen_up_approach=False,waypoints=[dict(xyz=lift_point.tolist(),rpy_deg=Rotation.from_matrix(end[:3,:3]).as_euler('xyz',degrees=True).tolist(),contact=False)])
                lift_plan=self.service(PlanPath,'/pinchy_motion/plan_path',request_for(lift,limits),seconds=180)
                self.assertTrue(lift_plan.success,lift_plan.message)
                lift_result=self.execute(lift_plan);self.assertTrue(lift_result.result.success,lift_result.result.message)
                retract_report=dict(success=True,distance_m=.010,duration_s=lift_plan.duration,message=lift_result.result.message)
            # Stop after completion still holds, and consumed plans cannot replay.
            from std_srvs.srv import Trigger
            stop=self.service(Trigger,'/pinchy_motion/stop',Trigger.Request());self.assertTrue(stop.success,stop.message)
            from pinchy_motion_interfaces.action import ExecutePlan
            replay=self.wait(self.execute_client.send_goal_async(ExecutePlan.Goal(plan_id=plan.plan_id,hardware_confirmed=self.hardware)),5)
            self.assertFalse(replay.accepted,'Consumed trajectory replay was accepted')
            report=dict(success=True,surface=self.surface,retract=retract_report,hardware='PTY emulator' if self.hardware else 'mock',physical_motors_moved=False,
                point_set_file=str(point_file),point_set_sha256=hashlib.sha256(point_file.read_bytes()).hexdigest(),
                execute_to_first_moving_reference_s=next((r['wall']-execute_requested for r in records if r['wall']>=execute_requested and max(abs(v) for v in r['velocity'])>1e-4),None),
                requested_tcp_speed_m_s=speed,requested_min_tcp_speed_m_s=minimum,live_reference_peak_tangential_acceleration_m_s2=peak_tangential_acceleration,range_report=json.loads(plan.speed_range_report_json or '[]'),live_reference_min_m_s=min(speeds),live_reference_max_m_s=max(speeds),
                cruise_samples=len(cruise),interior_waypoints_checked=len(waypoint_speeds),
                minimum_interior_waypoint_speed_m_s=min(waypoint_speeds),max_normal_error_deg=float(np.degrees(max(angles))),
                stand_off_range_m=[min(gaps),max(gaps)],encoder_model_xyz_span_m=np.ptp(measured,axis=0).tolist(),
                endpoint_result=result.result.message,consumed_plan_replay_rejected=True,offline_metrics=metrics)
            (self.folder/'continuous_execution_report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2),flush=True)
        finally:
            (self.folder/'controller_trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
            if controller is not None:self.node.destroy_subscription(controller)
            self.node.destroy_subscription(sub)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--emulated',action='store_true')
    parser.add_argument('--emulated-max-joint-speed',type=float,default=30.,help='Mock servo-profile cap only, deg/s')
    parser.add_argument('--surfaces',nargs='+',choices=['cylinder','plane','quarter_pipe'],default=['cylinder'])
    parser.add_argument('--support',choices=['original','pedestal'],default='original')
    parser.add_argument('--scene',type=Path,help='Explicit collision scene for the isolated test stack')
    parser.add_argument('--points',type=Path,help='Explicit point set for one selected surface')
    parser.add_argument('--retract',action='store_true',help='Also plan and execute a separate 10 mm normal pen-up retract')
    args=parser.parse_args()
    if args.points and len(args.surfaces)!=1:parser.error('Explicit points require one surface')
    if not .1<=args.emulated_max_joint_speed<=np.rad2deg(MAX_SPEED):parser.error('Emulated speed outside supported joint envelope')
    Runner.emulated_max_velocity=float(np.deg2rad(args.emulated_max_joint_speed))
    Runner.hardware=args.emulated;Runner.point_file=args.points;Runner.retract=args.retract
    for surface in args.surfaces:
        Runner.surface=surface;suffix='_pedestal' if args.support=='pedestal' else ''
        os.environ.update(PINCHY_TEST_CALIBRATED_MOCK='1',PINCHY_TEST_SAVED_LIMITS='1',
            PINCHY_TEST_SCENE=str(args.scene.resolve() if args.scene else ROOT/f'src/pinchy_moveit_config/config/nominal_{surface}{suffix}_dry_run.yaml'))
        Runner.setUpClass()
        try:Runner('runTest').qualify()
        finally:Runner.tearDownClass()


if __name__=='__main__':main()
