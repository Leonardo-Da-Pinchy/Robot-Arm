import sys,json,math
from pathlib import Path
import os
os.chdir(Path(__file__).resolve().parents[4])
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
sys.path.insert(0,'tools')
from test_ik import calibrated_solver
from pinchy_motion.point_sets import read
from pinchy_motion.paths import CartesianCurve,SpeedLaw,separated,timed_trace
from pinchy_motion.feasibility import axis_frame
from pinchy_motion.geometry import seconds,set_seconds,segment_coefficients,extrema,model_limits,prepare_trajectory
from pinchy_bringup.control_contract import JOINTS
_,solver=calibrated_solver();data=read('src/pinchy_motion/examples/point_sets/07_cylinder_continuous_3in_fast.json')
curve=CartesianCurve([axis_frame(p['xyz'],Rotation.from_euler('xyz',p['rpy_deg'],degrees=True).as_matrix()[:,2]) for p in data['waypoints']],data['style'])
saved=np.load(str(Path(__file__).with_name('large_branch.npz')));spline=CubicSpline(saved['d'],saved['q'],axis=0)
results=[]
for speed,acc in ((.007,.02),(.012,.08),(.014,.08),(.016,.08),(.018,.08),(.018,.02)):
 law=SpeedLaw(curve.length,speed,acc);times=separated(np.r_[np.arange(0,law.duration,.02),[law.duration,law.ramp_time,law.ramp_time+law.plateau_time],[law.time_at(s) for s in curve.waypoint_distances]],.001);times[-1]=law.duration
 tr=RobotTrajectory();tr.joint_trajectory.joint_names=JOINTS
 for t in times:
  t=round(float(t)*1e9)/1e9;s,v,a=law.at(t);point=JointTrajectoryPoint(positions=spline(s).tolist(),velocities=(spline(s,1)*v).tolist(),accelerations=(spline(s,2)*v*v+spline(s,1)*a).tolist());set_seconds(point,t);tr.joint_trajectory.points.append(point)
 peaks=np.zeros((2,6));when=np.zeros((2,6));phase=np.zeros((2,6));where=np.zeros((2,6,3))
 for first,last in zip(tr.joint_trajectory.points,tr.joint_trajectory.points[1:]):
  dt,c=segment_coefficients(first,last)
  for order in (1,2):
   vals=np.array([max(abs(x)) for x in extrema(c,dt,order)])
   for i in np.where(vals>peaks[order-1])[0]:
    peaks[order-1,i]=vals[i];when[order-1,i]=seconds(first)
    distance=law.at(seconds(first))[0];p=curve.evaluate(distance,'tool_axis')[:3,3];u=.04*np.arctan2(p[1]-.1,.4-p[0]);phase[order-1,i]=float(np.rad2deg(np.arctan2((p[2]-.15)/.0381,(u+.04*np.pi/12)/.0381))%360);where[order-1,i]=p
 limits={n:dict(lower=float(l),upper=float(h),velocity=math.radians(60),acceleration=math.radians(360)) for n,l,h in zip(JOINTS,solver.lower,solver.upper)}
 try:prepare_trajectory(tr,limits,rescale=False);outcome='pass'
 except ValueError as e:outcome=str(e)
 row=dict(tcp_speed_mm_s=speed*1000,tcp_acceleration_mm_s2=acc*1000,outcome=outcome,joints={n:dict(peak_velocity_deg_s=float(np.rad2deg(peaks[0,i])),peak_acceleration_deg_s2=float(np.rad2deg(peaks[1,i])),velocity_at_s=float(when[0,i]),acceleration_at_s=float(when[1,i]),velocity_phase_deg=float(phase[0,i]),acceleration_phase_deg=float(phase[1,i])) for i,n in enumerate(JOINTS)})
 results.append(row);print(speed,acc,outcome,row['joints']['wrist_roll'],row['joints']['wrist_flex'],flush=True)
Path(__file__).with_name('rates.json').write_text(json.dumps(results,indent=2))
