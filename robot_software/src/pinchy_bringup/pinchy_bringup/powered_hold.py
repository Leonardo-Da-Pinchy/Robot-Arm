"""Nonblocking powered Stop/recovery protocol used by the Cartesian gateway.

The same controller, stamped accepted targets, pause handshake and hold goal
as the commissioning sliders. Never disables torque or bypasses backend health.
"""
import math
import time
from rclpy.action import ActionClient
from rclpy.task import Future
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState, SpeedScalingFactor
from .control_contract import *


class PoweredHold:
    def __init__(self,node,hardware,callback_group=None):
        self.node,self.hardware=node,hardware
        self.commanded=None; self.applied=None; self.applied_stamp=-1
        self.state_stamp=-1; self.last_state=0.; self.paused=False
        self.safety='WAITING' if hardware else 'READY'
        self.phase='idle'; self.future=None; self.goal=None; self.generation=0
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scaling=node.create_publisher(SpeedScalingFactor,SCALING_INPUT,qos)
        self.reset=node.create_client(Trigger,SAFETY_RESET,callback_group=callback_group)
        self.client=ActionClient(node,FollowJointTrajectory,TRAJECTORY_ACTION,callback_group=callback_group)
        node.create_subscription(JointState,APPLIED_TARGETS,self.on_applied,qos,callback_group=callback_group)
        node.create_subscription(String,SAFETY_STATUS,self.on_safety,qos,callback_group=callback_group)
        node.create_subscription(JointTrajectoryControllerState,CONTROLLER_STATE,self.on_state,10,callback_group=callback_group)
        node.create_timer(0.05,self.tick,callback_group=callback_group)

    def on_safety(self,message): self.safety=message.data

    def on_applied(self,message):
        values=dict(zip(message.name,message.position))
        if all(name in values and math.isfinite(values[name]) for name in JOINTS):
            self.applied=[values[name] for name in JOINTS]
            self.applied_stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec

    def on_state(self,message):
        values=dict(zip(message.joint_names,message.output.positions))
        if all(name in values and math.isfinite(values[name]) for name in JOINTS):
            self.commanded=[values[name] for name in JOINTS]
            self.last_state=time.monotonic()
            self.state_stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
            self.paused=message.speed_scaling_factor==0.0

    def request(self,recover=False):
        if self.phase!='idle': return self.future
        self.generation+=1
        self.future=Future(); self.recover=recover
        self.phase='pause'; self.deadline=time.monotonic()+8
        self.request_stamp=self.node.get_clock().now().nanoseconds
        self.scaling.publish(SpeedScalingFactor(factor=0.0))
        return self.future

    def finish(self,success,message):
        self.phase='idle'
        if self.future is not None and not self.future.done(): self.future.set_result((success,message))

    def tick(self):
        if self.phase=='idle': return
        if time.monotonic()>self.deadline:
            self.finish(False,'Hold handshake timed out; controller remains paused'); return
        if self.phase!='pause' or not self.paused or self.state_stamp<self.request_stamp: return
        if self.commanded is None or time.monotonic()-self.last_state>0.5: return
        if self.hardware and (self.applied is None or self.applied_stamp<self.request_stamp): return
        self.target=list(self.applied if self.hardware else self.commanded)
        self.phase='goal'
        generation=self.generation
        self.client.send_goal_async(hold_goal(self.target),
            feedback_callback=lambda message:self.feedback(message,generation)).add_done_callback(
                lambda future:self.accepted(future,generation))

    def accepted(self,future,generation):
        if generation!=self.generation or self.phase!='goal': return
        try:
            self.goal=future.result()
            if not self.goal.accepted: self.finish(False,'Hold goal rejected; controller remains paused'); return
            self.goal.get_result_async().add_done_callback(lambda future:self.result(future,generation))
        except Exception as error: self.finish(False,str(error))

    def feedback(self,message,generation):
        if generation!=self.generation or self.phase!='goal': return
        positions=message.feedback.desired.positions
        if len(positions)!=6 or any(abs(a-b)>1e-8 for a,b in zip(positions,self.target)): return
        if self.recover and self.safety.startswith('HOLD:'):
            self.phase='reset'
            if not self.reset.service_is_ready(): self.finish(False,'Recovery service unavailable; remains paused'); return
            self.reset.call_async(Trigger.Request()).add_done_callback(lambda future:self.reset_done(future,generation))
        elif self.safety.startswith('HOLD:'):
            self.finish(False,'Hardware remains in safety hold; explicit recovery required')
        else:
            self.phase='result'; self.scaling.publish(SpeedScalingFactor(factor=1.0))

    def reset_done(self,future,generation):
        if generation!=self.generation or self.phase!='reset': return
        try:
            response=future.result()
            if not response.success: self.finish(False,response.message); return
            self.safety='READY'; self.phase='result'
            self.scaling.publish(SpeedScalingFactor(factor=1.0))
        except Exception as error: self.finish(False,str(error))

    def result(self,future,generation):
        if generation!=self.generation or self.phase=='idle': return
        try:
            response=future.result()
            self.finish(response.status==4 and response.result.error_code==0,'Stopped — powered hold established')
        except Exception as error: self.finish(False,str(error))
