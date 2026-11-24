#!/usr/bin/env python3
"""Record controller output and measured motion outside the hardware read loop."""
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import time

from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from pinchy_bringup.control_contract import APPLIED_TARGETS,SERVO_GOALS,SAFETY_STATUS
import rclpy
from rclpy.node import Node


class MotionMonitor(Node):
    def __init__(self):
        super().__init__('pinchy_motion_monitor')
        directory = Path(os.environ.get('ROS_LOG_DIR', '/tmp'))
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory/f'pinchy-motion-{os.getpid()}.jsonl'
        self.writer = logging.Logger('motion_file')
        self.handler = RotatingFileHandler(self.path, maxBytes=10*1024*1024, backupCount=2)
        self.handler.setFormatter(logging.Formatter('%(message)s'))
        self.writer.addHandler(self.handler)
        self.applied=None;self.servo_goals=None;self.safety='UNKNOWN'
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointState,APPLIED_TARGETS,lambda m:self.target_snapshot('applied',m),qos)
        self.create_subscription(JointState,SERVO_GOALS,lambda m:self.target_snapshot('servo_goals',m),qos_profile_sensor_data)
        self.create_subscription(String,SAFETY_STATUS,lambda m:setattr(self,'safety',m.data),qos)
        self.sub = self.create_subscription(JointTrajectoryControllerState,
            '/arm_controller/controller_state', self.sample, 10)
        self.get_logger().info(f'Motion diagnostics: {self.path} (30 MiB maximum with rotation)')

    def target_snapshot(self,field,message):
        names=list(message.name);positions=list(message.position)
        if len(names)!=len(positions) or len(set(names))!=len(names) or not all(math.isfinite(v) for v in positions):return
        stamp=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        previous=getattr(self,field)
        if previous is not None and stamp<previous['stamp_ns']:return
        setattr(self,field,dict(values=dict(zip(names,positions)),stamp_ns=stamp,received=time.monotonic()))

    def aligned_snapshot(self,field,names,now):
        snapshot=getattr(self,field)
        if snapshot is None or any(n not in snapshot['values'] for n in names):return None
        return dict(positions=[snapshot['values'][n] for n in names],stamp_ns=snapshot['stamp_ns'],
                    age_s=now-snapshot['received'])

    def sample(self, message):
        measured = list(message.feedback.positions)
        commanded = list(message.output.positions)
        if len(commanded) != len(message.joint_names) or len(measured) != len(commanded): return
        if not all(math.isfinite(value) for value in commanded+measured): return
        now=time.monotonic()
        self.writer.info(json.dumps(dict(
            stamp_ns=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec,
            received_monotonic=now, joints=list(message.joint_names),
            commanded=commanded, measured=measured,
            error_radians=[actual-target for actual, target in zip(measured, commanded)],
            reference_positions=list(message.reference.positions),
            reference_velocity=list(message.reference.velocities),
            adapter_applied=self.aligned_snapshot('applied',message.joint_names,now),
            servo_goals=self.aligned_snapshot('servo_goals',message.joint_names,now),
            safety=self.safety,
            scaling=message.speed_scaling_factor), allow_nan=False))

    def destroy_node(self):
        self.handler.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = MotionMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
