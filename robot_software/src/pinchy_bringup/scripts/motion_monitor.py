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
        self.sub = self.create_subscription(JointTrajectoryControllerState,
            '/arm_controller/controller_state', self.sample, 10)
        self.get_logger().info(f'Motion diagnostics: {self.path} (30 MiB maximum with rotation)')

    def sample(self, message):
        measured = list(message.feedback.positions)
        commanded = list(message.output.positions)
        if len(commanded) != len(message.joint_names) or len(measured) != len(commanded): return
        if not all(math.isfinite(value) for value in commanded+measured): return
        self.writer.info(json.dumps(dict(
            stamp_ns=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec,
            received_monotonic=time.monotonic(), joints=list(message.joint_names),
            commanded=commanded, measured=measured,
            error_radians=[actual-target for actual, target in zip(measured, commanded)],
            reference_velocity=list(message.reference.velocities),
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
