#!/usr/bin/env python3
"""Send bounded joint waypoints to the mock bench FollowJointTrajectory action."""

import argparse
import math
import time
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


def wait(node, future, seconds):
    deadline = time.monotonic() + seconds
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
    if not future.done():
        raise TimeoutError('Timed out waiting for ROS response')
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--point', type=float, nargs=6, action='append', required=True,
                        metavar='RAD', help='Six radians: shoulder pan/lift, elbow, wrist flex/roll, tool rotation')
    parser.add_argument('--seconds', type=float, default=3.0,
                        help='Seconds per waypoint (default: 3)')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Execution timeout in seconds; cancels on expiry (default: 30)')
    parser.add_argument('--cancel-after', type=float,
                        help='Request cancellation this many seconds after acceptance')
    args = parser.parse_args()
    for name, value in [('seconds', args.seconds), ('timeout', args.timeout),
                        ('cancel-after', args.cancel_after)]:
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error(f'--{name} must be finite and positive')
    limits = [(-1, 1)] * 5 + [(0, 1)]
    if any(not math.isfinite(p) or not low <= p <= high
           for point in args.point for p, (low, high) in zip(point, limits)):
        parser.error('Mock demo range: first five joints [-1, 1], tool rotation [0, 1] radians')

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('mock_trajectory_client')
    client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    goal_handle = None
    result_future = None
    exit_code = 1
    try:
        # Refuse accidental use against the later physical-arm description.
        model_future = rclpy.task.Future()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

        def on_description(message):
            if not model_future.done():
                model_future.set_result(message.data)

        subscription = node.create_subscription(String, '/robot_description', on_description, qos)
        model = ET.fromstring(wait(node, model_future, 10.0))
        plugins = [item.text for item in model.findall('./ros2_control/hardware/plugin')]
        if model.get('name') != 'pinchy_mock' or plugins != ['mock_components/GenericSystem']:
            raise RuntimeError('Expected pinchy_mock with GenericSystem mock hardware')
        node.destroy_subscription(subscription)
        if not client.wait_for_server(timeout_sec=10.0):
            raise TimeoutError('Mock trajectory action unavailable; check controller startup')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
                                       'wrist_flex', 'wrist_roll', 'pen_pitch']
        for index, positions in enumerate(args.point, start=1):
            point = JointTrajectoryPoint()
            point.positions = positions
            nanoseconds = round(index * args.seconds * 1_000_000_000)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 1_000_000_000)
            goal.trajectory.points.append(point)

        last_feedback = 0.0

        def feedback(message):
            nonlocal last_feedback
            if time.monotonic() - last_feedback >= 1.0:
                last_feedback = time.monotonic()
                node.get_logger().info(f'Actual radians: {list(message.feedback.actual.positions)}')

        # Resolve acceptance before processing Ctrl+C to obtain a cancellable goal handle.
        import signal
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            goal_handle = wait(node, client.send_goal_async(goal, feedback_callback=feedback), 10.0)
        finally:
            signal.signal(signal.SIGINT, previous)
        if not goal_handle.accepted:
            raise RuntimeError('Trajectory rejected')
        node.get_logger().info('Trajectory accepted')
        result_future = goal_handle.get_result_async()
        started = time.monotonic()
        requested_cancel = False
        while not result_future.done():
            elapsed = time.monotonic() - started
            if elapsed >= args.timeout:
                raise TimeoutError('Execution timeout; requesting cancellation')
            if args.cancel_after is not None and elapsed >= args.cancel_after and not requested_cancel:
                response = wait(node, goal_handle.cancel_goal_async(), 5.0)
                if not response.goals_canceling:
                    raise RuntimeError('Controller did not accept cancellation')
                requested_cancel = True
                node.get_logger().info('Cancellation accepted')
            rclpy.spin_once(node, timeout_sec=0.1)
        result = result_future.result()
        node.get_logger().info(f'Final action status={result.status}, '
                               f'error_code={result.result.error_code}: {result.result.error_string}')
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code == 0:
            exit_code = 0
        elif requested_cancel and result.status == GoalStatus.STATUS_CANCELED:
            exit_code = 0
    except (RuntimeError, TimeoutError, KeyboardInterrupt, ET.ParseError) as error:
        node.get_logger().error(str(error) or 'Interrupted; requesting cancellation')
    finally:
        if goal_handle is not None and goal_handle.accepted and result_future is not None and not result_future.done():
            try:
                response = wait(node, goal_handle.cancel_goal_async(), 5.0)
                if not response.goals_canceling:
                    node.get_logger().warning('Cancellation not acknowledged')
                result = wait(node, result_future, 5.0)
                node.get_logger().info(f'Cleanup action status={result.status}')
            except (RuntimeError, TimeoutError, KeyboardInterrupt) as error:
                node.get_logger().error(f'Could not confirm stop: {error}')
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
