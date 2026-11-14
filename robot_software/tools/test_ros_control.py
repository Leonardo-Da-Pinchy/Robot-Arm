#!/usr/bin/env python3
"""ROS integration tests: mock hardware and the real plugin on an emulated PTY.

Source ROS and the built workspace first. Uses an isolated ROS_DOMAIN_ID.
No physical servo device is opened. Logs stay in /tmp/pinchy-ros-tests-*.
"""
import importlib.util
import math
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = str(80 + os.getpid() % 100)
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
          'wrist_flex', 'wrist_roll', 'pen_pitch']
ROOT = Path(__file__).resolve().parents[1]


class SerialEmulator:
    def __init__(self, folder):
        import json
        self.ready = folder / 'emulator_ready.json'
        self.report = folder / 'emulator_report.json'
        self.process = subprocess.Popen([sys.executable, str(ROOT / 'tools/servo_bus_emulator.py'),
            str(self.ready), str(self.report)], start_new_session=True)
        deadline = time.monotonic() + 10
        while not self.ready.exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        self.path = json.loads(self.ready.read_text())['port']

    def close(self):
        import json
        self.process.send_signal(signal.SIGINT)
        self.process.wait(timeout=10)
        report = json.loads(self.report.read_text())
        assert all(value == 0 for value in report['torque'].values()), report
        assert report['eeprom_writes'] == 0, report

    def fault(self, servo, address, value):
        import json
        temporary = self.ready.parent / 'fault.tmp'
        temporary.write_text(json.dumps(dict(servo=servo, address=address, value=value)))
        temporary.replace(self.ready.parent / 'fault.json')

    def feedback_offset(self, servo, ticks):
        import json
        temporary = self.ready.parent / 'fault.tmp'
        temporary.write_text(json.dumps(dict(servo=servo, feedback_offset=ticks)))
        temporary.replace(self.ready.parent / 'fault.json')
        time.sleep(0.15)


class MotionProfileTests(unittest.TestCase):
    def test_selected_speed_and_acceleration_bound_sampled_quintic_motion(self):
        spec = importlib.util.spec_from_file_location('profile_sliders', ROOT/'src/pinchy_bringup/scripts/joint_sliders.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        for distance, speed, acceleration in [(0.5, 0.1, 0.2), (0.01, 0.1, 0.01),
                (0.5, math.radians(30), math.radians(180)), (0.0, 0.05, 0.1)]:
            with self.subTest(distance=distance):
                duration = module.motion_duration([distance], [speed], acceleration)
                positions = [distance*(10*(i/1000)**3-15*(i/1000)**4+6*(i/1000)**5) for i in range(1001)]
                dt = duration/1000
                velocities = [(b-a)/dt for a, b in zip(positions, positions[1:])]
                accelerations = [(b-a)/dt for a, b in zip(velocities, velocities[1:])]
                self.assertLessEqual(max(map(abs, velocities)), speed)
                self.assertLessEqual(max(map(abs, accelerations)), acceleration)
                self.assertLess(abs(velocities[0]), 1e-4)
                self.assertLess(abs(velocities[-1]), 1e-4)


class Pipeline:
    hardware = False
    bench = False
    gui_target = 0.12
    hardware_velocity = 0.1

    @classmethod
    def setUpClass(cls):
        cls.folder = Path(tempfile.mkdtemp(prefix='pinchy-ros-tests-'))
        print(f'\n{cls.__name__} artifacts: {cls.folder}', flush=True)
        domain = 80 + (os.getpid() + int(cls.hardware) + int(cls.bench)) % 100
        os.environ['ROS_DOMAIN_ID'] = str(domain)
        cls.context = Context()
        rclpy.init(context=cls.context, domain_id=domain)
        cls.bus = SerialEmulator(cls.folder) if cls.hardware else None
        command = ['ros2', 'launch', 'pinchy_bringup',
                   'hardware_control.launch.py' if cls.hardware else 'mock_control.launch.py']
        if cls.bus:
            path = cls.folder / 'emulated_calibration.yaml'
            if cls.bench:
                capture = subprocess.run([str(ROOT / '.venv/bin/python'), str(ROOT / 'tools/prepare_ros_bench.py'),
                    '--hardware', '--port', cls.bus.path, '--output', str(path)],
                    capture_output=True, text=True, timeout=15)
                if capture.returncode:
                    cls.bus.close()
                    raise RuntimeError(capture.stdout + capture.stderr)
                command += ['unloaded_bench:=true']
            else:
                config = yaml.safe_load((ROOT / 'src/pinchy_bringup/config/servo_calibration.example.yaml').read_text())
                config.update(calibration_confirmed=True, port=cls.bus.path)
                config.update(speed=500, acceleration=32)
                for joint in config['joints'].values(): joint['max_velocity'] = cls.hardware_velocity
                path.write_text(yaml.safe_dump(config))
            command += [f'calibration:={path}', 'motion_enabled:=true']
        cls.log = (cls.folder / 'launch.log').open('w')
        env = dict(os.environ, ROS_LOG_DIR=str(cls.folder / 'ros_logs'))
        cls.process = subprocess.Popen(command, stdout=cls.log, stderr=subprocess.STDOUT,
                                       env=env, start_new_session=True)
        cls.node = rclpy.create_node(cls.__name__.lower(), context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)
        cls.actual = {}

        def on_state(message):
            cls.actual = dict(zip(message.name, message.position))

        cls.sub = cls.node.create_subscription(JointState, '/joint_states', on_state, 10)
        cls.client = ActionClient(cls.node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        controller_client = cls.node.create_client(ListControllers, '/controller_manager/list_controllers')
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            cls.executor.spin_once(timeout_sec=0.1)
            if cls.process.poll() is not None: break
            if cls.client.server_is_ready() and all(j in cls.actual for j in JOINTS) and controller_client.service_is_ready():
                future = controller_client.call_async(ListControllers.Request())
                cls.executor.spin_until_future_complete(future, timeout_sec=1)
                if future.done() and all(c.state == 'active' for c in future.result().controller) and len(future.result().controller) == 2:
                    cls.node.destroy_client(controller_client)
                    return
        cls.tearDownClass()
        raise RuntimeError(f'Controller startup failed: {cls.folder / "launch.log"}')

    @classmethod
    def tearDownClass(cls):
        if cls.process.poll() is None:
            os.killpg(cls.process.pid, signal.SIGINT)
            try: cls.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(cls.process.pid, signal.SIGKILL); cls.process.wait()
        cls.log.close(); cls.client.destroy(); cls.executor.shutdown(); cls.node.destroy_node()
        if cls.bus:
            cls.bus.close()
        cls.context.shutdown()

    def wait(self, future, seconds=10):
        self.executor.spin_until_future_complete(future, timeout_sec=seconds)
        self.assertTrue(future.done(), 'ROS response timed out')
        return future.result()

    def assert_bench_client_refuses_regular_robot(self):
        import json
        path = self.folder / 'refused-bench-test.jsonl'
        result = subprocess.run(['ros2', 'run', 'pinchy_bringup', 'ros_bench_test.py',
            '--log', str(path)], capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn('Requires the explicit physical unloaded-bench launch', result.stderr)
        self.assertTrue(all(json.loads(line)['event'] != 'command' for line in path.read_text().splitlines()))

    def goal(self, points, seconds=1.5, names=JOINTS):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = names
        for i, values in enumerate(points, 1):
            point = JointTrajectoryPoint(); point.positions = values
            ns = round(i * seconds * 1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 1_000_000_000)
            goal.trajectory.points.append(point)
        return self.wait(self.client.send_goal_async(goal))

    def test_00_active_pointer_tcp_is_published(self):
        from tf2_ros import Buffer, TransformListener
        from rclpy.time import Time
        buffer = Buffer()
        listener = TransformListener(buffer, self.node)
        try:
            deadline = time.monotonic()+5
            while time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=0.05)
                if buffer.can_transform('base_link', 'pointer_tip', Time()): break
            self.assertTrue(buffer.can_transform('base_link', 'pointer_tip', Time()))
            local = buffer.lookup_transform('pen_holder_link', 'pointer_tip', Time()).transform
            for actual, expected in zip((local.translation.x, local.translation.y, local.translation.z), (0, -0.149, 0.0188)):
                self.assertAlmostEqual(actual, expected, places=12)
            self.assertAlmostEqual(local.rotation.x, math.sqrt(0.5), places=12)
            self.assertAlmostEqual(local.rotation.w, math.sqrt(0.5), places=12)
            point = buffer.lookup_transform('base_link', 'pointer_tip', Time()).transform
            nominal = buffer.lookup_transform('base_link', 'pen_tip', Time()).transform
            self.assertAlmostEqual(point.translation.x, nominal.translation.x, places=12)
            self.assertAlmostEqual(point.translation.y, nominal.translation.y, places=12)
            self.assertAlmostEqual(point.translation.z, nominal.translation.z, places=12)
        finally:
            listener.unregister()

    def test_01_waypoints_feedback(self):
        target = [0.1, -0.1, 0.05, -0.05, 0.1, 0.1]
        handle = self.goal([target, [0.0] * 6])
        self.assertTrue(handle.accepted)
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(result.result.error_code, 0)
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        for name in JOINTS: self.assertAlmostEqual(self.actual[name], 0, delta=0.01)

    def test_02_cancel(self):
        handle = self.goal([[0.2] * 6], seconds=5)
        self.assertTrue(handle.accepted)
        for _ in range(5): self.executor.spin_once(timeout_sec=0.05)
        response = self.wait(handle.cancel_goal_async())
        self.assertTrue(response.goals_canceling)
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)

    def test_03_reject_incorrect_joint_set(self):
        handle = self.goal([[0.0] * 6], names=['unknown'] + JOINTS[1:])
        self.assertFalse(handle.accepted)

    def test_04_sliders_send_feedback(self):
        from python_qt_binding.QtWidgets import QApplication
        app = QApplication.instance() or QApplication([])
        spec = importlib.util.spec_from_file_location('joint_sliders', ROOT / 'src/pinchy_bringup/scripts/joint_sliders.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        gui_node = rclpy.create_node('slider_integration_test', context=self.context)
        window = module.Sliders(gui_node, self.hardware)
        try:
            deadline = time.monotonic() + 10
            while not window.send_button.isEnabled() and time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertTrue(window.send_button.isEnabled())
            # Both slider and numeric input clamp inward to every URDF bound.
            for name, spin, slider in zip(JOINTS, window.inputs, window.sliders):
                limit = window.description.find(f"joint[@name='{name}']/limit")
                low, high = float(limit.get('lower')), float(limit.get('upper'))
                spin.setValue(high + 10)
                self.assertLessEqual(spin.value(), high)
                self.assertLessEqual(slider.value()/1000, high)
                spin.setValue(low - 10)
                self.assertGreaterEqual(spin.value(), low)
                self.assertGreaterEqual(slider.value()/1000, low)
            window.copy_actual()
            window.inputs[0].setValue(self.gui_target); window.speed.setValue(5); window.send()
            deadline = time.monotonic() + 10
            while window.pending and time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertFalse(window.pending); self.assertEqual(window.status.text(), 'Reached target')
            deadline = time.monotonic() + 2
            while abs(window.actual['shoulder_pan'] - self.gui_target) >= 0.01 and time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertAlmostEqual(window.actual['shoulder_pan'], self.gui_target, delta=0.01)
            # Stop requested before goal acceptance must not be lost.
            window.inputs[0].setValue(0.25); window.speed.setValue(3)
            window.send(); window.cancel()
            deadline = time.monotonic() + 7
            while window.pending and time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertFalse(window.pending)
            self.assertEqual(window.status.text(), 'Stopped — holding commanded position')
        finally:
            window.timer.stop(); window.client.destroy(); window.executor.shutdown(); window.deleteLater(); app.processEvents(); gui_node.destroy_node()


class MockPipeline(Pipeline, unittest.TestCase):
    def test_09_guided_startup_refuses_existing_robot_graph(self):
        import json
        from test_start_arm import Bus, profile
        path = self.folder / 'startup-calibration.json'
        config = profile(Bus()); config['port'] = '/this-port-must-never-be-opened'
        path.write_text(json.dumps(config))
        result = subprocess.run([str(ROOT/'.venv/bin/python'), str(ROOT/'tools/start_arm.py'),
            '--hardware', '--calibration', str(path), '--domain', os.environ['ROS_DOMAIN_ID'],
            '--no-gui'], capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 1, result.stdout+result.stderr)
        self.assertIn('ROS domain already has robot control/model publishers', result.stdout)
        self.assertNotIn('Cannot open', result.stderr)

    def test_08_bench_client_refuses_regular_robot(self):
        self.assert_bench_client_refuses_regular_robot()

    def test_07_slider_executable_accepts_ros_launch_arguments(self):
        process = subprocess.Popen(['ros2', 'run', 'pinchy_bringup', 'joint_sliders.py',
            '--ros-args', '-r', '__node:=slider_launch_smoke'], stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, start_new_session=True)
        try:
            try:
                output, _ = process.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                pass  # The GUI event loop is running, awaiting user input.
            else:
                self.fail(f'Slider executable exited during startup: {output}')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
            try:
                process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.communicate(timeout=5)
                self.fail('Slider process failed to exit after SIGTERM')

    def test_05_cli_timeout_cancels(self):
        result = subprocess.run(['ros2', 'run', 'pinchy_bringup', 'mock_trajectory.py',
            '--point', '0', '0', '0', '0', '0', '0', '--seconds', '5', '--timeout', '0.3'],
            capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn('Cleanup action status=5', result.stdout + result.stderr)

    def test_06_cli_success_and_explicit_cancel(self):
        base = ['ros2', 'run', 'pinchy_bringup', 'mock_trajectory.py',
                '--point', '0.1', '0', '0', '0', '0', '0']
        for extra, expected in [(['--seconds', '1'], 'Final action status=4'),
                                (['--seconds', '5', '--cancel-after', '0.3'], 'Final action status=5')]:
            result = subprocess.run(base + extra, capture_output=True, text=True, timeout=20)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn(expected, result.stdout + result.stderr)


class HardwarePipeline(Pipeline, unittest.TestCase):
    hardware = True
    hardware_velocity = math.radians(30)

    def test_05_target_telemetry_supports_late_subscribers_without_hold(self):
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        statuses = []
        status = self.node.create_subscription(String, '/arm_safety/status', lambda msg: statuses.append(msg.data), qos)
        try:
            for _ in range(6):
                samples = []
                sub = self.node.create_subscription(JointState, '/arm_safety/held_positions',
                    lambda msg: samples.append(msg), qos)
                try:
                    deadline = time.monotonic()+3
                    while (not samples or not statuses) and time.monotonic() < deadline:
                        self.executor.spin_once(timeout_sec=0.02)
                    self.assertTrue(samples)
                    self.assertEqual(list(samples[-1].name), JOINTS)
                    self.assertEqual(len(samples[-1].position), 6)
                    self.assertGreater(samples[-1].header.stamp.sec, 0)
                    self.assertTrue(all(math.isfinite(value) for value in samples[-1].position))
                    self.assertIn('READY', statuses)
                    self.assertFalse(any(value.startswith('HOLD:') for value in statuses))
                finally:
                    self.node.destroy_subscription(sub)
            self.assertIsNone(self.process.poll())
        finally:
            self.node.destroy_subscription(status)

    def test_05_motion_log_records_command_and_measured_feedback(self):
        import json
        deadline = time.monotonic()+5
        samples = []
        while not samples and time.monotonic() < deadline:
            for path in (self.folder/'ros_logs').glob('pinchy-motion-*.jsonl'):
                samples = [json.loads(line) for line in path.read_text().splitlines()]
            if not samples: time.sleep(0.05)
        self.assertTrue(samples, 'Motion diagnostics did not receive controller state')
        sample = samples[-1]
        self.assertEqual(sample['joints'], JOINTS)
        self.assertEqual(len(sample['commanded']), 6)
        self.assertEqual(len(sample['measured']), 6)
        self.assertEqual(len(sample['error_radians']), 6)

    def test_05_stop_holds_command_with_large_feedback_lag_and_allows_next_move(self):
        from python_qt_binding.QtWidgets import QApplication
        self.bus.feedback_offset(1, -31) # About 2.7 degrees of loaded-motor lag
        app = QApplication.instance() or QApplication([])
        spec = importlib.util.spec_from_file_location('stop_sliders', ROOT/'src/pinchy_bringup/scripts/joint_sliders.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        node = rclpy.create_node('stop_slider_test', context=self.context)
        window = module.Sliders(node, True)
        try:
            deadline = time.monotonic()+10
            while not window.send_button.isEnabled() and time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertTrue(window.send_button.isEnabled())
            window.inputs[0].setValue(0.6); window.speed.setValue(5); window.send()
            deadline = time.monotonic()+2
            while time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertTrue(window.pending)
            window.cancel()
            # A delayed pre-Stop telemetry sample must not move the arm back.
            window.saved_hold_stamp = window.pause_stamp-1
            window.submit_hold()
            self.assertTrue(window.pause_pending)
            self.assertIn('waiting for fresh', window.status.text())
            deadline = time.monotonic()+6
            while window.pending and time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertFalse(window.pending)
            self.assertEqual(window.status.text(), 'Stopped — holding commanded position')
            held = dict(window.commanded)
            deadline = time.monotonic()+0.5
            while time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            for name in JOINTS: self.assertAlmostEqual(window.commanded[name], held[name], delta=0.001)
            service = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
            try:
                controllers = self.wait(service.call_async(ListControllers.Request())).controller
                self.assertTrue(all(controller.state == 'active' for controller in controllers))
            finally: self.node.destroy_client(service)
            self.bus.feedback_offset(1, 0)
            for _ in range(20): app.processEvents(); time.sleep(0.02)
            window.inputs[0].setValue(held['shoulder_pan']+0.03); window.speed.setValue(3); window.send()
            deadline = time.monotonic()+6
            while window.pending and time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertFalse(window.pending)
            self.assertEqual(window.status.text(), 'Reached target')
        finally:
            self.bus.feedback_offset(1, 0)
            window.timer.stop(); window.client.destroy(); window.executor.shutdown()
            window.deleteLater(); app.processEvents(); node.destroy_node()

    def test_05_consecutive_slider_moves_near_velocity_limit(self):
        from python_qt_binding.QtWidgets import QApplication
        self.bus.feedback_offset(1, 8)
        app = QApplication.instance() or QApplication([])
        spec = importlib.util.spec_from_file_location('consecutive_sliders', ROOT/'src/pinchy_bringup/scripts/joint_sliders.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        node = rclpy.create_node('consecutive_slider_test', context=self.context)
        window = module.Sliders(node, True)
        try:
            deadline = time.monotonic()+10
            while not window.send_button.isEnabled() and time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertTrue(window.send_button.isEnabled())
            self.assertAlmostEqual(window.speed.maximum(), 30, places=2)
            self.assertAlmostEqual(window.acceleration.maximum(), 180, places=1)
            for target in (-0.10, -0.17, 0.30):
                window.inputs[0].setValue(target); window.speed.setValue(30); window.acceleration.setValue(180)
                window.send()
                deadline = time.monotonic()+12
                while window.pending and time.monotonic() < deadline:
                    app.processEvents(); time.sleep(0.01)
                self.assertFalse(window.pending)
                self.assertEqual(window.status.text(), 'Reached target', f'target={target}')
                for _ in range(10): app.processEvents(); time.sleep(0.02)
        finally:
            window.timer.stop(); window.client.destroy(); window.executor.shutdown()
            window.deleteLater(); app.processEvents(); node.destroy_node()
            self.bus.feedback_offset(1, 0)

    def test_05_feedback_offset_does_not_jump_first_command(self):
        self.bus.feedback_offset(2, 8) # Accepted real-servo residual: about 0.70 degrees
        try:
            for _ in range(5): self.executor.spin_once(timeout_sec=0.05)
            # Isolate the feedback offset without resetting joints left at a
            # different position by the consecutive-move regression above.
            target = [self.actual[name] for name in JOINTS]
            target[1] += 0.03
            handle = self.goal([target], seconds=3)
            self.assertTrue(handle.accepted)
            result = self.wait(handle.get_result_async())
            self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED, result.result.error_string)
            self.assertEqual(result.result.error_code, 0)
        finally:
            self.bus.feedback_offset(2, 0)

    def test_07_bench_client_refuses_regular_robot(self):
        self.assert_bench_client_refuses_regular_robot()

    def test_05_small_residual_and_controller_abort_do_not_latch_velocity_fault(self):
        from python_qt_binding.QtWidgets import QApplication
        from control_msgs.msg import JointTolerance
        app = QApplication.instance() or QApplication([])
        spec = importlib.util.spec_from_file_location('settling_sliders', ROOT/'src/pinchy_bringup/scripts/joint_sliders.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        node = rclpy.create_node('settling_test', context=self.context)
        window = module.Sliders(node, True)
        def until(predicate, seconds=7):
            deadline = time.monotonic()+seconds
            while not predicate() and time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=0)
                app.processEvents(); time.sleep(0.01)
            self.assertTrue(predicate(), window.status.text())
        try:
            until(lambda: window.send_button.isEnabled())
            self.assertEqual(window.speed.value(), 30)
            self.assertEqual(window.acceleration.value(), 180)
            self.bus.feedback_offset(3, 14) # Exact ~1.23 degree residual from the real report.
            for _ in range(25): app.processEvents(); time.sleep(0.01)
            window.inputs[2].setValue(window.commanded['elbow_flex']+0.02)
            window.send(); until(lambda: not window.pending)
            self.assertEqual(window.status.text(), 'Reached target')
            self.assertFalse(window.safety_hold)
            # A client requesting tighter tolerance still receives an honest
            # failure, but its abort-to-measured hold must not fault the hardware.
            goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
            point = JointTrajectoryPoint(positions=[window.commanded[name] for name in JOINTS])
            point.time_from_start.nanosec = 500_000_000
            goal.trajectory.points = [point]
            goal.goal_tolerance = [JointTolerance(name=name, position=0.01) for name in JOINTS]
            handle = self.wait(self.client.send_goal_async(goal))
            result = self.wait(handle.get_result_async(), seconds=6)
            self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
            for _ in range(40): app.processEvents(); time.sleep(0.01)
            self.assertFalse(window.safety_hold, window.safety_reason)
            self.assertTrue(window.send_button.isEnabled())
            self.bus.feedback_offset(3, 0)
            for _ in range(25): app.processEvents(); time.sleep(0.01)
            window.inputs[2].setValue(0.01)
            window.send(); until(lambda: not window.pending)
            self.assertEqual(window.status.text(), 'Reached target')
            self.assertIsNone(self.process.poll())
        finally:
            self.bus.feedback_offset(3, 0)
            window.timer.stop(); window.client.destroy(); window.executor.shutdown()
            window.deleteLater(); app.processEvents(); node.destroy_node()

    def test_06_hardware_fault_prevents_success(self):
        from python_qt_binding.QtWidgets import QApplication
        app = QApplication.instance() or QApplication([])
        spec = importlib.util.spec_from_file_location('safety_sliders', ROOT/'src/pinchy_bringup/scripts/joint_sliders.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        node = rclpy.create_node('safety_slider_test', context=self.context)
        window = module.Sliders(node, True)
        try:
            deadline = time.monotonic()+8
            while not window.send_button.isEnabled() and time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertTrue(window.send_button.isEnabled())
            handle = self.goal([[0.1]*6], seconds=5); self.assertTrue(handle.accepted)
            future = handle.get_result_async()
            self.bus.fault(3, 63, 60)
            deadline = time.monotonic()+6
            while not window.safety_hold and time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertTrue(window.safety_hold)
            self.assertIn('four consecutive', window.status.text())
            self.assertFalse(window.send_button.isEnabled())
            service = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
            try:
                controllers = self.wait(service.call_async(ListControllers.Request())).controller
                self.assertTrue(controllers)
                self.assertTrue(all(controller.state == 'active' for controller in controllers))
            finally: self.node.destroy_client(service)
            self.assertFalse(future.done(), 'Paused movement must not report false success')
            # Operator recovery cannot bypass an active overtemperature fault.
            window.cancel()
            deadline = time.monotonic()+4
            while (window.recovering or window.pending) and time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertTrue(window.safety_hold)
            self.assertIn('Still holding', window.safety_reason)
            self.bus.fault(3, 63, 33)
            deadline = time.monotonic()+0.5
            while time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            window.send()
            self.assertFalse(window.pending)
            self.assertTrue(window.safety_hold)
            self.assertFalse(window.send_button.isEnabled())
            window.cancel()
            deadline = time.monotonic()+6
            while (window.safety_hold or window.pending) and time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertFalse(window.safety_hold, window.safety_reason)
            self.assertFalse(window.pending, window.status.text())
            window.speed.setValue(30); window.acceleration.setValue(180)
            window.inputs[0].setValue(0.02)
            window.send()
            deadline = time.monotonic()+6
            while window.pending and time.monotonic() < deadline:
                app.processEvents(); time.sleep(0.01)
            self.assertEqual(window.status.text(), 'Reached target')
            self.assertTrue(window.motion_history[-1]['samples'])
            self.assertIn('deg/s²', window.labels[0].text())
            window.show_graph(); app.processEvents(); window.graph.refresh()
            self.assertGreaterEqual(len(window.graph.axes[0].lines), 3)
            self.assertGreaterEqual(len(window.graph.axes[1].lines), 2)
            self.assertTrue(all(math.isfinite(v) for v in window.measured_acceleration))
        finally:
            if window.graph is not None: window.graph.close(); window.graph.timer.stop()
            window.timer.stop(); window.client.destroy(); window.executor.shutdown()
            window.deleteLater(); app.processEvents(); node.destroy_node()


class UnloadedBenchPipeline(Pipeline, unittest.TestCase):
    hardware = True
    bench = True
    gui_target = 0.02

    def test_01_waypoints_feedback(self):
        result = subprocess.run(['ros2', 'run', 'pinchy_bringup', 'ros_bench_test.py',
            '--seconds', '1', '--log', str(self.folder / 'bench-test.jsonl')],
            capture_output=True, text=True, timeout=40)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('PASS: IDs [1, 2, 3, 4, 5, 6]', result.stdout)
        for servo in range(1, 7):
            self.assertIn(f'ID {servo}', result.stdout)

    def test_02_cancel(self):
        handle = self.goal([[0.02] * 6], seconds=5)
        self.assertTrue(handle.accepted)
        response = self.wait(handle.cancel_goal_async())
        self.assertTrue(response.goals_canceling)
        self.assertEqual(self.wait(handle.get_result_async()).status, GoalStatus.STATUS_CANCELED)


if __name__ == '__main__':
    unittest.main(verbosity=2)
