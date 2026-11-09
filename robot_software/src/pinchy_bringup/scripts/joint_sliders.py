#!/usr/bin/env python3
"""Stage six joint targets; Send executes them through FollowJointTrajectory."""
from collections import deque
import numpy as np
import argparse
import math
import signal
import sys
import time
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState, JointTolerance, SpeedScalingFactor
from python_qt_binding.QtCore import QTimer, Qt
from python_qt_binding.QtGui import QFontDatabase
from python_qt_binding.QtWidgets import (
    QApplication, QDoubleSpinBox, QGridLayout, QLabel, QPushButton, QSlider, QWidget,
    QComboBox, QCheckBox, QFileDialog)
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
          'wrist_flex', 'wrist_roll', 'pen_pitch']


def motion_duration(distances, velocities, acceleration):
    """Duration for a quintic profile with bounded peak speed/acceleration."""
    if (not distances or len(distances) != len(velocities) or
            not math.isfinite(acceleration) or acceleration <= 0 or
            any(not math.isfinite(d) or d < 0 for d in distances) or
            any(not math.isfinite(v) or v <= 0 for v in velocities)):
        raise ValueError('Require finite distances, positive speeds and positive acceleration')
    return 1.05*max(0.5, max(1.875*d/v for d, v in zip(distances, velocities)),
                    max(math.sqrt((10/math.sqrt(3))*d/acceleration) for d in distances))


class Sliders(QWidget):
    def __init__(self, node, hardware):
        super().__init__()
        self.node, self.hardware = node, hardware
        self.executor = SingleThreadedExecutor(context=node.context)
        self.executor.add_node(node)
        self.actual, self.commanded, self.description, self.goal = {}, {}, None, None
        self.pending = False
        self.stopping = False
        self.pause_pending = False
        self.resume_hold = False
        self.generation = 0
        self.hold_positions = []
        self.last_feedback = None
        self.last_command_feedback = None
        self.safety_hold = False
        self.safety_ready = not hardware
        self.safety_reason = ''
        self.recovering = False
        self.saved_hold = []
        self.motion_history = []
        self.active_record = None
        self.encoder_window = deque(maxlen=20)
        self.measured_speed = np.zeros(6)
        self.measured_acceleration = np.zeros(6)
        self.graph = None
        self.setWindowTitle('Pinchy REAL hardware targets' if hardware else 'Pinchy MOCK motor targets')
        grid = QGridLayout(self)
        grid.setColumnStretch(1, 1)
        grid.addWidget(QLabel('Radians — sliders set targets; Send requests movement.\n'
                             'Joint limits enforced; self-collision checking is not enabled.'), 0, 0, 1, 4)
        self.inputs, self.sliders, self.labels = [], [], []
        for row, name in enumerate(JOINTS, 1):
            grid.addWidget(QLabel(name), row, 0)
            slider = QSlider(Qt.Horizontal)
            spin = QDoubleSpinBox(); spin.setDecimals(3); spin.setSingleStep(0.01)
            slider.setRange(0, 0); spin.setRange(0, 0)
            slider.valueChanged.connect(lambda value, box=spin: box.setValue(value / 1000.0))
            spin.valueChanged.connect(lambda value, bar=slider: bar.setValue(round(value * 1000)))
            label = QLabel('Actual: waiting')
            # Reserve space before feedback arrives; signs and digit counts must
            # not resize the readout column or push the slider around.
            label.setFont(QFontDatabase.systemFont(QFontDatabase.FixedFont))
            label.setFixedWidth(label.fontMetrics().horizontalAdvance(
                '+000.000 rad | +00000.0 deg/s | +0000000.0 deg/s² (estimated)') + 12)
            spin.setFixedWidth(spin.fontMetrics().horizontalAdvance('-000.000') + 36)
            self.inputs.append(spin); self.sliders.append(slider); self.labels.append(label)
            grid.addWidget(slider, row, 1); grid.addWidget(spin, row, 2); grid.addWidget(label, row, 3)
        self.speed = QDoubleSpinBox(); self.speed.setDecimals(2)
        self.speed.setRange(0.1, 30); self.speed.setValue(3); self.speed.setSuffix(' deg/s')
        self.acceleration = QDoubleSpinBox(); self.acceleration.setDecimals(1)
        self.acceleration.setRange(0.1, 180); self.acceleration.setValue(10); self.acceleration.setSuffix(' deg/s²')
        grid.addWidget(QLabel('Maximum speed'), 7, 0); grid.addWidget(self.speed, 7, 1)
        grid.addWidget(QLabel('Maximum acceleration'), 7, 2); grid.addWidget(self.acceleration, 7, 3)
        self.estimate = QLabel('Estimated move duration: waiting for controller feedback')
        grid.addWidget(self.estimate, 8, 0, 1, 4)
        for control in [*self.inputs, self.speed, self.acceleration]:
            control.valueChanged.connect(self.update_estimate)
        self.send_button = QPushButton('Send'); self.send_button.setEnabled(False)
        self.send_button.clicked.connect(self.send)
        reset = QPushButton('Copy actual to targets'); reset.clicked.connect(self.copy_actual)
        cancel = QPushButton('Stop / hold'); cancel.clicked.connect(self.cancel)
        grid.addWidget(self.send_button, 9, 0); grid.addWidget(reset, 9, 1); grid.addWidget(cancel, 9, 2)
        self.status = QLabel('Waiting for model, controller, and feedback')
        grid.addWidget(self.status, 10, 0, 1, 4)
        graphs = QPushButton('Motion graphs / recordings'); graphs.clicked.connect(self.show_graph)
        grid.addWidget(graphs, 9, 3)
        self.reset_client = node.create_client(Trigger, '/arm_safety/reset')
        self.held_sub = node.create_subscription(Float64MultiArray, '/arm_safety/held_positions',
            lambda message: setattr(self, 'saved_hold', list(message.data)),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.scaling = node.create_publisher(SpeedScalingFactor, '/arm_controller/speed_scaling_input',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.model_sub = node.create_subscription(String, '/robot_description', self.on_model,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.state_sub = node.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.command_sub = node.create_subscription(JointTrajectoryControllerState,
            '/arm_controller/controller_state', self.on_controller_state, 10)
        self.safety_sub = node.create_subscription(String, '/arm_safety/status', self.on_safety,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = QTimer(self); self.timer.timeout.connect(self.poll); self.timer.start(20)

    def on_model(self, message):
        self.description = None
        self.send_button.setEnabled(False)
        root = ET.fromstring(message.data)
        expected = 'pinchy_hardware/FeetechSystem' if self.hardware else 'mock_components/GenericSystem'
        if [p.text for p in root.findall('./ros2_control/hardware/plugin')] != [expected]:
            self.status.setText('Model does not match selected mock/hardware mode'); return
        if any(root.find(f"joint[@name='{name}']/limit") is None for name in JOINTS):
            self.status.setText('Model has older joint names; restart the control launch for CAD 4.'); return
        self.description = root
        velocities = [float(root.find(f"joint[@name='{name}']/limit").get('velocity')) for name in JOINTS]
        self.speed.setMaximum(math.degrees(min(math.radians(30), max(velocities))))
        raw_acceleration = root.find("./ros2_control/hardware/param[@name='acceleration']")
        if self.hardware and raw_acceleration is not None:
            self.acceleration.setMaximum(min(180, 0.8*8.7*int(raw_acceleration.text)))
        if root.get('name') == 'pinchy_bench_hardware':
            self.setWindowTitle('Pinchy REAL unloaded bench — temporary encoder zeros')
        for name, spin, slider in zip(JOINTS, self.inputs, self.sliders):
            limit = root.find(f"joint[@name='{name}']/limit")
            low, high = float(limit.get('lower')), float(limit.get('upper'))
            slider.setRange(math.ceil(low * 1000), math.floor(high * 1000))
            spin.setRange(slider.minimum() / 1000, slider.maximum() / 1000)
        if len(self.actual) == 6:
            self.copy_actual()

    def on_state(self, message):
        import time
        values = dict(zip(message.name, message.position))
        if not all(name in values and math.isfinite(values[name]) for name in JOINTS):
            return
        first = not self.actual
        self.actual = {name: values[name] for name in JOINTS}
        self.last_feedback = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        if self.encoder_window and stamp <= self.encoder_window[-1][0]:
            self.encoder_window.clear()
        self.encoder_window.append((stamp, list(self.actual.values())))
        while len(self.encoder_window) > 3 and stamp-self.encoder_window[0][0] > 0.25:
            self.encoder_window.popleft()
        if len(self.encoder_window) >= 4:
            times = np.array([row[0]-stamp for row in self.encoder_window])
            if times[-1]-times[0] > 0.05:
                coefficients = np.polynomial.polynomial.polyfit(times,
                    np.array([row[1] for row in self.encoder_window]), 2)
                self.measured_speed = coefficients[1]
                self.measured_acceleration = 2*coefficients[2]
        for i, (name, label) in enumerate(zip(JOINTS, self.labels)):
            label.setText(f'{self.actual[name]:+.3f} rad | '
                f'{math.degrees(self.measured_speed[i]):+.1f} deg/s | '
                f'{math.degrees(self.measured_acceleration[i]):+.1f} deg/s² (estimated)')
        if first and self.description is not None:
            self.copy_actual()

    def copy_actual(self):
        for name, spin in zip(JOINTS, self.inputs):
            if name in self.actual: spin.setValue(self.actual[name])

    def on_controller_state(self, message):
        values = dict(zip(message.joint_names, message.output.positions))
        if all(name in values and math.isfinite(values[name]) for name in JOINTS):
            self.commanded = {name: values[name] for name in JOINTS}
            self.last_command_feedback = time.monotonic()
            if self.active_record is not None:
                record = self.active_record
                elapsed = (self.node.get_clock().now().nanoseconds-record['start_ns'])*1e-9
                if len(record['samples']) < 10000 and all(name in self.actual for name in JOINTS):
                    record['samples'].append((elapsed, [self.actual[name] for name in JOINTS],
                        self.measured_speed.tolist(), [self.commanded[name] for name in JOINTS]))
            if not self.pending: self.update_estimate()
            stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            if self.pause_pending and message.speed_scaling_factor == 0.0 and stamp >= self.pause_stamp:
                self.pause_pending = False
                self.submit_hold()

    def feedback_fresh(self):
        now = time.monotonic()
        return all(stamp is not None and now-stamp < 0.5 for stamp in
                   (self.last_feedback, self.last_command_feedback))

    def on_safety(self, message):
        if message.data == 'READY' and not self.safety_hold:
            self.safety_ready = True
            return
        if not message.data.startswith('HOLD:'): return
        self.safety_hold = True
        self.safety_ready = False
        self.safety_reason = 'Safety hold — '+message.data[5:].strip()+'; correct cause, then Stop / hold to recover'
        self.generation += 1 # Ignore outstanding action results and hold handshakes.
        self.pending = self.stopping = self.pause_pending = self.resume_hold = False
        self.goal = None
        self.recovering = False
        self.active_record = None
        self.status.setText(self.safety_reason)
        self.send_button.setEnabled(False)

    def move_duration(self):
        distances = [abs(spin.value()-self.commanded[name]) for name, spin in zip(JOINTS, self.inputs)]
        velocities = [min(math.radians(self.speed.value()), math.radians(30),
            float(self.description.find(f"joint[@name='{name}']/limit").get('velocity'))) for name in JOINTS]
        return motion_duration(distances, velocities, math.radians(self.acceleration.value()))

    def update_estimate(self, *args):
        if self.description is not None and len(self.commanded) == len(JOINTS):
            self.estimate.setText(f'Estimated move duration: {self.move_duration():.1f} s (joint limits applied)')

    def poll(self):
        import time
        self.executor.spin_once(timeout_sec=0)
        if self.safety_hold:
            self.status.setText(self.safety_reason); self.send_button.setEnabled(False); return
        fresh = self.feedback_fresh()
        self.send_button.setEnabled(self.description is not None and fresh and
                                    self.safety_ready and not self.pending and not self.stopping and self.client.server_is_ready())

    def send(self):
        if self.safety_hold:
            self.status.setText(self.safety_reason); return
        if self.pending or self.stopping or not self.safety_ready or self.description is None or not self.feedback_fresh():
            self.status.setText('Waiting for fresh motor and controller command feedback'); return
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
        start = JointTrajectoryPoint()
        start.positions = [self.commanded[name] for name in JOINTS]
        start.velocities = [0.0]*len(JOINTS)
        start.accelerations = [0.0]*len(JOINTS)
        point = JointTrajectoryPoint(); point.positions = [spin.value() for spin in self.inputs]
        point.velocities = [0.0]*len(JOINTS)
        point.accelerations = [0.0]*len(JOINTS)
        # The same profile drives both the estimate and the actual trajectory.
        # Explicit t=0 and zero endpoint velocity/acceleration prevent jumps
        # and let the controller use a smooth quintic interpolation.
        seconds = self.move_duration()
        ns = round(seconds * 1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 1_000_000_000)
        goal.trajectory.points = [start, point]
        self.active_record = dict(start_ns=self.node.get_clock().now().nanoseconds,
            duration=seconds, start=list(start.positions), target=list(point.positions), samples=[])
        self.motion_history.append(self.active_record)
        self.motion_history = self.motion_history[-10:]
        self.pending = True; self.status.setText(f'Sending smooth {seconds:.1f} second trajectory')
        self.dispatch(goal)

    def dispatch(self, goal, hold=False):
        self.generation += 1
        generation = self.generation
        feedback = (lambda message: self.hold_started(message, generation)) if hold else None
        self.client.send_goal_async(goal, feedback_callback=feedback).add_done_callback(
            lambda future: self.accepted(future, generation, hold))

    def accepted(self, future, generation, hold):
        handle = future.result()
        if handle.accepted:
            handle.get_result_async().add_done_callback(lambda result: self.finished(result, generation, hold))
        if generation != self.generation: return
        self.goal = handle
        if not handle.accepted:
            if self.stopping and not hold: return
            self.pending = False
            self.status.setText('Stop hold rejected; controller remains paused' if hold else 'Controller rejected the goal')
            return
        self.status.setText('Establishing powered hold' if hold else 'Moving — actual column shows controller feedback')

    def finished(self, future, generation, hold):
        if generation != self.generation or (self.stopping and not hold): return
        result = future.result(); self.pending = False; self.goal = None
        record = self.active_record
        def finish_recording():
            if self.active_record is record: self.active_record = None
        QTimer.singleShot(500, finish_recording)
        success = result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code == 0
        if hold:
            self.stopping = False
            if success:
                for value, spin in zip(self.hold_positions, self.inputs): spin.setValue(value)
                self.status.setText('Stopped — holding commanded position')
                return
        self.status.setText('Reached target' if success else
                            f'Stopped: status {result.status}, {result.result.error_string}')

    def cancel(self):
        # JTC's cancel callback holds measured feedback. With servo lag that
        # creates a command jump. Pause first, then replace the goal with a hold.
        if self.safety_hold:
            if self.recovering: return
            if len(self.saved_hold) != len(JOINTS) or not self.feedback_fresh() or not self.reset_client.service_is_ready():
                self.safety_reason = 'Safety hold — waiting for feedback/recovery service; check power and connection'
                return
            self.recovering = True
            self.stopping = True
            self.pending = True
            self.pause_pending = True
            self.pause_stamp = self.node.get_clock().now().nanoseconds
            self.scaling.publish(SpeedScalingFactor(factor=0.0))
            self.safety_reason = 'Recovering — replacing interrupted trajectory with powered hold'
            return
        if not self.pending or self.stopping: return
        self.stopping = True; self.pause_pending = True
        self.pause_stamp = self.node.get_clock().now().nanoseconds
        self.scaling.publish(SpeedScalingFactor(factor=0.0))
        self.status.setText('Stop requested — waiting for controller pause; torque stays on')

    def submit_hold(self):
        self.hold_positions = list(self.saved_hold) if self.recovering else [self.commanded[name] for name in JOINTS]
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
        start = JointTrajectoryPoint(positions=self.hold_positions, velocities=[0.0]*len(JOINTS))
        end = JointTrajectoryPoint(positions=self.hold_positions, velocities=[0.0]*len(JOINTS))
        end.time_from_start.nanosec = 250_000_000
        goal.trajectory.points = [start, end]
        # Completion means the command is held, not that loaded feedback has
        # caught up. Hardware bounds/health/speed checks remain active.
        goal.goal_tolerance = [JointTolerance(name=name, position=-1.0) for name in JOINTS]
        self.resume_hold = True
        self.dispatch(goal, hold=True)

    def hold_started(self, message, generation):
        if generation != self.generation or not self.resume_hold: return
        desired = message.feedback.desired.positions
        if len(desired) == len(JOINTS) and all(abs(a-b) < 1e-9 for a, b in zip(desired, self.hold_positions)):
            # Resume only after feedback from this specific replacement goal
            # confirms the constant target is installed. The old move is gone.
            self.resume_hold = False
            if self.recovering:
                self.reset_client.call_async(Trigger.Request()).add_done_callback(self.recovered)
            else:
                self.scaling.publish(SpeedScalingFactor(factor=1.0))

    def recovered(self, future):
        try:
            result = future.result()
            success, reason = result.success, result.message
        except Exception as error:
            success, reason = False, str(error)
        self.recovering = False
        if not success:
            self.pending = self.stopping = False
            self.safety_reason = 'Still holding — '+reason+'; correct cause, then Stop / hold again'
            return
        self.safety_hold = False
        self.safety_ready = True
        self.scaling.publish(SpeedScalingFactor(factor=1.0))
        self.status.setText('Recovered — holding position; Send starts a new move')

    def show_graph(self):
        if self.graph is None:
            self.graph = MotionGraph(self)
        self.graph.show(); self.graph.raise_()

    def closeEvent(self, event):
        if self.pending:
            self.cancel()
            self.status.setText('Wait for the action result before closing')
            event.ignore()
        else:
            event.accept()


class MotionGraph(QWidget):
    """Last ten moves, each with planned and encoder-measured traces."""
    def __init__(self, owner):
        super().__init__()
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
        self.owner = owner
        self.setWindowTitle('Planned versus measured joint motion')
        self.resize(950, 650)
        grid = QGridLayout(self)
        self.joint = QComboBox(); self.joint.addItems(JOINTS)
        self.overlay = QCheckBox('Overlay previous moves (last 10)')
        save = QPushButton('Save recordings CSV'); save.clicked.connect(self.save_csv)
        grid.addWidget(self.joint, 0, 0); grid.addWidget(self.overlay, 0, 1); grid.addWidget(save, 0, 2)
        grid.addWidget(QLabel('Dashed: planned quintic. Solid: encoder measurements. '
            'Dotted: controller command. Speed is a 0.25 s filtered estimate.\n'
            'Time starts when Send is pressed; communication adds a small timing offset.'), 1, 0, 1, 3)
        self.figure = Figure(); self.canvas = FigureCanvasQTAgg(self.figure)
        self.axes = self.figure.subplots(2, 1, sharex=True)
        grid.addWidget(self.canvas, 2, 0, 1, 3)
        self.timer = QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(500)

    def refresh(self):
        if not self.isVisible(): return
        joint = self.joint.currentIndex()
        for axis in self.axes: axis.clear(); axis.grid(True, alpha=0.25)
        records = self.owner.motion_history if self.overlay.isChecked() else self.owner.motion_history[-1:]
        for index, record in enumerate(records):
            duration = record['duration']; t = np.linspace(0, duration, 240); u = t/duration
            start, delta = record['start'][joint], record['target'][joint]-record['start'][joint]
            label = f'Move {self.owner.motion_history.index(record)+1}'
            color = f'C{index%10}'
            self.axes[0].plot(t, np.degrees(start+delta*(10*u**3-15*u**4+6*u**5)), '--', color=color, label=label+' planned')
            self.axes[1].plot(t, np.degrees(delta*(30*u**2-60*u**3+30*u**4)/duration), '--', color=color)
            samples = record['samples']
            if samples:
                times = [row[0] for row in samples]
                self.axes[0].plot(times, [math.degrees(row[1][joint]) for row in samples], color=color, label=label+' measured')
                self.axes[0].plot(times, [math.degrees(row[3][joint]) for row in samples], ':', color=color)
                self.axes[1].plot(times, [math.degrees(row[2][joint]) for row in samples], color=color)
        self.axes[0].set_ylabel('Position (deg)'); self.axes[1].set_ylabel('Speed (deg/s)')
        self.axes[1].set_xlabel('Seconds since Send')
        if records: self.axes[0].legend(fontsize=8)
        self.figure.tight_layout(); self.canvas.draw_idle()

    def save_csv(self):
        import csv
        path, _ = QFileDialog.getSaveFileName(self, 'Save motion recordings', 'joint-motion.csv', 'CSV (*.csv)')
        if not path: return
        with open(path, 'w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['move', 'joint', 'seconds', 'planned_deg', 'planned_deg_s',
                'measured_deg', 'measured_deg_s', 'controller_command_deg'])
            for number, record in enumerate(self.owner.motion_history, 1):
                for elapsed, actual, speed, command in record['samples']:
                    u = min(1, max(0, elapsed/record['duration']))
                    for joint, name in enumerate(JOINTS):
                        delta = record['target'][joint]-record['start'][joint]
                        planned = record['start'][joint]+delta*(10*u**3-15*u**4+6*u**5)
                        velocity = delta*(30*u**2-60*u**3+30*u**4)/record['duration']
                        writer.writerow([number, name, elapsed, math.degrees(planned), math.degrees(velocity),
                            math.degrees(actual[joint]), math.degrees(speed[joint]), math.degrees(command[joint])])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true', help='Explicitly select real hardware mode')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    app = QApplication(sys.argv[:1]); node = rclpy.create_node('pinchy_joint_sliders')
    window = Sliders(node, args.hardware); window.show()
    def stop(signum, frame):
        window.cancel()
        def quit_after_hold():
            if window.pending:
                QTimer.singleShot(50, quit_after_hold)
            else:
                app.quit()
        quit_after_hold()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        return app.exec_()
    finally:
        window.timer.stop(); window.client.destroy(); window.executor.shutdown()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
