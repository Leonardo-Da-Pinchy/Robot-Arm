"""Prompted startup owns only selected motors and never disables holds at handoff."""
import copy
import io
import json
import os
import fcntl
import pty
import select
import struct
import termios
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from datetime import datetime
from contextlib import redirect_stdout
import unittest
from unittest.mock import patch

from calibrate import new_profile, CONFIG_KEYS, ROOT
from servo_setup import MockSetupBus
from start_arm import GuidedStartup, walkthrough, validate_profile
from startup_dashboard import CaptureWindow


class Bus:
    def __init__(self):
        self.servo_id = 1
        self.motors = {i: MockSetupBus(i) for i in range(1, 7)}
        self.events = []
        self.motion_settings = []
        for motor in self.motors.values(): motor.position = 2058
    def read(self): return self.motors[self.servo_id].read()
    def setup_state(self):
        result = self.motors[self.servo_id].setup_state(); result['model_raw'] = 777
        return result
    def byte(self, address):
        if address != 65: raise AssertionError(address)
        return 0
    def word(self, address):
        motor = self.motors[self.servo_id]
        if address == 42: return motor.goal
        if address == 48: return motor.limit
        raise AssertionError(address)
    def set_torque_limit(self, value):
        self.events.append((self.servo_id, 'limit', value)); self.motors[self.servo_id].limit = value
    def move(self, position, speed, acceleration):
        self.events.append((self.servo_id, 'goal', position))
        self.motion_settings.append((self.servo_id, speed, acceleration))
        self.motors[self.servo_id].move(position, speed, acceleration)
    def enable(self):
        self.events.append((self.servo_id, 'on', 1)); self.motors[self.servo_id].enable()
    def release(self):
        self.events.append((self.servo_id, 'off', 0)); self.motors[self.servo_id].release()
    def close(self): pass


def profile(bus):
    result = new_profile(True, '/test-port')
    for servo_id, name in __import__('calibrate').JOINTS.items():
        bus.servo_id = servo_id; state = bus.setup_state()
        result['joints'][name].update(zero_tick=2048, direction=1, min_tick=1984,
            max_tick=2112, min_radians=-64*2*3.141592653589793/4096,
            max_radians=64*2*3.141592653589793/4096)
        if servo_id == 6:
            result['joints'][name].update(min_tick=2032, min_radians=-16*2*3.141592653589793/4096)
        result['observations'][name] = dict(zero=dict(configuration={key:state[key] for key in CONFIG_KEYS}),
                                           positive_direction=dict(encoder_delta=32))
    return result


class Clock:
    def __init__(self): self.now = 0
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


class CaptureTests(unittest.TestCase):
    def test_capture_needs_one_second_without_motion_and_resets_when_ineligible(self):
        gate = CaptureWindow()
        self.assertEqual(gate.update(0, 2048, True), 0)
        self.assertEqual(gate.update(1, 2052, True), 0)
        self.assertEqual(gate.update(1.5, 2053, True), 0.5)
        self.assertEqual(gate.update(2, 2052, True), 1)
        self.assertEqual(gate.update(4, 2052, False), 0)
        self.assertEqual(gate.update(5, 2052, True), 0)


class StartupTests(unittest.TestCase):
    def setUp(self):
        self.bus = Bus(); self.profile = profile(self.bus); self.clock = Clock()
        self.start = GuidedStartup(self.bus, self.profile, clock=self.clock, sleep=self.clock.sleep)

    def test_one_at_a_time_keeps_previous_hold_and_others_free(self):
        self.start.preflight()
        with redirect_stdout(io.StringIO()): self.start.lock_motor(1)
        self.assertEqual(self.bus.motors[1].torque, 1)
        self.assertEqual(self.bus.motors[2].torque, 0)
        before = len(self.bus.events)
        with redirect_stdout(io.StringIO()): self.start.lock_motor(2)
        self.assertTrue(all(event[0] == 2 for event in self.bus.events[before:]))
        self.assertFalse(any(event[1] == 'off' for event in self.bus.events))
        self.assertEqual(self.bus.motors[1].goal, 2048)
        self.assertEqual(self.bus.motors[2].goal, 2048)

    def test_current_goal_staged_before_zero_and_ram_limit_before_enable(self):
        with redirect_stdout(io.StringIO()): self.start.lock_motor(1)
        events = self.bus.events
        self.assertEqual(events[0], (1, 'limit', 300))
        self.assertEqual(events[1], (1, 'goal', 2058))
        self.assertEqual(events[-1], (1, 'goal', 2048))
        goals = [event[2] for event in events if event[1] == 'goal']
        self.assertEqual(goals, sorted(goals, reverse=True))

    def test_higher_ros_caps_do_not_speed_up_initial_zeroing(self):
        self.profile.update(speed=500, acceleration=32)
        with redirect_stdout(io.StringIO()): self.start.lock_motor(1)
        self.assertTrue(self.bus.motion_settings)
        self.assertTrue(all(speed == 100 and acceleration == 5 for _, speed, acceleration in self.bus.motion_settings))

    def test_folded_pose_rejected_before_any_write(self):
        self.bus.motors[1].position = 2400 # More than 25 degrees from zero
        with self.assertRaises(ValueError): self.start.lock_motor(1)
        self.assertEqual(self.bus.events, [])

    def test_default_window_allows_twenty_degrees_with_slow_monotonic_move(self):
        self.bus.motors[1].position = 2276 # About 20 degrees from zero
        with redirect_stdout(io.StringIO()): self.start.lock_motor(1)
        goals = [event[2] for event in self.bus.events if event[1] == 'goal']
        self.assertEqual(goals, sorted(goals, reverse=True))
        self.assertEqual(goals[-1], 2048)
        self.assertAlmostEqual(self.clock.now, 4.7, delta=0.1)
        self.assertEqual(self.bus.motors[1].torque, 1)

    def test_brief_tracking_lag_pauses_then_recovers(self):
        self.bus.motors[1].position = 2150
        original = self.start.state
        lag_reads = []
        def state(servo_id, full=False):
            result = original(servo_id, full)
            if not full and self.bus.motors[1].goal < 2150 and len(lag_reads) < 2:
                lag_reads.append(len(self.bus.events))
                result['position'] += 20
            return result
        output = io.StringIO()
        with patch.object(self.start, 'state', side_effect=state), redirect_stdout(output):
            self.start.lock_motor(1)
        self.assertIn('caught up; continuing', output.getvalue())
        self.assertEqual(lag_reads[0], lag_reads[1]) # No new goals during catch-up
        self.assertEqual(self.bus.motors[1].goal, 2048)

    def test_persistent_stall_pauses_goals_and_reports_actionable_failure(self):
        self.bus.motors[1].position = 2150
        def blocked_move(position, speed, acceleration):
            self.bus.events.append((1, 'goal', position))
            self.bus.motors[1].goal = position
        with patch.object(self.bus, 'move', side_effect=blocked_move), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, 'motor could not catch up.*Support this joint'):
                self.start.lock_motor(1)
        goals = [event[2] for event in self.bus.events if event[1] == 'goal']
        self.assertGreater(goals[-1], 2048) # Stops advancing instead of pushing to zero
        self.assertGreater(self.clock.now, 3)

    def test_small_inward_startup_can_approach_zero_from_below_operating_bound(self):
        joint = self.profile['joints']['wrist_roll']; joint.update(min_tick=2048, min_radians=0)
        self.bus.motors[5].position = 2038
        before = copy.deepcopy(self.profile)
        with redirect_stdout(io.StringIO()): self.start.lock_motor(5)
        self.assertEqual(self.bus.motors[5].goal, 2048)
        self.assertEqual(self.profile, before)

    def test_cad_range_still_enforced_inside_near_zero_window(self):
        self.bus.motors[6].position = 2008 # -3.52 degrees, beyond CAD -3
        with self.assertRaises(ValueError): self.start.lock_motor(6)
        self.assertEqual(self.bus.events, [])

    def test_existing_hold_loss_stops_next_motor_before_writes(self):
        with redirect_stdout(io.StringIO()): self.start.lock_motor(1)
        before = list(self.bus.events); self.bus.motors[1].position += 40
        with self.assertRaises(RuntimeError): self.start.lock_motor(2)
        self.assertEqual(self.bus.events, before)

    def test_abort_releases_only_motors_this_startup_touched(self):
        answers = iter(['', 'q'])
        with redirect_stdout(io.StringIO()):
            completed = walkthrough(self.start, lambda message, monitor: next(answers))
        self.assertFalse(completed)
        self.assertEqual(self.start.held, [1])
        self.start.release()
        self.assertEqual([e[0] for e in self.bus.events if e[1] == 'off'], [1])

    def test_missing_calibration_is_rejected(self):
        self.profile['joints']['elbow_flex']['zero_tick'] = None
        with self.assertRaises(ValueError): validate_profile(self.profile)
        self.assertEqual(self.bus.events, [])

    def test_temperature_spike_requires_two_cool_rechecks_and_no_alarm(self):
        initial = self.bus.read(); initial['temperature_raw'] = 65
        def byte(address): return 35 if address == 63 else 0
        with patch.object(self.bus, 'read', return_value=initial), \
             patch.object(self.bus, 'byte', side_effect=byte), \
             patch.object(self.bus, 'word', return_value=35 << 8):
            self.assertTrue(self.start.state(1)['temperature_spike_unconfirmed'])

    def test_confirmed_high_temperature_still_stops_startup(self):
        initial = self.bus.read(); initial['temperature_raw'] = 65
        with patch.object(self.bus, 'read', return_value=initial), \
             patch.object(self.bus, 'byte', side_effect=lambda address: 65 if address == 63 else 0), \
             patch.object(self.bus, 'word', return_value=65 << 8):
            for _ in range(3): self.start.state(1)
            with self.assertRaisesRegex(ValueError, 'temperature limit reached'):
                self.start.state(1)

    def test_hold_failure_reports_measured_error_and_torque(self):
        with redirect_stdout(io.StringIO()): self.start.lock_motor(1)
        self.bus.motors[1].position = 2060
        with self.assertRaisesRegex(RuntimeError, r'error=\+12 ticks.*torque=1'):
            self.start.monitor()

    def test_six_tick_residual_is_accepted_without_disabling_torque(self):
        with redirect_stdout(io.StringIO()): self.start.lock_motor(1)
        self.bus.motors[1].position = 2054
        self.start.monitor()
        self.assertEqual(self.bus.motors[1].torque, 1)

    def test_single_small_hold_excursion_recovers_without_new_goals(self):
        with redirect_stdout(io.StringIO()): self.start.lock_motor(1)
        original = self.start.state
        reads = []
        def state(servo_id, full=False):
            result = original(servo_id, full)
            if not reads: result['position'] += 12
            reads.append(result)
            return result
        before = list(self.bus.events)
        with patch.object(self.start, 'state', side_effect=state): self.start.monitor()
        self.assertEqual(len(reads), 2)
        self.assertEqual(self.bus.events, before)

    def test_torque_loss_stops_immediately_without_rechecks(self):
        with redirect_stdout(io.StringIO()): self.start.lock_motor(1)
        self.bus.motors[1].torque = 0
        before = self.clock.now
        with self.assertRaisesRegex(RuntimeError, 'torque=0'): self.start.monitor()
        self.assertEqual(self.clock.now, before)

    def test_preflight_will_not_take_over_unknown_powered_motors(self):
        self.bus.motors[2].torque = 1
        with self.assertRaises(ValueError): self.start.preflight()
        self.assertEqual(self.bus.events, [])


@unittest.skipUnless((ROOT/'install/setup.bash').exists() and (ROOT/'.venv/bin/python').exists(),
                     'Requires built ROS workspace and servo SDK environment')
class StartupIntegrationTests(unittest.TestCase):
    def test_startup_fault_preserves_hold_until_operator_releases(self):
        self.check_fault_hold(False)

    def test_ros_handoff_fault_preserves_holds_until_operator_exits(self):
        self.check_fault_hold(True)

    def check_fault_hold(self, handoff):
        folder = Path(tempfile.mkdtemp(prefix='pinchy-startup-hold-test-'))
        ready, report = folder/'ready.json', folder/'report.json'
        initial = folder/'initial.json'
        initial.write_text(json.dumps({str(i): 2058 for i in range(1, 7)}))
        emulator = subprocess.Popen(['/usr/bin/python3', str(ROOT/'tools/servo_bus_emulator.py'),
            str(ready), str(report), str(initial)], start_new_session=True)
        process = None
        try:
            deadline = time.monotonic()+10
            while not ready.exists() and time.monotonic() < deadline: time.sleep(0.05)
            saved = profile(Bus()); saved['port'] = json.loads(ready.read_text())['port']
            calibration = folder/'arm.json'; calibration.write_text(json.dumps(saved))
            output = folder/'terminal.txt'
            with output.open('w') as handle:
                options = ['--no-gui', '--domain', str(60+os.getpid()%10)] if handoff else ['--wizard-only']
                process = subprocess.Popen([str(ROOT/'.venv/bin/python'), str(ROOT/'tools/start_arm.py'),
                    '--hardware', *options, '--calibration', str(calibration),
                    '--log', str(folder/'startup.jsonl')], stdin=subprocess.PIPE,
                    stdout=handle, stderr=subprocess.STDOUT, text=True, start_new_session=True)
                process.stdin.write('\n'*(6 if handoff else 1)); process.stdin.flush()
                marker = 'ID 6 locked' if handoff else 'ID 1 locked'
                deadline = time.monotonic()+20
                while time.monotonic() < deadline and process.poll() is None:
                    if marker in output.read_text(): break
                    time.sleep(0.05)
                self.assertIn(marker, output.read_text())
                temporary = folder/'fault.tmp'; temporary.write_text(json.dumps(dict(servo=6 if handoff else 1, address=63, value=70)))
                temporary.replace(folder/'fault.json')
                marker = 'SAFETY HOLD: movement is blocked' if handoff else 'SAFETY HOLD: no further movement'
                deadline = time.monotonic()+20
                while time.monotonic() < deadline and process.poll() is None:
                    if marker in output.read_text(): break
                    time.sleep(0.05)
                self.assertIn(marker, output.read_text())
                time.sleep(0.2); self.assertIsNone(process.poll())
                release_at = time.time()
                if handoff: process.send_signal(signal.SIGINT)
                else: process.stdin.write('q\n'); process.stdin.flush()
                process.wait(timeout=15)
                process.stdin.close()
            self.assertEqual(process.returncode, 0 if handoff else 1)
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT); process.wait(timeout=10)
            emulator.send_signal(signal.SIGINT); emulator.wait(timeout=10)
        result = json.loads(report.read_text())
        self.assertEqual(result['eeprom_writes'], 0)
        self.assertTrue(all(value == 0 for value in result['torque'].values()))
        self.assertTrue(all(event['time'] >= release_at for event in result['torque_events'] if event['value'] == 0))

    def test_live_terminal_auto_capture_and_continuous_ros_handoff(self):
        self.live_dashboard(False)

    def test_ros_cli_keeps_terminal_input_and_powered_handoff(self):
        self.live_dashboard(True)

    def live_dashboard(self, via_ros):
        folder = Path(tempfile.mkdtemp(prefix='pinchy-dashboard-test-'))
        ready, report = folder/'ready.json', folder/'report.json'
        initial = folder/'initial.json'
        initial.write_text(json.dumps({str(i): 2058 for i in range(1, 7)}))
        emulator = subprocess.Popen(['/usr/bin/python3', str(ROOT/'tools/servo_bus_emulator.py'),
            str(ready), str(report), str(initial)], start_new_session=True)
        process = None
        master, slave = pty.openpty()
        fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 24, 120, 0, 0))
        transcript = bytearray()
        events = folder/'startup.jsonl'
        def drain():
            while select.select([master], [], [], 0)[0]:
                try: transcript.extend(os.read(master, 65536))
                except OSError: break
        def log():
            return [json.loads(line) for line in events.read_text().splitlines()] if events.exists() else []
        try:
            deadline = time.monotonic()+10
            while not ready.exists() and time.monotonic() < deadline: time.sleep(0.05)
            saved = profile(Bus()); saved.update(port=json.loads(ready.read_text())['port'], torque_limit=1000)
            calibration = folder/'arm.json'; calibration.write_text(json.dumps(saved))
            command = [str(ROOT/'.venv/bin/python'), str(ROOT/'tools/start_arm.py')]
            if via_ros:
                command = ['/bin/bash', '-c', 'source /opt/ros/jazzy/setup.bash\n'
                    'source "$1/install/setup.bash"\nshift\nexec ros2 run pinchy_bringup start_arm "$@"',
                    'ros-startup-test', str(ROOT)]
            process = subprocess.Popen([*command,
                '--hardware', '--calibration', str(calibration), '--domain', str(80+os.getpid()%10),
                '--no-gui', '--log', str(events)], stdin=slave, stdout=slave, stderr=slave,
                env=dict(os.environ, TERM='xterm-256color'), start_new_session=True)
            deadline = time.monotonic()+15
            while time.monotonic() < deadline and process.poll() is None:
                drain()
                if b'Capture window' in transcript: break
                time.sleep(0.05)
            self.assertIn(b'Capture window', transcript)
            time.sleep(2.2); drain()
            self.assertFalse(any(e['event'] == 'locking' for e in log()))
            os.write(master, b's') # One key for all six, no Enter presses
            deadline = time.monotonic()+55
            while time.monotonic() < deadline and process.poll() is None:
                drain()
                if b'READY: ROS owns the arm' in transcript: break
                time.sleep(0.05)
            self.assertIn(b'READY: ROS owns the arm', transcript, transcript.decode(errors='replace'))
            self.assertIn(b'\x1b[33m', transcript) # Yellow while locking
            self.assertIn(b'\x1b[32m', transcript) # Green when held
            self.assertEqual([e['id'] for e in log() if e['event'] == 'locked'], [1,2,3,4,5,6])
            adopted_at = datetime.fromisoformat(next(e['time'] for e in log() if e['event'] == 'ros_ready')).timestamp()
            os.killpg(process.pid, signal.SIGINT); process.wait(timeout=15)
            self.assertEqual(process.returncode, 0)
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
            drain(); (folder/'terminal.txt').write_bytes(transcript)
            os.close(master); os.close(slave)
            emulator.send_signal(signal.SIGINT); emulator.wait(timeout=10)
        result = json.loads(report.read_text())
        self.assertEqual(result['eeprom_writes'], 0)
        self.assertTrue(all(value == 0 for value in result['torque'].values()))
        self.assertTrue(all(e['time'] >= adopted_at for e in result['torque_events'] if e['value'] == 0))
        print(f'Live dashboard evidence: {folder}', flush=True)

    def test_terminal_exit_releases_motor_held_during_walkthrough(self):
        for termination in (signal.SIGTERM, signal.SIGHUP):
            with self.subTest(signal=termination):
                folder = Path(tempfile.mkdtemp(prefix='pinchy-startup-exit-test-'))
                ready, report = folder/'ready.json', folder/'report.json'
                initial = folder/'initial.json'
                initial.write_text(json.dumps({str(i): 2058 for i in range(1, 7)}))
                emulator = subprocess.Popen(['/usr/bin/python3', str(ROOT/'tools/servo_bus_emulator.py'),
                    str(ready), str(report), str(initial)], start_new_session=True)
                process = None
                try:
                    deadline = time.monotonic()+10
                    while not ready.exists() and time.monotonic() < deadline: time.sleep(0.05)
                    saved = profile(Bus()); saved['port'] = json.loads(ready.read_text())['port']
                    calibration = folder/'arm.json'; calibration.write_text(json.dumps(saved))
                    output = folder/'terminal.txt'
                    with output.open('w') as handle:
                        process = subprocess.Popen([str(ROOT/'.venv/bin/python'), str(ROOT/'tools/start_arm.py'),
                            '--hardware', '--wizard-only', '--calibration', str(calibration),
                            '--log', str(folder/'startup.jsonl')], stdin=subprocess.PIPE,
                            stdout=handle, stderr=subprocess.STDOUT, text=True, start_new_session=True)
                        process.stdin.write('\n'); process.stdin.flush()
                        deadline = time.monotonic()+10
                        while time.monotonic() < deadline and process.poll() is None:
                            if 'ID 1 locked' in output.read_text(): break
                            time.sleep(0.05)
                        self.assertIn('ID 1 locked', output.read_text())
                        process.send_signal(termination); process.wait(timeout=10)
                        process.stdin.close()
                    self.assertEqual(process.returncode, 0, output.read_text())
                finally:
                    if process is not None and process.poll() is None:
                        process.send_signal(signal.SIGINT); process.wait(timeout=10)
                    emulator.send_signal(signal.SIGINT); emulator.wait(timeout=10)
                result = json.loads(report.read_text())
                self.assertEqual(result['eeprom_writes'], 0)
                self.assertTrue(all(value == 0 for value in result['torque'].values()))
                self.assertEqual([(e['id'], e['value']) for e in result['torque_events']], [(1, 1), (1, 0)])

    def test_guided_cli_keeps_holds_powered_through_ros_handoff(self):
        folder = Path(tempfile.mkdtemp(prefix='pinchy-guided-startup-test-'))
        ready, report = folder/'bus-ready.json', folder/'bus-report.json'
        initial = folder/'initial.json'
        initial.write_text(json.dumps({str(i): 2058 for i in range(1, 7)}))
        emulator = subprocess.Popen(['/usr/bin/python3', str(ROOT/'tools/servo_bus_emulator.py'),
            str(ready), str(report), str(initial)], start_new_session=True)
        process = None
        try:
            deadline = time.monotonic()+10
            while not ready.exists() and time.monotonic() < deadline: time.sleep(0.05)
            port = json.loads(ready.read_text())['port']
            saved = profile(Bus()); saved['port'] = port
            saved['torque_limit'] = 1000
            saved['joints']['wrist_roll'].update(min_tick=2048, min_radians=0)
            calibration = folder/'arm.json'; calibration.write_text(json.dumps(saved))
            original = calibration.read_bytes()
            events = folder/'startup.jsonl'; output = folder/'terminal.txt'
            with output.open('w') as handle:
                process = subprocess.Popen([str(ROOT/'.venv/bin/python'), str(ROOT/'tools/start_arm.py'),
                    '--hardware', '--calibration', str(calibration), '--domain', str(60+os.getpid()%10),
                    '--no-gui', '--log', str(events)], stdin=subprocess.PIPE,
                    stdout=handle, stderr=subprocess.STDOUT, text=True, start_new_session=True)
                process.stdin.write('\n'*6); process.stdin.close()
                deadline = time.monotonic()+50
                while time.monotonic() < deadline and process.poll() is None:
                    if 'READY: ROS owns the arm' in output.read_text(): break
                    time.sleep(0.1)
                self.assertIn('READY: ROS owns the arm', output.read_text(), str(folder)+'\n'+output.read_text())
                process.send_signal(signal.SIGINT); process.wait(timeout=15)
            self.assertEqual(process.returncode, 0, output.read_text())
            self.assertEqual(calibration.read_bytes(), original)
            log = [json.loads(line) for line in events.read_text().splitlines()]
            self.assertEqual([e['id'] for e in log if e['event'] == 'locked'], [1,2,3,4,5,6])
            adopted_at = datetime.fromisoformat(next(e['time'] for e in log if e['event'] == 'ros_ready')).timestamp()
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
            emulator.send_signal(signal.SIGINT); emulator.wait(timeout=10)
        result = json.loads(report.read_text())
        self.assertEqual(result['eeprom_writes'], 0)
        self.assertTrue(all(value == 0 for value in result['torque'].values()))
        on_events = [e['id'] for e in result['torque_events'] if e['value'] == 1]
        self.assertEqual(on_events, [1,2,3,4,5,6])
        off_events = [e for e in result['torque_events'] if e['value'] == 0]
        self.assertTrue(off_events)
        self.assertTrue(all(e['time'] >= adopted_at for e in off_events), result['torque_events'])
        print(f'Continuous-hold startup evidence: {folder}', flush=True)


if __name__ == '__main__': unittest.main()
