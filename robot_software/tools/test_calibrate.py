"""Verify guided calibration saves measurements without sending motor writes."""
import copy
import json
import math
from pathlib import Path
import tempfile
import signal
import subprocess
import time
import unittest
import xml.etree.ElementTree as ET

from calibrate import (JOINTS, capture, load_profile, new_profile, read_checked,
                       run_session, save_profile, stable_sample, UnstablePosition)
from servo_setup import MockSetupBus
from calibrate import ROOT
from calibrate import capture_limit, run_limits_session, TICK, adjust_zero, run_zero_adjustment


class ReadOnlyBus(MockSetupBus):
    def __init__(self):
        super().__init__()
        self.positions = {i: 2000+i*10 for i in JOINTS}
        self.sequence = None
        self.offset = 85

    def read(self):
        state = super().read()
        state['position'] = self.sequence.pop(0) if self.sequence else self.positions[self.servo_id]
        return state

    def setup_state(self):
        state = super().setup_state()
        state.update(model_raw=777, offset_raw=self.offset)
        return state

    def move(self, *args): raise AssertionError('Motor write during calibration')
    def enable(self, *args): raise AssertionError('Torque write during calibration')
    def release(self, *args): raise AssertionError('Torque write during calibration')
    def write_byte(self, *args): raise AssertionError('Register write during calibration')
    def set_torque_limit(self, *args): raise AssertionError('Register write during calibration')
    def set_id(self, *args): raise AssertionError('EEPROM write during calibration')


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.bus = ReadOnlyBus()
        self.profile = new_profile(True, '/test-port')
        self.quiet = lambda *args: None
        self.sleep = lambda _: None

    def sample(self, servo=1):
        return stable_sample(self.bus, servo, True, self.sleep)

    def session(self, ids, answers, **kwargs):
        iterator = iter(answers)
        saved = []
        run_session(self.bus, self.profile, ids, 'zero', True,
                    lambda value: saved.append(copy.deepcopy(value)),
                    ask=lambda _: next(iterator), output=self.quiet, sleep=self.sleep, **kwargs)
        return saved

    def test_all_six_sequential_captures_read_only(self):
        saved = self.session(list(JOINTS), ['']*6)
        self.assertEqual(len(saved), 6)
        for servo, name in JOINTS.items():
            self.assertEqual(self.profile['joints'][name]['zero_tick'], self.bus.positions[servo])
            self.assertIsNone(self.profile['joints'][name]['direction'])
            self.assertIsNone(self.profile['joints'][name]['min_tick'])
        self.assertFalse(self.profile['calibration_confirmed'])

    def test_select_one_without_touching_other_captures(self):
        self.session([3], [''], single=True)
        self.assertEqual(self.profile['joints']['elbow_flex']['zero_tick'], 2030)
        for servo, name in JOINTS.items():
            if servo != 3: self.assertIsNone(self.profile['joints'][name]['zero_tick'])

    def test_resume_skips_completed_and_quit_keeps_progress(self):
        first = self.session(list(JOINTS), ['', 'q'])
        self.assertEqual(len(first), 1)
        second = self.session(list(JOINTS), ['']*5)
        self.assertEqual(len(second), 5)
        self.assertEqual(self.profile['joints']['shoulder_pan']['zero_tick'], 2010)

    def test_recapture_invalidates_only_affected_joint(self):
        self.session([1, 2], ['', ''])
        joint = self.profile['joints']['shoulder_pan']
        joint.update(direction=1, min_tick=1900, max_tick=2100, min_radians=-0.1, max_radians=0.1)
        self.profile['calibration_confirmed'] = True
        self.bus.positions[1] = 2025
        self.session([1], [''], single=True)
        self.assertEqual(joint['zero_tick'], 2025)
        self.assertIsNone(joint['direction']); self.assertIsNone(joint['min_tick'])
        self.assertEqual(self.profile['joints']['shoulder_lift']['zero_tick'], 2020)
        self.assertFalse(self.profile['calibration_confirmed'])

    def prepared_shoulder(self, direction=1):
        self.bus.positions[2] = 2020
        capture(self.profile, 2, 'zero', self.sample(2))
        self.bus.positions[2] += direction*32
        capture(self.profile, 2, 'direction', self.sample(2))
        self.bus.positions[2] = 1900
        capture_limit(self.profile, 2, 'first', self.sample(2))
        self.bus.positions[2] = 2150
        capture_limit(self.profile, 2, 'second', self.sample(2))

    def test_small_zero_correction_preserves_sign_and_physical_limits(self):
        for direction in (1, -1):
            self.prepared_shoulder(direction)
            others = {name: copy.deepcopy(joint) for name, joint in self.profile['joints'].items() if name != 'shoulder_lift'}
            self.bus.positions[2] = 2040
            adjust_zero(self.profile, 2, self.sample(2))
            joint = self.profile['joints']['shoulder_lift']
            self.assertEqual((joint['zero_tick'], joint['direction']), (2040, direction))
            self.assertEqual((joint['min_tick'], joint['max_tick']), (1900, 2150))
            expected = sorted(direction*(ticks-2040)*TICK for ticks in (1900, 2150))
            self.assertEqual([joint['min_radians'], joint['max_radians']], expected)
            self.assertFalse(self.profile['calibration_confirmed'])
            self.assertEqual(others, {name: joint for name, joint in self.profile['joints'].items() if name != 'shoulder_lift'})

    def test_large_zero_correction_is_rejected_without_mutation(self):
        self.prepared_shoulder()
        before = copy.deepcopy(self.profile)
        self.bus.positions[2] = 2220
        with self.assertRaisesRegex(ValueError, 'exceeds 10 degrees'):
            adjust_zero(self.profile, 2, self.sample(2))
        self.assertEqual(before, self.profile)

    def test_zero_adjustment_session_is_read_only_and_saves_only_on_enter(self):
        self.prepared_shoulder(); self.bus.positions[2] = 2040
        saved = []
        run_zero_adjustment(self.bus, self.profile, 2, True, lambda value: saved.append(copy.deepcopy(value)),
                            ask=lambda _: '', output=self.quiet, sleep=self.sleep)
        self.assertEqual(len(saved), 1)
        self.assertEqual(saved[0]['joints']['shoulder_lift']['zero_tick'], 2040)

    def test_zero_correction_tightens_travel_when_new_reference_would_exceed_cad(self):
        self.prepared_shoulder()
        joint = self.profile['joints']['shoulder_lift']
        model = ET.parse(ROOT/'src/pinchy_description/urdf/pinchy_arm.urdf').getroot()
        lower = float(model.find("joint[@name='shoulder_lift']/limit").get('lower'))
        joint['min_tick'] = math.ceil(joint['zero_tick']+lower/TICK)
        old_min, old_max = joint['min_tick'], joint['max_tick']
        self.bus.positions[2] = 2040
        adjust_zero(self.profile, 2, self.sample(2))
        self.assertGreater(joint['min_tick'], old_min)
        self.assertLessEqual(joint['max_tick'], old_max)
        self.assertGreaterEqual(joint['min_radians'], lower)

    def test_direction_in_both_encoder_senses(self):
        for delta in [8, -8, 32, -32, 512, -512, 1024, -1024]:
            profile = new_profile(True, '/test-port')
            self.bus.positions[1] = 2010
            capture(profile, 1, 'zero', self.sample())
            self.bus.positions[1] += delta
            capture(profile, 1, 'direction', self.sample())
            self.assertEqual(profile['joints']['shoulder_pan']['direction'], 1 if delta > 0 else -1)
            self.assertFalse(profile['calibration_confirmed'])

    def test_direction_rejects_wrap_and_tiny_motion(self):
        capture(self.profile, 1, 'zero', self.sample())
        for position in [2010, 2011, 2017, 3035, 985, 4090, 3]:
            self.bus.positions[1] = position
            with self.assertRaises(ValueError): capture(self.profile, 1, 'direction', self.sample())
        self.assertIsNone(self.profile['joints']['shoulder_pan']['direction'])

    def test_configuration_change_requires_new_zero(self):
        capture(self.profile, 1, 'zero', self.sample())
        self.bus.positions[1] += 32; self.bus.offset += 1
        with self.assertRaises(ValueError): capture(self.profile, 1, 'direction', self.sample())

    def test_moving_joint_not_captured(self):
        self.bus.sequence = [2000, 2010, 2020, 2030, 2040]
        with self.assertRaises(UnstablePosition): self.sample()

    def test_torque_enabled_refused_without_releasing_it(self):
        self.bus.torque = 1
        with self.assertRaises(ValueError): read_checked(self.bus, 1, True)
        self.assertEqual(self.bus.torque, 1)

    def test_atomic_save_backup_and_backend_separation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'arm.json'
            capture(self.profile, 1, 'zero', self.sample()); save_profile(path, self.profile)
            capture(self.profile, 2, 'zero', self.sample(2)); save_profile(path, self.profile)
            self.assertEqual(load_profile(path, True, '/test-port')['joints']['shoulder_lift']['zero_tick'], 2020)
            backup = json.loads(path.with_name('arm.json.bak').read_text())
            self.assertIsNone(backup['joints']['shoulder_lift']['zero_tick'])
            with self.assertRaises(ValueError): load_profile(path, False, '/test-port')
            load_profile(path, False, '/test-port', status_only=True)

    def test_limits_in_both_encoder_directions(self):
        for direction in [1, -1]:
            profile = new_profile(True, '/test-port'); self.bus.positions[1] = 2010
            capture(profile, 1, 'zero', self.sample())
            profile['joints']['shoulder_pan']['direction'] = direction
            self.bus.positions[1] = 2010-direction*32
            capture_limit(profile, 1, 'lower', self.sample())
            self.bus.positions[1] = 2010+direction*64
            capture_limit(profile, 1, 'upper', self.sample())
            joint = profile['joints']['shoulder_pan']
            self.assertAlmostEqual(joint['min_radians'], -32*TICK)
            self.assertAlmostEqual(joint['max_radians'], 64*TICK)
            self.assertEqual(joint['min_tick'], 1978 if direction == 1 else 1946)
            self.assertEqual(joint['max_tick'], 2074 if direction == 1 else 2042)
            self.assertFalse(profile['calibration_confirmed'])

    def test_second_position_requires_first_but_first_can_be_positive(self):
        capture(self.profile, 1, 'zero', self.sample())
        self.profile['joints']['shoulder_pan']['direction'] = 1
        self.bus.positions[1] = 2042
        with self.assertRaises(ValueError): capture_limit(self.profile, 1, 'second', self.sample())
        capture_limit(self.profile, 1, 'first', self.sample())
        self.assertEqual(self.profile['observations']['shoulder_pan']['safe_position_1']['position'], 2042)

    def test_pen_pitch_limit_does_not_expand_cad_range(self):
        capture(self.profile, 6, 'zero', self.sample(6))
        self.profile['joints']['pen_pitch']['direction'] = 1
        self.bus.positions[6] = 2060-64
        with self.assertRaises(ValueError): capture_limit(self.profile, 6, 'lower', self.sample(6))
        self.bus.positions[6] = 2060-16
        capture_limit(self.profile, 6, 'lower', self.sample(6))
        self.assertAlmostEqual(self.profile['observations']['pen_pitch']['safe_position_1']['angle_radians'], -16*TICK)

    def test_safe_positions_sort_in_any_order_and_include_zero(self):
        for direction in [1, -1]:
            for first, second in [(64, -32), (-32, 64), (16, 64), (-64, -16), (0, 32)]:
                with self.subTest(direction=direction, first=first, second=second):
                    profile = new_profile(True, '/test-port'); self.bus.positions[1] = 2010
                    capture(profile, 1, 'zero', self.sample())
                    profile['joints']['shoulder_pan']['direction'] = direction
                    self.bus.positions[1] = 2010+direction*first
                    capture_limit(profile, 1, 'first', self.sample())
                    self.bus.positions[1] = 2010+direction*second
                    capture_limit(profile, 1, 'second', self.sample())
                    joint = profile['joints']['shoulder_pan']
                    self.assertAlmostEqual(joint['min_radians'], min(0, first, second)*TICK)
                    self.assertAlmostEqual(joint['max_radians'], max(0, first, second)*TICK)
                    self.assertEqual(joint['zero_tick'], 2010)
                    self.assertEqual(joint['direction'], direction)
                    self.assertFalse(profile['calibration_confirmed'])

    def test_legacy_partial_endpoint_resumes_as_first_safe_position(self):
        capture(self.profile, 1, 'zero', self.sample())
        self.profile['joints']['shoulder_pan']['direction'] = 1
        self.bus.positions[1] = 1978; capture_limit(self.profile, 1, 'first', self.sample())
        observed = self.profile['observations']['shoulder_pan']
        observed['limit_lower'] = observed.pop('safe_position_1')
        self.bus.positions[1] = 2042
        saved = []
        run_limits_session(self.bus, self.profile, [1], True,
                           lambda p: saved.append(copy.deepcopy(p)), ask=lambda _: '',
                           output=self.quiet, sleep=self.sleep)
        self.assertEqual(len(saved), 1)
        self.assertEqual(self.profile['joints']['shoulder_pan']['min_tick'], 1978)
        self.assertEqual(observed['safe_position_2']['position'], 2042)

    def test_limits_resume_after_lower_capture(self):
        capture(self.profile, 1, 'zero', self.sample())
        self.profile['joints']['shoulder_pan']['direction'] = 1
        saved = []
        actions = iter([('', 1978), ('q', None)])
        def ask(_):
            answer, position = next(actions)
            if position is not None: self.bus.positions[1] = position
            return answer
        run_limits_session(self.bus, self.profile, [1], True,
                           lambda p: saved.append(copy.deepcopy(p)), ask=ask,
                           output=self.quiet, sleep=self.sleep)
        self.assertEqual(len(saved), 1)
        self.assertIsNone(self.profile['joints']['shoulder_pan']['min_tick'])
        self.bus.positions[1] = 2042
        run_limits_session(self.bus, self.profile, [1], True,
                           lambda p: saved.append(copy.deepcopy(p)), ask=lambda _: '',
                           output=self.quiet, sleep=self.sleep)
        self.assertEqual(len(saved), 2)
        self.assertEqual(self.profile['joints']['shoulder_pan']['min_tick'], 1978)

    def test_direction_recapture_invalidates_endpoint_observations(self):
        capture(self.profile, 1, 'zero', self.sample())
        self.profile['joints']['shoulder_pan']['direction'] = 1
        self.bus.positions[1] = 1978; capture_limit(self.profile, 1, 'lower', self.sample())
        self.bus.positions[1] = 2042; capture_limit(self.profile, 1, 'upper', self.sample())
        capture(self.profile, 1, 'direction', self.sample())
        self.assertNotIn('limit_lower', self.profile['observations']['shoulder_pan'])
        self.assertIsNone(self.profile['joints']['shoulder_pan']['min_tick'])


@unittest.skipUnless((ROOT/'.venv/bin/python').is_file(), 'Requires existing servo SDK virtual environment')
class CLIIntegrationTests(unittest.TestCase):
    def test_guided_real_sdk_capture_on_virtual_bus_sends_no_writes(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory); ready, report = folder/'ready.json', folder/'report.json'
            emulator = subprocess.Popen(['/usr/bin/python3', str(ROOT/'tools/servo_bus_emulator.py'),
                str(ready), str(report)], start_new_session=True)
            try:
                deadline = time.monotonic()+10
                while not ready.exists() and time.monotonic() < deadline: time.sleep(0.05)
                port = json.loads(ready.read_text())['port']; path = folder/'arm.json'
                command = [str(ROOT/'.venv/bin/python'), str(ROOT/'tools/calibrate.py'),
                           '--hardware', '--port', port, '--output', str(path)]
                result = subprocess.run(command, input='\n'*6, capture_output=True, text=True, timeout=20)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                profile = json.loads(path.read_text())
                self.assertTrue(all(j['zero_tick'] == 2048 for j in profile['joints'].values()))
                self.assertFalse(profile['calibration_confirmed'])
                result = subprocess.run(command+['--id', '2'], input='\n', capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                self.assertTrue(path.with_name('arm.json.bak').is_file())
            finally:
                emulator.send_signal(signal.SIGINT); emulator.wait(timeout=10)
            state = json.loads(report.read_text())
            self.assertEqual(state['writes'], 0)
            self.assertTrue(all(value == 0 for value in state['torque'].values()))
            # The PTY no longer exists: status must not try to open the bus.
            result = subprocess.run(command+['--status'], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)


if __name__ == '__main__':
    unittest.main()
