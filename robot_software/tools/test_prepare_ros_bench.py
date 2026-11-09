"""Bench zeros are relative encoder values and never arm calibration."""
import copy
import unittest

from prepare_ros_bench import JOINTS, TICK, make_profile
from servo_chain import MockChainBus


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.ids = [1, 2, 3, 4, 5, 6]
        self.bus = MockChainBus(self.ids)
        self.states = self.bus.read(self.ids, full=True)
        for state in self.states.values(): state['model_raw'] = 777

    def test_measured_zeros_narrow_limits_and_no_bus_writes(self):
        self.states[1]['position'] = 2088
        profile = make_profile(self.states, self.ids, '/test-port', True)
        self.assertEqual(profile['profile_kind'], 'unloaded_ros_bench')
        self.assertTrue(profile['calibration_confirmed'])
        self.assertEqual(profile['joints']['shoulder_pan']['zero_tick'], 2088)
        for joint in profile['joints'].values():
            self.assertLessEqual(joint['zero_tick']-joint['min_tick'], 64)
            self.assertLessEqual(joint['max_tick']-joint['zero_tick'], 64)
            self.assertLessEqual(joint['max_radians'], 64*TICK)
            self.assertGreaterEqual(joint['min_radians'], -64*TICK)
        for motor in self.bus.motors.values():
            self.assertEqual(motor.torque, 0); self.assertEqual(motor.goal, 0)

    def test_cad_pen_pitch_lower_limit_is_respected(self):
        profile = make_profile(self.states, self.ids, '/test-port', True)
        self.assertGreaterEqual(profile['joints']['pen_pitch']['min_radians'], -0.05236)

    def test_mock_capture_cannot_claim_hardware_calibration(self):
        self.assertFalse(make_profile(self.states, self.ids, '/test-port', False)['calibration_confirmed'])

    def test_wrap_edges_are_not_crossed(self):
        self.states[1]['position'] = 4; self.states[2]['position'] = 4092
        profile = make_profile(self.states, self.ids, '/test-port', True)
        self.assertEqual(profile['joints']['shoulder_pan']['min_tick'], 0)
        self.assertEqual(profile['joints']['shoulder_lift']['max_tick'], 4095)

    def test_faults_and_wrong_model_reject_profile(self):
        for key, value in [('model_raw', 123), ('torque', 1), ('mode', 1), ('status_raw', 4), ('resolution_raw', 2)]:
            states = copy.deepcopy(self.states); states[1][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                make_profile(states, self.ids, '/test-port', True)


if __name__ == '__main__':
    unittest.main()
