"""Multi-motor motion, bus errors and grouped packet behavior without hardware."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from servo_chain import ChainBus, MockChainBus, Motor, health, position_motors, release_motors, run_test
from servo_bench import error_text


class RecordingChain(MockChainBus):
    def __init__(self):
        super().__init__([1, 2])
        self.writes = []
        self.releases = []

    def limit(self, servo_id):
        self.writes.append(('limit', servo_id))
        super().limit(servo_id)

    def goals(self, targets, speeds, acceleration):
        self.writes.append(('goals', targets.copy(), speeds.copy()))
        super().goals(targets, speeds, acceleration)

    def enable(self, servo_id):
        self.writes.append(('enable', servo_id))
        super().enable(servo_id)

    def release(self, servo_id):
        self.releases.append(servo_id)
        super().release(servo_id)


MOTORS = [Motor(1, 1000, 3000), Motor(2, 1000, 3000)]


class ChainTests(unittest.TestCase):
    def test_explicit_five_tick_tolerance_accepts_and_reports_four_tick_error(self):
        class Residual(RecordingChain):
            def goals(self, targets, speeds, acceleration):
                super().goals(targets, speeds, acceleration)
                for i, target in targets.items():
                    if target == 2304:
                        self.motors[i].position = target - 4

        bus = Residual()
        result = run_test(bus, MOTORS, pattern='together', tolerance=5)
        self.assertEqual(result['stages'][0]['outward']['errors_ticks'], {1: -4, 2: -4})
        self.assertEqual(result['tolerance_ticks'], 5)
        self.assertTrue(all(s['torque'] == 0 for s in result['after'].values()))
        bus = Residual()
        with patch('servo_chain.time.monotonic', side_effect=range(100)):
            with self.assertRaisesRegex(TimeoutError, 'allowed \\+/-2'):
                run_test(bus, MOTORS, pattern='together', timeout=0.01)
        self.assertEqual(bus.releases, [1, 2])

    def test_invalid_or_excessive_tolerance_does_not_write(self):
        for tolerance in [0, 11, 2.5]:
            bus = RecordingChain()
            with self.assertRaises(ValueError):
                run_test(bus, MOTORS, tolerance=tolerance)
            self.assertEqual(bus.writes, [])
        bus = RecordingChain()
        with self.assertRaisesRegex(ValueError, 'Delta must exceed tolerance'):
            run_test(bus, MOTORS, delta=4, tolerance=5)
        self.assertEqual(bus.writes, [])

    def test_position_uses_explicit_tolerance_and_preserves_real_error(self):
        class Residual(RecordingChain):
            def goals(self, targets, speeds, acceleration):
                super().goals(targets, speeds, acceleration)
                for i, target in targets.items():
                    if target == 2304:
                        self.motors[i].position = target - 4

        result = position_motors(Residual(), [1, 2], 2304, 1000, 3000, tolerance=5)
        self.assertEqual(result['errors_ticks'], {1: -4, 2: -4})
        self.assertEqual(result['tolerance_ticks'], 5)

    def test_release_disables_all_selected_motors_without_goals(self):
        bus = RecordingChain()
        bus.motors[1].torque = bus.motors[2].torque = 1
        result = release_motors(bus, [1, 2])
        self.assertEqual(bus.releases, [1, 2])
        self.assertEqual(bus.writes, [])
        self.assertTrue(all(s['torque'] == 0 for s in result['after'].values()))

    def test_high_temperature_captures_two_rechecks_without_overwriting(self):
        state = dict(position=2048, mode=0, torque=0, voltage_raw=124, temperature_raw=61)
        device = SimpleNamespace(read=Mock(return_value=state),
                                 byte=Mock(side_effect=[0, 35, 0]),
                                 word=Mock(return_value=(35 << 8) | 124))
        bus = ChainBus.__new__(ChainBus)
        bus.bus = device
        result = bus.read([2])[2]
        self.assertEqual(result['temperature_raw'], 61)
        self.assertEqual(result['temperature_recheck_raw'], 35)
        self.assertEqual(result['temperature_block_raw'], 35)
        self.assertTrue(result['temperature_spike_unconfirmed'])
        device.word.assert_called_once_with(62)
        health(result, 2)

    def test_normal_temperature_does_not_add_retry_traffic(self):
        device = SimpleNamespace(read=Mock(return_value=dict(temperature_raw=35)),
                                 byte=Mock(return_value=0), word=Mock())
        bus = ChainBus.__new__(ChainBus)
        bus.bus = device
        result = bus.read([2])[2]
        self.assertNotIn('temperature_recheck_raw', result)
        device.byte.assert_called_once_with(65)
        device.word.assert_not_called()

    def test_cool_motor_alarm_reports_actual_cause_and_readings(self):
        state = dict(mode=0, voltage_raw=124, temperature_raw=35, status_raw=32)
        with self.assertRaisesRegex(ValueError, 'load/overload; temperature=35 C, voltage=12.4 V, status=0x20'):
            health(state, 2)

    def test_temperature_limit_reports_temperature_without_inventing_alarm(self):
        state = dict(mode=0, voltage_raw=124, temperature_raw=61, status_raw=0)
        with self.assertRaisesRegex(ValueError, 'temperature limit reached.*temperature=61 C.*status=0x00'):
            health(state, 2)

    def test_alarm_decoder_preserves_unknown_bits_and_multiple_faults(self):
        state = dict(mode=0, voltage_raw=124, temperature_raw=35, status_raw=145)
        with self.assertRaisesRegex(ValueError, 'voltage, unknown bits 0x90'):
            health(state, 2)

    def test_grouped_goal_write_does_not_enable_mock_torque(self):
        bus = MockChainBus([1, 2])
        bus.goals({1: 2304, 2: 1792}, {1: 300, 2: 300}, 1)
        self.assertEqual(bus.motors[1].goal, 2304)
        self.assertEqual(bus.motors[1].position, 2048)
        self.assertEqual(bus.motors[1].torque, 0)

    def test_current_goals_precede_explicit_enable_and_target(self):
        bus = RecordingChain()
        bus.motors[1].position, bus.motors[2].position = 3838, 224
        position_motors(bus, [1, 2], 2048, 0, 4095)
        hold = bus.writes.index(('goals', {1: 3838, 2: 224}, {1: 300, 2: 300}))
        target = bus.writes.index(('goals', {1: 2048, 2: 2048}, {1: 300, 2: 300}))
        for servo_id in [1, 2]:
            enabled = bus.writes.index(('enable', servo_id))
            self.assertLess(hold, enabled)
            self.assertLess(enabled, target)

    def test_dropped_current_goal_prevents_enable_and_target(self):
        class DroppedGoal(RecordingChain):
            def goals(self, targets, speeds, acceleration):
                self.writes.append(('goals', targets.copy(), speeds.copy()))

        bus = DroppedGoal()
        with self.assertRaisesRegex(RuntimeError, 'goal readback failed'):
            position_motors(bus, [1, 2], 1800, 1000, 3000)
        self.assertFalse(any(write[0] == 'enable' for write in bus.writes))
        self.assertEqual(bus.releases, [1, 2])

    def test_position_centers_motors_from_opposite_encoder_ends(self):
        bus = RecordingChain()
        bus.motors[1].position = 6
        bus.motors[2].position = 4092
        result = position_motors(bus, [1, 2], 2048, 0, 4095)
        goals = [write[1] for write in bus.writes if write[0] == 'goals']
        self.assertEqual(goals, [{1: 6, 2: 4092}, {1: 2048, 2: 2048}])
        self.assertEqual(result['errors_ticks'], {1: 0, 2: 0})
        self.assertTrue(all(s['position'] == 2048 and s['torque'] == 0
                            for s in result['after'].values()))
        self.assertEqual(bus.releases, [1, 2])

    def test_position_supports_one_motor(self):
        bus = RecordingChain()
        result = position_motors(bus, [1], 1800, 1000, 3000)
        self.assertEqual(result['after'][1]['position'], 1800)
        self.assertEqual(bus.releases, [1])
        self.assertEqual(bus.motors[2].position, 2048)

    def test_position_preflight_failure_prevents_all_writes(self):
        for target, lower, upper in [(3100, 1000, 3000), (1000, 1000, 1000),
                                      (2048, -1, 4095)]:
            bus = RecordingChain()
            with self.assertRaises(ValueError):
                position_motors(bus, [1, 2], target, lower, upper)
            self.assertEqual(bus.writes, [])
        bus = RecordingChain()
        bus.motors[2].position = 0
        with self.assertRaisesRegex(ValueError, '180 degrees'):
            position_motors(bus, [1, 2], 3000, 0, 4095)
        self.assertEqual(bus.writes, [])

    def test_position_stall_releases_every_motor(self):
        class Stalled(RecordingChain):
            def goals(self, targets, speeds, acceleration):
                for servo_id, target in targets.items():
                    self.motors[servo_id].goal = target

        bus = Stalled()
        with patch('servo_chain.time.monotonic', side_effect=range(100)):
            with self.assertRaises(TimeoutError):
                position_motors(bus, [1, 2], 1800, 1000, 3000, timeout=0.01)
        self.assertEqual(bus.releases, [1, 2])

    def test_position_rejects_enabled_motor_before_writes(self):
        bus = RecordingChain()
        bus.motors[2].torque = 1
        with self.assertRaisesRegex(ValueError, 'torque already enabled'):
            position_motors(bus, [1, 2], 2048, 0, 4095)
        self.assertEqual(bus.writes, [])

    def test_all_patterns_and_exact_return_targets(self):
        bus = RecordingChain()
        result = run_test(bus, MOTORS)
        stages = result['stages']
        self.assertEqual([s['stage'] for s in stages],
                         ['individual-1', 'individual-2', 'together', 'opposite', 'mixed'])
        self.assertEqual(stages[0]['targets'], {1: 2304, 2: 2048})
        self.assertEqual(stages[1]['targets'], {1: 2048, 2: 2304})
        self.assertEqual(stages[2]['targets'], {1: 2304, 2: 2304})
        self.assertEqual(stages[3]['targets'], {1: 2304, 2: 1792})
        self.assertEqual(stages[4]['speeds_raw'], {1: 150, 2: 300})
        for stage in stages:
            self.assertEqual({i: s['position'] for i, s in stage['returned']['feedback'].items()},
                             {1: 2048, 2: 2048})
        self.assertEqual(bus.releases, [1, 2])
        self.assertTrue(all(state['torque'] == 0 for state in result['after'].values()))

    def test_any_invalid_future_target_prevents_all_writes(self):
        bus = RecordingChain()
        bus.motors[2].position = 1010
        with self.assertRaisesRegex(ValueError, 'opposite'):
            run_test(bus, MOTORS)
        self.assertEqual(bus.writes, [])
        self.assertEqual(bus.releases, [])

    def test_duplicate_ids_are_rejected(self):
        bus = RecordingChain()
        with self.assertRaises(ValueError):
            run_test(bus, [Motor(1, 1000, 3000), Motor(1, 1000, 3000)])
        self.assertEqual(bus.writes, [])

    def test_missing_motor_prevents_any_write(self):
        bus = RecordingChain()
        del bus.motors[2]
        with self.assertRaises(KeyError):
            run_test(bus, MOTORS)
        self.assertEqual(bus.writes, [])

    def test_low_voltage_at_one_motor_prevents_any_write(self):
        class LowVoltage(RecordingChain):
            def read(self, ids, full=False):
                states = super().read(ids, full)
                states[2]['voltage_raw'] = 51
                return states

        bus = LowVoltage()
        with self.assertRaisesRegex(ValueError, '5.1 V'):
            run_test(bus, MOTORS)
        self.assertEqual(bus.writes, [])

    def test_mismatched_speed_units_prevent_any_write(self):
        class DifferentUnits(RecordingChain):
            def read(self, ids, full=False):
                states = super().read(ids, full)
                if full:
                    states[2]['phase_raw'] = 4
                return states

        bus = DifferentUnits()
        with self.assertRaisesRegex(ValueError, 'different speed units'):
            run_test(bus, MOTORS)
        self.assertEqual(bus.writes, [])

    def test_group_send_failure_releases_every_motor(self):
        class SendFailure(RecordingChain):
            def goals(self, targets, speeds, acceleration):
                super().goals(targets, speeds, acceleration)
                if targets[1] != 2048:
                    raise RuntimeError('Packet failed')

        bus = SendFailure()
        with self.assertRaisesRegex(RuntimeError, 'Packet failed') as caught:
            run_test(bus, MOTORS)
        self.assertIn('individual-1/outward; return-to-start command not attempted', error_text(caught.exception))
        goals = [write for write in bus.writes if write[0] == 'goals']
        self.assertEqual(len(goals), 2)  # Startup hold, outward; no return goal.
        self.assertEqual(bus.releases, [1, 2])
        self.assertTrue(all(m.torque == 0 for m in bus.motors.values()))

    def test_return_failure_is_reported_separately(self):
        class ReturnFailure(RecordingChain):
            def goals(self, targets, speeds, acceleration):
                super().goals(targets, speeds, acceleration)
                if len([w for w in self.writes if w[0] == 'goals']) == 3:
                    raise RuntimeError('Return packet failed')

        bus = ReturnFailure()
        with self.assertRaisesRegex(RuntimeError, 'Return packet failed') as caught:
            run_test(bus, MOTORS)
        self.assertIn('individual-1/return; return-to-start command attempted', error_text(caught.exception))
        self.assertEqual(bus.releases, [1, 2])

    def test_alarm_during_movement_releases_every_motor(self):
        class Alarm(RecordingChain):
            def read(self, ids, full=False):
                states = super().read(ids, full)
                if self.motors[1].position != 2048:
                    states[2]['status_raw'] = 1
                return states

        bus = Alarm()
        with self.assertRaisesRegex(ValueError, 'servo alarm: voltage'):
            run_test(bus, MOTORS)
        self.assertEqual(bus.releases, [1, 2])

    def test_timeout_releases_every_motor(self):
        class Stalled(RecordingChain):
            def goals(self, targets, speeds, acceleration):
                for servo_id, target in targets.items():
                    self.motors[servo_id].goal = target

        bus = Stalled()
        # Increasing clock values let the initial hold pass, then time out a leg.
        with patch('servo_chain.time.monotonic', side_effect=range(100)):
            with self.assertRaises(TimeoutError):
                run_test(bus, MOTORS, timeout=0.01)
        self.assertEqual(bus.releases, [1, 2])

    def test_interrupt_releases_every_motor(self):
        class Interrupted(RecordingChain):
            def goals(self, targets, speeds, acceleration):
                super().goals(targets, speeds, acceleration)
                raise KeyboardInterrupt()

        bus = Interrupted()
        with self.assertRaises(KeyboardInterrupt):
            run_test(bus, MOTORS)
        self.assertEqual(bus.releases, [1, 2])

    def test_one_release_failure_does_not_skip_other_motors(self):
        class ReleaseFailure(RecordingChain):
            def release(self, servo_id):
                super().release(servo_id)
                if servo_id == 1:
                    raise RuntimeError('Disconnected')

        bus = ReleaseFailure()
        with self.assertRaisesRegex(RuntimeError, 'cut motor power'):
            run_test(bus, MOTORS)
        self.assertEqual(bus.releases, [1, 2])

    def test_explicit_mixed_speed_limits_checked_before_writes(self):
        for speeds in [[300], [300, 1001], [0, 300]]:
            bus = RecordingChain()
            with self.assertRaises(ValueError):
                run_test(bus, MOTORS, mixed_speeds=speeds)
            self.assertEqual(bus.writes, [])

    def test_goals_are_queued_then_sent_in_one_group_packet(self):
        sdk = Mock()
        sdk.SyncWritePosEx.return_value = True
        sdk.groupSyncWrite.txPacket.return_value = 0
        bus = ChainBus.__new__(ChainBus)
        bus.bus = SimpleNamespace(sdk=sdk, check=Mock())
        bus.goals({1: 2304, 2: 1792}, {1: 300, 2: 600}, 1)
        self.assertEqual(sdk.SyncWritePosEx.call_args_list,
                         [((1, 2304, 300, 1),), ((2, 1792, 600, 1),)])
        sdk.groupSyncWrite.txPacket.assert_called_once_with()
        self.assertEqual(sdk.groupSyncWrite.clearParam.call_count, 2)

    def test_queue_failure_does_not_transmit_partial_group(self):
        sdk = Mock()
        sdk.SyncWritePosEx.side_effect = [True, False]
        bus = ChainBus.__new__(ChainBus)
        bus.bus = SimpleNamespace(sdk=sdk, check=Mock())
        with self.assertRaises(RuntimeError):
            bus.goals({1: 2304, 2: 1792}, {1: 300, 2: 300}, 1)
        sdk.groupSyncWrite.txPacket.assert_not_called()
        self.assertEqual(sdk.groupSyncWrite.clearParam.call_count, 2)


if __name__ == '__main__':
    unittest.main()
