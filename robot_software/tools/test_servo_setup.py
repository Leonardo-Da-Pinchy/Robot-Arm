"""Setup and motion failure tests without physical motors."""
import unittest

from servo_setup import MockSetupBus, configure, motion_test


class RecordingBus(MockSetupBus):
    def __init__(self):
        super().__init__()
        self.writes = []
        self.voltage = 120
        self.responders = None
        self.fail_motion = False
        self.fail_id = False

    def read(self):
        return dict(super().read(), voltage_raw=self.voltage)

    def scan(self):
        return self.responders if self.responders is not None else super().scan()

    def write_byte(self, address, value):
        self.writes.append(('byte', address, value))
        super().write_byte(address, value)

    def set_id(self, new_id):
        self.writes.append(('id', new_id))
        if self.fail_id:
            raise RuntimeError('ID write failed')
        super().set_id(new_id)

    def set_torque_limit(self, value):
        self.writes.append(('limit', value))
        super().set_torque_limit(value)

    def move(self, position, speed, acceleration):
        self.writes.append(('goal', position))
        if self.fail_motion and position != self.position:
            raise RuntimeError('Movement write failed')
        super().move(position, speed, acceleration)

    def enable(self):
        self.writes.append(('enable',))
        super().enable()

    def release(self):
        self.writes.append(('release',))
        super().release()


class SetupTests(unittest.TestCase):
    def test_configure_current_settings_does_not_rewrite_eeprom(self):
        bus = RecordingBus()
        self.assertEqual(configure(bus, 1)['changes'], [])
        self.assertEqual(bus.writes, [])

    def test_changes_id_and_mode_then_relocks_without_changing_zero(self):
        bus = RecordingBus()
        bus.mode = 1
        result = configure(bus, 2)
        self.assertEqual(result['after']['stored_id'], 2)
        self.assertEqual(result['after']['mode'], 0)
        self.assertEqual(result['after']['eeprom_lock'], 1)
        self.assertEqual(bus.writes, [('byte', 55, 0), ('byte', 33, 0),
                                      ('id', 2), ('byte', 55, 1)])

    def test_configuration_failure_relocks(self):
        bus = RecordingBus()
        bus.fail_id = True
        with self.assertRaisesRegex(RuntimeError, 'ID write failed'):
            configure(bus, 2)
        self.assertEqual(bus.lock, 1)

    def test_multiple_servo_response_or_torque_on_prevents_configuration(self):
        for multiple in [False, True]:
            bus = RecordingBus()
            if multiple:
                bus.responders = [{'servo_id': 1}, {'servo_id': 2}]
            else:
                bus.torque = 1
            with self.assertRaises(ValueError):
                configure(bus, 2)
            self.assertEqual(bus.writes, [])

    def test_usb_voltage_blocks_all_motor_writes(self):
        bus = RecordingBus()
        bus.voltage = 51
        with self.assertRaisesRegex(ValueError, '5.1 V'):
            motion_test(bus, 20, 2000, 2100)
        self.assertEqual(bus.writes, [])

    def test_fine_speed_units_select_working_default(self):
        class FineUnits(RecordingBus):
            def setup_state(self):
                return dict(super().setup_state(), phase_raw=12)

        bus = FineUnits()
        self.assertEqual(motion_test(bus, 20, 2000, 2100)['speed_raw'], 50)
        bus = FineUnits()
        with self.assertRaisesRegex(ValueError, 'Fine speed units'):
            motion_test(bus, 20, 2000, 2100, speed=5)
        self.assertEqual(bus.writes, [])

    def test_boundary_crossing_rejected_before_enable(self):
        bus = RecordingBus()
        bus.position = 4092
        with self.assertRaises(ValueError):
            motion_test(bus, 20, 4000, 4095)
        self.assertEqual(bus.writes, [])

    def test_current_pose_goal_precedes_enable_and_round_trip_releases(self):
        bus = RecordingBus()
        result = motion_test(bus, 20, 2000, 2100)
        self.assertLess(bus.writes.index(('goal', 2048)), bus.writes.index(('enable',)))
        self.assertIn(('goal', 2068), bus.writes)
        self.assertEqual(result['after']['position'], 2048)
        self.assertEqual(result['after']['torque'], 0)
        self.assertEqual(bus.writes[-1], ('release',))

    def test_larger_move_returns_to_original_position(self):
        bus = RecordingBus()
        bus.position = 4075
        result = motion_test(bus, -512, 3000, 4095, timeout=20)
        self.assertEqual(result['target_ticks'], 3563)
        self.assertIn(('goal', 3563), bus.writes)
        self.assertEqual(result['returned']['position'], 4075)
        self.assertEqual(result['after']['torque'], 0)

    def test_large_target_outside_interval_does_not_write(self):
        bus = RecordingBus()
        bus.position = 4075
        with self.assertRaises(ValueError):
            motion_test(bus, -512, 4000, 4095)
        self.assertEqual(bus.writes, [])

    def test_return_uses_original_target_despite_arrival_error(self):
        class ArrivalError(RecordingBus):
            def move(self, position, speed, acceleration):
                super().move(position, speed, acceleration)
                if position == 1024:
                    self.position = 1022

        bus = ArrivalError()
        result = motion_test(bus, -1024, 0, 4095)
        self.assertEqual(result['returned']['position'], 2048)

    def test_motion_error_attempts_release(self):
        bus = RecordingBus()
        bus.fail_motion = True
        with self.assertRaisesRegex(RuntimeError, 'Movement write failed'):
            motion_test(bus, 20, 2000, 2100)
        self.assertEqual(bus.torque, 0)
        self.assertEqual(bus.writes[-1], ('release',))

    def test_release_failure_reports_uncertain_motor_state(self):
        class BrokenRelease(RecordingBus):
            def release(self):
                raise RuntimeError('Disconnected')

        with self.assertRaisesRegex(RuntimeError, 'cut motor power'):
            motion_test(BrokenRelease(), 20, 2000, 2100)

    def test_release_must_be_confirmed_by_feedback(self):
        class StuckTorque(RecordingBus):
            def release(self):
                pass

        with self.assertRaisesRegex(RuntimeError, 'readback failed'):
            motion_test(StuckTorque(), 20, 2000, 2100)

    def test_invalid_motion_parameters_do_not_write(self):
        for kwargs in [dict(delta=0), dict(delta=1025), dict(speed=0), dict(speed=1001),
                       dict(acceleration=0), dict(timeout=0)]:
            bus = RecordingBus()
            args = dict(delta=20, lower=2000, upper=2100)
            args.update(kwargs)
            with self.assertRaises(ValueError):
                motion_test(bus, **args)
            self.assertEqual(bus.writes, [])

    def test_maximum_speed_1000_is_accepted(self):
        bus = RecordingBus()
        result = motion_test(bus, 20, 2000, 2100, speed=1000)
        self.assertEqual(result['speed_raw'], 1000)
        self.assertEqual(result['after']['torque'], 0)


if __name__ == '__main__':
    unittest.main()
