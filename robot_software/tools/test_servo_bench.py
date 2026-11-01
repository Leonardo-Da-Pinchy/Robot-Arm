"""Command guards and failure handling without connected hardware."""
import unittest
from unittest.mock import patch

from servo_bench import MockBus, move_relative


class RecordingBus(MockBus):
    def __init__(self):
        super().__init__()
        self.commands = []

    def move(self, position, speed, acceleration):
        self.commands.append(position)


class BenchTests(unittest.TestCase):
    def test_invalid_targets_do_not_write(self):
        for delta, lower, upper in [(65, 2000, 2100), (20, 2000, 2050),
                                     (20, 2050, 2100)]:
            bus = RecordingBus()
            with self.assertRaises(ValueError):
                move_relative(bus, delta, lower, upper, 30, 5, 1)
            self.assertEqual(bus.commands, [])

    def test_disabled_torque_does_not_write(self):
        bus = RecordingBus()
        bus.torque = 0
        with self.assertRaises(ValueError):
            move_relative(bus, 20, 2000, 2100, 30, 5, 1)
        self.assertEqual(bus.commands, [])

    def test_timeout_attempts_hold_at_measured_position(self):
        bus = RecordingBus()
        with patch('servo_bench.time.monotonic', side_effect=[0, 2]):
            with self.assertRaises(TimeoutError):
                move_relative(bus, 20, 2000, 2100, 30, 5, 1)
        self.assertEqual(bus.commands, [2068, 2048])


if __name__ == '__main__':
    unittest.main()
