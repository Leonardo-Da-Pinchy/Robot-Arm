# Faster startup and capture without an extra angle window

November 17, 2026. The owner requested faster zeroing, then explicitly clarified
that removing the startup range means removing **only the extra ±90° cap**,
while retaining saved/CAD limits.

- Zeroing ramp: **0.225 rad/s ≈12.9 deg/s**, twice the previous 0.1125 rad/s.
  Each joint's configured velocity still caps it. The minimum short-move
  duration remains 0.35 s.
- Default capture: the saved interval clipped to CAD bounds, retaining the
  existing tiny near-zero settling allowance. No additional angle cap.
- Automatic capture still requires **one second still**, after the existing
  explicit arming key. Noise/trend checks are unchanged.
- Optional `--max-start-degrees 90` restores the former extra cap;
  `--max-start-degrees 25` adds a narrower one. An explicit cap must be finite
  and in (0,180]. It cannot expand saved/CAD limits.
- Raw servo startup caps stay 150 speed / 8 acceleration. Existing torque
  allowance, thermal confirmation, lag/stall pauses, powered prior holds and
  ROS handoff remain unchanged. Calibration files are not edited.

**All 37 startup tests passed in 152.43 seconds**, including:

- Command-timing test: approximately 20° zeroing in 1.6 virtual seconds when the
  configured speed permits it; the lower-profile-speed regression remains 3.5 s.
- Capture at approximately 100° inside a mock saved ±105° interval; an explicit
  90° cap rejects the same pose before any write.
- Saved-range/CAD rejection, invalid option rejection, stationary capture,
  moving/jittering encoders, current-goal staging, existing-hold monitoring,
  temporary tracking lag, stalls, torque loss and confirmed temperatures.
- Real terminal/curses + PTY servo-emulator startup from 100°, all six sequential
  locks, live ROS handoff with no torque release, and eventual explicit exit.
- Direct CLI, ROS CLI and Cartesian startup entry points, powered runtime fault
  holds, handoff faults and explicit release/terminal exit regressions.

The actual wide-start PTY test began at tick 3186 on joint 1 (about 100°) and
retained normal saved/CAD enforcement. That synthetic test profile did not
modify the owner's profile. Its archived [bus report](wide_start_bus_report.json)
records no EEPROM writes. Full [startup events](startup.jsonl.gz), compressed
[terminal capture](terminal.txt.gz), [test results/artifact paths](results.json)
and [source identities](source_hashes.json) are retained.

Reproduce from `robot_software/`:

```bash
PYTHONPATH=tools .venv/bin/python -m unittest test_start_arm
```

Tests use mocks and newly created pseudo-terminals, never physical USB motors.
No physical arm was moved or restarted during this change. The ordinary `arm`
and `motion` startup entry points pick up the updated tools on their next run;
no new startup flag is required.
