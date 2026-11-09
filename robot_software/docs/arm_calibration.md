# Capture assembled-arm joint calibration

This tool **reads only**. It never moves motors, enables/releases torque, or
changes RAM/EEPROM. Support the assembled arm with torque off and stop other
serial tools/drivers. Move joints gently; do not force a gearbox or a stop.

## Zero each joint

From `robot_software/`:

```bash
calibrate() { .venv/bin/python tools/calibrate.py --hardware "$@"; }
calibrate
```

It visits motors 1–6 in order. For each, align that joint to its **CAD zero
angle**, hold it still, then press Enter. Other joints need not be at zero.
Use `s` to skip or `q` to stop. Ctrl+C also stops; previously saved joints remain.

**Zero means the physical CAD/reference alignment.** It does not mean encoder
2048 or the midpoint between physical stops. The tool cannot see or judge the
arm's alignment. Encoder readings are sampled five times; unstable readings
are rejected instead of saving an accidental moving position.

To show a fixed all-zero CAD reference in RViz, in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Robot-Arm/robot_software
source install/setup.bash
ROS_DOMAIN_ID=43 ros2 launch pinchy_bringup calibration_reference.launch.py
```

This display has no controller or serial connection. Domain 43 separates its
synthetic joint states from physical robot feedback. For each calibration step,
compare the selected child link **relative to its parent**, rather than trying
to make the entire arm match at once:

| ID | Joint | Parent → child |
| --- | --- | --- |
| 1 | `shoulder_pan` | `base_link` → `shoulder_link` |
| 2 | `shoulder_lift` | `shoulder_link` → `upper_arm_link` |
| 3 | `elbow_flex` | `upper_arm_link` → `forearm_link` |
| 4 | `wrist_flex` | `forearm_link` → `wrist_pitch_link` |
| 5 | `wrist_roll` | `wrist_pitch_link` → `wrist_roll_link` |
| 6 | `pen_pitch` | `wrist_roll_link` → `pen_holder_link` |

## Select, resume or inspect

```bash
calibrate --id 2             # Capture/re-capture only motor 2
calibrate                   # Resume: skip zeros already saved
calibrate --redo            # Revisit all six, including existing zeros
calibrate --status          # Show saved progress; no serial connection
```

Saved data defaults to `robot_software/calibration/arm.json`. It is JSON that
the hardware launch can read as YAML. Each capture saves atomically, preserving
the previous version in `arm.json.bak`. Recapturing a zero clears that joint's
direction/limit entries so older measurements cannot silently be reused.

## Direction checks after zero capture

```bash
calibrate --step direction
calibrate --step direction --id 2
```

For each joint, start from its saved zero, then manually move any comfortable
amount **up to 90° in the positive CAD direction**. No exact angle is needed;
the tool only uses the sign of the encoder change. About 0.7° minimum movement
is needed to distinguish direction reliably. Stay within clear, comfortable
travel; you do not need to reach 90°. Press Enter while holding still. These joint
frames have +Z pointing into the motor from the driven horn face, so positive
joint rotation appears clockwise when viewed directly at that horn face.
The script checks that the encoder change is small and records whether the
encoder increases (`direction: +1`) or decreases (`direction: -1`). It accepts
8–1024 ticks of displacement and rejects tiny changes, movement beyond 90°,
encoder wrap and changed servo configuration.
This step uses the operator's known movement sense; it does not infer that
sense from encoder readings alone.

## Capture conservative usable limits

After zeros and directions are saved:

```bash
calibrate --step limits
calibrate --step limits --id 2
```

For each motor, record **safe position 1** and **safe position 2**, in either
order or direction. You do not need to decide which is the lower/upper limit.
The tool converts both using the saved zero/direction, includes the already
recorded zero position, and takes the minimum and maximum as the initial bounds.
The saved zero is not changed. For example, two positive samples produce a
range from zero to the larger positive angle; opposite-side samples produce a
range between those samples.

Use small positions with clear travel for initial powered checks. About ±5°
can be enough where comfortable; pen pitch still has its CAD −3° lower bound.
These examples do not establish measured clearance on your assembled robot.
Do not push against hard stops or try to discover maximum mechanical travel.

Keep the arm supported. Press `p` to read the current calibrated angle, `s` to
skip the motor, or `q` to save progress and quit. Position 1 saves immediately;
a resumed all-motor session continues at position 2. `--id` deliberately revisits
that motor from position 1; `--redo` revisits completed entries. Partial captures
from the earlier lower/upper workflow can also resume without losing their
first measurement.

The two new samples must differ by at least eight ticks. CAD/EEPROM bounds and
encoder-wrap checks remain active. Bounds are sorted correctly for either
encoder direction. Recapturing zero/direction invalidates older endpoint data.
Recorded safe endpoints do not prove collision clearance everywhere between
them or across arbitrary multi-joint configurations; the range remains subject
to review before powered use.

The file still remains **unconfirmed** afterward. We review it, the arm's
starting pose and support/power-cut readiness before enabling motor holding
or testing small powered movements.

## Flags

| Flag | Meaning |
| --- | --- |
| `--id N` | One motor, 1–6; default visits all incomplete entries |
| `--step zero`, `direction`, or `limits` | Choose measurement stage; default zero |
| `--redo` | Revisit completed entries in an all-motor session |
| `--status` | Display saved progress without accessing motors |
| `--output PATH` | Different calibration file |
| `--port PATH` | Different adapter path; default is our established USB by-ID path |
| `--hardware` | Physical reads; omitted means mock, which cannot overwrite a physical capture |

Without the shell function, run `.venv/bin/python tools/calibrate.py --hardware`
with the same flags. No additional Python dependencies are needed beyond the
existing servo-tool environment. Do not mix mock and physical captures in one file.

## Before enabling assembled-arm motion

The file intentionally keeps `calibration_confirmed: false`; usable limits
remain unset until captured. Zero capture alone does not establish direction, safe travel, tracking
accuracy or load capacity. We still need to review conservative joint/encoder
bounds, encoder-wrap margin, support/power-cut arrangements, and the physical
model alignment before enabling hardware control. Do not reuse the unloaded
bench profile on the assembled arm or try to sweep mechanically to hard stops.

Validation: 18 tests passed, including sequential/selected capture, endpoint
validation/resume, direction-aware limits, invalidation,
configuration/stability rejection and real SDK transactions over a virtual
serial bus. The emulator observed **zero motor writes**. The passive reference
launch published all six positions at zero. No physical zero was recorded
automatically; each requires the operator's alignment and Enter input.
