# Test motors on the same TTL bus

First program, power-cycle, inspect and label each motor **individually** using
[servo_bench.md](servo_bench.md). Label by its verified bus ID: **1, 2, …, 6**.
Do not chain motors that still share factory ID 1; software cannot reliably
detect duplicate physical IDs.

## 1. Connect two motors

Turn USB and external power off before wiring. Connect the URT-2 TTL port to
motor 1, then motor 1's second bus connector to motor 2. Use the confirmed 12 V
supply. Start with two **unassembled, unloaded motors** with clear shaft travel.
Do not run another servo/ROS program on the same serial port.

From `robot_software/`, using the same `PORT` as the single-motor guide:

```bash
PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078770-if00
chain() { .venv/bin/python tools/servo_chain.py --hardware --port "$PORT" "$@"; }
chain inspect --ids 1 2
```

This reads both IDs without enabling torque. Check both report mode 0, torque 0,
roughly 12 V, and no alarms. Ensure each starting position has room for the
requested movement in **both directions** for the full sequence.

If preflight says torque is already enabled, release the connected unloaded
motors first (this sends no movement goals):

```bash
chain release --ids 1 2
```

For six connected motors, use `chain release --ids 1 2 3 4 5 6`. Release removes
holding effort; assembled mechanisms must be supported before using it.

## 2. Move them to a common starting position

On unloaded motors with clear travel over the selected interval:

```bash
chain position --ids 1 2 --target 2048 --min 0 --max 4095 --speed 300 --timeout 30
chain inspect --ids 1 2
```

`--target 2048` is the absolute encoder midpoint (about 180° from encoder zero).
`--min 0 --max 4095` permits the full single-turn range for this preparation
move. This may rotate a shaft by up to **180°**; use narrower bounds if needed.
Before moving, the command writes and verifies each motor's current-position
goal, then explicitly enables torque. It moves to the target, checks arrival
within `--tolerance` (default two ticks), then disables torque. It does **not** return to the old positions or rewrite encoder offsets,
limits or EEPROM. Equal readings do not guarantee identical horn orientation or
define calibrated arm-joint zero. Released shafts can be displaced afterward.

Use only the IDs physically connected. To prepare all six, replace `--ids 1 2`
with `--ids 1 2 3 4 5 6`. Inspect again before testing.

## 3. Run the sequence

Example only if **both readings are near 2048** and 1000–3000 is mechanically clear:

```bash
chain test --motor 1:1000:3000 --motor 2:1000:3000 --delta 256 --speed 300 --mixed-speeds 150 300
```

It runs these stages, returning to the original positions after each:

1. Motor 1 moves while motor 2 holds; then motor 2 moves while motor 1 holds.
2. Both move in the same encoder direction.
3. They move in opposite encoder directions.
4. Both start together with different requested speeds.

It checks **all stages before any writes**, uses a temporary 30% output limit,
monitors each motor's position, voltage, temperature and alarms, and attempts to
disable torque on every selected motor on completion, failure or Ctrl+C. If a
motor cannot be released because the bus failed, cut motor power. EEPROM and
calibration settings are unchanged.

**If a reading is near 0/4095, this example will be rejected.** Use the positioning
command above and inspect again. Alternatively use a one-direction `individual`
or `together` pattern with suitable signed delta and bounds. The script never
wraps an encoder and rejects preparation moves larger than 180°.

## 4. Expand to six unloaded motors

Power off USB and external power before extending the TTL chain:
**URT-2 → 1 → 2 → 3 → 4 → 5 → 6**. Keep shaft/horn travel clear, then power on.
Stop any standalone watcher before these commands.

Check that all six respond with mode 0, torque 0, roughly 12 V and no alarms:

```bash
chain inspect --ids 1 2 3 4 5 6
```

Prepare a common starting position (up to 180° of motion per unloaded shaft):

```bash
chain position --ids 1 2 3 4 5 6 --target 2048 --min 0 --max 4095 \
  --speed 300 --timeout 30 --tolerance 5 --watch-temps --log /tmp/position-six.jsonl
chain inspect --ids 1 2 3 4 5 6
```

Once all six are near 2048 and 1000–3000 is mechanically clear, run:

```bash
chain test \
  --motor 1:1000:3000 --motor 2:1000:3000 --motor 3:1000:3000 \
  --motor 4:1000:3000 --motor 5:1000:3000 --motor 6:1000:3000 \
  --delta 256 --speed 300 --mixed-speeds 150 300 150 300 150 300 \
  --tolerance 5 --watch-temps --log /tmp/chain-six.jsonl
```

This runs each motor individually, then all together, alternating directions,
and alternating speeds. Each stage returns to its start; the run ends with all
six torque-disabled. The log records any failure and temperature spike for
diagnosis. Passing this unloaded test does not establish loaded-arm supply or
holding capacity.

`--tolerance 5` accepts an arrival error up to five encoder ticks (about 0.44°)
for basic bench motion. This is separate from `--min`/`--max`, which always bound
the permitted travel. The default remains two ticks. Actual signed errors are
reported as `errors_ticks`; success does not claim zero error or arm accuracy.
The movement delta must exceed the tolerance. Startup hold checks retain their
separate two-tick check before any deliberate move.

## Flags

| Flag | Meaning |
| --- | --- |
| `--hardware --port PATH` | Physical bus; omit both for mock operation |
| `--baud N` | Connection baud before `test`/`inspect`; default 1000000 |
| `--ids 1 2` | Selected IDs for `inspect`, `position` or `release` (the latter two also accept one motor) |
| `--target N` | `position`: absolute encoder target 0–4095; 2048 is midrange |
| `--min N --max N` | `position`: absolute allowed interval for every selected motor |
| `--motor ID:MIN:MAX` | Repeat per motor; absolute allowed encoder interval |
| `--pattern NAME` | `individual`, `together`, `opposite`, `mixed`, or `all` (default) |
| `--delta N` | Signed movement, ±4–1024 ticks; default 256 = 22.5° |
| `--speed N` | Raw speed 1–1000; default 300 |
| `--mixed-speeds A B …` | One raw speed per motor in `--motor` order; default alternates half/full speed |
| `--acceleration N` | Raw acceleration 1–10; default 1 |
| `--timeout N` | Seconds allowed per outward/return leg; default 20, maximum 30 |
| `--tolerance N` | `test`/`position`: arrival tolerance 1–10 ticks; default 2; bench example uses 5 |

Opposite direction alternates signs in `--motor` order. Directions describe
encoder changes; clockwise appearance depends on mounting/viewpoint. Raw speed
comparisons require matching speed-unit settings across motors; fine units need
values ≥50. A grouped packet sends all goals together, but does **not** guarantee
identical motion or simultaneous arrival. The different-speed stage is most
visible with enough distance; acceleration can dominate very short moves.

Stage progress prints to stderr; the final JSON report on stdout contains per-
motor feedback and approximate arrival times from polling. These are bench
observations, not precise synchronization or loaded-arm accuracy measurements.
Add motors 3–6 using more `--motor` entries after the two-motor test works.

Progress distinguishes `outward` from `return`. If the outward move fails a
check, the test stops and releases torque instead of commanding the return.
Errors identify the stage/leg and whether a return command was attempted.
`position` intentionally has no return leg; only `test` runs out-and-back stages.

If a run reports a health error, inspect both IDs before retrying. Errors include
the temperature, voltage and status byte, with alarm names such as voltage,
encoder, current or load/overload. An alarm does not necessarily mean the motor
is hot. A high temperature triggers an immediate direct recheck, a
voltage/temperature block read and another alarm-status read. An isolated spike
is logged as unconfirmed and allowed only when **both rechecks are below 60 °C,
agree within 2 °C, and both alarm checks are clear**. Otherwise the test stops.
All original/repeat readings stay in the log; actual alarm flags still stop it.

For continuous temperature output and packet logging, see
[watching servo temperatures](servo_temperature.md). Use `chain watch` while
idle, or add `--watch-temps --log /tmp/chain-motion.jsonl` to the motion command.
Do not run a separate watcher alongside a motion script on the same port.

Mock preview (does not simulate timing):

```bash
python3 tools/servo_chain.py test --motor 1:1000:3000 --motor 2:1000:3000
```

[Manufacturer grouped-write example](https://github.com/ftservo/FTServo_Python/blob/main/sms_sts/sync_write.py)
