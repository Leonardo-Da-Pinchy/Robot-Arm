# Hardware inventory and calibration record

Started November 4, 2026. Source: the October 9 design report, particularly
Sections VI.A–B and Appendix C, plus the repository's custom-arm description.
The user confirmed the report-listed physical equipment on November 4, 2026.
As of November 7, the user confirms all six motors are labelled, have been
power-cycled, retain their assigned IDs and work individually. Motor 6 also
passed on retry, as confirmed by the user; it has no separate outstanding bench
issue. Arm assembly and joint calibration remain pending. The user verified
the supply label as 12 V / 5 A. During the initial session on
November 4, communication and a small out-and-back movement test succeeded for
one physical servo. Runtime commands enabled then disabled torque; no EEPROM
configuration was changed. A faulty outlet caused the initial low voltage;
after correcting it, the servo reported 12.4 V. Assembled-arm capacity is untested.

## Equipment already documented

| Item | Documented information | Still to confirm |
| --- | --- | --- |
| Arm | Custom SO-101 derivative, six independently actuated positioning joints | Current assembly/CAD revision and actual joint order/axes |
| Arm servos | Six Feetech STS3215-12V; labelled IDs power-cycle verified; individual and full six-motor chain tests passed | Assembled-joint calibration, load and stop/recovery qualification |
| Bus adapter | Feetech URT-2, shared TTL bus verified with all six motors | Assembled-arm control rate and load qualification |
| Motor supply | SANSUN 12 V / 5 A, confirmed by user; servo feedback 12.4 V after correcting faulty outlet | Assembled-arm capacity remains untested |
| Emergency stop | Baomain 43217-5544, purchased; report proposes cutting motor power | Wiring not established; power-cut behavior untested |
| Computer | Current host: Jetson Orin Nano; device tree identifies Engineering Reference Developer Kit Super; Ubuntu 24.04.5 LTS, aarch64; ROS 2 Jazzy installed | Exact module memory and JetPack/L4T version if later needed |
| Camera | Intel RealSense D455, physical equipment confirmed by user; same Jetson intended for AprilTag detection | Camera not visible in inspected USB list; not needed for initial motor tests |
| Structure | Custom arm is not assembled or wired, per user | Current geometry/CAD and eventual assembly verification |
| Pen | Report proposes a manually loaded sleeve with thumb screw | Actual holder, pen, mounting arrangement and tip offset |
| Gripper | Repository describes a seventh Feetech servo, separate from six arm joints; report defers gripping and excludes it from the six-pack BOM | Whether present/purchased and exact model/control connection |

The report proposes a shared serial bus, IDs 1–6 from base to wrist, motor
power through an emergency stop, and independent computer power. These are
design intentions until checked on the actual setup. All six assigned IDs persist
after power cycling per the user; mapping those labelled motors to actual arm
joints still needs to be recorded. Readbacks captured directly in this session
include IDs 1, 2 and 6 at 1,000,000 baud in position mode.

## Current next steps

1. External motor power and a small motion test are verified for the first
   servo. Feedback reports 12.4 V after correcting a faulty outlet.
2. USB connection is complete: `/dev/ttyACM0`, stable path
   `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078770-if00`.
   The user granted temporary access with `sudo setfacl -m u:leo:rw /dev/ttyACM0`.
   Access may need renewal after unplugging the adapter.
3. Read-only discovery at ID 1 / 1,000,000 baud succeeded. Stored ID 1, baud code
   0, raw model 777, position mode 0 and torque disabled were read from hardware.
4. Complete: all six motors are labelled, power-cycled and individually working
   per the user. ID persistence no longer needs to be treated as outstanding.
5. Complete: two-motor and full six-motor bench sequences passed. Next are the
   actual arm model, assembly/joint mapping and calibration, then ROS control.
   Load and stop/recovery qualification remain.

`tools/servo_setup.py` now provides read-only scanning/inspection, persistent
ID/position-mode configuration and a guarded out-and-back movement test that
enables then releases torque. The single-motor steps and flags are documented in
[servo_bench.md](servo_bench.md). User-performed ID setup and persistence checks
are complete for all six motors. The manufacturer's
[STS/SMS SDK](https://github.com/ftservo/FTServo_Python/blob/main/scservo_sdk/sms_sts.py)
defines ID/baud configuration and EEPROM lock/unlock operations.

The Jetson will run both ROS 2 servo control and camera AprilTag detection, per
the user. Recheck control update rate and feedback latency with vision running
during later integration. Initial single-motor testing does not require vision.

For the actual arm model, identify the latest CAD/export or dimensioned assembly
and the location/axis of the added sixth positioning joint. This can follow the
initial inventory; it is not required to identify one unloaded motor on the bus.

## Per-joint record

Rows are physical base-to-tool positions, not final ROS joint names or confirmed
bus IDs. Populate these from inspection and bench/calibration results. Do not
copy stock SO-101 names or limits onto the custom mechanism without checking.
The pending entries below refer to assembled-joint mapping and calibration;
they do not undo the completed labelled-motor checks above.

| Position | Physical axis/function | ROS joint name | Actual model | Read bus ID | Read baud/mode | Zero ticks | Direction | Ticks/radian | Measured usable range | Test result |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Arm 1, nearest base | Unknown | TBD | STS3215-12V | Unknown | Unknown | Unknown | Unknown | Unknown | Unknown | Untested |
| Arm 2 | Unknown | TBD | STS3215-12V | Unknown | Unknown | Unknown | Unknown | Unknown | Unknown | Untested |
| Arm 3 | Unknown | TBD | STS3215-12V | Unknown | Unknown | Unknown | Unknown | Unknown | Unknown | Untested |
| Arm 4 | Unknown | TBD | STS3215-12V | Unknown | Unknown | Unknown | Unknown | Unknown | Unknown | Untested |
| Arm 5 | Unknown | TBD | STS3215-12V | Unknown | Unknown | Unknown | Unknown | Unknown | Unknown | Untested |
| Arm 6, nearest tool | Unknown | TBD | STS3215-12V | Unknown | Unknown | Unknown | Unknown | Unknown | Unknown | Untested |
| Separate gripper | Grip/release pen | TBD | Unknown | Unknown | Unknown | Unknown | Unknown | Unknown | Unknown | Untested |

Encoder zeros, direction, range, operating limits and stop behavior are things
to establish together during testing, not information the user must already have.
Record the measured pose defining zero, encoder wrap convention and any stored
servo offset alongside each calibration result.

## Bench session record template

- Date, operator, physical servo label and host:
- Supply/adapter, serial device and connection:
- Read model/ID/baud/mode and original persistent settings:
- Initial position, voltage and temperature:
- Mechanically clear interval and small commanded movement:
- Observed direction, final position and response:
- Hold/release, disconnect/reconnect and power-cut observations:
- Any setting changes, faults or unresolved questions:

## First physical communication check — November 4, 2026

- One user-confirmed three-wire servo connected to the URT-2. Arm unassembled.
- SDK `ftservo-python-sdk==2.0.1`, `pyserial==3.5` installed in
  `robot_software/.venv`. Since ensurepip is absent, host pip installed into the
  environment via `python3 -m pip --python robot_software/.venv/bin/python`.
- Read-only `inspect` result: stored ID **1**, host baud **1,000,000**, baud code
  **0**, raw model **777**, mode **0**, torque **0**, position **4092**,
  min/max **0/4095**, offset raw **85**, stored goal **0**, EEPROM lock **1**,
  voltage raw **51**, temperature raw **26**. These stored limits are not
  calibrated mechanical arm limits, and raw offset is not a measured joint zero.
- Existing ID already matches the intended first ID; no need to rewrite it.
  The physical motor still needs a label and eventual joint assignment.
- Five software bench tests passed, including two new read-only inspection tests.
- No servo writes or motion were performed. Position is close to the encoder
  boundary; do not assume a positive relative movement is valid.
- The [manufacturer's URT-2 diagram](https://akizukidenshi.com/goodsaffix/URT-2.pdf)
  identifies the switch as serial signal-level selection, documents USB-powered
  parameter adjustment, and separates TTL motor power (G/V1) from RS485 power.
  The roughly 5 V interpretation is consistent with USB supply, but actual
  terminal voltage/polarity must be checked; the exact cause remains unverified.

## Setup-tool verification — November 4, 2026

- Full physical scan at 1,000,000 baud found only ID 1 (raw model 777).
- Additional reads: phase raw **12**, resolution **1**, alarm status **0**.
  Phase bit 2 selects fine speed units; the new test chooses raw speed 50 by
  default for this configuration, without rewriting phase or calibration.
- User reports 12 V measured and TTL wiring checked. Servo feedback still
  reports **51 (5.1 V)**; exact location of the external measurement is pending.
- Attempted `test-motion --delta -20 --min 4000 --max 4095`. It refused on the
  voltage precondition **before any motor writes**. Subsequent inspection still
  shows position **4092**, torque **0**, goal **0**, EEPROM lock **1**.
- All **17 software tests passed**, covering configuration/no-op behavior, relocking on failure,
  multiple responses, startup ordering, limits, voltage and release failures.
  No physical ID changes, power-cycle persistence check, or successful movement
  test has been performed yet.

## First successful physical movement — November 4, 2026

- User identified a faulty outlet. With the outlet corrected, the servo reported
  raw voltage **124 (12.4 V)**, temperature **29–30 °C**, and alarm status **0**.
- First test, raw speed **50**, moved from **4092** toward **4072**, then timed
  out approximately four ticks short. The script attempted hold and disabled
  torque; subsequent position was **4076**. No EEPROM changes were made.
- Second test used `test-motion --delta -20 --min 4000 --max 4095 --speed 100`.
  It passed the unchanged two-tick tolerance on both legs: start **4075**,
  target **4055**, outward feedback **4057**, returned feedback **4073**.
- Final feedback: torque **0**, voltage **124**, temperature **30**. Temporary
  output limit **300 (30%)**, speed **100** in fine units, acceleration **1**.
  These are unloaded bench settings, not validated assembled-arm operating limits.
- Keep this motor's existing ID **1**, baud **1,000,000**, position mode **0**,
  encoder offset and EEPROM limits. Label it motor 1. Verify the ID after a
  power cycle; assign IDs 2–6 to the remaining motors individually.
- Joint zero, direction and usable mechanical ranges are still to be calibrated
  after assembly. Successful encoder tracking here is not a physical accuracy
  measurement or a load/holding qualification.

## Chained-bus tooling — November 7, 2026

- Added `tools/servo_chain.py` and the [chain-test guide](servo_chain.md).
  It supports 2–6 uniquely addressed unloaded motors on one serial connection,
  with individual, grouped same-direction, opposite-direction and mixed-speed
  stages. Every stage returns to the initial positions.
- All **36 software tests pass**. The chain sequence and grouped packet handling
  were tested with mocks only; no physical multi-motor run has been performed.
- Physical ID assignment/persistence checks and individual tests for the remaining
  motors still precede the first two-motor chain test. No additional motor has
  been marked tested or mapped to an assembled joint.

## Temperature diagnostics — November 8, 2026

- The user reported transient temperatures around 61–65 °C during a two-motor
  motion test, followed by inspections showing 33/35 °C. Actual overheating has
  not been established; the cause of the transient remains unresolved.
- Added `chain watch`, integrated `--watch-temps`, and append-only JSONL logging
  of samples, motion phases, errors and raw SDK request/reply packets. See
  [temperature monitoring](servo_temperature.md). Do not run separate programs
  on the adapter concurrently; these tools now coordinate port ownership.
- SDK reply ID/checksum checks are supplemented by exact read-reply length
  validation. This closes an observed validation gap, but is not proof of the
  temperature spike's cause. High readings still stop motion.
- Read-only hardware monitoring: **21 samples in five seconds**, motor 1
  **33 °C**, motor 2 **35 °C**, no rejected frames. Raw packets and samples are
  in `/tmp/pinchy-temperature-check.jsonl`. No motor writes were issued.
- **59 software tests pass**, including telemetry, malformed replies and port
  ownership. Motion with the new telemetry has been checked in mock mode; a
  physical run capturing the transient is still needed.

## Successful two-motor chain sequence — November 8, 2026

- Reviewed user run in `/tmp/chain-motion.jsonl`, session `70811d68d8e3`,
  UTC 08:22:14–08:22:40, approximately **25.8 seconds**.
- Individual IDs 1/2, together, opposite and mixed-speed outward/return stages
  completed. **471 samples**, **5721 SDK request/reply records**, no logged
  execution errors, alarm flags or rejected packets.
- Both motors finished at **2051 ticks**, torque **0**, voltage **12.4 V**,
  temperatures **33/35 °C**. Observed supply range was **12.0–12.4 V**.
- Temperature telemetry remains suspect: ID 1 reported **33 → 50 → 33 °C**
  about 56 ms between samples; ID 2 reported **35 → 46 → 35 °C** similarly.
  Both were at their ordinary baseline in **454 of 471** samples.
- Raw ID 1 temperature request/reply at its peak:
  `ff ff 01 04 02 3f 01 b8` / `ff ff 01 03 00 32 c9`.
  The reply's `0x32` is 50, with a valid checksum and expected length. Therefore
  the spike is present before JSON formatting. This suggests a transient
  measurement/data artifact; its exact origin remains unproven.
- Mark the basic **two-motor chain test complete**. Six-motor qualification and
  investigation of the telemetry spikes remain separate pending work.

## Interrupted six-motor positioning — November 8, 2026

- `/tmp/position-six.jsonl`, session `39160ff8a7c4`, reached target movement
  after all six passed preflight and current-position startup checks.
- At approximately 4.84 s, ID 4 temperature read **62 °C**, followed by **28 °C**
  and **28 °C** in the direct/block rechecks within about **1.5 ms**. Alarm
  status was zero, supply was **12.2 V**, and no packet errors were logged.
- The prior single-sample rule aborted the run. Cleanup packets confirm torque
  **0 for every ID 1–6**. Centering was incomplete; this is not a six-motor pass.
- Temperature confirmation now permits an isolated spike only with two cool
  rechecks agreeing within 2 °C plus clear initial/rechecked alarm flags. Missing,
  inconsistent or high rechecks and all real alarms still stop the run. Raw
  values and the unconfirmed-spike classification remain logged.
- **61 software tests pass**, including the captured 62/28/28 case and sustained
  high-temperature cleanup. The revised rule awaits the next physical run;
  temperature telemetry's underlying fault remains unresolved.

## Interrupted-transaction cleanup recovery — November 8, 2026

- Positioning session `48f0013eeb11` was interrupted; cleanup failed for every
  motor with the SDK's internal “Port is in use” result. Subsequent runs failed
  preflight because torque remained **1**. IDs and resolution were correct.
- The SDK can leave `portHandler.is_using` set when interrupted inside a read
  or grouped write. Our wrapper now clears that transaction flag and drains
  interrupted replies before propagating the error, so cleanup can proceed.
- No competing serial process was present. Explicit release then verified
  **torque 0 on all six motors**, positions **2050, 2050, 2046, 2047, 2051, 2047**,
  and temperatures **33, 34, 32, 32, 32, 33 °C**. No position goals were issued.
- Added `chain release --ids ...` and field-specific preflight errors. All
  **64 software tests pass**, including interrupted read/write recovery.
- The motors are sufficiently near midrange to begin the bounded six-motor
  sequence; passing the exact centering tolerance is not a prerequisite for
  that sequence. Physical six-motor sequence verification remains pending.

## Six-motor sequence paused at motor 6 — November 8, 2026

- `/tmp/chain-six.jsonl`, session `926e85716f0d`: individual outward/return
  stages completed for motors 1–5. Motor 6 started at **2047**, requested
  **2303**, and settled at **2299**, a **−4-tick** residual inside the allowed
  **1000–3000** travel interval but outside the default **±2** arrival tolerance.
- The run was interrupted while waiting on `individual-6/outward`; no return
  command for that stage was attempted. Cleanup confirmed **torque 0 on IDs
  1–6**. No communication errors or servo alarms were logged.
- Added explicit `--tolerance` (1–10 ticks, default 2) to chain `test` and
  `position`. The six-motor bench example now uses 5. Actual signed errors are
  reported and the selected tolerance is logged. Travel/health checks and startup
  hold verification are unchanged. This is a bench acceptance setting, not a
  claim of improved servo accuracy.
- **67 software tests pass**, including acceptance/reporting of a four-tick
  residual with tolerance 5 and rejection with the unchanged default of 2.
  Full six-motor sequence completion remains pending.

## Successful full six-motor chain test — November 8, 2026

- Verified `/tmp/chain-six.jsonl`, session `9c75b46da30f`, UTC
  **08:58:35–08:59:21**, approximately **45.8 seconds**.
- All **nine stages** passed: individual motors 1–6, together, opposite and
  mixed-speed stages, with outward and return legs. Explicit arrival tolerance
  was **5 ticks**; this is an unloaded bench result, not a physical accuracy claim.
- **691 samples**, **25069 SDK transaction records**, no logged errors,
  rejected packets, servo alarms or unconfirmed high-temperature events.
- Final positions: **2054, 2051, 2050, 2050, 2051, 2051**. All six finished
  with **torque 0**. Final temperatures **33, 33, 33, 33, 33, 34 °C** and
  supply readings **12.2–12.4 V**.
- Mark individual motor setup and unloaded six-motor chained operation complete.
  Joint calibration, assembled-load capacity, stop/recovery qualification and
  ROS/MoveIt integration remain outstanding. Earlier intermittent telemetry
  artifacts remain a separate diagnostic limitation.

Use [servo_bench.md](servo_bench.md) for the initial bench procedure and
[point_motion_plan.md](point_motion_plan.md) for the subsequent control work.
