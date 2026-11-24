# 90 deg/s ceiling and tracking-boundary diagnostics

November 19, 2026. Explicit owner request: raise maximum joint speed to 90 deg/s,
investigate/fix the physical halfway pause, and explain speed-range tradeoffs.
No physical motor was commanded, servo firmware setting written or live driver
restarted. Speed-range timing remains a proposal, not implemented.

## Operating ceiling

Updated all matching layers: shared MAX_SPEED, the model generator and six-joint URDF velocity limits,
saved profile max_velocity and C++ profile validation. Default selected speed
is still 30 deg/s; acceleration ceilings, measured zeros/directions/position
bounds, startup ramp, thermal confirmation, powered Stop/fault/recovery and
raw servo speed/acceleration/torque settings remain unchanged. The previous
saved profile is backed up under calibration/backups and the update is audited
in the profile. A structural comparison confirms no other calibration changes.
Load these hardware/model changes through the next normal startup, not a hot
restart of a powered arm. A 90 deg/s selectable cap is not physical qualification.
The existing timed-trajectory safety margin remains in force.

## Complete high-speed loop software qualification

A separate explicit test point file requests example 07's unchanged geometry at
18 mm/s, 80 mm/s² and joint caps 90 deg/s / 360 deg/s². It does not change the
packaged example's saved 7 mm/s / 30 deg/s defaults. Both isolated complete stacks
pass planning, collision-checked approach, timing, execution, constant cruise at
all 59 interior input waypoints, subsequent Stop/hold and consumed-plan rejection.

| Result | Mock | Driver + PTY servo emulator |
| --- | --- | --- |
| Initial planning | 85 s | 86 s |
| Whole approach + trace | 16.29 s | 16.53 s |
| Live cruise samples | 679 | 679 |
| Cruise minimum/maximum | See JSON report | 17.923–18.088 mm/s |
| Interior input waypoints at cruise | 59/59 | 59/59 |
| Execute → moving reference | See JSON report | 0.154 s |

The test helper now honors point-file joint caps instead of always substituting
30/180. The PTY fixture explicitly selects its 90 deg/s profile; all existing
fixtures retain 30 deg/s defaults. Its raw acceleration=32 conservatively limits
modeled joint acceleration to 222.72 deg/s² despite the request for 360; that
more restrictive case still passes. The mock uses the 360 deg/s² ceiling.
The PTY emulator models protocol/command following, not loaded servo dynamics.
These results do not establish physical tracking or a physical pause repair.

## Halfway policy and downstream observation

A new regression checks both cylinder curves at 3, 7, 14 and 18 mm/s with 20/80
mm/s² ramps. The 180-degree knot and its neighboring region stay at requested
cruise, with zero commanded TCP acceleration. There is only one pair of endpoint
ramps per complete stroke. Full reference traces independently confirm constant
speed through the interior. The off-surface endpoint-ramp policy is not inserting
a halfway ramp in these tested paths.

The driver reuses observed goal registers 42/43 from its existing read sweep to
publish `/arm_diagnostics/servo_goal_positions`. Its preallocated realtime
publisher only uses trylock in the control thread; DDS publication runs on the
worker, with dropped diagnostic samples allowed. No extra serial transaction,
control command, firmware/EEPROM change or synchronous DDS publication is added.
The recorder separately stores reference, controller output, adapter targets,
observed servo goals and measured positions, with source stamps/receipt ages,
safety and scaling. Missing/stale boundaries are reported as unknown.

The read-only analysis tool localizes low-feedback-progress windows to the
first observed boundary that stops advancing. Synthetic records exercise
separate driver, goal-readback and downstream-feedback cases, missing/stale
observations and explicit holds. The historical physical recording correctly
remains unknown at the adapter boundary. The new joined PTY 18 mm/s recording
has no low-feedback-progress events.

This instrumentation does **not** fix or identify the physical root cause by
itself. The user was asked for one saved-settings example-07 loop after the next
normal startup. Until that data arrives and a correction is physically checked,
the physical halfway pause remains open; servo firmware profiling, packet delivery
and mechanical/load/feedback effects have not been conclusively separated.

## Checks

- 44 focused path/panel/point-file/model/recorder tests pass.
- Seven recorder/boundary tests pass (three overlap the batch above).
- 29 startup unit tests pass; the higher cap does not change zeroing behavior.
- 30 C++ hardware tests pass, including 90 deg/s acceptance, out-of-range rejection,
  dropped broadcast retry, health handling and powered holds.
- Four ROS driver/PTY integration tests pass: a reversal above the old 60 deg/s
  envelope, joined telemetry recording, Stop under feedback lag and hardware-fault
  handling. No physical serial device is opened.
- Both complete high-speed loop executions pass. Build and diff checks pass.

Reports, controller traces, a joined recorder excerpt, point file, emulator audit,
check logs and source hashes are stored alongside this file. See the
[investigation](../../../cylinder_speed_investigation.md) and
[speed-range proposal](../../../bounded_speed_proposal.md).
