# Cylinder speed limits and halfway tracking pause

November 19, 2026. Numerical replays and read-only recordings only. No physical
motion was commanded, live process restarted, controller/driver setting changed,
or runtime speed-range feature implemented. Existing uncommitted work remains.

## Higher requested TCP speed

Speed does not change the geometric IK equations. The chosen continuous joint
path must additionally satisfy qdot = q'(s) v and qddot = q''(s) v² + q'(s) a.
A reachable complete branch can therefore fail timing at a higher speed.
Increasing joint caps can make it pass until a different cap becomes binding.

The owner's current planner log records 7, 12, 16, 14 and 18 mm/s requests for
example 07. The 18 mm/s search rejects complete candidates on wrist-roll
velocity (one rejection requires a time multiplier of about 1.18), alongside
other rejected candidates with wrist-flex position bounds or continuation
failures. The last finite-search rejection alone is not a diagnosis of every
candidate or proof that no other branch can work.

An independent replay of the previously qualified example-07 geometric branch
with joint caps **60 deg/s / 360 deg/s²** and TCP acceleration **80 mm/s²** gives:

| TCP cruise | Peak wrist-roll velocity | Joint-timing outcome |
| --- | --- | --- |
| 12 mm/s | 44.75 deg/s | Pass |
| 14 mm/s | 52.21 deg/s | Pass |
| 16 mm/s | 59.67 deg/s | Reject: velocity safety margin |
| 18 mm/s | 67.13 deg/s | Reject: wrist-roll velocity |

The checker reserves a 5% time margin: velocity requires 1.05× headroom and
acceleration 1.05²×. Thus a raw velocity just under 60 deg/s can still reject.
At 18 mm/s, wrist-roll peaks around **91° of the oval**, about 3.74 s into the
trace, rather than at the observed halfway pause. The largest acceleration
among all joints is only about 64.8 deg/s² on this branch at these settings;
raising the acceleration cap cannot fix its wrist-speed limit. Changing TCP
acceleration from 80 to 20 mm/s² moves the peak later but leaves its magnitude
essentially unchanged because it occurs during cruise. These are branch-specific
modeled results, not a new motor operating limit or a globally optimal speed.

## Why the smaller example can reject earlier

A numerical complete branch for example 04 passes its original 3 mm/s timing.
At TCP 10 mm/s, 80 mm/s² and joint caps 60/360, that branch needs approximately
**794 deg/s² wrist-flex acceleration**, peaking around 324° of the oval. At
6 mm/s it passes. Smaller Cartesian dimensions do not imply easier joint motion.

The older example uses `spline`: smooth XYZ but segmentwise S2 pen-direction
interpolation. Direction-rate jumps at knots contribute sharp joint curvature
and acceleration demand. In an explicitly separate numerical comparison,
refitting the same XYZ/normal knots using `spline_axis` passes 10 mm/s and then
becomes wrist-velocity limited around 12 mm/s. This supports interpolation
roughness as a contributor, rather than an intrinsic inability to reach the
positions. This comparison was **not collision- or execution-qualified**, and
no packaged point set or running planner was altered. Its geometry/normal
interpolation differs explicitly; it is not permission to silently substitute
curves in existing jobs.

## Halfway slowdown: what is actually established

The available physical recording contains a complete 14 mm/s example-07 run.
At roughly **185°**, for a 0.201 s interval:

- Controller output advances about 2.8 mm along the modeled TCP curve;
  reference and independently differentiated output both remain about 14 mm/s.
- Elbow target advances about eight encoder ticks and wrist-flex about five;
  measured elbow/wrist encoders remain unchanged. Only shoulder-lift feedback
  varies by one tick. The feedback-derived tip speed briefly approaches zero.
- Several joints reverse nearby; shoulder-lift crosses zero velocity there.
- Controller speed scaling stays at 1, the goal completes on schedule, and the
  hardware log contains no command-rate-limiter warning for that run.
- The five-constraint task Jacobian condition number is about 8.7 at 180–185°,
  versus roughly 11.6 near 90°; the modeled halfway posture is not flagged
  singular. Neither a planned dwell nor a modeled singularity is supported
  for this particular recorded run.

See the [recorded comparison plot](validation/feasibility/cylinder_speed_2026_11_19/halfway_tracking.png).
Encoder/model speed is not independent physical tip metrology. The 240 ms
smoothed instantaneous speed estimate is supplemented by raw joint feedback
and controller-output differences, rather than used alone to infer a pause.

**Root cause is unresolved.** The evidence separates continuous upstream
commands from a pause in measured motion; it does **not** isolate hardware-driver
packet/accepted-goal behavior, servo firmware/profile behavior, deadband,
backlash, friction or load effects. The historical recorder did not include
stamped adapter-applied targets, servo goal-register readback or external tip
measurements. Absence of a rate warning does not prove every command arrived.
The owner's observation that both loop sizes pause at different speeds remains
relevant; only this 14 mm/s large-loop event has been analyzed from raw data.
Do not generalize the one recording to a proven mechanism for every run.

A useful next physical qualification would passively record reference, controller
output, `/arm_safety/held_positions`, measured joints and safety/scaling on the
same clock through both loop sizes, and correlate with external observation.
This separates controller/adapter behavior from downstream tracking. Further
servo-profile, mechanical and load checks need operator-coordinated testing;
no setting should be changed merely on the deadband/backlash hypothesis.

## Proposed speed range

The owner requested recording an optional minimum/maximum TCP speed mode,
**without implementing it yet**. It can help these joint-timing bottlenecks by
slowing locally, provided the minimum and acceleration transitions remain
feasible. It does not repair the measured halfway tracking pause. See the
[proposal and TODO](bounded_speed_proposal.md).

Raw selected recordings, replay branches, peak-rate tables, scripts, source
hashes and the comparison plot are in
[the validation bundle](validation/feasibility/cylinder_speed_2026_11_19/validation.md).


## Follow-up: owner-approved 90 deg/s ceiling

Later on November 19, the owner explicitly requested increasing the maximum
joint velocity to 90 deg/s. The shared contract, URDF, saved operating profile
and driver validation now agree on that ceiling. Default selected speed remains
30 deg/s; acceleration caps, positional calibration, startup ramp and powered
safety behavior are retained. A pre-change operating-profile backup is stored
under `calibration/backups/arm-2026-11-19-before-90deg-s.json`. The earlier table
above remains a **60 deg/s historical comparison**, not the new active cap.

The same larger loop at 18 mm/s / 80 mm/s² passes complete collision-planned
approach/trace execution on mock and the real driver with a generated PTY servo
emulator. This resolves that software timing case at the new ceiling; it does
not demonstrate physical motion at those settings or resolve the physical pause.
The emulator's joint-acceleration profile is more restrictive than the requested
360 deg/s²; it still passes. See [follow-up evidence](validation/moveit/joint90_tracking/validation.md).

The halfway timing regression explicitly checks both cylinder curves at 3, 7,
14 and 18 mm/s with 20/80 mm/s² endpoint ramps: the halfway region remains on
the constant-speed plateau. Complete controller reference traces also show no
mid-loop ramp. This rules out the drawing-policy off-surface endpoint ramps as
the planned halfway pause in these cases; it does not rule out the servo's own
internal acceleration/profile behavior or delayed command delivery.

The existing read sweep now publishes observed servo goal registers on
`/arm_diagnostics/servo_goal_positions`, through a nonblocking realtime-publisher
worker with no extra serial reads. The recorder joins reference/output,
adapter-applied targets, servo goal-register readback, measured positions,
safety/scaling and source timestamps. These distinguish unadvanced adapter
commands, unadvanced servo goal readback and feedback failing to follow advancing
goals. Old logs remain explicitly **unobserved** at missing boundaries rather
than being classified as mechanical faults. Diagnostic publishing does not alter
commands, firmware settings or safety behavior.

The user has been asked for one saved-settings example-07 run at the next normal
startup, with the new recording. No physical motor has been moved by the agent.
Until that data isolates the boundary and a corrective change is physically
checked, the physical halfway pause remains **unfixed/unqualified**. Do not
claim that new telemetry or passing emulator tests repaired it.

Read an existing recording without connecting to the robot:

```bash
python3 tools/analyze_motion_tracking.py /tmp/pinchy-motion-<PID>.jsonl
```

The default FK model is the shared source URDF. For changed tool/base geometry,
pass `--model` with the exact session model. A reported low-feedback-progress
window is evidence, not an automatic fault or proof of a particular mechanism.
