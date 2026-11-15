# IK reliability and Cartesian transit investigation

November 14, 2026. Commands and UI usage are in [Cartesian control](cartesian_control.md).
The [canonical roadmap](implementation_roadmap.md) governs remaining product work.

## Short findings

- The model has six independent positioning axes at generic configurations.
  The tested zero pose is full rank but poorly conditioned, not universally
  singular and not a five-axis arm. A small translation with locked orientation
  can require large wrist motion or leave the allowed ranges.
- No general MoveIt/KDL configuration defect was found. Native KDL recovered
  160/160 collision-valid known full poses in the calibrated-model benchmark,
  including 96 biased toward near-straight wrist/elbow and saved boundaries.
  The independent bounded search also recovered all 160. This does not prove
  mathematical completeness for every pose or guarantee a connecting path.
- The operator examples confuse position reachability with full-pose
  reachability. Both XYZ points have collision-free modeled solutions.
  The second also allows the exact requested pen direction with about 2.9° of
  free axial twist. Exact RPY intentionally prohibits that adjustment.
- The gateway now offers Exact RPY, pen direction/free twist, and position-only
  modes. It never silently converts a failed exact request into a relaxed one.
- Native KDL remains the fast phase. A bounded global multi-start fallback
  broadens candidate search instead of repeating mostly local seeds. MoveIt
  still checks every candidate, route and timed interpolation; no second motor
  controller was introduced.
- Optional straight TCP transit uses position LERP plus quaternion SLERP.
  This is a single checked transit, not continuous constant-speed drawing.

## Physical feedback boundary bug and current scope

The operator's real arm reported wrist-roll -0.00153398 rad (-0.0878906°, one
encoder tick) while its accepted/controller hold command was exactly zero and
its lower command limit was zero. The gateway used strict command bounds on
this measured state and rejected it **before IK**. The adapter correctly
reported READY because this was within its existing feedback settling margin.
This was a gateway bug, not evidence of an unreachable target.

The earlier broad benchmarks used valid witness poses and mostly ideal
feedback; they did not establish correct behavior for this physical boundary
case. The fix distinguishes measured feedback (10-tick margin matching the
adapter) from strict held commands, IK candidates and trajectories. MoveIt still
checks raw measured geometry for collisions. No clamping, command-range widening,
calibration changes or motor-protection changes were made.

The current user task requires XYZ and pen direction with twist free. UI/CLI/
service default to **tool-axis/free-twist**. Position-only is explicitly optional
and is never substituted for a direction request.
A focused calibrated-profile PTY test injects one negative wrist-roll tick with
a zero goal, then plans and executes the operator's XYZ (.404189,-.000513,.245081)
without a hardware fault. Three small regressions check strict commands, rejection
beyond the feedback margin and preservation of collision checks.

A subsequent screenshot's XYZ (.329988,.124285,.155302) was checked read-only
against the live model. Position-only IK found a collision-free candidate in
about 0.058 s; MoveIt found a connecting route from the held state in about
0.046 s. Holding the screenshot's pen direction reproduced its reported
68 mm / 15.63° residual failure. No real motor goal was dispatched by these checks.

## Free-twist MoveIt correction and targeted evidence

The earlier free-twist mode went directly to the bounded direction solver;
unlike exact RPY it did not call native MoveIt IK. It now searches native KDL
with up to 24 full-pose candidates spanning one complete turn about the requested
tool +z, then uses continuous bounded 5-DOF search for feasible twist intervals
between samples. The tangent frame is aligned to the starting tool frame; an
arbitrary input twist cannot change the requested direction or bias that frame.
All candidates still pass strict saved command bounds and MoveIt collision checks.

Targeted coverage includes 96 fresh geometry-only known-reachable pen-direction
cases with randomized irrelevant twist, plus eight independently collision-valid
witnesses inside the working cube planned through native MoveIt/OMPL using the
saved profile on the real hardware adapter attached to an emulator. The eight
resolved axes were verified against the requested axes; no physical motor goals
were sent. The prior 160-pose benchmark concerned full poses, not this mode.

The reported XYZ=(0.329988,0.124285,0.155302), RPY=(-113.168,-86.871,-108.553)°
was compared under identical geometry and two ranges:

| Range set | Result |
| --- | --- |
| Nominal CAD | A collision-checked full route exists preserving the requested direction. One result uses joint 4 +69.0°, joint 5 -93.7°, joint 6 +92.6°. |
| Saved physical profile | Native full-turn twist search and continuous bounded direction search did not find a match. The nominal candidate exceeds saved joint-4 maximum +63.9°, joint-5 minimum 0°, and joint-6 maximum +90.4°. |

This confirms a concrete discrepancy between the nominal model and saved
usable ranges, not an approval to extend physical motion. It does not prove
that the listed candidate is the only mathematical branch. Hardware calibration
and torque protections remain unchanged. The eight reachable directed cases
still pass under the stricter profile; we do not call the failing physical-profile
case fixed by accepting a position-only or out-of-bounds result.

Bringup preserves nominal model limits as diagnostic metadata alongside the
strict calibrated joint limits. On a failed direction search, an isolated
**range-review-only** calculation may find a collision-free nominal-range witness
and name which saved bounds it exceeds. It cannot produce an execution token,
enter the normal solution cache or change hardware/calibration. An older running
bringup without this metadata retains ordinary diagnostics until restarted.

Singularity diagnostics now use the selected task dimension: six for Exact RPY,
five for pen direction and three for XYZ-only. At zero, pen-direction conditioning
has minimum nonzero singular value about 0.0365 (condition ~68), versus 0.0119
(~209) for exact RPY. The unconstrained twist's zero singular value is excluded
from the five-dimensional metric. No new safety stop or torque release is added.

The secondary range review remains pending physical checks. Joint 5's recorded
safe samples were +131.48° and +4.22°; the script also included zero, producing
[0,+131.48]°. Those samples were not proof of a negative mechanical stop. Changing
the starting joint angles or merely renaming zero does not add physical reach;
verify real travel/mounting and update model/calibration consistently if needed.
Do not widen the saved ranges solely because a nominal solution exists.

## The saved limits matter

The tested profile is `calibration/arm.json`; no saved zero, direction, bound,
EEPROM or motor-controller protection was changed. Values below are rounded:

| Joint | Lower | Upper |
| --- | ---: | ---: |
| Shoulder pan | -96.68° | +108.90° |
| Shoulder lift | -91.14° | +87.63° |
| Elbow flex | -85.08° | +88.42° |
| Wrist bend | -90.88° | +63.90° |
| Wrist roll | -143.96° | +141.77° |
| Pen pitch | -0.70° | +99.14° |

These are saved usable ranges, not proof of absolute mechanical travel. Motor 5
now has negative travel after recapture; pen pitch still has very little negative
motion from zero. The earlier one-sided motor-5 profile caused several recorded
failures; historical comparisons above refer to that earlier profile.
Many common full orientations need that negative travel or the other wrist
branch. Re-measure only with the operator and real mechanism if these ranges
are suspected to be too narrow; do not widen them merely to make IK succeed.

The active tip is the approved long RViz `pointer_tip` proxy. The physical
single attachment endpoint remains unmeasured. A model-reachable proxy pose is
not proof of physical tip placement or clearance around the new base/camera.

## Singularity versus Euler-angle representation

Kinematic conditioning uses the geometric Jacobian for the active tip. We scale
translation by 0.4 m before its singular-value calculation, so the threshold is
an explicit dimensionless diagnostic rather than an arbitrary mix of metres and
radians. At zero, the smallest scaled singular value is about 0.0119 and the
condition number is about 209. At q=[0.1,-0.3,0.6,-0.3,0.6,0.3] rad these are
about 0.1103 and 21.8. This illustrates a much better-conditioned bent posture;
it is not a physical move recommendation or a claim about collision clearance.

Below 0.02 the UI/status warns of sensitive posture. This is informational;
there is no new hardware fault or torque release. Near such postures, a small
TCP step can demand a large joint change, slower travel or a different branch.
Straight Cartesian paths can also approach a joint limit or singularity midway
although both endpoints are valid. Such a path is rejected if incomplete or if
it jumps; free-space transit may find another route.

RPY near pitch ±90° has an additional **representation** ambiguity: roll/yaw
numbers can change greatly while the actual rotation changes little. IK uses
rotation matrices/quaternions and the analytic geometric Jacobian, not Euler
angle derivatives. Euler gimbal lock alone does not mean the robot is singular.

## Search and validation

[ik.py](../src/pinchy_motion/pinchy_motion/ik.py) caches the exact live URDF chain,
computes FK and analytic task Jacobians, and supplements KDL with bounded SciPy
least-squares searches. The fallback uses current state, successful-solution
cache, range midpoint and deterministic Halton seeds across the full saved
joint ranges. Each request has a six-second fallback budget and 64 seed limit,
with cancellation/model-change checks. Full pose converges to 0.1 mm / 0.001 rad
model residual; relaxed modes apply that angular tolerance to pen +z direction
or explicitly ignore orientation. These are numerical tolerances, not measured
physical accuracy. Failed exact searches do not return approximate poses.

The fallback independently enforces bounds and asks MoveIt state validity for
every candidate. MoveIt's FK verifies the final timed endpoint. OMPL can retry
with an alternate collision-free IK configuration if the first free-space route
fails. Finite search can still miss a solution, and endpoint existence does not
prove that a collision-free route exists from the current state. Error messages
separate IK constraints, collision rejection, route failure and incomplete lines.

For pen-direction mode, the target orientation's +z is preserved and rotation
about it is free. For position-only, RPY is explicitly ignored. The response
includes `resolved_target`; the panel reports its actual solved RPY. Execution
residual is relative to that resolved full pose, not an ignored twist angle.

## LERP and SLERP

These are complementary operations: LERP interpolates XYZ along a straight line;
SLERP interpolates quaternion orientation along the shortest rotation. Check
**Straight TCP path (LERP + SLERP)** to use MoveIt's `GetCartesianPath` service.
Unchecked uses OMPL free-space transit, whose TCP can curve around obstacles.
In relaxed-orientation modes the endpoint orientation is first resolved, then
the line interpolates to that explicitly reported orientation.

The line service uses 2 mm Cartesian steps, collision checking and relative/
absolute joint-jump checks (5× average / 0.2 rad). Only fraction=1 succeeds;
partial results never receive an execution token. Joint timing is checked with
the same JTC spline validator and limits as ordinary transit. Dense command
samples must stay within 1 mm / 0.01 rad of the LERP/SLERP shape; these checks
are discrete, not a continuous swept-volume proof. No automatic curved detour
is substituted when a straight path fails.

A line trajectory still becomes a timed joint trajectory sent to the same
FollowJointTrajectory action. Joint servo commands are not Cartesian commands.
Step 11 retains ordered paths, continuous IK branches, corner/transition handling
and measured constant contact TCP speed. Acceleration/deceleration belongs in
pen-up motion wherever feasible; this checkbox does not qualify contact drawing.

## Working-region limitations

The blue cube remains X=[0.10,0.50], Y=[-0.20,0.20], Z=[0,0.40] m. Its far upper
corners are 0.6708 m radially from the base. The sum of the live chain's fixed
translation lengths is only 0.6394 m: by the triangle inequality those corners
cannot be reached even before considering joint bounds, collisions or orientation.
A numerical radial-reach optimization estimates 0.6225 m for the current proxy,
ignoring collisions/loading. That estimate is not a certified useful reach or
a mathematical maximum proof. The chain-length upper bound is conservative.

The report's working cube is a designated object/platform region, not a promise
of arbitrary six-axis poses throughout it. Actual supported drawing placements
must be checked in steps 8/12/14. The floor remains the unmeasured example floor;
no tabletop plane, camera mount or base-wing geometry was guessed.

## Validation and reproduction

Source ROS and the built workspace, then from `robot_software/`:

```bash
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 tools/test_ik.py
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 tools/benchmark_ik.py
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 tools/benchmark_ik.py --count 96 --stress --output /tmp/ik-stress.json
QT_QPA_PLATFORM=offscreen /usr/bin/python3 tools/test_cartesian.py
```

The benchmark launches its own isolated-domain mock with the **actual saved
profile's limits**, independently obtains target poses from MoveIt FK, and checks
witness/candidate collision validity. Generated witness joints are never passed
as IK seeds. Fresh searches prevent cache leakage between benchmark poses.
Mock default without `calibration:=...` retains nominal CAD limits for generic
commissioning; use the saved profile option for realistic physical-arm rehearsal.
The tools never open the real serial device. New transit execution tests use
mock or the real adapter attached to a PTY emulator; no physical motors moved.

The [validation report](validation/moveit/report.json) indexes logs and JSON,
including the ordinary/stress benchmark, operator examples, reach calculation,
GUI screenshots and regressions. [MoveIt KDL documentation](https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.rst)
explains numerical seeds/restarts; the [Cartesian interpolator implementation](https://moveit.picknik.ai/main/api/html/cartesian__interpolator_8cpp_source.html)
shows position/quaternion interpolation. Installed Jazzy service fields and
runtime behavior were checked, rather than assuming a newer API was present.

## Operating-limit recapture

The [calibration guide](arm_calibration.md) now documents
`calibrate --step review-limits --ids 4 5 6`. It captures negative and positive
manual endpoints, shows signed encoder angles and existing/model ranges,
rejects same-sided samples, retains zero/direction, backs up the session and
commits each joint only after both endpoints are complete. It performs no motor
writes and refuses torque-enabled capture. The hardware profile is unchanged
until the operator supplies those measurements. Recalibration can unlock real
travel excluded by conservative samples; it cannot guarantee arbitrary poses
or establish geometry/clearance that has not been physically verified.

## Motor 5 operating-range recapture result

The owner recaptured motor 5 from [0,+131.48]° to [-143.96,+141.77]° without
changing zero/direction. Joints 4 and 6 remained unchanged. Startup/model/encoder
validation passes. The three recent logged directed requests at
(0.464938,0.137252,0.246784), (0.380140,0.137252,0.246784) with RPY
(-114.504,-87.032,-65.441)°, and the latter XYZ with RPY
(-6.575,-67.874,-172.817)° now all produce complete collision-checked free-twist
plans on a mock using the new saved bounds. No physical execution was requested.

The older screenshot target (0.329988,0.124285,0.155302), RPY
(-113.168,-86.871,-108.553)° remains range-blocked: the diagnostic candidate uses
pen pitch about +95.1°, beyond the still-saved +90.35°. That is a separate pending
physical range check; it was not silently allowed. Evidence and the exact profile
snapshot are in [the recapture checks](validation/moveit/motor5_recapture_pose_checks.json).
Restart physical bringup/planning to load the recaptured limits.

## Motor 6 recapture and wider startup window

Motor 6 was subsequently recaptured from [-1.67,+90.35]° to [-0.70,+99.14]°,
with zero 1288 and positive direction retained. Structural startup/model/encoder
checks pass. The older screenshot request that needed about +95° now produces a
complete collision-checked free-twist plan with the saved ranges in mock.
The earlier motor-6 block above is historical, superseded by this capture.

Startup now accepts an outer ±90° window, clipped to each saved/CAD interval
with only a 10-tick settling allowance near zero. It retains the gradual ramp,
prior powered holds and health/lag checks; this is guided single-joint zeroing,
not a MoveIt collision-planned route. The operator must support/clear the sweep.


## Latest real-session investigation and fixes

Inspected `python3_76672_1794700513473.log` and its matching controller log.
There were 28 pose requests, six directed-IK rejections, four partial-straight
warnings and three canceled/stopped searches. Old logs do not record every
successful plan or the source of every cancellation. No hardware fault was found
in the inspected controller tail. New PLAN_EVENT records include actual/held
start, mode, target quaternion, model hash, outcome and IK diagnostics so future
failures can be replayed without guessing their start state.

[Saved numerical results](validation/moveit/latest_session_numeric_ik.json) and
[MoveIt collision results](validation/moveit/latest_session_collision_checks.json)
record independent bounded IK and differential-evolution checks of five distinct
rejected XYZ/direction requests found:

| XYZ (m) | Finding under current saved limits |
| --- | --- |
| (0.336922,0.193282,0.189951) | Numerical residual remains at pen_pitch upper limit +99.14° (about 1.85 mm / 3.90° for the bounded candidate). |
| (0.336922,0.175257,0.189951) | Same wrist limit, best observed residual about 0.62 mm / 1.36°. |
| (0.371725,0.154079,0.064397) | XYZ/direction IK exists, but pen holder/wrist links collide with example_floor. |
| (0.250592,0.154079,0.098469) | XYZ/direction IK exists, but wrist_roll_link collides with example_floor. |
| (0.250592,0.035642,0.098469) | Both searches retain substantial residual near wrist_flex upper and wrist_roll lower bounds. |

The previous message incorrectly prioritized a range diagnostic over known
collision rejections. It now reports the actual collision/state rejection first.
The example floor has not been moved: the actual tabletop dimensions/transform
remain an owner measurement. These numerical searches are evidence of difficulty,
not mathematical proofs that every alternative branch is impossible.

The old straight-path implementation constrained full quaternion interpolation
between resolved endpoints even in free-twist mode. That could reject a valid
pen-direction route. Free-twist tracing now follows the pointing direction only,
with local bounded continuation, a nearest-previous-solution preference to avoid
unnecessary joint jumps, strict task residuals, complete collision checks and
partial-path rejection. Original session starts were not fully logged, so the
four old straight failures cannot be claimed as faithfully replayed successes.

Step 11 uses the same continuation for ordered paths; see
[ordered paths](cartesian_paths.md). Software timing checks do not establish
physical accuracy, contact safety or universal workspace reachability. The blue
working box is a requested work region, not a guarantee of every direction at
every point. Joint limits, self/floor collision, singularities and continuous
route connectivity still matter; required missing wrist travel cannot be created
by switching IK algorithms.


The path-timing regression also reproduced a near-duplicate IK knot: a waypoint
and a regular grid sample were only 0.35 micrometres apart after native IK's
small endpoint residual. Returning the same q within task tolerance at both
samples created a flat interval and artificial joint-acceleration spikes. Fixed
waypoint knots now take priority over redundant regular samples within 0.1 mm.
The joint fit is C2 (continuous acceleration), and both controller polynomial
extrema and TCP geometry/speed are still checked. The reproduced 10 mm/s mock
contact case now measures modeled interior speed approximately 9.9999–10.0001
mm/s; this is a software result, not a physical drawing measurement.


## Nearby goal after changing arm orientation

Latest session `python3_82998_1794705626775.log` includes the screenshot request
XYZ (0.471461,-0.003679,0.239280) and RPY (-130.083,-86.586,4.025)°.
The displayed current pose is only about 16.5 mm away but differs in pen direction
by 53.9°. The original request remains unsolved: bounded search and independent
differential evolution both retain approximately 8.06 mm / 6.42° residual under
the current saved ranges. That residual is a combined optimization result, not
an instruction to move to that approximate pose or proof of global impossibility.

In isolated mock using the actual saved angular bounds, the same XYZ with current
pointing direction produces a collision-checked plan in both OMPL transit and
straight/free-twist transit. Nothing is physically commanded. The UI now compares
goal distance/direction against live feedback and offers Copy current direction
without altering XYZ, planning or moving. Goal fields intentionally remain stable
as the arm moves; existing queue rows are not rewritten. Error text reports the
requested direction change and numerical residual instead of a generic paragraph.
Evidence is indexed under `nearby_goal_direction_screenshot` in the validation
report. Three focused UI/geometry checks, ten IK regressions and the saved-range
mock reproduction passed. Joint/TCP speed caps and calibration were not changed.
