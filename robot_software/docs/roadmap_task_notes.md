# Roadmap implementation notes

These are supporting details for the numbered rows in the canonical
[roadmap](implementation_roadmap.md), not a separate status or task list.
Status and ordering belong to the roadmap; these notes preserve implementation
caveats, ownership boundaries and pending measurements from its expanded rows.

## Step 1: Motor setup

Hardware inventory and six unique motor IDs; individual and chained motor tests, labels and power-cycle checks.

## Step 2: Assembly and calibration

Assemble the arm, integrate CAD 4, and record joint order, zeros, directions and initial bounds. Model/TCP accuracy still needs measurement in later steps.

## Step 3: ROS control

Real and mock ros2_control; measured joint states, trajectory action, RViz and six-joint sliders. Owner successfully used the physical workflow in demos; bringup is complete for starting MoveIt.

## Step 4: Startup and safety

Guided sequential zero/hold startup, operating speed/acceleration controls, live readouts, planned/measured graphs, recording, Stop/hold and explicit runtime fault recovery. Finite command-rate excesses are limited without a safety latch; defaults are 30 deg/s and 180 deg/s², with current selectable caps of 92.14 deg/s and 398.5 deg/s² ([conditional estimate](estimated_operating_limits.md)). Live target telemetry uses a nonblocking worker and stamped updates. Physical fault/duty qualification remains in steps 8 and 23.

## Step 5: Scope and objects

Owner selected a 100 × 100 mm flat panel; an 80 mm outside-diameter × 100 mm tall cylinder/cylindrical mug, handle away; and a concave quarter pipe with 100 mm inner radius, 100 mm axial length and 90° span, with 3–5 mm wall if printed. See [nominal geometries](nominal_geometries.md). Placement, supported drawing regions, pen, artwork limits, accuracy, speed and runtime acceptance remain undecided/unqualified.

## Step 6: Team interfaces

Agree both integration contracts with the server and AprilTag teammates. Obtain example payloads and recorded pose samples so development can proceed with fakes.

## Step 7: TCP definition

Defined `pointer_tip` at the modeled pointer endpoint, with an active-tool profile, visible marker, geometry/FK checks and mock/emulated ROS TF validation. Owner authorized this temporary proxy; physical endpoint/model agreement remains unmeasured in step 8, and actual pen calibration is step 13. See [tool frames](tool_frames.md).

## Step 8: Hardware qualification

Owner decision, November 15, 2026: accept this milestone as complete for moving
forward. Retain accepted bench work, assembled operation and captured ranges.
Outstanding load, tracking, useful-speed, temperature/power, accuracy/repeatability
and Stop/recovery measurements move to verification steps 23–24. Closing this
milestone is not a claim that all physical qualification measurements passed.
Startup capture-window feedback and shoulder/elbow settling tolerance are
implemented; selectable GUI limits remain settings rather than certified
physical operating limits. Final pen/TCP calibration remains step 13.

## Step 9: MoveIt setup

Added `pinchy_moveit_config`: live calibrated model/limits, six-joint `arm` group, KDL plus bounded multi-start IK with explicit exact-pose/pen-direction/position modes, OMPL planning, reviewed adjacent-only self-collision exclusions, example floor and existing-controller mapping. Live goal handles, numerical input bounds, approximate reach samples and optional collision geometry are visible; floor placement remains unmeasured. Shared RViz view and hardware protections retained. Final base-wing/fixture geometry remains pending from the owner; reconcile it before physical planning near those parts. See [MoveIt handoff](moveit_handoff.md).

## Step 10: Poses and saved points

Owner decision, November 15, 2026: accept the functional pose/saved-point
workflow as complete for moving forward. Continue improvements throughout the
project. This acceptance does not certify physical accuracy, repeatability,
fixed-TCP drift, loaded duty or every fault/recovery scenario. Those measured
checks remain in steps 23–24; final pen/TCP calibration remains step 13.
The retained implementation details below describe evidence and limitations,
not additional conditions for reopening step 10.

Implemented synchronized live XYZ/RPY fields/RViz handles and CLI plus typed plan/execute interfaces: explicit IK constraints, preview-only planning, optional straight TCP LERP/SLERP transit with partial-path rejection, checked timed paths, explicit execution, current pose/progress, powered Stop/cancel and recovery. Mock/emulated validation covers rejection and execution; the new guided entry preserves holds through planner startup on a fake bus. Measured physical pose/orientation/repeatability checks remain pending, using the temporary pointer proxy with its unmeasured attachment offset. Pen-up queues now default to collision-planned destination routing and share alternate-configuration retries with point moves; stale input results are cleared. Added measured/model TCP capture, saved/loaded point sets, read-only queue snapshots and a checked fixed-TCP rotation test mode with examples; the seven-point session queue is saved. File/queue workflows and rotation execution passed mock/PTY tests; the ±3° rotation example showed about 0.066 mm modeled TCP drift, not measured physical accuracy. See [point sets](point_sets.md). See [Cartesian guide](cartesian_control.md) and the [reported queue replay](pen_up_queue_replay.md).

## Step 11: Continuous strokes

Ordered waypoint queue, typed PlanPath service and JSON CLI; continuous free-twist IK, polyline/explicit spline traces and OMPL visits, full collision/trajectory validation, preview and existing powered execution. Mock contact timing holds modeled TCP speed through samples with explicit pen-up ramps; sharp contact corners, incomplete paths and infeasible speed are rejected. Physical contact remains disabled until pen/surface qualification; measured speed/variation and loaded operation remain pending. Added whole-stroke joint optimization and explicit alternative-start search with a checked pen-up approach. Both recorded broad arcs now pass modeled timing at an explicitly requested 3 mm/s and 20 mm/s² TCP ramps under default joint caps; this does not qualify faster speeds or full drawing coverage. Explicit contact endpoint ramps are supported; short ramp-only strokes are reported. Broader posture/geometry and physical qualification remain the priority; see step 12 and the [stroke contract](stroke_planning.md). See [ordered paths](cartesian_paths.md).

Continuous air-test software is now complete at the documented level. The
[41-point two-inch cylinder example](continuous_cylinder_test.md) runs as one
trace at 3 mm/s, retaining normals and a 5 mm stand-off. Live controller
references passed mock and real-driver/PTY execution through all 39 interior
waypoints (2.986–3.012 mm/s). `enforce_tcp_speed` rejects hidden slowing and
interior sharp-corner stops. Interior-witness backward/forward search now joins
complete branches before validation. Collision transport retains every original
sample in small batches; strict bounds, scene/model snapshots, powered Stop and
replay rejection remain. See [validation](validation/feasibility/nominal_surfaces/validation.md).
Physical contact remains disabled; actual nib speed/accuracy remain unmeasured.

The Cartesian planning display now reports elapsed time, current stage, known
check/sample counts and stage-only percentage/ETA, with search attempts and
indeterminate stages where totals are unknown. Both saved 41-point plane and
cylinder requests passed real panel/mock service checks. See
[planning progress](planning_progress.md); no planning or safety checks are relaxed.

## Step 12: Drawing feasibility

Prioritize continuous drawing feasibility before job integrations. A current-model study found posture-dependent nearby failures, a continuation failure with a valid broader IK witness, singularity sensitivity, side-normal range concerns and broad-arc timing failures; front-180° coverage is not established. Read [motion feasibility](motion_feasibility.md), use the selected [nominal geometries](nominal_geometries.md), verify the real TCP/wrist ranges, and establish object frames/placement, seating, solid collision geometry and fixture clearances. Two historical broad-arc regressions now pass whole-stroke selection at 3 mm/s with 20 mm/s² ramps and default joint caps; full-region coverage and a physically useful speed remain unqualified. Extend checks across the selected geometries, placements, alternate starts and complete drawing/lift paths. See the [stroke contract](stroke_planning.md).

Added opt-in nominal collision scenes: upright panel, solid cylinder and a
closed, conservatively faceted concave quarter-pipe mesh, each with an assumed
fixture and the unchanged example floor. All three two-inch loops plus their
separately checked 10 mm normal retracts pass at 3 mm/s; 35/35 central endpoint
witnesses were found for each. The larger endpoint study found 36/65 cylinder,
36/45 plane and 36/45 quarter-pipe witnesses. Finite unresolved samples are not
impossibility proofs; endpoint witnesses are not connected-path coverage.
The longer search confirms lowest-row `nominal_fixture/pen_holder_link`
collisions on all three assumed fixtures, plus unresolved cylinder side normals
near saved pen-pitch limits. These are hypothetical placements, not measured
final hardware. Keep this gate
open for placement/fixture confirmation, supported regions and complete paths
beyond those witnesses. Step 14 still owns general artwork mapping and lifts.

Expanded follow-up: three proposed support scenes now clear the lower-row
holder collisions without removing collision checking or changing the objects,
floor, TCP or joint limits. Pedestal endpoint results are 45/65 cylinder and
45/45 each for plane/quarter pipe. All nine complete lower/upper/vertical paths
and their exact-end retracts pass strict 3 mm/s preflight, with independent FK.
Six full-system executions on mock/real-driver PTY (wider cylinder arc and saved
plane/pipe loops, each backend) also execute their retracts and reject replay.
The tested cylinder interval is **−75°…+45°**, not symmetric ±60° at this offset
placement. Plane full width and pipe full 90° are tested at five heights over a
90 mm interior axial span; these witnesses do not guarantee arbitrary artwork.
A necessary geometry bound excludes current-placement +75°/+90° cylinder poses
by 19.28/61.03 mm of wrist reach even with unrestricted joints/free twist.
A finite 58-placement sweep and a 48-start coupled placement/configuration
optimization found no exact three-endpoint full-180 witness. Neither is a
global impossibility proof. Keep the
physical gate open for actual setup measurements and a reviewed placement/tool
change or explicit owner coverage decision. See [expanded evidence](validation/feasibility/surface_gate/validation.md)
and the installed `examples/surface_feasibility` point sets. No physical motor
moved; original uncommitted work and baseline scenes are retained.

## Step 13: Pen calibration

Install the actual pen, calibrate tip/axis and holder compression/contact offset, update active tool visuals/collision geometry and check compliance. Prove repeatable contact without bottoming out. Encoder position is not a force measurement.

## Step 14: Surface mapping

Generate surface-following poses from simple 2D lines/curves for the selected plane, convex cylinder and concave quarter pipe, preserving surface distance and the correct surface normals. Add approach, contact, pen-up travel, stroke transitions and retract. Allow only intended pen-tip contact; keep holder/arm/fixture collision checks.

## Step 15: SVG import

Parse the supported SVG subset, units/viewBox/transforms and artwork placement; sample native curves to an explicit geometric tolerance, preserve stroke boundaries/corners, and validate unsupported elements and input limits. Reject invalid jobs with clear reasons.

## Step 16: Job executor

Build the robot job executor: validate → register → map → plan/preview → wait for Start → execute → complete/hold/fail. One active job; track measured progress and results. Honor hardware safety holds and discard interrupted motion on recovery; never silently resume an old drawing.

## Step 17: Physical drawing

Draw from a local file on the fixed cylinder: start with dry runs, then single lines, curves and multiple separated strokes. Measure the marks and tune TCP, contact, mapping and operating speed.

## Step 18: AprilTag integration

Consume teammate AprilTag output through the agreed ROS adapter. Establish camera/base/platform/object transforms using the pending base-wing camera mount geometry and verified optical-frame offset, check timestamps/quality, reject stale or missing registration, and associate the approved plan with the specific object pose. Detection implementation belongs to the teammate.

## Step 19: Registration testing

Verify registration against independent physical references over several placements and reseatings. Define behavior for occlusion, stale updates and detected movement during a job. A changed pose invalidates a preview/plan; never change the drawing frame underneath an executing trajectory.

## Step 20: HTTPS client

Implement the Jetson HTTPS bridge: retrieve/claim a job, download artwork/settings, validate and stage locally, report preview/readiness/progress/results, and consume Start/Stop/cancel requests. Protect against duplicate execution, reconnect/retry errors, partial downloads and stale jobs. Use a fake server until real endpoints are ready.

## Step 21: Browser workflow

Connect the finished browser/server workflow: upload, object selection, scale/placement, preview, explicit Start, status/progress, Stop and rejection details. Existing canvas/SVG submission is a starting point. A browser Stop must reach the existing ROS hold mechanism.

## Step 22: End-to-end integration

Complete the end-to-end path: browser → HTTPS server → Jetson job → AprilTag registration → validated drawing trajectory → real motors, with status/results back to the server/browser. Demonstrate an entire job without terminal commands per stroke.

## Step 23: Verification

The owner deferred remaining step-8 measurements to verification on November 15.
Record loaded shoulder/elbow lifting and holding (droop/stall and tracking),
useful TCP speed and acceleration including low-speed smoothness, independently
measured accuracy/repeatability and model/TCP agreement, sustained temperature
and supply/current behavior, and physical Stop/hold and explicit recovery under
representative loading. Emulator results qualify software behavior, not these
physical measurements. Final measured acceptance remains step 24, and installed
pen/TCP calibration remains step 13.

Validate failures and repeatability: invalid/oversized artwork, unreachable/colliding paths, tag loss/movement, HTTP disconnects/duplicates, servo faults, Stop, power loss and process restart. Establish supported stop/recovery behavior and verify interrupted jobs do not auto-run again.

## Step 24: Final acceptance

Final measured acceptance and handoff: repeated drawings across placements, accuracy/speed/contact/duty results, documented limits, one normal operator entry point, dependency/config versioning, persistent job records and a reproducible final demo/runbook.



## Startup refinement — November 17

Owner requested faster zeroing and clarified removal of only the extra ±90°
capture cap, retaining saved/CAD limits. Default ramp is now 0.225 rad/s
(~12.9 deg/s), capped by configured joint velocity. Automatic capture still
requires one second still after arming. Raw startup servo caps 150/8, torque
allowance, health checks, tracking/stall pauses, prior powered holds and handoff
remain unchanged. Optional `--max-start-degrees` can add a smaller window.
See [startup validation](validation/startup/2026-11-17/validation.md).


## November 17: Execute latency and larger/faster cylinder

Execute previously repeated dense collision validation after a successful
preview. Exact trajectory/model/scene fingerprints now permit reuse of that
validation while retaining fresh readiness, actual-state collision, start,
scene and pre-dispatch Stop/cancel checks. Final gateway/transport plus eight
mock/PTY integration checks pass, including thermal hold recovery. See
[implementation](execute_latency.md) and
[results](validation/moveit/execute_latency/validation.md).

Example 07 adds a 61-point 76.2 mm oval at strict 7 mm/s, versus 50.8 mm and
3 mm/s before. Nominal cylinder dimensions and default joint caps are retained;
it explicitly uses a lower hypothetical placement and its matching scene.
Both complete mock and real-driver/PTY executions pass all 59 interior input
waypoints at cruise; Execute starts the moving reference in 0.31–0.38 s.
Smooth direction interpolation is an explicit new `spline_axis` mode; old
files retain their semantics. See [usage](continuous_cylinder_test.md#larger-faster-example)
and [validation](validation/feasibility/fast_cylinder/validation.md).
This refines step 11 without closing step 12: full coverage, actual placement,
fixture/tool agreement and physical contact/speed qualification remain open.
No physical motors moved or existing live hardware processes restarted.


## November 19: speed diagnosis and optional range TODO

Higher requested TCP speed can fail timed joint rates after geometric IK has
succeeded. Example-07 replay at 18 mm/s / 80 mm/s² needs about 67 deg/s wrist-roll
velocity against the 60 deg/s cap; the physical-session search log also rejects
on wrist-roll velocity. The older smaller example has additional direction
interpolation acceleration spikes. See the [diagnosis](cylinder_speed_investigation.md).

An existing physical 14 mm/s recording also shows a distinct halfway feedback
pause while controller output keeps moving. It is not an intentional planned
stop in that recording. The downstream cause is unresolved; do not declare
backlash, firmware, driver behavior or a mechanical cause proven. Record adapter
applied targets and external observations in operator-coordinated verification
(step 23). No physical movement or tuning was commanded for this investigation.

The owner asks to remember optional TCP speed bounds but **not implement yet**.
Added this as a step-11 TODO: explicit minimum/maximum, smooth look-ahead timing,
preserved path/normal and joint constraints, no interior stops or hidden drops
below the minimum, and reported endpoint-ramp exceptions. Solve timing on a
complete geometric branch rather than rerunning IK at every possible speed;
benchmark cost and validate line quality later. See the [proposal](bounded_speed_proposal.md).


## November 19 follow-up: 90 deg/s and tracking instrumentation

The owner requests 90 deg/s maximum joint velocity, retaining the default
30 deg/s. Updated the shared contract, six-joint URDF, saved profile (with backup)
and C++ validation together. The 18 mm/s / 80 mm/s² larger loop now passes full
mock and driver/PTY execution. Position calibration, acceleration maxima,
startup zeroing ramp, thermal confirmation and powered fault handling remain.
Software timing success is not physical speed or tracking qualification.

Explicit tests show both circle interiors retain cruise speed, with ramps only
at the whole-stroke endpoints. The downstream physical pause remains unresolved;
nonblocking servo-goal readback telemetry and a joined recorder are implemented
to isolate it on the next operator-run example. Read-only analysis leaves old
missing observations unknown. Step 23 must retain physical reversal/tracking
verification; no physical motion was commanded for this work. See
[results](validation/moveit/joint90_tracking/validation.md).

Bounded-speed timing remains a TODO, not implemented. Expected overhead is small
per candidate relative to whole-stroke IK; total runtime may improve by avoiding
rejected candidates. Drawing quality, smooth acceleration and useful minimum
speed are practical drawbacks to test. See the [proposal](bounded_speed_proposal.md).


## November 22: completed bounded TCP timing and conditional caps

The owner subsequently authorized implementation, superseding the November 19
TODO. Range mode now preserves explicit min/max through panel, files, CLI and
PlanPath; selected full geometric branches receive smooth constrained timing,
then shared JTC, geometry, normal, speed and collision validation. No hidden
minimum relaxation or stops at interior samples are allowed. Existing fixed
mode remains the default. Example 08 (4–18 mm/s) passes complete mock and
real-driver/PTY execution at all 59 interior waypoints, with Stop/cancel,
scene invalidation, single-use preview and thermal-fault regressions retained.
See [usage](bounded_speed_proposal.md) and [recorded tests](validation/moveit/bounded_speed/validation.md).

The coupled CAD/manufacturer graph estimate, less the requested 10-unit margins
and rounded down, sets software maxima to 92.14 deg/s and 398.5 deg/s². Both
exceed the old 90/360 floors. Defaults stay 30/180; position calibration,
raw 1000/64 servo profile, startup behavior and shared safety stack are retained.
These are conditional nominal performance estimates, not a physically qualified
safe envelope. See [method and bounds](estimated_operating_limits.md).
No physical motor moved or live hardware process restarted. Step 12 coverage,
physical line quality/speed and the recorded halfway tracking pause remain open.
