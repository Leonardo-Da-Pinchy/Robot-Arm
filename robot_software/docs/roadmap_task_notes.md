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

Guided sequential zero/hold startup, operating speed/acceleration controls, live readouts, planned/measured graphs, recording, Stop/hold and explicit runtime fault recovery. Finite command-rate excesses are limited without a safety latch; defaults are 30 deg/s and 180 deg/s², with selectable caps of 60 deg/s and 360 deg/s². Live target telemetry uses a nonblocking worker and stamped updates. Physical fault/duty qualification remains in steps 8 and 23.

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

## Step 12: Drawing feasibility

Prioritize continuous drawing feasibility before job integrations. A current-model study found posture-dependent nearby failures, a continuation failure with a valid broader IK witness, singularity sensitivity, side-normal range concerns and broad-arc timing failures; front-180° coverage is not established. Read [motion feasibility](motion_feasibility.md), use the selected [nominal geometries](nominal_geometries.md), verify the real TCP/wrist ranges, and establish object frames/placement, seating, solid collision geometry and fixture clearances. Two historical broad-arc regressions now pass whole-stroke selection at 3 mm/s with 20 mm/s² ramps and default joint caps; full-region coverage and a physically useful speed remain unqualified. Extend checks across the selected geometries, placements, alternate starts and complete drawing/lift paths. See the [stroke contract](stroke_planning.md).

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

