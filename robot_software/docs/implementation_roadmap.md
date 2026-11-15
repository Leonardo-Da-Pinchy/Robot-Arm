# Final-product roadmap

Updated November 14, 2026. This is the current ordered plan through the finished
product, superseding the older checkpoint-only task list. The owner reports
that the assembled arm, startup, ROS control and sliders worked well for the
demos. This confirms demonstrated operation; it does not establish measured
Cartesian accuracy or continuous-duty qualification.

Use the [technical handoff](checkpoint_handoff.md) for hardware details,
calibration, commands, code locations and the existing runtime safety contract.
Use the [Cartesian guide](cartesian_control.md) and [MoveIt handoff](moveit_handoff.md)
for the implemented planning/pose layer. The [point/path plan](point_motion_plan.md)
retains background for continuous paths. Read [motion feasibility](motion_feasibility.md)
for the current priority findings, exact regression cases and remaining planner work.
Older status tables in those documents are historical; this roadmap governs
current scope and ordering. Check the code before assuming external components
have arrived.

**This is the canonical task list.** When asked for the full list, read this
file, reconcile any newly confirmed decisions/statuses here first, and reproduce
the numbered table below with its existing numbers, status labels and formatting.
Do not maintain a separate chat-only version. Preserve supporting details in
this file or its linked technical documents. An unchanged status request does
not require a gratuitous file edit.

## Product and ownership

The product accepts SVG artwork through the team's website, with drawing scale
and placement settings. The Jetson retrieves approved jobs over **HTTPS**, gets
a validated object/platform pose from the teammate's AprilTag subsystem, checks
the complete motion, and draws the artwork on a known stationary cylinder or
cylindrical mug. It reports preview, progress, completion and clear failures;
software Stop holds the arm. Start with one color and a manually loaded pen.
Preserve separate strokes and keep the pen approximately normal to the surface.

The current robot has **six positioning motors and no separate gripper motor**.
Automatic pen pickup, extra actuators, arbitrary surface reconstruction and
multi-color tool changes are extensions, not dependencies for this product.
Initial geometry is a known constant-radius surface, with handles and fixtures
excluded from the drawing region. The report's front-180-degree drawing region
must be checked against the actual assembled arm's reachable poses.

| Owner | Deliverable and boundary |
| --- | --- |
| Web/server teammate | Hosted HTTPS service, job storage/endpoints and browser workflow. Existing `website/` is a canvas/SVG payload prototype, not proof of an implemented robot job queue. |
| AprilTag teammate | Camera acquisition/calibration and tag-based pose estimation, with timestamps, frames and quality/validity output. Exact exported interface still needs agreement. |
| Robot software | HTTPS client on Jetson; validation and job lifecycle; ROS adapter for tag output and frame calibration; drawing geometry, planning, execution, progress and Stop/recovery integration. |
| Shared hardware/integration | Fixture geometry/seating, camera-to-base registration, final pen calibration, operating limits and measured product acceptance. |

Pending owner measurements include the actual tabletop plane and edges in
`base_link`, alongside the new base-wing/camera transforms and installed tool
geometry. The example floor stays at its recorded location until those values
are supplied. Loaded shoulder/elbow lifting must also be checked physically;
startup’s verified 100% RAM torque allowance is not proof of adequate torque.

## Bringup complete; planning/pose layer added

**Steps 1–4 are complete for moving on to MoveIt: working and demo-tested.**
This closes the bringup foundation, not the full product or measured hardware
qualification. Accuracy, loaded duty and broader fault testing remain in steps
8 and 23. The pending base/camera geometry can require integration changes,
without reopening the completed motor bench work.

The current sliders are a joint commissioning tool. The Cartesian application now has a separate entry point, `ros2 run
pinchy_motion start_motion`, using the same startup and hardware stack without
automatically opening joint sliders. The complete drawing/job application is
still future work. Do not create a second servo-control path.

| Layer | Responsibility / code boundary |
| --- | --- |
| Bringup | Guided startup/zeroing, driver/controllers, measured joint states and transforms. `pinchy_bringup` assembles launches; the driver remains in `pinchy_hardware`. |
| MoveIt configuration | `pinchy_moveit_config`: live-model planning group, full-pose IK, limits, collision setup and execution-controller mapping. |
| Motion application | `pinchy_motion` plus typed interfaces: XYZ/RPY pose plan/preview/execute, progress/results and powered Stop/recovery. Ordered path requests are implemented in step 11. |
| Drawing/job application | Separate application logic for HTTPS jobs, AprilTag registration, artwork mapping and job lifecycle; exact package split remains to be implemented. |
| Product launch | `mock_motion.launch.py` starts the mock pose workflow; `start_motion` enters the existing physical startup then opens planning. A future complete drawing-job launch can assemble additional components; application logic stays separate from bringup. |

Execution remains: **pose/path → MoveIt planning and IK → timed joint trajectory
→ existing ROS trajectory controller → existing motor driver**. Collision
checking and full-path validation belong above the motor driver; startup and
hardware protections remain active underneath.

## Pending base redesign — owner follow-up

November 13 owner update: the robot base is being modified with wings that
integrate a **marker holder and camera holder**. The owner will return with the
updated geometry and exact mounting transforms. These transforms are pending;
do not invent offsets or treat the current base/camera mounting model as final.
Keep the owner's term "marker holder" until its role/datum is confirmed; do not
assume it changes the moving sixth-joint pen holder or requires automatic pickup.

When available, incorporate the updated base/wing meshes and collision geometry,
confirm whether the arm's mounting frame changed, and add the supplied transforms
from a named base datum to each holder datum. State translation units, rotation
convention and frame directions. For the camera, distinguish the mount/body datum
from the optical frame and physically verify the resulting camera-to-base chain.
These changes affect MoveIt scene clearance and AprilTag registration (steps 9,
12 and 18–19). Preserve existing joint calibration unless the revised assembly
actually changes its mapping or reference poses.

**Step 7 software is now implemented:** `pointer_tip` is the owner-approved
endpoint of the existing RViz pointer, used as a temporary reference while only
the single servo-mounted attachment is installed. It is not a measured endpoint
of that attachment. Its offset from the sixth-joint link is separate from the
base-wing mounting transforms. See [tool frames](tool_frames.md). MoveIt setup
can proceed with this explicit proxy while the base redesign is pending.
Reconcile the new base geometry and transforms before physical planning around
the wings or camera registration.

## Required contact-speed behavior

During drawing contact, maintain approximately constant tool-tip speed along
the surface. Put acceleration/deceleration in pen-up motion off the object
wherever feasible: reach marking speed before contact and lift before terminal
braking. Do not stop at sampled contact waypoints. If a corner/short stroke
cannot meet this, plan a lift or obtain explicit approval for blending/slowdown
rather than silently changing the drawing. Safety stops remain exceptions.
The [versioned drawing policy](../src/pinchy_moveit_config/config/drawing_motion_policy.yaml)
records this requirement. Qualified speed/variation are TBD; the implemented
ordered-path interface validates modeled contact timing in mock, while physical
contact execution remains disabled pending qualification. Steps 8 and 12–17 must
establish measured drawing behavior. See [ordered paths](cartesian_paths.md).

## Ordered work through completion

✅ **Complete** · 🟨 **Partial** · ⬜ **Remaining** · 🔜 **Next**

Complete means demonstrated or implemented at the stated level. Step 7 is
software-complete using the authorized proxy, with physical agreement still
unmeasured. External integration identifies a teammate dependency, not a claim
about their implementation progress.

| # | Status | Task |
| --- | --- | --- |
| 1 | ✅ **Complete** | Hardware inventory and six unique motor IDs; individual and chained motor tests, labels and power-cycle checks. |
| 2 | ✅ **Complete** | Assemble the arm, integrate CAD 4, and record joint order, zeros, directions and initial bounds. Model/TCP accuracy still needs measurement in later steps. |
| 3 | ✅ **Complete** | Real and mock ros2_control; measured joint states, trajectory action, RViz and six-joint sliders. Owner successfully used the physical workflow in demos; bringup is complete for starting MoveIt. |
| 4 | ✅ **Complete** | Guided sequential zero/hold startup, operating speed/acceleration controls, live readouts, planned/measured graphs, recording, Stop/hold and explicit runtime fault recovery. Finite command-rate excesses are limited without a safety latch; defaults are 30 deg/s and 180 deg/s², with selectable caps of 60 deg/s and 360 deg/s². Live target telemetry uses a nonblocking worker and stamped updates. Physical fault/duty qualification remains in steps 8 and 23. |
| 5 | ⬜ **Remaining** | Confirm final scope and measurable acceptance: supported object/region, pen, artwork limits, placement/shape accuracy, speed and runtime. Treat report targets as targets until measured or explicitly revised. |
| 6 | ⬜ **Remaining — coordinate now** | Agree both integration contracts with the server and AprilTag teammates. Obtain example payloads and recorded pose samples so development can proceed with fakes. |
| 7 | ✅ **Software complete — RViz proxy** | Defined `pointer_tip` at the modeled pointer endpoint, with an active-tool profile, visible marker, geometry/FK checks and mock/emulated ROS TF validation. Owner authorized this temporary proxy; physical endpoint/model agreement remains unmeasured in step 8, and actual pen calibration is step 13. See [tool frames](tool_frames.md). |
| 8 | 🟨 **Partial** | Qualify assembled operation: usable joint ranges, loaded holding, speed/acceleration, temperature/current, tracking error, Stop and recovery. Startup is faster with explicit capture-window feedback and shared shoulder/elbow settling tolerance; loaded lifting still needs physical verification. Measure useful tool speed and low-speed stepping; selectable GUI limits are not certified operating limits. |
| 9 | ✅ **Software complete — current model** | Added `pinchy_moveit_config`: live calibrated model/limits, six-joint `arm` group, KDL plus bounded multi-start IK with explicit exact-pose/pen-direction/position modes, OMPL planning, reviewed adjacent-only self-collision exclusions, example floor and existing-controller mapping. Live goal handles, numerical input bounds, approximate reach samples and optional collision geometry are visible; floor placement remains unmeasured. Shared RViz view and hardware protections retained. Final base-wing/fixture geometry remains pending from the owner; reconcile it before physical planning near those parts. See [MoveIt handoff](moveit_handoff.md). |
| 10 | 🟨 **Software complete — physical tests pending** | Implemented synchronized live XYZ/RPY fields/RViz handles and CLI plus typed plan/execute interfaces: explicit IK constraints, preview-only planning, optional straight TCP LERP/SLERP transit with partial-path rejection, checked timed paths, explicit execution, current pose/progress, powered Stop/cancel and recovery. Mock/emulated validation covers rejection and execution; the new guided entry preserves holds through planner startup on a fake bus. Nearby physical pose/orientation/repeatability checks await the operator, using the temporary pointer proxy with its unmeasured attachment offset. See [Cartesian guide](cartesian_control.md). |
| 11 | 🟨 **Software implemented — qualification pending** | Ordered waypoint queue, typed PlanPath service and JSON CLI; continuous free-twist IK, polyline/explicit spline traces and OMPL visits, full collision/trajectory validation, preview and existing powered execution. Mock contact timing holds modeled TCP speed through samples with explicit pen-up ramps; sharp contact corners, incomplete paths and infeasible speed are rejected. Physical contact remains disabled until pen/surface qualification; measured speed/variation and loaded operation remain pending. Added bounded whole-stroke alternative configuration search from the fixed held start, with complete-candidate timing/interpolation/collision checks. The recorded broad arcs still fail; robust starting-posture selection and collision-planned pen-up repositioning remain unresolved. Keep IK/continuity as the priority before other feature work; see step 12. See [ordered paths](cartesian_paths.md). |
| 12 | 🔜 **Next — feasibility gate / geometry pending** | Prioritize continuous drawing feasibility before job integrations. A current-model study found posture-dependent nearby failures, a continuation failure with a valid broader IK witness, singularity sensitivity, side-normal range concerns and broad-arc timing failures; front-180° coverage is not established. Read [motion feasibility](motion_feasibility.md), verify the real TCP/wrist ranges, then establish cylinder radius, height, frame, seating and fixture clearances. Check alternative starting postures and complete drawing/lift paths at useful speed. A narrower software timing example is not product qualification. |
| 13 | ⬜ **Remaining — pen needed** | Install the actual pen, calibrate tip/axis and holder compression/contact offset, update active tool visuals/collision geometry and check compliance. Prove repeatable contact without bottoming out. Encoder position is not a force measurement. |
| 14 | ⬜ **Remaining** | Generate surface-following poses from simple 2D lines/curves using cylinder arc length and surface normals. Add approach, contact, pen-up travel, stroke transitions and retract. Allow only intended pen-tip contact; keep holder/arm/fixture collision checks. |
| 15 | ⬜ **Remaining** | Parse the supported SVG subset, units/viewBox/transforms and artwork placement; sample native curves to an explicit geometric tolerance, preserve stroke boundaries/corners, and validate unsupported elements and input limits. Reject invalid jobs with clear reasons. |
| 16 | ⬜ **Remaining** | Build the robot job executor: validate → register → map → plan/preview → wait for Start → execute → complete/hold/fail. One active job; track measured progress and results. Honor hardware safety holds and discard interrupted motion on recovery; never silently resume an old drawing. |
| 17 | ⬜ **Remaining** | Draw from a local file on the fixed cylinder: start with dry runs, then single lines, curves and multiple separated strokes. Measure the marks and tune TCP, contact, mapping and operating speed. |
| 18 | ⬜ **Remaining — external integration** | Consume teammate AprilTag output through the agreed ROS adapter. Establish camera/base/platform/object transforms using the pending base-wing camera mount geometry and verified optical-frame offset, check timestamps/quality, reject stale or missing registration, and associate the approved plan with the specific object pose. Detection implementation belongs to the teammate. |
| 19 | ⬜ **Remaining — shared validation** | Verify registration against independent physical references over several placements and reseatings. Define behavior for occlusion, stale updates and detected movement during a job. A changed pose invalidates a preview/plan; never change the drawing frame underneath an executing trajectory. |
| 20 | ⬜ **Remaining — external integration** | Implement the Jetson HTTPS bridge: retrieve/claim a job, download artwork/settings, validate and stage locally, report preview/readiness/progress/results, and consume Start/Stop/cancel requests. Protect against duplicate execution, reconnect/retry errors, partial downloads and stale jobs. Use a fake server until real endpoints are ready. |
| 21 | 🟨 **Partial — external integration** | Connect the finished browser/server workflow: upload, object selection, scale/placement, preview, explicit Start, status/progress, Stop and rejection details. Existing canvas/SVG submission is a starting point. A browser Stop must reach the existing ROS hold mechanism. |
| 22 | ⬜ **Remaining** | Complete the end-to-end path: browser → HTTPS server → Jetson job → AprilTag registration → validated drawing trajectory → real motors, with status/results back to the server/browser. Demonstrate an entire job without terminal commands per stroke. |
| 23 | ⬜ **Remaining** | Validate failures and repeatability: invalid/oversized artwork, unreachable/colliding paths, tag loss/movement, HTTP disconnects/duplicates, servo faults, Stop, power loss and process restart. Establish supported stop/recovery behavior and verify interrupted jobs do not auto-run again. |
| 24 | ⬜ **Remaining** | Final measured acceptance and handoff: repeated drawings across placements, accuracy/speed/contact/duty results, documented limits, one normal operator entry point, dependency/config versioning, persistent job records and a reproducible final demo/runbook. |

## Interface contracts to agree early (step 6)

### HTTPS service: robot is the client

The server teammate supplies the URL/endpoints and deployment. We implement the
Jetson client and its ROS handoff; no external web request should directly write
to servo registers. Exact routes and polling cadence are still undecided.

Agree on:

- Stable job ID, payload/schema version, drawing format/bytes or download URL,
  content identity, object model/radius, scale/placement and units, and requested
  drawing speed/other supported settings. Define who converts SVG units to mm.
- How jobs are offered, claimed and acknowledged, and how the same job cannot
  run twice after a retry/reconnect/reboot. Initially allow one active robot job.
- Preview and readiness responses, explicit Start approval for the validated
  artwork/settings/object pose, progress/completion/failure reporting, and
  Start/Stop/cancel commands with acknowledgments. Edited settings invalidate
  the old plan. A queued job is not authorization to start moving immediately.
- HTTPS certificate validation, authentication if required by the hosted
  service, request timeouts, download limits and retry/backoff behavior. Keep
  credentials outside versioned code and restrict artwork downloads to trusted
  sources. Validate complete content locally before any motion.
- Heartbeat/stale-command rules and network-loss policy. Local Stop/hold stays
  available without the server. Browser Stop depends on network delivery and
  acknowledgment; it is not a physical emergency stop. Do not automatically
  resume a drawing after reconnect.

### AprilTag subsystem: consume validated poses

Agree on:

- Actual topic/message or API, parent/child frames, transform direction,
  timestamps/clock basis, metres/radians, tag IDs/layout and fixture/object model
  identity. Confirm whether the output is camera→platform, base→platform or
  another transform; do not apply an extrinsic transform twice.
- Camera intrinsics and camera-to-base calibration ownership; platform-to-object
  geometry and seating. The robot needs a complete, checked transform into its
  base frame, not just image coordinates or tag IDs.
- Quality/uncertainty, detection validity and required visible tags. The report
  proposes at least two tags; confirm this with the teammate's actual estimator.
  A ROS message type alone does not define the quality policy.
- A sample recording, stale-data threshold, consistency checks, and the policy
  for missing tags or a moved object during execution. Plan against a validated
  pose snapshot; require a new check/plan if placement changes. Avoid reacting
  to normal pose-estimation noise by moving the path during contact.

## Final measurements and definition of done

The design report proposes the following; retain or revise them explicitly
against the physical system instead of claiming the demo already met them:

| Measurement | Design baseline / remaining decision |
| --- | --- |
| Pen-tip positioning | ≤2 mm at independently measured poses; report proposes 20 grid points across three heights. |
| Camera/fixture registration | ≤2 mm at physical surface references; report proposes ten placements including rotations, plus fixture reseating tests. |
| Drawing shape | Maximum stroke-centerline deviation ≤2 mm after translation/rotation alignment only, with no scaling/warping; every intended stroke and no unintended connecting marks, across five placements. |
| Drawing placement | Measure independently before scan alignment; final acceptance threshold remains to be chosen. |
| Reach and region | Report proposes ≥0.50 m radial tip reach, a 0.4 m working-region cube and the cylinder's accessible front 180 degrees. Verify actual reachable pen-normal and pen-up poses; revise scope explicitly if needed. |
| Input size | Provisional 50 strokes and 1 m mapped drawing path; verify supported SVG subset and workload limits. The existing website's 1 MB request limit is a separate constraint. |
| Motion quality | Measure continuous tool speed in mm/s, minimum useful speed, variation, path error, pen orientation tolerance and total drawing duration; thresholds remain to be chosen. |
| Loaded duty | Report proposes ten minutes at the most extended permitted pose plus acceleration/braking tests; verify temperatures, supply current and tracking without faults. |
| Fault handling | Rejected jobs never move; software Stop retains a powered hold where hardware permits; corrected runtime faults recover by explicit operator action. Power loss/firmware protection cannot guarantee a hold. |
| Operator workflow | Normal setup/start/preview/stop/recovery works without development commands; completed/interrupted jobs and relevant calibration/configuration are recorded. |

Overall success is a repeatable single-color drawing from a website-submitted
job on the registered physical object, within agreed measured limits, with
clear progress, Stop and failure behavior. A fixed-object/local-file test is an
integration step, not completion of the web/AprilTag product.

## Parallel work and immediate next actions

The numbered list is the dependency spine, not a requirement that every task
wait for the previous row. Server/tag contract discussions can proceed separately; the current owner
priority is to resolve IK/posture-dependent routing and drawing continuity
before implementing other product features. The AprilTag
teammate can provide recorded detections while the web teammate supplies sample
job/status exchanges. Use a known fixed object frame to develop mapping/contact
before their live components are available.

The report’s 0.4 m cube is the designated front working region, separate from
broad pose-input bounds. The owner-authorized blue box spans X [0.10, 0.50],
Y [-0.20, 0.20], Z [0.00, 0.40] m in base_link; its 0.10 m front offset is
provisional and not a certified collision-free reach envelope. Drawing IK must preserve the pen normal while
allowing axial pen rotation as described in report sections IV.B and VI.F.

Current directed point-to-point operation defaults to pen direction with free twist.
MoveIt searches twist variants before continuous bounded search; position-only
is explicit and never a fallback execution for failed direction requests.
Measured-state boundary settling is distinct from strict command limits; the
one-tick wrist-roll regression and live read-only XYZ/route checks are recorded.
The reported direction plans under CAD ranges but not the saved profile; physical
wrist-range review uses `calibrate --step review-limits --ids 4 5 6`, preserving
zeros/directions and requiring negative/positive endpoints. No saved bounds have
been expanded automatically.

See the [IK investigation](ik_investigation.md) for the calibrated-model benchmark,
condition warnings and explicit orientation/path modes. The blue cube is not an
arbitrary-pose reach guarantee; its far upper corners exceed the modeled chain
length even before considering collisions.

The owner recaptured motor 5 to [-143.96,+141.77]° and motor 6 to
[-0.70,+99.14]°, with both zeros/directions unchanged. All four previously reported
directed goals now plan in calibrated mock; physical execution and
repeatability still need operator checks. Guided startup has a ±90° outer
capture window clipped to saved/CAD ranges, with the same bounded ramp and
powered-hold monitoring. See the IK investigation for the recapture evidence.

The immediate software priority remains **steps 11–12**: qualify IK routing and
whole-stroke continuity under the contact-speed policy before other feature work.
The bounded alternative search is an initial implementation, not closure of
the saved broad-arc failures or full drawing coverage. Steps 9–10 now provide the software pose
workflow; the returning operator must first verify the nearby physical poses,
orientation and Stop behavior described in [the MoveIt handoff](moveit_handoff.md).
Step 7 uses an explicit proxy; physical tip/model agreement remains unmeasured.
Measure step 8 alongside this work. Install/calibrate the actual pen before
contact drawing and final accuracy tests. Do not repeat completed motor bench
setup as a prerequisite.
