# Final-product roadmap

Updated November 13, 2026. This is the current ordered plan through the finished
product, superseding the older checkpoint-only task list. The owner reports
that the assembled arm, startup, ROS control and sliders worked well for the
demos. This confirms demonstrated operation; it does not establish measured
Cartesian accuracy or continuous-duty qualification.

Use the [technical handoff](checkpoint_handoff.md) for hardware details,
calibration, commands, code locations and the existing runtime safety contract.
Use the [point/path plan](point_motion_plan.md) for Cartesian planning detail.
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

## Bringup complete; next layer is planning

**Steps 1–4 are complete for moving on to MoveIt: working and demo-tested.**
This closes the bringup foundation, not the full product or measured hardware
qualification. Accuracy, loaded duty and broader fault testing remain in steps
8 and 23. The pending base/camera geometry can require integration changes,
without reopening the completed motor bench work.

The current sliders are a joint commissioning tool. The normal product will
have a separate top-level entry point using the same startup and hardware stack;
sliders will be optional. Do not create a second servo-control path.

| Layer | Responsibility / code boundary |
| --- | --- |
| Bringup | Guided startup/zeroing, driver/controllers, measured joint states and transforms. `pinchy_bringup` assembles launches; the driver remains in `pinchy_hardware`. |
| MoveIt configuration | Planned `pinchy_moveit_config`: planning group, IK solver, limits, collision setup and execution-controller mapping. |
| Motion application | Planned separate motion package: pose/path requests, plan/preview/execute, progress/results and Stop. |
| Drawing/job application | Separate application logic for HTTPS jobs, AprilTag registration, artwork mapping and job lifecycle; exact package split remains to be implemented. |
| Product launch | A future normal-operation command starts the required components together. It may live in `pinchy_bringup`; planning/drawing logic stays in its own packages. No new product launch command exists yet. |

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
| 4 | ✅ **Complete** | Guided sequential zero/hold startup, operating speed/acceleration controls, live readouts, planned/measured graphs, recording, Stop/hold and explicit runtime fault recovery. Finite command-rate excesses are limited without a safety latch; defaults are 30 deg/s and 180 deg/s². Live target telemetry uses a nonblocking worker and stamped updates. Physical fault/duty qualification remains in steps 8 and 23. |
| 5 | ⬜ **Remaining** | Confirm final scope and measurable acceptance: supported object/region, pen, artwork limits, placement/shape accuracy, speed and runtime. Treat report targets as targets until measured or explicitly revised. |
| 6 | ⬜ **Remaining — coordinate now** | Agree both integration contracts with the server and AprilTag teammates. Obtain example payloads and recorded pose samples so development can proceed with fakes. |
| 7 | ✅ **Software complete — RViz proxy** | Defined `pointer_tip` at the modeled pointer endpoint, with an active-tool profile, visible marker, geometry/FK checks and mock/emulated ROS TF validation. Owner authorized this temporary proxy; physical endpoint/model agreement remains unmeasured in step 8, and actual pen calibration is step 13. See [tool frames](tool_frames.md). |
| 8 | 🟨 **Partial** | Qualify assembled operation: usable joint ranges, loaded holding, speed/acceleration, temperature/current, tracking error, Stop and recovery. Measure useful tool speed and low-speed stepping; selectable GUI limits are not certified operating limits. |
| 9 | 🔜 **Next** | Configure MoveIt 2: six-joint group, active tool/base frames, IK, joint limits, controller connection, self-collision review and planning-scene geometry for the table/fixture. Updated base wings/holder geometry and mounting transforms are pending from the owner; reconcile them before physical planning near those parts. Use the current ROS stack rather than rewriting IK/dynamics. |
| 10 | ⬜ **Remaining** | Add plan/preview/execute for a Cartesian pose in a named frame. Validate orientation, reach, collision and fresh robot state; reject invalid requests before movement. Test mock first, then nearby physical moves with the temporary pointer proxy explicitly distinguished from the unmeasured attachment tip. |
| 11 | ⬜ **Remaining** | Add ordered Cartesian paths with continuous IK and full-path validation. Distinguish visiting poses from tracing a path; reject incomplete paths and joint jumps. Keep approximately constant tool speed through intermediate samples, with measured minimum useful speed, stated tolerance and explicit corner/start/end exceptions. |
| 12 | ⬜ **Remaining — geometry pending** | Establish a known fixed cylinder and fixture model: radius, height, frame, seating, drawing region and clearances. Prove all drawing and lift/travel poses are reachable. This fixed-frame setup allows drawing development before tags arrive. |
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
wait for the previous row. Agree server/tag contracts and build their fake
inputs now; implement the HTTPS adapter alongside motion work. The AprilTag
teammate can provide recorded detections while the web teammate supplies sample
job/status exchanges. Use a known fixed object frame to develop mapping/contact
before their live components are available.

The next robot implementation is **steps 9–11**: MoveIt setup using the explicit
`pointer_tip` proxy, Cartesian poses and continuous paths. Step 7 software is
implemented; the real attachment/TCP comparison remains unmeasured. Measure
step 8 alongside that work. Install/calibrate the actual pen before contact
drawing and final accuracy tests. Do not repeat completed motor bench setup as a prerequisite.
