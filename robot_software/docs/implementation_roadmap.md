# Final-product roadmap

Updated November 24, 2026. This is the current ordered plan through the finished
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
the complete motion, and draws the artwork on a known stationary surface. The owner-selected
nominal geometries are a flat panel, cylinder/cylindrical mug and concave quarter
pipe; dimensions and scope decisions are in [nominal geometries](nominal_geometries.md). It reports preview, progress, completion and clear failures;
software Stop holds the arm. Start with one color and a manually loaded pen.
Preserve separate strokes and keep the pen approximately normal to the surface.

The arm has **six positioning motors**. The owner is also developing a grabber
using a small blue **Fitec FS90 PWM servo**; Jetson GPIO/PWM configuration and
grabber programming are now planned under step 13. Automatic pen pickup and
multi-color tool changes are not implied by this addition. An optional I2C ADC
for a purchased force-sensor setup follows pen calibration and servo programming;
its necessity and exact hardware remain to be confirmed. Arbitrary surface
reconstruction remains an extension.
The design report initially specified a cylinder/cylindrical mug; the owner
has now added the flat panel and concave quarter pipe as nominal geometries.
Handles and fixtures remain excluded from the drawing region. The report's front-180-degree drawing region
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
assume it requires automatic pickup. November 24: the owner also confirms ongoing
end-effector/pen-holder updates and will provide camera TF information and pen
locations relative to the holders. Track these explicitly in step 13a; supplied
geometry and measured transforms are required before updating the shared model.

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
rather than silently changing the drawing. The owner also permits smooth acceleration/braking on the surface at stroke
ends; the explicit `allow_contact_ramps` option reports these exceptions and
short ramp-only strokes. Interior contact cruise speed remains checked. Each
SVG pen-down subpath is intended as one continuous motion; see the
[stroke contract](stroke_planning.md). Safety stops remain exceptions.
The [versioned drawing policy](../src/pinchy_moveit_config/config/drawing_motion_policy.yaml)
records this requirement. Qualified speed/variation are TBD; the implemented
ordered-path interface validates modeled contact timing in mock, while physical
contact execution remains disabled pending qualification. Steps 12–17 and verification steps 23–24 must
establish measured drawing behavior. See [ordered paths](cartesian_paths.md).

## Ordered work through completion

✅ **Complete** · 🟨 **Partial** · ⬜ **Remaining** · 🔜 **Next**

Complete means demonstrated or implemented at the stated level. Step 7 is
software-complete using the authorized proxy, with physical agreement still
unmeasured. External integration identifies a teammate dependency, not a claim
about their implementation progress. Expanded caveats and pending inputs are
retained in [implementation notes](roadmap_task_notes.md) and the linked guides.
The owner accepted steps 8 and 10 as complete for moving forward on November 15.
Outstanding accuracy, repeatability, load, speed, thermal and safety measurements
are retained in verification steps 23–24; acceptance does not claim those
measurements have already passed.

| # | Status | Task |
| --- | --- | --- |
| 1 | ✅ **Complete** | **Motor setup** — Inventory, IDs, bench tests and power-cycle checks. |
| 2 | ✅ **Complete** | **Assembly and calibration** — Model, zeros, directions and bounds recorded; physical accuracy still needs measurement. |
| 3 | ✅ **Complete** | **ROS control** — Real/mock control, feedback, RViz and sliders demonstrated. |
| 4 | ✅ **Complete** | **Startup and safety** — Guided zero/hold, motion controls, recordings and Stop/recovery implemented; physical qualification remains. |
| 5 | 🟨 **Partial — nominal geometries selected** | **Scope and objects** — Panel 100 × 100 mm; cylinder Ø80 × 100 mm; quarter pipe R100 × L100 mm, 90°. Placement/acceptance pending. [Details](nominal_geometries.md). |
| 6 | ⬜ **Remaining — coordinate now** | **Team interfaces** — Agree HTTPS and AprilTag contracts; obtain sample payloads and recordings. |
| 7 | ✅ **Software complete — RViz proxy** | **TCP definition** — `pointer_tip` proxy implemented; physical tip agreement unmeasured. |
| 8 | ✅ **Complete — owner accepted** | **Hardware qualification** — Accepted for moving forward; remaining measurements carried into verification steps 23–24. |
| 9 | ✅ **Software complete — current model** | **MoveIt setup** — IK, collision planning and controller integration implemented; final scene geometry pending. |
| 10 | ✅ **Complete — owner accepted** | **Poses and saved points** — Functional workflow accepted for moving forward; refinements and physical measurements continue through steps 23–24. [Usage](point_sets.md). |
| 11 | ✅ **Software complete — dry-run tested** | **Continuous strokes** — Fixed-speed/range timing and execution pass mock/PTY tests; broader reliability remains in step 12. [Timing](bounded_speed_proposal.md). |
| 12 | 🟨 **Partial — reliability/coverage work remaining** | **Drawing feasibility and reliability** — Improve complete-stroke configuration selection, joint reserve and qualified drawing regions. [Work plan](complete_stroke_reliability.md). |
| 13 | ⬜ **Remaining — pen needed** | **Pen calibration** — Install pen; measure tip, axis and compliance; update the tool model. |
| 13a | ⬜ **Remaining — owner geometry/TF pending** | **Base and end-effector updates** — Incorporate revised geometry, camera TF, pen holder and pen-location frames. [Details](roadmap_task_notes.md#step-13a-base-and-end-effector-updates). |
| 13b | ⬜ **Remaining — grabber in development** | **Grabber servo programming** — Configure Jetson GPIO/PWM and control the Fitec FS90 grabber. [Details](roadmap_task_notes.md#step-13b-grabber-servo-programming). |
| 13c | ⬜ **Optional — after calibration and servo programming** | **Force-sensor ADC** — Integrate the purchased I2C ADC and sensor if needed; hardware details pending. [Details](roadmap_task_notes.md#step-13c-optional-force-sensor-adc). |
| 14 | ⬜ **Remaining** | **Surface mapping** — Map lines/curves onto the plane, cylinder and quarter pipe, including normals and pen-up transitions. |
| 15 | ⬜ **Remaining** | **SVG import** — Handle units, transforms and curves; preserve strokes and reject unsupported input. |
| 16 | ⬜ **Remaining** | **Job executor** — Validate, preview, Start, execute and report results; one active job with explicit Stop/recovery. |
| 17 | ⬜ **Remaining** | **Physical drawing** — Local-file dry runs, then lines, curves and separate strokes; measure the marks. |
| 18 | ⬜ **Remaining — external integration** | **AprilTag integration** — Consume teammate poses through verified camera/base transforms; reject stale registration. |
| 19 | ⬜ **Remaining — shared validation** | **Registration testing** — Measure placement/reseating error; invalidate plans when the object moves. |
| 20 | ⬜ **Remaining — external integration** | **HTTPS client** — Retrieve/claim jobs and report status; handle duplicates, downloads and reconnects. |
| 21 | 🟨 **Partial — external integration** | **Browser workflow** — Connect upload, placement, preview, Start, Stop and progress; prototype exists. |
| 22 | ⬜ **Remaining** | **End-to-end integration** — Browser → HTTPS → registration → checked trajectory → drawing, without per-stroke terminal commands. |
| 23 | ⬜ **Remaining** | **Verification** — Check accuracy/repeatability, loads, speeds, temperatures, faults and recovery; prevent unintended replay. |
| 24 | ⬜ **Remaining** | **Final acceptance** — Measure quality, speed, accuracy and duty; deliver the reproducible operator workflow. |

November 24 step-12 follow-up: the requested horizontal 100 mm circle and exact
four-stroke square at the paper center are saved with 4–18 mm/s drawing timing.
After the owner's wrist-flex recapture, strict-normal circle/square previews
still fail to connect complete paths. A 5°-tilted circle passes a full mock
preview but has only about 0.16° wrist-flex reserve; square tilt witnesses are
numerical only. The optional normal cone remains analysis-only, not implemented.
The [complete-stroke work plan](complete_stroke_reliability.md) is now the active
step-12 follow-up, with geometry/calibration dependencies in 13/13a and physical
acceptance in 23–24. Existing [surface coverage results](validation/feasibility/surface_gate/validation.md)
remain valid at their documented scope; full cylinder 180° coverage and physical
contact remain unresolved. See [paper placement and files](tabletop_paper_examples.md).

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

The immediate priority is **step 12**: review the proposed physical supports and
resolve the front-180° requirement. Nine boundary/height paths and six complete
mock/driver-emulator executions now pass with proposed supports. At the current
cylinder placement a necessary reach bound rules out part of the requested
front 180°; a 58-placement search and 48-start coupled placement/configuration
optimization did not establish an alternative. Neither is a global proof.
See the [expanded gate evidence](validation/feasibility/surface_gate/validation.md).
Measured tool/table/fixture/base-wing geometry and a placement/tool change or
explicit coverage decision are still needed before treating the gate as closed. Step 11 now
passes continuous cylinder execution on mock and PTY at 3 mm/s, with every
interior waypoint traversed at speed; physical speed/contact measurements
remain in steps 13 and 23–24.
Whole-stroke optimization and explicit start selection now resolve the two
recorded broad cases at 3 mm/s with gentler ramps; they do not establish full
drawing coverage or physical qualification. Steps 9–10 now provide the software pose
workflow; the returning operator must first verify the nearby physical poses,
orientation and Stop behavior described in [the MoveIt handoff](moveit_handoff.md).
Step 7 uses an explicit proxy; physical tip/model agreement remains unmeasured.
Measure step 8 alongside this work. Install/calibrate the actual pen before
contact drawing and final accuracy tests. Do not repeat completed motor bench
setup as a prerequisite.
