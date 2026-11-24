# Drawing motion feasibility investigation

**Finding: there is a real coverage/continuity concern in the current model.**
It is not established that the required front 180° can be drawn. Some nearby
pose failures are posture-dependent; some example side-normal poses remain
unsolved even with an independent numerical search. Solving individual XYZs
is insufficient. This investigation sends no real motor commands and changes
no zero, direction, position bound, safety policy or active collision scene.

The design report requires inward surface normals with pen twist free, preserves
stroke geometry, and calls for five perpendicular poses across the front 180°
plus measured drawing accuracy. It supplies no final cylinder size/placement.
At the time of the investigation the owner had not selected those. The owner
has since selected [nominal geometries](nominal_geometries.md), including an
80 mm diameter × 100 mm tall cylinder/mug. The historical examples below used
a hypothetical 80 mm diameter cylinder; they remain unapproved physical
placements and do not qualify the newly selected full object geometries.
The shared `pointer_tip` is still an unmeasured RViz proxy with a 149 mm axial
offset from joint six. Verify the real pen/TCP and physical joint ranges before
deciding that mechanical changes are required.

## Current planner follow-up

The original investigation and first bounded beam attempt below are historical.
The subsequent [stroke planner](stroke_planning.md) now passes both recorded
broad arcs at an explicitly requested **3 mm/s with 20 mm/s² TCP ramps**, retaining
saved position bounds and default **30 deg/s / 180 deg/s²** joint caps.
The first case needs another starting configuration and a checked pen-up route;
the second can pass whole-stroke optimization from its original held start.
Independent mock collision/FK and actual controller-polynomial validation passed.
This resolves those cases at those settings, not their original 10 mm/s request,
full front-180° coverage, the selected entire objects or physical contact.

See [new results](validation/feasibility/stroke_start_search/report.json) and
[validation details](validation/feasibility/stroke_start_search/validation.md).

The later [nominal-surface study](validation/feasibility/nominal_surfaces/validation.md)
adds actual example object/fixture collision solids, two-inch loops on all three
geometries and separately checked retracts. The cylinder also passes live
controller execution on mock/PTY with strict 3 mm/s cruise. Wider endpoint
sampling still leaves many targets unresolved; physical/full-area coverage is
not established. Historical claims below about missing object solids describe
the original investigation, not these new opt-in scenes.

The [expanded surface gate](validation/feasibility/surface_gate/validation.md)
now resolves the assumed fixture conflicts using explicit support proposals,
passes nine complete boundary/height paths and six mock/PTY system executions
with retracts, and introduces a necessary wrist-reach bound. At the current
cylinder placement, +75°/+90° cannot fit the upstream link reach even with
unrestricted joints and free axial twist. Other unresolved points remain finite
search outcomes; alternate placement feasibility is not globally disproven.

## What the original investigation found

| Question | Evidence / practical consequence |
| --- | --- |
| Can a twist make the next move difficult? | Yes. A continuous-arc search at center X=0.40 m, Z=0.20 m stopped at 52.9%; independent search found collision-checked IK for the **exact failed intermediate target** in about 0.08 s, in a different configuration. This is a continuation/planning limitation, not an unreachable-point result. At zero, 3 of 10 ±2 mm / ±5° neighbours failed a local step search but had solutions in a broader search. A different configuration can require a large joint change; it is not automatically connected while tracing the same stroke. |
| Can free twist create more headroom? | Yes. A sampled pose near wrist-roll's end had only 0.57° reserve; a different configuration at the **same XYZ and direction** had about 17.19° minimum reserve. This is a configuration witness, not an authorized repositioning trajectory. |
| Are there genuine near-singular configurations? | Yes. One collision-valid posture had normalized five-dimensional task condition number about 273. A sampled alternative at the same task improved it to about 103. The local neighbour search improved from 5/10 to 6/10; other neighbours required broader configuration changes. High condition number is a warning, not proof that every movement fails. |
| Is the whole front 180° covered? | Not demonstrated. An IK-only grid over 15 example placements/heights found at most 9/13 sampled angles per row. Red/unresolved samples are finite search outcomes, not proofs of impossibility. Some front-center samples also fail at closer placements. |
| Are the side failures only numerical? | For center X=0.35 m, radius=0.04 m, surface Z=0.15 m, a side-normal target retained about 9.45 mm / 14.69° combined residual under both bounded search and independent differential evolution. A hypothetical expansion of **only joint 6** to 180° produced geometric solutions around +124°, beyond its saved +99.14° and CAD +100°. Those diagnostic configurations were not collision checked, authorized or sent to hardware. |
| Does reachable IK guarantee constant speed? | No. A sampled 120° arc had collision-valid IK and independent FK agreement throughout, but the selected continuous joint branch touched limits, demanded steep wrist rotation and failed controller-polynomial timing/bounds checks at 10 mm/s. Retrying sampled alternative starts did not establish a passing version. |
| Can any curved, changing-normal motion maintain speed? | A narrower off-center 35° segment, angles −45° to −10°, passed the shared interpolation/collision timing assay at 5 mm/s with 20 mm/s² ramps and 30°/s, 180°/s² joint caps. Modeled steady speed was 4.981–5.053 mm/s. It still approached a joint limit, so this is not a robust operating envelope or physical speed qualification. |

The condition number uses the existing task Jacobian with translation scaled
by 0.4 m. It applies to XYZ plus pen direction (five constraints), not exact RPY
(six constraints). Euler RPY near ±90° pitch can jump without a corresponding
robot singularity; the solver and these checks use matrices/unit directions.

A wrist flip/branch difficulty, a joint limit, a collision and a singularity are
different problems. A goal can have IK elsewhere in joint space but lack a
continuous collision-free connection from the current posture. A small XYZ
change can also request a large pointing-direction change. Do not equate either
with numerical solver failure or silently remove the direction constraint.

## What was added

- [tools/analyze_motion_feasibility.py](../tools/analyze_motion_feasibility.py): repeatable saved-range neighbourhood,
  twist-family, arc-continuation and timing analysis with JSON and a plot. It
  never creates an execution action client or opens a serial port.
- [pinchy_motion/feasibility.py](../src/pinchy_motion/pinchy_motion/feasibility.py): exact cylinder-arc/normal generation, same-task
  nullspace exploration, joint margins and necessary sampled rate estimates.
- Current-pose UI warns about less than 5° travel reserve; **Joint limits /
  bounds** now includes a Reserve column. These are readouts, not new stops.
- Pose/path preview messages report low reserve anywhere along the actual JTC
  interpolation, including extrema between waypoints. Incomplete traces retain the exact failed target and try a separate diagnostic
  IK search. If it succeeds, runtime errors explicitly distinguish a local
  continuation failure from an unreachable point; this diagnostic never becomes
  an executable plan. Visit poses is for pen-up routing.

Alternative twists are **not automatically executed**. A pen-up approach to a
better starting posture needs collision-planned repositioning, and the complete
remaining stroke must be checked. Looking ahead through the stroke and choosing
configurations jointly is preferable to greedily choosing the nearest solution
at each sample. The original investigation used local runtime continuation; its diagnostic
made failures visible without establishing global path completeness. The
bounded search added afterwards is described below and does not close the gate.

## Run the analysis

From `robot_software/`, with ROS and the workspace sourced:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 python3 tools/analyze_motion_feasibility.py \
  --moveit --sweep --global-endpoint --output-dir /tmp/arm-feasibility
```

This may take several minutes. `--moveit` launches a separate calibrated **mock**
on an isolated domain; it does not connect to the real driver or alter its
scene. Without that flag the results are IK-only and explicitly unqualified for
collisions. Close-to-limit feedback outside the saved command interval is not a
valid input configuration for this offline test.

For a quicker generic neighbourhood check, use `--neighbourhood-only`. Add
`--joint-deg J1 J2 J3 J4 J5 J6` to diagnose a particular configuration copied from
the joint-limit window, in base-to-tip order. Values must fit saved bounds.
`--reuse-sweep /path/report.json` reuses only an IK sweep with matching
model/profile hashes. `--tcp-speed` selects the main numerical timing assay in
m/s; the separately labeled narrow example uses up to 5 mm/s and 20 mm/s².

The output `report.json` distinguishes local success, broader-search witnesses,
configuration margins, conditioning, continuation and timing. `feasibility.png`
shows sampled arc coverage and neighbourhood comparisons.

## Limits of the evidence

MoveIt checked the shared robot self-collision geometry and the existing example
floor. **No cylinder, fixture, final tabletop or base-wing collision geometry was
added.** Independent MoveIt FK checked the reported continuation states. Timing
assays validate the actual joint-controller polynomials, bounded joint rates,
approximate TCP geometry and the interior steady-speed portion (5% tolerance).

The numerical assay's ramps lie on the hypothetical arc; they are **not approved
pen-up approaches**. No contact execution plan is created. Real drawing needs
off-surface acceleration/lift/retract geometry and object/holder clearance.
The 5 mm/s result is not an approved physical drawing speed, minimum useful
speed, proof of contact accuracy, or proof of coverage of arbitrary artwork.
Small numerical reserve and finite sampling leave sensitivity to model error.

## Next engineering decision

Treat continuous drawing feasibility as the priority before HTTP/job features:

1. Measure the final installed pen/TCP and verify actual joint ranges/model
   correspondence, especially the wrist. Relabeling zero alone does not create
   extra physical travel; correct frame origins must still represent the arm.
2. Select the object size/placement and intended marking band. Add its actual
   collision/fixture geometry and pen-up clearance.
3. Search alternative approach postures and configurations across **whole
   strokes**, including lookahead, reserve, collisions and requested speed.
   Verify any proposed runtime planner improvement against these failing arcs.
4. If the required region still needs unavailable travel, explicitly decide on
   placement, reduced drawing region, supported pen-up transitions or mechanical
   changes. Do not silently shrink the advertised region or loosen normals.
5. Physically measure speed, low-speed stepping, tracking, contact and repeatability.

Evidence: [full report](validation/feasibility/current_model/report.json),
[summary plot](validation/feasibility/current_model/feasibility.png),
[independent endpoint check](validation/feasibility/independent_endpoint_global.json),
[narrow timing assay](validation/feasibility/narrow_segment_slower_ramp/report.json),
[exact continuation failure and broader witness](validation/feasibility/continuation_failure/report.json).

Validation passed: nine diagnostic unit cases, three UI checks, seven path
regressions and two mock ROS workflow regressions. The collision/FK investigation
used isolated mock services; no physical motion or automatic restart was performed.


## Historical first bounded whole-stroke selection follow-up

The runtime now calls `paths.select_trace`: retain the fast continuation if it
passes; otherwise explore local task-nullspace alternatives with a beam of up
to four distinct endpoint configurations/history paths. Every history begins at
the exact held start. Candidate steps remain within 0.15 rad and strict saved
bounds, with MoveIt state checks. The search ranks travel, changes between steps
and joint reserve, and keeps complete histories before selecting a result.
Exact RPY has no free twist; position-only remains explicitly selected.

A complete candidate must pass the existing controller-polynomial bounds,
geometry and speed validation plus sampled MoveIt collision checks before it
can win. Failed complete candidates allow another retained history to be tried.
The final gateway validation, start/model/scene checks and explicit Execute
remain in place. Planning logs record selection method and candidate rejection
reasons. Cancellation/model changes and the existing 45-second trace budget
still reject without a partial plan. There is no new motor writer or automatic
repositioning, and contact remains mock-only.

This is **partial planner progress**, not robust whole-stroke starting-posture
selection. Beam pruning can lose useful configurations; rank costs do not
prove singularity avoidance or requested speed. Corners split pen-up legs;
selection does not search jointly across those legs. It does not search an
alternate initial posture and collision-plan an approach to it. That remains
necessary when the held posture cannot connect to a useful drawing branch.

Repeat the comparison with ROS/workspace sourced:

```bash
OPENBLAS_NUM_THREADS=1 python3 tools/check_stroke_selection.py \
  --moveit --output-dir /tmp/stroke-selection-checked
```

[Recorded comparison](validation/feasibility/whole_stroke_selection/report.json):

| Case | Local / bounded selection result |
| --- | --- |
| Center X=0.40 m, Z=0.20 m, −60° to +60° | Both fail; original 52.9% continuation witness reproduced. No complete alternative was retained. |
| Center X=0.35 m, Z=0.15 m, −60° to +60° | Both fail wrist-flex interpolation bounds; the retained complete alternative also fails. |
| Center X=0.35 m, Z=0.15 m, −45° to −10° | Both pass the same narrow numerical contact-speed assay; fast continuation is retained. |

The comparison uses saved ranges, the shared mock collision scene and
independent MoveIt FK for accepted candidates. It is not coverage, fixture,
physical-speed or contact qualification. The numerical ramps still lie on the
hypothetical arc and are not approved pen-up geometry.

Validation also includes a controlled branch trap that needs an early twist
choice, real-model alternative selection through the shared timed validator,
collision/exact-mode rejection and cancellation. A pre-existing operator replay
incorrectly expected a trace for rounded coordinates within micrometres of its
current XYZ; it now checks duplicate-point rejection and separately requests a
real 2 mm translation with the deliberately copied direction. This changes the
test fixture, not runtime direction constraints or duplicate-point policy.

The [validation record](validation/feasibility/whole_stroke_selection/validation.md)
records 12 path, 10 IK and nine feasibility cases plus five mock workflow cases.

The IK layer remains in the right architectural location (KDL/bounded candidates
above MoveIt collision/routing and the shared controller), but the drawing
feasibility gate is **not ready to close**. Continue posture/whole-stroke work
before HTTPS/job features. No physical motors moved or were restarted.


## Whole-stroke optimization and explicit pen-up start selection

`stroke_search.smooth_trace` jointly adjusts a complete joint history using
sparse task residuals, joint smoothness and travel reserve. Exact reprojection
restores strict XYZ/normal constraints, with adaptive sampling and the same
0.15 rad step guard. A low optimizer cost is never acceptance: the timed
controller polynomial, task geometry, requested contact speed and collisions
must all pass. Ordinary traces keep the held start fixed.

`plan_pen_up_approach` explicitly allows `select_stroke` to retry different
starting configurations against the whole remaining path before committing to
an approach. The caller collision-plans a pen-up OMPL route from the held state
to the winning first configuration. Its exact endpoint, zero boundary
derivatives, controller interpolation and collisions are validated before the
combined preview token exists. Stop/model changes still abort; no partial trace
or unchecked same-task joint jump is executable. This search now has a 360-second
setup budget; it is finite and can still miss useful branches.

The [owner's stroke contract](stroke_planning.md) permits smooth contact endpoint
ramps when explicitly requested. Off-surface ramps remain preferred. Interior
contact speed, normal direction and drawing geometry remain checked, with no
silent speed scaling for contact. Physical contact is still disabled.

| Exact historical arc | Passing follow-up at 3 mm/s, 20 mm/s² |
| --- | --- |
| X=0.40 m, Z=0.20 m, R=0.04 m, −60° to +60° | Alternative start plus whole-stroke optimization; modeled steady contact speed 2.996–3.004 mm/s, with a collision-checked pen-up route. Fixed-start local/beam search still fails at 52.9%. |
| X=0.35 m, Z=0.15 m, R=0.04 m, −60° to +60° | Fixed-start whole-stroke optimization; modeled steady contact speed 2.990–3.008 mm/s. Explicit alternative-start mode also passes at 2.989–3.009 mm/s with a checked route. |

Reproduce the [report](validation/feasibility/stroke_start_search/report.json):

```bash
OPENBLAS_NUM_THREADS=1 python3 tools/check_stroke_selection.py --moveit \
  --tcp-speed .003 --tcp-acceleration .02 --budget 120 \
  --output-dir /tmp/stroke-start-checked-3mm
```

The numerical assay's ramps still lie on the hypothetical arc, not an approved
physical pen-up lead-in. The newly checked pen-up route reaches the selected
start but does not qualify actual object/holder clearance: the scene still
contains only the shared robot and example floor, with no final object,
fixture, tabletop or base-wing geometry. This is modeled software validation.
The original faster requests are not silently retimed to 3 mm/s. Three mm/s is
not established as a physically useful drawing speed or a minimum speed.

The comparison tool now explicitly caps its timing assays at the shared default
joint speed/acceleration; its earlier comparison used model maxima. Historical
reports remain unchanged. This distinction matters when comparing timing.

A [geometry-only pen-length comparison](validation/feasibility/stroke_start_search/pen_length_comparison.json)
varied only the axial holder-to-TCP offset in an in-memory model: 100, 125,
149 and 175 mm. Each of the first three solved all ten sampled XYZ/normal tasks;
175 mm solved nine. This finite five-angle-per-arc test neither establishes a
length optimum nor proves the unresolved sample impossible. It checks no
collisions, full strokes, actual pen body, stiffness or tool calibration. No
active TCP/model/calibration was changed. Choose installed pen length using
measured geometry and complete drawing/clearance checks, not endpoint counts.


Final [runtime preview events](validation/feasibility/stroke_start_search/runtime_broad_previews.json)
also pass both broad cases through the actual PlanPath service with 13 sampled
spline waypoints and explicit full-stroke contact endpoint ramps. Interior speeds
are 2.988–3.009 and 2.999–3.003 mm/s respectively. The initial exact-arc assay
above and final spline-request previews are distinct tests. Runtime integration
required isolating start-candidate seeds from the continuation cache, accounting
for nonuniform path sample spacing, preserving endpoint joint curvature, and
densely reprojecting exact task constraints. No validation tolerance was relaxed.
