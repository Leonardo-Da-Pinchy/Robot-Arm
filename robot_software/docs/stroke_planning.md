# Continuous stroke operation and planning contract

Owner clarification, November 14, 2026: each continuous pen-down SVG stroke
should be one continuous motion, rather than a sequence of stopped point moves.
An SVG path element can contain multiple moveto-separated subpaths; those are
separate strokes and must not acquire unintended connecting marks.

## Intended operation

1. Map the stroke to the selected surface, preserving geometry and stroke
   boundaries. Request XYZ and the pen direction normal to that surface. Rotation
   about the round pen axis is free; it is not a request to ignore the normal.
2. Sample to a geometric tolerance and check the complete stroke before moving.
   Choose configurations jointly, accounting for joint reserve, collisions,
   continuity, singularity sensitivity and the actual timed joint trajectory.
3. Retry alternative twists/branches and starting postures when a branch fails.
   A valid independent IK witness is not a connected stroke. A different start
   requires a separately collision-planned pen-up approach from the held state.
4. Preview the complete approach and stroke, then execute only on explicit Start.
   Prefer approximately constant TCP surface speed through the stroke interior.
   Do not stop at sampled interior waypoints or silently alter artwork/normals.
5. Prefer pen-up acceleration/braking where feasible. The owner explicitly permits
   smooth acceleration/braking on the surface at stroke ends as well. Report
   endpoint ramps and short strokes that cannot reach cruise speed. This does
   not authorize arbitrary interior slowdown or a silent lower requested speed.
6. If no complete candidate passes, reject without partial motion and explain
   whether the difficulty is task constraints, branch continuity, collision,
   position bounds or timing. A proposed lift/split, geometry change or lower
   speed must be visible before execution. Stop/fault recovery never resumes an
   interrupted drawing automatically.

An exact sharp corner at nonzero speed requires infinite acceleration. Retain
current explicit curve/lift/corner handling; do not hide corner blending.

## Implemented planning options

`PlanPath` keeps its existing fixed-held-start tracing by default. Three opt-in
fields expose stroke setup, endpoint ramps and strict dry-run timing:

- `plan_pen_up_approach: true`: waypoint 0 is the trace start; its incoming
  contact label must be false. Search starting IK configurations against the
  entire remaining trace. Choose only a candidate whose timed trace and OMPL
  pen-up approach both pass collision/trajectory checks. The combined preview
  starts at the original held command and uses the existing ExecutePlan action.
- `allow_contact_ramps: true`: permit the same smooth speed law's endpoint ramps
  during contact. Validate modeled speed against that law on ramps and within
  5% of requested cruise speed on the plateau. Contact speed extrema include
  the ramps, so a minimum of zero is expected when contact starts from rest.
  Ramp-only short strokes are explicitly reported. Joint limits remain strict;
  contact trajectories are not uniformly slowed behind the caller's back.

- `enforce_tcp_speed: true`: require the requested cruise speed for a continuous
  pen-up dry run too. Reject hidden slowdown, short ramp-only traces and sharp
  polyline corners. Report checked cruise extrema and trace/cruise time windows.

All three fields are available in the JSON CLI and panel. Rebuild interface and
motion packages, then restart the planning server, panel and other interface
clients together. Do not automatically restart physical bringup. Physical
contact-tagged requests remain disabled pending tool/contact qualification.

`stroke_search.py` supplements local continuation with sparse joint optimization
of a complete branch, exact task reprojection and adaptive jump checks. The
start-search option can change the initial configuration because a checked
pen-up route is included; ordinary fixed-start traces retain the exact start.
An interior bottleneck can also seed a candidate connected backward and forward
along the same curve; only complete histories can proceed to validation.
Start-candidate generation uses a separate cache so continuation history cannot
consume its seed budget. Optimization uses the actual spacing between samples,
and dense exact reprojection controls nonlinear FK/speed error. The joint spline
preserves endpoint curvature instead of assuming zero joint curvature on a
curved Cartesian task. The existing controller interpolation, scene/model/start checks and powered
Stop remain authoritative. Finite search is not a completeness guarantee.

## Reachability and tool length

Six positioning axes do not imply every Cartesian pose is reachable. Link
lengths, joint ranges, self/object collisions and singularities restrict both
endpoints and connecting paths. Exact XYZ/RPY constrains six quantities. XYZ
plus a normal constrains five and leaves one local degree of freedom, but that
freedom can run out at bounds or require a different disconnected branch.

The current `pointer_tip` is an unmeasured RViz proxy. Its axial holder-frame
offset is about 149 mm; that is not a measured total pen length. Shortening that
offset changes required wrist positions and can improve travel/lever arm while
reducing holder clearance. Lengthening can improve clearance while harming
reach or conditioning. Neither guarantees a continuous stroke. Tool lengths must
be compared at the same surface tasks with holder geometry, joint ranges and
complete paths checked. Do not edit the active TCP to manufacture a passing
result; measure the installed tool and update model/collision geometry together.

See [nominal geometries](nominal_geometries.md), [motion feasibility](motion_feasibility.md)
and the canonical [roadmap](implementation_roadmap.md).

The [two-inch cylinder test](continuous_cylinder_test.md) provides an executable
continuous dry run and repeatable live-controller speed qualification.


## Bounded-speed timing

November 22: the owner authorized implementation of the optional minimum/maximum
TCP range. It is now implemented in planner, panel, saved point sets and CLI;
fixed speed remains the default. See [the implemented contract](bounded_speed_proposal.md).
Range mode permits smooth interior speed variation only inside its explicit
band; it does not permit waypoint stops, a hidden lower minimum, changed
geometry/normals or altered safety behavior. Endpoint-ramp exceptions remain
explicit, and physical contact remains disabled pending qualification.
