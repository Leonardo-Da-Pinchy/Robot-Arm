# Saved point sets and fixed-TCP tests

Point sets are a lightweight commissioning workflow above the shared PlanPath /
ExecutePlan layer. They are not the final SVG/HTTPS job executor. Loading or
capturing points never sends a motor goal. Plan the complete queue, inspect the
RViz preview, then explicitly Execute when the operator intends to move.

## Panel workflow

- **Add goal** captures the XYZ/RPY fields into the queue.
- **Capture current TCP** captures fresh measured/model TCP feedback, independently
  of the goal fields. This is still the current unmeasured `pointer_tip` proxy.
- Select a queued row to inspect its fields and goal marker. **Update selected**
  writes edited fields back to that row while retaining its contact label.
- **Save points…** writes a reusable JSON file containing the poses, contact
  labels, orientation/path modes and absolute joint/TCP settings.
- **Load points…** validates the complete file before replacing the queue. It
  invalidates the old execution token; planning and execution remain separate.
- The example selector plus **Load example** stages an installed example. Saving
  an example defaults to a personal copy rather than overwriting the packaged
  template. Queue/file controls are blocked during an active plan or execution.
- **Plan queued path → inspect RViz → Execute previewed plan** executes the whole
  accepted queue through the existing trajectory controller. Powered Stop and
  recovery retain their existing behavior. No file resumes automatically.

Load/save does not expand limits, clamp points, change the active TCP or import
old timed motor commands. Wrong frames/units, invalid numbers/settings, more than
80 points and unsupported modes are rejected. The panel uses base_link, metres,
RPY degrees and pointer_tip. File loading on hardware rejects contact tags;
physical contact remains disabled pending qualification.

## Included examples

| Example | Behavior |
| --- | --- |
| [Fixed TCP: small tilts](../src/pinchy_motion/examples/point_sets/01_fixed_tcp_tilts.json) | Seven poses at XYZ approximately (0.422999, −0.011669, 0.277707) m; full orientations tilt ±3° about local tool X/Y and return. Uses 5 deg/s and 30 deg/s² joint caps. |
| [Small pen-up pose loop](../src/pinchy_motion/examples/point_sets/02_small_pen_up_square.json) | Five destinations around a 5 mm square near the same bent modeled posture. Routed pen-up visits stop at destinations and do not promise a square connecting TCP trace. Uses 5 deg/s and 30 deg/s². |
| [Captured session points](../src/pinchy_motion/examples/point_sets/03_captured_session_points.json) | The owner's seven poses recovered from the November 15 session's latest submitted queue. Pen-direction/free-twist visits; position/orientation/contact values match the log. Joint and TCP acceleration settings were not logged, so shared defaults are recorded. |
| [Cylinder continuous two-inch dry run](../src/pinchy_motion/examples/point_sets/04_cylinder_continuous_2in.json) | 41-point oval, 50.8 mm surface width × 50.8 mm height, 5 mm air gap, constrained normal, strict 3 mm/s cruise and checked pen-up start route. Passed mock/PTY execution. |
| [Plane continuous two-inch dry run](../src/pinchy_motion/examples/point_sets/05_plane_continuous_2in.json) | Same dimensions/speed on an upright nominal panel; complete mock preflight and separate 10 mm retract passed. |
| [Quarter-pipe continuous two-inch dry run](../src/pinchy_motion/examples/point_sets/06_quarter_pipe_continuous_2in.json) | Same dimensions/speed inside the nominal concave R100 mm quarter pipe; complete mock preflight and separate 10 mm retract passed. |

See [continuous test instructions](continuous_cylinder_test.md) for assumed
placements, scene presets, settings and repeatable qualifications.

All examples are staging data and require a new scene/start/model-checked preview.
They are not certified physical placements or contact jobs. The illustrative
bent posture and the owner's list use the current proxy TCP, not a calibrated
physical nib. Final object/fixture/table/base-wing clearance is still pending.

## Fixed TCP means checking the connecting motion

**Fixed TCP rotations + approach** (`style: fixed_tcp`) requires Exact RPY,
identical XYZ at every waypoint (within 1 micrometre) and pen-up labels. The first
leg is an ordinary collision-planned approach to point 1; it may translate.
Subsequent legs use MoveIt Cartesian interpolation and quaternion SLERP, so the
TCP stays at the common XYZ while orientation changes. There are intentional
stops at poses for this commissioning test; it is not a drawing stroke mode.

The actual timed joint-controller interpolation is checked after setting stop
boundaries and timing. Rotation legs must remain within 1 mm of the common
modeled TCP, obey full-pose interpolation tolerances, joint bounds/rates and
collisions, and satisfy every endpoint pose. A partial Cartesian leg rejects the
whole queue. There is no free-space detour fallback for a fixed-TCP rotation.
Joint speed/acceleration caps control timing; linear TCP-speed controls do not
apply while XYZ is held fixed and are disabled for this mode.

The example passed independent MoveIt FK with about 0.066 mm maximum sampled
model drift on mock and a real-driver/PTY emulator workflow (see validation).
This is not measured physical TCP accuracy. Servo tracking, model agreement,
actual pen length and compliance still need operator measurements.

## JSON and CLI

The same files can be planned without the panel:

```bash
ros2 run pinchy_motion pose path /absolute/path/to/points.json
```

The result provides a plan ID and a separate Execute command. Saving/loading does
not store or reuse an execution token. Files use schema_version 1, explicit
`frame`, `tcp_frame`, `units`, `orientation_mode`, `style`, `joint_speed_deg_s`,
`joint_acceleration_deg_s2`, TCP settings, and ordered `waypoints` containing
`xyz`, `rpy_deg` and `contact`. Absolute joint settings are checked against the
current session caps. Existing legacy CLI JSON with scaling fractions remains
supported by the CLI; panel imports require absolute caps to avoid ambiguous
speed changes between different live profiles.

The updated panel publishes a read-only transient-local JSON snapshot on
`/pinchy_motion/queued_points` (std_msgs/String). Read or export it without
planning or moving:

```bash
ros2 run pinchy_motion pose queue
ros2 run pinchy_motion pose queue --out /tmp/current-points.json
```

This exposes future queue changes for inspection. An old already-running panel
has no queue publisher; its latest *submitted* list can be recovered from the
planner log, but unsent edits cannot be inferred from that log.

## Recovered submitted list

These positions were recovered with the corresponding full RPY values and
contact flags in the linked file:

| Point | X (m) | Y (m) | Z (m) |
| --- | ---: | ---: | ---: |
| 1 | 0.405899 | 0.049812 | 0.246919 |
| 2 | 0.405899 | 0.105772 | 0.246919 |
| 3 | 0.413339 | 0.105772 | 0.246919 |
| 4 | 0.340563 | 0.105772 | 0.246919 |
| 5 | 0.370790 | 0.122193 | 0.157282 |
| 6 | 0.328032 | 0.007273 | 0.157282 |
| 7 | 0.455855 | −0.001240 | 0.247397 |

## Installation and qualification

Rebuild pinchy_motion_interfaces and pinchy_motion, then reopen the planning
server/panel together to load the new mode, examples and queue publisher. The
implementation was built and tested without restarting the live physical
hardware stack or sending it execution requests. Keep guided hardware startup
and its established powered holds independent; this feature adds no servo writer.

See [validation](validation/moveit/point_sets/validation.md), [ordered paths](cartesian_paths.md),
[the stroke contract](stroke_planning.md), and the canonical [roadmap](implementation_roadmap.md).


## Extended feasibility sets

Nine preflighted lower-horizontal, vertical and upper-horizontal sets are
installed in `examples/surface_feasibility/{cylinder,plane,quarter_pipe}/`.
Use **Load points…** to select one. These require the explicitly paired
`nominal_*_pedestal_dry_run.yaml` proposed scene in isolated testing; loading a
file never installs a support or changes the live scene. See the
[expanded gate](validation/feasibility/surface_gate/validation.md) for tested
angles/widths, executed cases, setup concepts and remaining physical decisions.

Planning now exposes live elapsed time, stage counts/percentages and approximate
stage time remaining in the Cartesian window. See [planning progress](planning_progress.md).


The larger/faster cylinder is example **07**, a 76.2 mm surface oval at strict
7 mm/s, with an explicit lower imaginary-cylinder placement and `spline_axis`
interpolation. See [dimensions, matching scene and qualification commands](continuous_cylinder_test.md#larger-faster-example).
Loading it preserves its required speed and requests a checked pen-up approach;
it never installs collision geometry or executes automatically.


Example **08 — Cylinder: 3-inch dry run, TCP range 4–18 mm/s** demonstrates the
optional range mode with the same lower imaginary placement/scene as 07.
`tcp_min_speed_m_s` zero/absent preserves fixed timing; a positive value explicitly
sets the interior floor, with `tcp_speed_m_s` as the ceiling. The band and mode
round-trip through save/load, panel requests and CLI. See [usage and bounds](bounded_speed_proposal.md).
