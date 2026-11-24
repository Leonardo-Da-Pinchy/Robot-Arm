# Ordered Cartesian paths (step 11)

Use the existing motion startup and panel. Build the new interface packages and
restart the planning server/panel together; do not leave an old server running.
The driver, calibration, RViz layout, powered Stop/recovery and confirmed thermal
checks are shared with joint bringup. No second servo writer is introduced.

## Panel

Set a goal using XYZ/RPY or the draggable axes, then **Add goal** to the queue.
Repeat; select a row to remove or reorder it. **Update selected** replaces that
row with the current top fields while retaining its contact label. The queue defaults to **Move
between poses / pen-up routing**, which collision-plans routes to destinations.
Choose an explicit trace style when the connecting TCP geometry matters. The
upper Straight TCP checkbox applies to the single-pose Plan button; the queue
has its own path-style selector. **Plan queued** creates a preview
only. Inspect the trajectory and duration before using the existing Execute
button. Stop discards motion; recovery does not resume the interrupted path.

- **Polyline** traces straight segments. Collinear intermediate points do not
  cause stops. Sharp pen-up corners stop; a sharp contact corner is rejected.
- **Smooth through points** explicitly chooses a natural cubic XYZ curve through
  the points. Inspect its shape: it can overshoot between points. The complete
  curve, joint limits and collisions are checked.
- **Move between poses / pen-up routing** (`visits`, the queue default) uses OMPL between poses and stops at each. It can route around
  obstacles and does not promise a straight TCP path or constant TCP speed.

Pen direction/free twist remains the default. Traces interpolate the tool +z
pointing direction on the unit sphere and solve twist continuously; Exact RPY
uses quaternion SLERP. Position-only imposes no orientation constraint. No mode
silently relaxes a failed direction request. A valid endpoint alone does not
prove the requested connecting trace is feasible.

TCP speed is in mm/s in the panel; TCP acceleration is in mm/s². Joint speed and
acceleration controls remain independent caps. Pen-up transit may be uniformly
slowed to meet joint caps, with its duration/scale reported. Planning can take
several seconds because intermediate states and controller interpolation are
validated. Nothing moves during planning.

## CLI and interfaces

Save a JSON file (coordinates in metres, RPY in degrees):

```json
{
  "frame": "base_link",
  "style": "polyline",
  "orientation_mode": "tool_axis",
  "tcp_speed_m_s": 0.01,
  "tcp_acceleration_m_s2": 0.05,
  "waypoints": [
    {"xyz": [0.32, 0.00, 0.24], "direction": [0, 0, -1]},
    {"xyz": [0.32, 0.02, 0.24], "direction": [0, 0, -1]}
  ]
}
```

These are format examples, not certified reachable goals. Begin with nearby
points copied from the current pose. Each point can instead have `rpy_deg` and
an optional `frame`. Run `ros2 run pinchy_motion pose path /absolute/path.json`.
It prints a plan ID and the separate execution command. Use `pose --help` for
available commands. Plans expire after 180 seconds and are invalidated by target,
model, scene or start-state changes.

`/pinchy_motion/plan_path` uses `pinchy_motion_interfaces/srv/PlanPath`: ordered
PoseStamped waypoints, orientation mode, style, TCP speed/acceleration,
per-segment contact labels and joint scaling. Contact label i describes the
approach from the current pose/previous waypoint to waypoint i. The response
contains the checked RobotTrajectory, resolved waypoint frames, duration and
modeled contact-speed range. Execution uses the existing ExecutePlan action and
joint trajectory controller. The magenta RViz line shows the planned TCP path.

Limits: 80 waypoints, 3000 timed samples and 120 seconds per combined plan.
Rotation-only traces are rejected; use pose transit or Visit poses.

## Contact timing is software validated, not physically qualified

A JSON waypoint may specify `"contact": true`. This is currently **mock only**;
physical requests with contact tags are rejected until the pen and contact
geometry/speed are qualified. Pen-up physical path previews/execution are
available through the existing explicit hardware confirmation.

Contact timing requires pen-up lead-in/out long enough for smooth acceleration
and braking. It keeps approximately constant modeled TCP speed through interior
contact samples (5% planning tolerance), rejects insufficient ramps or joint
speed/acceleration excesses, and never silently slows or rounds contact geometry.
Explicit spline style is a different user-selected curve, not automatic corner
blending. Measured physical speed, minimum useful drawing speed, final TCP and
surface contact remain steps 8, 12–14 and 17. A free-space lead-in is not proof
that an actual surface can be approached without marking it prematurely.


## Validation and replay

From a sourced build, focused non-physical checks are:

```bash
OPENBLAS_NUM_THREADS=1 python3 tools/test_paths.py
OPENBLAS_NUM_THREADS=1 python3 tools/test_ik.py
OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen PINCHY_TEST_SAVED_LIMITS=1 python3 tools/test_cartesian.py MockCartesian.test_01h_ordered_free_twist_path_and_contact_timing MockCartesian.test_01i_queued_path_gui_and_cli MockCartesian.test_01j_free_twist_straight_transit HardwareCartesian.test_01h_ordered_free_twist_path_and_contact_timing HardwareCartesian.test_01i_queued_path_gui_and_cli HardwareCartesian.test_01j_free_twist_straight_transit
```

HardwareCartesian here means the real C++ adapter connected to a PTY servo
emulator, never the USB motor port. The saved-limit option applies the owner's
angular bounds with synthetic emulator encoder zeros. MockCartesian uses nominal
CAD bounds. Logs identify which mode/profile was exercised. The tests validate
preview without movement, whole-path execution, intermediate speed and direction,
GUI/CLI entry, infeasible contact rejection and free-twist single-segment transit.
The dedicated near-duplicate-sample case reproduces the copied-pose numerical
error; fixed waypoint samples are kept and nearby redundant grid samples removed.
Joint interpolation has continuous acceleration, with polynomial extrema and
actual TCP geometry checked afterwards. See the validation report for final logs.

Focused validation passed: seven path/math cases, ten IK regressions, six mock/saved-range emulator workflow cases, and geometry/collision/Stop/style checks. Initial failures and corrected reruns are retained in [the validation report](validation/moveit/report.json); these are scoped software/emulator checks, not a physical acceptance test.

Selectable TCP ceilings are now 400 mm/s and 2000 mm/s²; starting values remain 20 mm/s and 50 mm/s². Joint ceilings are 60 deg/s and 360 deg/s², with 30/180 defaults retained. Joint feasibility can still require slower pen-up timing or reject contact timing; these ceilings are not measured physical drawing speeds.


Current-model broad arcs can fail despite individually reachable samples; the
[motion feasibility investigation](motion_feasibility.md) records posture, joint
range, interpolation and speed issues. A bounded whole-stroke alternative search now retries local configuration
choices from the fixed held start when continuation or timed validation fails.
It retains up to four distinct histories, then accepts only a complete timed,
collision-checked trace. It does not insert same-pose joint jumps or automatic
pen-up repositioning. A later whole-stroke optimizer and explicit starting-configuration search now
pass both recorded broad arcs at 3 mm/s and 20 mm/s² ramps under default joint
caps. Drawing-region coverage and physical contact remain unqualified. See
[the newer results](validation/feasibility/stroke_start_search/report.json).
See the [selection comparison](validation/feasibility/whole_stroke_selection/report.json).


## Whole-stroke setup and optional contact endpoint ramps

See [the stroke contract](stroke_planning.md) for the owner's intended operation.
The panel now has **Choose stroke start + pen-up route** and **Allow contact
endpoint speed ramps**. Both default off so existing requests keep their
semantics. Their JSON/service fields are `plan_pen_up_approach` and
`allow_contact_ramps` (booleans). The first requires waypoint 0's incoming
segment to be pen-up and at least two waypoints; subsequent points describe
one smooth/collinear trace. Visit poses is not a stroke mode.

A setup request searches alternative start configurations and optimizes complete
joint histories before accepting the checked pen-up approach and stroke. Normal
direction and geometry are retained. Explicit contact ramps permit smooth
endpoint acceleration/braking; the interior plateau still has 5% modeled speed
checks. A short ramp-only stroke is identified in the response. Contact speed
min/max include ramps; they are not plateau-only statistics.

Rebuild `pinchy_motion_interfaces` and `pinchy_motion` and restart planning
clients/server together to load the extended interface. Contact remains
mock-only and physical execution still requires the existing explicit controls.


The [reported pen-up queue replay](pen_up_queue_replay.md) records the exact
zero-start screenshot targets, corrected defaults and remaining endpoint-axis
constraint distinction. Unspecified service/JSON styles use routed visits for
pen-up and polyline for contact; explicitly selected traces remain strict.


## Saved queues and rotation tests

The panel can **Capture current TCP**, **Save points…**, **Load points…** and
load example queues. These stage poses and settings; use the existing Plan then
Execute controls. **Fixed TCP rotations + approach** explicitly plans a pen-up
approach to the first pose and checks constant modeled XYZ during subsequent
full-orientation rotations. See [saved point sets](point_sets.md) for the saved
seven-point session list, examples, file format and read-only queue-export CLI.

Planning now exposes live elapsed time, stage counts/percentages and approximate
stage time remaining in the Cartesian window. See [planning progress](planning_progress.md).


`style: spline_axis` explicitly adds smooth unit pen-direction interpolation to
the existing natural cubic XYZ spline in pen-direction mode. Existing `spline`
files retain segmentwise S2 interpolation; full RPY remains SLERP. Degenerate
smooth directions reject. Whole-stroke timing, interpolation, collision and
normal checks still apply. Example 07 demonstrates the new mode; see the
[larger cylinder test](continuous_cylinder_test.md#larger-faster-example).


`PlanPath.tcp_min_speed > 0` now explicitly selects bounded TCP timing; zero
retains fixed-speed behavior. The positive interior minimum must be below
`tcp_speed`, which becomes the upper bound. Scene/model/start checks and Execute
semantics are unchanged. The response's `speed_range_report_json` contains
per-trace achieved/mean speed, slowdown locations and solver timing. See the
[bounded-speed contract](bounded_speed_proposal.md).
