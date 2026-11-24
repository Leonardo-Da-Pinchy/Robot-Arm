# Continuous cylinder commissioning path

Example **04 — Cylinder: continuous 2-inch dry run, 3 mm/s** is one continuous
surface-following oval with 41 points. It spans **50.8 mm across the cylinder in
surface arc length and 50.8 mm vertically**, not 41 stopped pose visits. The
nominal cylinder is 80 mm diameter × 100 mm tall, upright at center
`[0.40, 0.10, 0.22]` m in `base_link`. The modeled TCP stays **5 mm outside** it,
with pen +Z pointing into the cylinder. This is an imaginary-object air test.
The axial pen twist is free; the normal is constrained.

The trace is about 169.7 mm long and takes **56.8 seconds**, plus the checked
pen-up approach. Requested TCP cruise is **3 mm/s**, endpoint acceleration is
**20 mm/s²**, and joint caps stay **30 deg/s / 180 deg/s²**. Endpoint ramps last
about 0.236 s each. The approach ends at rest, then the single trace accelerates,
traverses every interior point at cruise, and decelerates at its end. Loading
points, planning and execution remain separate operations.

## Larger, faster example

**07 — Cylinder: continuous 3-inch dry run, 7 mm/s** adds a 61-point oval
spanning **76.2 × 76.2 mm in surface coordinates**, 1.5 times the original
span at 2.33 times its cruise speed. It retains the nominal Ø80 × 100 mm cylinder,
5 mm air gap, free axial pen twist, 30 deg/s joint speed and 180 deg/s² joint
acceleration caps. It is a separate example; 04 remains available unchanged.

This example explicitly uses a **different imaginary placement**: cylinder
center `[0.40, 0.10, 0.15]` m, oval centered at −15° on the cylinder. Its matching
scene is `fast_cylinder_pedestal_dry_run.yaml`, with a proposed Ø40 × 20 mm
pedestal centered at Z=0.09 m. This keeps the entire 76.2 mm vertical loop on the
100 mm tall cylinder. It does not revise approved nominal object dimensions or
establish this placement as a physical setup.

Select **Smooth points and pen direction** (`spline_axis`), as stored in the
file. XYZ follows the same natural cubic interpolation as the existing spline;
pen direction follows a normalized cubic interpolant through the supplied unit
normals. This removes the angular-rate jumps of independently SLERPing each
segment. It changes the interpolation explicitly: old spline files retain their
original behavior. Full-RPY interpolation is unchanged. Opposing directions
whose interpolant becomes undefined reject. Every requested knot, actual JTC
polynomial, normal, stand-off, speed and collision sample must still pass.

The trace takes about **36.92 seconds**, plus roughly four seconds of pen-up
approach in the qualified mock run. Initial whole-stroke planning can still
take minutes. Execute's fresh checks reuse that exact checked preview and no
longer repeat all dense collision requests. See [Execute latency](execute_latency.md).

To use its matching mock scene:

```bash
ros2 launch pinchy_moveit_config mock_motion.launch.py \
  scene:="$PWD/src/pinchy_moveit_config/config/fast_cylinder_pedestal_dry_run.yaml"
```

Complete mock and driver/PTY executions pass; see the [recorded results](validation/feasibility/fast_cylinder/validation.md).

Choose example 07, then Plan queued path and inspect the preview before Execute.
Loading an example never moves the arm or silently changes the active scene.
This is an air test; physical contact remains disabled.

Repeat qualification on isolated mock/PTY stacks only:

```bash
OPENBLAS_NUM_THREADS=1 python3 tools/qualify_continuous_execution.py \
  --points src/pinchy_motion/examples/point_sets/07_cylinder_continuous_3in_fast.json \
  --scene src/pinchy_moveit_config/config/fast_cylinder_pedestal_dry_run.yaml
# Add --emulated to use the real driver with a generated PTY servo emulator.
```

Speed changes do not change the geometric IK equations, but they change joint
velocity and acceleration requirements along the whole curve. At constant TCP
speed, joint velocity grows proportionally to speed, while the curvature term
in joint acceleration grows with speed squared. The 400 mm/s input ceiling is
not the limiting factor here: wrist joint rates and configuration continuity
are. The initial 10 mm/s candidates were rejected by joint timing checks.
Neither a global 10 mm/s impossibility nor physical speed capability is proved.
Faster requests can take longer if more whole-stroke candidates are rejected;
a feasible faster trajectory can have fewer 20 ms samples to validate.

## Run it in the panel

After building the changed interface and reopening the planning server and
panel together, choose the cylinder example from **Example point sets…**, then
**Load example → Plan queued path → inspect preview → Execute previewed plan**.
It loads **Smooth through points**, **Pen direction / free twist**, **Choose
stroke start + pen-up route**, and **Require requested TCP cruise speed**.
Keep these settings for this test. **Visit poses** intentionally stops at poses
and clears the required-speed option. Plan searches may take several minutes. The [planning-progress display](planning_progress.md)
shows elapsed time, current stage, counted checks and approximate stage time
remaining where measurable. Its percentage describes that stage, not the whole job.

For an isolated simulation including the assumed cylinder/fixture solids:

```bash
cd /home/leo/Robot-Arm/robot_software
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  pinchy_motion_interfaces pinchy_motion pinchy_moveit_config
source install/setup.bash
ROS_DOMAIN_ID=181 ros2 launch pinchy_moveit_config mock_motion.launch.py \
  calibration:="$PWD/calibration/arm.json" \
  scene:="$PWD/src/pinchy_moveit_config/config/nominal_cylinder_dry_run.yaml"
```

The same saved file can be staged through the CLI on the intended session's ROS
domain (181 only for the isolated simulation above):

```bash
ROS_DOMAIN_ID=181 ros2 run pinchy_motion pose path \
  "$PWD/src/pinchy_motion/examples/point_sets/04_cylinder_continuous_2in.json"
```

The CLI prints a separate Execute command. Files contain poses/settings, never a
reusable execution token. A consumed, stopped, edited or stale plan cannot be
replayed. The packaged examples never install collision objects automatically.

For a later physical **air** test, use the established guided startup and the
current matching model/scene. These coordinates are hypothetical; do not place
a real cylinder there based on this example. Actual tabletop/fixtures, pen/TCP
and base wings still need their measured geometry. Physical contact-tagged
requests remain disabled. No physical motors were moved to develop this test.

## Speed and configuration behavior

`enforce_tcp_speed: true` extends the existing contact-speed contract to pen-up
dry runs. It rejects joint-rate excess instead of silently stretching the time,
checks cruise within **5%** and checks smooth endpoint ramps against their speed
law. Short ramp-only traces and sharp polyline corners reject this strict
request. There is no implicit curve replacement, geometric relaxation or
intermediate waypoint stop. Contact endpoint ramps still require the separate
`allow_contact_ramps` permission.

The planner evaluates the whole stroke before choosing a starting posture. If
local continuation stalls at an interior target, it can search another IK
configuration there, connect backward and forward along the unchanged curve,
and optimize the complete history. A witness alone is never executable. The
exact task, joint bounds/rates, actual controller polynomial and every collision
sample must pass, together with the pen-up approach.

MoveIt supplies IK/routing/collision services; the shared layer uses its existing
analytic Jacobian to evaluate TCP velocity `J_position(q) × qdot` and times the
stroke by Cartesian arc length. Ordinary joint retiming alone does not establish
constant TCP speed. The existing trajectory controller and hardware adapter
remain the only execution path; powered Stop/fault behavior is retained.

Collision samples retain the original 20 ms / 0.008 rad density. Small bounded
batches pipeline service transport; no samples are skipped. Scene, start, model and the exact trajectory fingerprint are checked again
before execution; identical previews reuse their completed collision checks.
See [Execute latency](execute_latency.md). Whole-stroke
search is finite, with a 360-second setup budget; a rejection is not an
impossibility proof.

## Repeat the software qualification

With ROS and the workspace sourced:

```bash
OPENBLAS_NUM_THREADS=1 python3 tools/qualify_continuous_execution.py
OPENBLAS_NUM_THREADS=1 python3 tools/qualify_continuous_execution.py --emulated
OPENBLAS_NUM_THREADS=1 python3 tools/qualify_nominal_paths.py \
  --output-dir /tmp/nominal-surface-paths
OPENBLAS_NUM_THREADS=1 python3 tools/qualify_nominal_paths.py \
  --grid-only --full-grid --endpoint-budget 3 --output-dir /tmp/nominal-surface-grid
```

The execution harness starts its own isolated mock or PTY servo emulator. It
accepts no physical serial port and never attaches to the running real robot.
It checks live controller reference speed at all 39 interior input points,
normal direction, gap, encoder/model path extent, completion and consumed-plan
rejection. The surface study separately collision-checks a 10 mm retract from
the exact accepted end posture. That retract is analysis evidence, not an
extra move appended to the saved example.

See [recorded validation](validation/feasibility/nominal_surfaces/validation.md),
[saved point sets](point_sets.md), [stroke planning](stroke_planning.md), and the
[roadmap](implementation_roadmap.md). Commanded software speed does not measure
physical nib speed, tracking or drawing accuracy; those remain in steps 23–24.


## Explicit speed-range example

Example **08** uses the same 76.2 mm oval, lower cylinder placement and scene as
07, with **4–18 mm/s** interior speed, 80 mm/s² TCP acceleration and selected
30 deg/s / 180 deg/s² joint caps. It intentionally slows through joint-rate
bottlenecks without stopping at interior points. Old fixed-speed examples are
unchanged. Minimum/maximum and endpoint exceptions are visible in the preview;
see [the range guide](bounded_speed_proposal.md) and
[full-stack validation](validation/moveit/bounded_speed/validation.md).
This does not establish physical pen speed or fix the separately recorded
physical halfway tracking pause.
