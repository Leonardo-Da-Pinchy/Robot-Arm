# Bounded TCP drawing speed

Implemented November 22, 2026 following the owner's authorization. Fixed-speed
mode remains available and is the default for existing point sets. This feature
changes planned timing, not stroke geometry, normals or the hardware control path.

## Use it

In the Cartesian panel choose a continuous trace style, enable **Use TCP speed
range**, set the minimum field and use the existing TCP speed field as the
maximum. Plan queued path, review the achieved range/mean and slowdown sections,
then Execute. Loading/saving a point set preserves the band and never executes.
Example **08 — Cylinder: 3-inch dry run, TCP range 4–18 mm/s** is packaged with
its existing explicit lower imaginary-cylinder/pedestal placement. It retains
30 deg/s / 180 deg/s² selected joint caps and a checked pen-up approach.
See [the matching scene](continuous_cylinder_test.md#larger-faster-example).

JSON files and CLI use metres/seconds:

```json
{
  "tcp_min_speed_m_s": 0.004,
  "tcp_speed_m_s": 0.018,
  "tcp_acceleration_m_s2": 0.08
}
```

These are timing fields within a complete point set, not a standalone job.
Minimum zero retains fixed-speed behavior. Positive minimum must be at least
1 mm/s and strictly below maximum. The ROS `PlanPath` field is `tcp_min_speed`;
`tcp_speed` becomes the upper bound. A positive minimum always enforces the band,
independently of the legacy fixed-speed `enforce_tcp_speed` flag.

Range mode supports polyline (without sharp corners), spline and spline_axis.
Visits and fixed-position rotations reject it. No style, path or normal is
silently substituted. An infeasible minimum rejects that candidate, allowing
existing whole-stroke configuration search to try another connected branch.
A failure is not a global impossibility proof.

## Endpoints and contact

The positive speed floor applies to the stroke interior. A single smooth ramp
starts from rest and reaches the band's lower region; another ends at rest.
Their geometric windows are determined before timing and cannot expand to hide
an interior bottleneck. Short ramp-only range requests reject. Pen-up lead-in/out
remains preferred; contact endpoint ramps still require `allow_contact_ramps`.
Explicit range mode permits interior speed variation within the band, without
stops at sampled waypoints. Physical contact remains disabled pending qualification.

## How timing works

First select a complete geometric joint path q(s) using the existing IK and
whole-stroke search. The added solver selects squared TCP path speed x(s)=v²
using a cubic B-spline and a constrained linear program. It respects the selected
joint velocities, joint accelerations, TCP tangential acceleration and positive
interior floor. It aims toward the upper speed without claiming globally optimal
branch choice or minimum execution time.

The joint constraints use qdot=q'(s)v and qddot=q''(s)v²+q'(s)a. This is one timing
solve per candidate, not IK repeated for every speed. Squared-speed bounds and
spline derivative bounds provide smooth interior timing; endpoints join with
zero acceleration. Numerical margins leave room for the final interpolation checks.

Acceptance still checks the actual JTC polynomials: exact joint position/rate
extrema, FK geometry/normal and TCP speed/tangential acceleration at intervals of
at most 10 ms, including the input waypoint times. Every checked interior speed
must stay in the requested band; the fixed-speed 5% tolerance is not used to
silently widen it. Collision density remains the established 20 ms / 0.008 rad.
These are dense software checks, not a continuous swept-volume or physical
tracking proof. No timed trajectory is retained unless every check and the
pen-up approach pass.

The preview reports modeled minimum/maximum/mean, duration and approximate
slowdown locations. `speed_range_report_json` contains per-trace metrics,
geometric slowdown intervals, and timing-solver duration. Scene/model/start
fingerprints, single-use previews and powered Stop/fault/recovery are unchanged.

## Cost and limits

Observed numerical timing-solver overhead is around a tenth of a second for the
large cylinder branch; full interpolation/geometry validation takes seconds.
Complete planning still includes IK/configuration search and collision services.
A range can reduce retries, but no fixed latency is promised. See the
[recorded full-stack results](validation/moveit/bounded_speed/validation.md).

Drawbacks include changes in ink deposition/line appearance, more speed
transitions, and sensitivity to an overly low useful-speed floor. A high minimum
can still make the entire stroke infeasible. Fixed mode remains appropriate
when uniform speed matters more than feasibility. A range cannot solve missing
IK, collisions, joint-position limits or the separate physical halfway tracking
pause. Bounds describe the modeled controller reference, not measured physical
pen speed; physical drawing/speed quality remains part of verification.

The solver uses SciPy already required by the stack; no new package installation
is needed. Its design follows the separation between geometric planning and
[constrained path timing](https://hungpham2511.github.io/toppra/index.html), but
it is not a drop-in use of TOPPRA or MoveIt's geometry-altering TOTG adapter.

Rebuild/reopen planner and panel together because PlanPath changed. The updated
estimated joint ceilings also require the next normal hardware startup; no
physical process is automatically restarted. See
[the operating-envelope estimate](estimated_operating_limits.md).
