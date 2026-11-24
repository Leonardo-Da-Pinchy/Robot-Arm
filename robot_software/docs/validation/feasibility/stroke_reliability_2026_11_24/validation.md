# Tabletop reach and planning analysis — November 24, 2026

Analysis only: no production code, saved examples, joint limits or live scenes
were changed; no physical motors moved. Experiments used offline numerical
solves and one isolated mock planning preview without an execution client.
Current wrist flex is -93.955 to +91.230 degrees. The nominal 149 mm axial tip
offset and 18.8 mm transverse offset remain unmeasured tool assumptions.

## Observed constraint

Full vertical-normal circle planning with updated saved limits fails around
47.4% at its furthest tested branch; square side 1 fails around 95.9%. The
standard start-selection search accepts up to eight attempts and here found
three circle starting configurations. These are finite-search failures, not
proofs of unreachability. Both fail during connected geometric IK, before
4–18 mm/s timing is accepted.

An independent 9-start SLSQP study maximized pen-axis alignment while enforcing
exact XYZ and saved bounds at nine representative positions. At the original
height, the nearest circle point retained approximately 0.1115 degrees of tilt,
with wrist flex at its upper bound and wrist roll at its lower bound. Other
sampled positions had perpendicular witnesses, some essentially on bounds.
This identifies a narrow joint-limited orientation region. It is not a global
minimum proof, collision assay or whole-area coverage result.

## Platform height

Assumption: only paper is raised relative to the robot; XY center remains
[0.2043352923, 0] m and the 5 mm air gap remains. Base table is inferred from
CAD feet at Z=-0.0024 m. If robot and paper move together there is no relative
height change, so platform thickness by itself has no effect.

| Added paper height | Best normal-error range found at nine points |
| --- | --- |
| 0 | 0–0.1115 degrees |
| 38.1 mm / 1.5 in | 7.937–13.090 degrees |
| 50.8 mm / 2 in | 10.656–16.507 degrees |

At the raised heights, the best fits repeatedly saturate wrist-flex and
wrist-roll bounds. No point in that finite nine-point set had a <=5 degree
witness at either raised height. This is evidence that raising paper alone
worsens this close placement, not a universal statement about higher surfaces.
All numbers depend on actual tool/frame geometry and calibrated limits.

## Normal-cone feasibility

A true 5-degree cone means acos(pen_axis dot inward_normal)<=5 degrees,
with axial twist free; it is not independent +/-5 degree Euler roll/pitch boxes.
Current production orientation tolerances were not changed. Offline comparisons
supplied a deliberately tilted exact normal to existing routines.

A fixed 5-degree tilt toward -Y passes full circle numerical geometry and
controller-polynomial timing. The same target also passes full isolated mock
planning, including collision validation and a checked pen-up approach. It
retains the requested circle geometry and 4–18 mm/s range. Mock planning took
157.87 seconds; trajectory duration is 24.31 seconds including approach;
modeled interior speed is 4.079–17.920 mm/s. Minimum wrist-flex reserve is only
about 0.16 degrees. This is a useful witness, not physical qualification or a
robust operating envelope. No execution occurred.

All four square sides have separate numerical geometry/timing witnesses inside
5 degrees: sides 1 and 4 at 5 degrees toward -Y, side 2 at 2.5 degrees toward +X,
and side 3 at 5 degrees toward +Y. These are independent, non-collision-checked
stroke witnesses. Some reserves are extremely small. Neither square lifts nor
repositioning between those chosen orientations have been qualified. This is
not a complete runnable square-job result.

Not every chosen tilt works. Several other fixed directions fail to continue,
and some complete waypoint chains overshoot joint bounds during interpolation.
Consequently a cone feature needs whole-stroke, smooth orientation selection
and final interpolation/cone/collision validation. Independent pointwise random
normal choices would be insufficient.

## Tool changes

Shortening the modeled axial offset improves some individual poses, but finite
continuation tests with 5 or 10 mm shorter offsets did not obtain a complete
vertical-normal circle. Combined shorter offsets and raised planes also failed
those limited continuation attempts. This does not disprove other configurations;
it means no particular hardware cut length is established by these tests.
Measure the real TCP, then co-design placement, tool offset and usable region.

## Search limitations and improvements to evaluate

- Current whole-stroke approach selection repeatedly follows a local branch
  from sampled starts. Bidirectional bottleneck exploration is applied only to
  the first failed start in that loop. A limited start count is not a complete
  search over all possible connected paths.
- The current whole-stroke smoothing routine is reached only after a complete
  seed history is obtained (directly or bidirectionally). Incomplete branches
  cannot yet benefit from an independent globally constrained seed-generation
  stage. A soft interior joint-margin heuristic can also struggle when the only
  seed lives near a bound; final hard safety limits must remain authoritative.
- Compare multi-solution-per-waypoint graph search with full-stroke optimization
  that can start from an approximate path. Score joint reserve, conditioning,
  curvature/rate demand and pen tilt; validate candidate edges and full timed
  interpolation. Adaptively refine at bottlenecks instead of only adding time.
- A proposed cone optimizer should keep XYZ fixed, enforce actual angle to the
  normal, prefer zero tilt and adequate joint reserve, and penalize rapid tilt
  changes. Default zero tolerance and explicit user opt-in would preserve the
  current surface-normal contract. No feature implemented here.
- Compute has no fixed multiplier: enumerating K normals multiplies work,
  whereas a coupled inequality-constrained solve relaxes orientation constraints
  within the existing six joint variables and may avoid failed retries. Hard
  cases can still cost more; benchmark complete stroke planning. The fixed-tilt
  mock result above is not a benchmark of a cone implementation.

## SVG implication

SVG parsing cannot create reach or connected joint paths. Support should mean
supported SVG geometry scaled and placed inside a qualified surface region,
with explicit stroke/corner/lift and speed policies. Current evidence does not
support promising arbitrary artwork at the requested tabletop placement.
Qualify geometry/frame accuracy and entire connected strokes with reserve first,
then test broader families of scaled/rotated artwork and operating conditions.

## Evidence and primary references

- [tabletop-options-study.json](tabletop-options-study.json): exact-position finite tilt minima.
- [tabletop-tilt-path-study.json](tabletop-tilt-path-study.json): fixed-tilt geometric/timing comparisons.
- [tabletop-square-side3-cone-study.json](tabletop-square-side3-cone-study.json): remaining square-side witness.
- [tabletop-tool-height-study.json](tabletop-tool-height-study.json): limited tool/height continuation probes.
- [tilted_circle_mock_result.json](tilted_circle_mock_result.json): full mock circle success.
- [tilted_circle_mock_launch.log.gz](tilted_circle_mock_launch.log.gz): planning-stage evidence.
- [strict_normal_mock_results.json](strict_normal_mock_results.json): strict-normal full failures.

The original temporary analyzer scripts are archived beside these reports.
They retain scratch output/import paths under /tmp; reproduce in a sourced ROS
workspace with those scripts placed at their original /tmp names, or adjust only
their scratch paths. They are not production planners. No measured physical speed, contact force or line quality
is claimed.

Tesseract Descartes documents the waypoint-IK ladder-graph approach:
https://tesseract-robotics.github.io/tesseract_nanobind/user-guide/planning/
MoveIt documents configurable orientation constraints (their component tolerance
box is not automatically an exact circular tool-normal cone):
https://moveit.picknik.ai/main/api/html/classkinematic__constraints_1_1OrientationConstraint.html


The current ordered follow-up is the [step-12 work plan](../../../complete_stroke_reliability.md). This bundle records analysis, not implementation of normal tolerance or physical qualification.
