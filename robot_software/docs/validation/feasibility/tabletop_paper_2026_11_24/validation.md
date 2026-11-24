# Exact tabletop paper targets: retained, not qualified

November 24, 2026. [Usage, placement and owner decisions](../../../tabletop_paper_examples.md).
The owner chose the exact requested placement and deferred wrist-range review.
No physical motors moved; no limits, calibration or base height were changed.

`placement.json` records current transformed CAD mesh bounds and input hashes.
Front feet are at X=64.635292 mm; adding 139.7 mm puts both centers at
X=204.335292 mm, Y=0. Inferred tabletop Z=−2.4 mm, air-trace Z=+2.6 mm.
`placement.png` is a top-view bounds/datum sketch, not collision geometry.

The circle is 100 mm diameter with 65 samples. The exact 100 mm square has four
11-sample straight sides, each followed by a separate 10 mm lift. The owner
selected separate strokes rather than rounded corners. Drawing files retain
4–18 mm/s, 80 mm/s² TCP acceleration and selected 30/180 joint limits. Lifts
have ordinary fixed-speed endpoint ramps and do not claim the positive drawing
floor while starting/stopping. All points are pen-up, not contact-qualified.

## Full-stack preflight

`check_paper_plane.py` launched its own calibrated mock with the explicit paper
collision patch and unchanged example floor. It creates planning/state/FK clients
only, never an execute client. The initial zero state was collision-free.

| Request | Result | Planning time |
| --- | --- | --- |
| Circle | Rejected; no alternative start IK found | 40.24 s |
| Square side 1 | Rejected; no alternative start IK found | 40.19 s |
| Square side 2 | Rejected; no alternative start IK found | 40.15 s |
| Square side 3 | Rejected; no alternative start IK found | 40.11 s |
| Square side 4 | Rejected; no alternative start IK found | 40.18 s |

Every rejection retains no plan ID or partial trajectory. Full messages,
configuration and diagnostics are in `preflight_report.json`, `preflight.txt`
and `mock_launch.log.gz`. **These results do not qualify execution** and finite
searches do not prove global impossibility. Separate lift execution was not
attempted because its preceding side has no accepted trajectory.

The isolated center-pose diagnostic found no saved-limit IK witness. A CAD-only,
non-collision-checked witness reaches the same center with wrist flex 87.1385°
and pen pitch −2.7558°, both outside saved bounds (about +63.90° and −0.703°
respectively). This witness is diagnostic, not an executable solution or evidence
that wider bounds would admit the full outlines. Actual travel review and
complete path/collision/timing requalification remain necessary.

```bash
python3 tools/inspect_paper_placement.py
OPENBLAS_NUM_THREADS=1 python3 tools/check_paper_plane.py \
  --output-dir /tmp/tabletop-paper-preflight
```

Run the second command in the sourced ROS workspace. It launches a separate
mock on its own domain, never attaches to physical bringup.

## Geometry and compatibility checks

Ten nominal-path tests pass, including fresh CAD datum agreement, circle radius
through the continuous interpolant, dimensions, downward pen axis/upward surface
normal, exact collinear square sides, joined corners, separate lifts and saved
file round-trips. Existing 21 path tests and eight range-timing tests also pass.
All nine installed example files and the opt-in scene were verified after build.
Logs and source hashes are included. The new read-only speed display was
[tested separately](../../moveit/tcp_speed_2026_11_24/validation.md).
