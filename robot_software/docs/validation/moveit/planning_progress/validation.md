# Planning-progress validation

November 15, 2026. No physical motor commands or live hardware restarts.

The actual Cartesian panel loaded the saved plane and cylinder files and called
PlanPath on separate calibrated mocks, with their nominal collision scenes.
Both completed successfully. The tests observed counted IK/timing/collision
stages, configuration attempt context, live elapsed time, and a hidden progress
bar after the final result. No execution action was requested.

| Case | Distinct live progress samples recorded | Last observed server elapsed time |
| --- | --- | --- |
| Plane, 41 points | 422 | 160.42 s |
| Cylinder, 41 points | 502 | 213.40 s |

These are observed concurrent-run timings, not promised planning durations.
Both searches can take longer from other configurations or on a busier machine.
Whole-job remaining time is deliberately unknown; only known-stage progress and
rate-based stage estimates are shown. Percentages reset when the stage or
candidate changes. Search attempts and optimizer evaluation limits are maxima,
not a fixed total required workload.

[Plane telemetry](plane_progress.json), [cylinder telemetry](cylinder_progress.json),
[plane screenshot](plane_panel.png), [cylinder screenshot](cylinder_panel.png).
Screenshots capture the live counted stages; the final wording was compacted to
three lines after these captures to keep the total-ETA label visible.

The focused unit suite checks stage-ETA reset, inactive/new-request reset,
accurate collision counts including the final partial batch, stale telemetry,
old request IDs, Stop-message priority and final-result priority, alongside the
existing IK, path, timing, scene, file and panel regressions. See
[unit output](unit_tests.txt) for the passing count. Four additional mock
integration regressions passed: contact timing, scene-change invalidation,
powered Stop with subsequent planning, and action cancellation. Their original
artifacts are `/tmp/pinchy-cartesian-tests-ufk5vilr`.

Optional progress callbacks preserve the same sample grids, solver bounds,
optimizer budgets, timing checks, polynomial validation and collision density.
The existing controller/driver and safety implementations are unchanged.
Read-only progress is attached to accepted planning operations only; execution
validation retains its existing behavior.

[User guide](../../../planning_progress.md). Build: `pinchy_motion`, successful.
