# Execute preview latency

Execute previously sent every dense trajectory state through MoveIt's collision
service a second time. Long continuous paths have thousands of samples, so a
successful preview could still be followed by a long idle interval before the
controller received its goal. This was validation work, not motor acceleration
or IK being solved again.

The preview now stores a SHA-256 fingerprint of the complete validated trajectory
message, immutable live robot model and collision-scene digest. The fingerprint
uses canonical message field values, avoiding nondeterministic CDR padding bytes.
Both pose and ordered-path previews receive it only after their full existing
bounds, rates, interpolation, geometry, timing and collision checks pass.

Execute consumes the preview once and retains all fresh checks:

- Preview age, target revision, model and full collision-scene snapshot.
- Measured-state collision check, controller/feedback/safety readiness.
- Held command and measured start before and after validation.
- Exact trajectory fingerprint; missing or changed validation rejects.
- A second scene/readiness check, and Stop/cancel/model checks before dispatch.

An identical checked trajectory in the identical model and scene reuses its
preview collision validation. No collision samples are removed from planning.
Scene changes and stale feedback during motion still request powered hold; the
hardware adapter, thermal guards, controller and Stop/recovery protocol are
unchanged. No result is reused after restart, expiration, consumption or edits.

The panel immediately displays “Checking current scene and start before
execution”, followed by actual controller progress. Server logs contain
`EXECUTION_DISPATCH` with preflight duration and trajectory point count.
The qualification runner records request-to-first-moving-controller-reference
latency; this includes controller sampling and startup ramps and is distinct
from preflight duration.

Validation commands, with ROS/workspace sourced:

```bash
PYTHONPATH=tools:$PYTHONPATH python3 -m unittest test_execution_preview test_trajectory_validation
PYTHONPATH=tools:$PYTHONPATH python3 -m unittest \
  test_cartesian.MockCartesian.test_01_plan_preview_execute_full_pose \
  test_cartesian.MockCartesian.test_05_stop_retains_powered_hold_and_allows_new_plan \
  test_cartesian.MockCartesian.test_06_action_cancel_is_canceled_and_holds
```

The equivalent hardware-fixture tests connect only to a generated PTY servo
emulator. Results are software timing evidence, not physical arm measurements.
After building, reopen the planning server and Cartesian panel together; the
hardware driver does not need a restart for this change.


The full 1,936-point example reached its first moving controller reference in
**0.31–0.38 s** on mock/PTY; fresh checks took 0.28–0.35 s before dispatch.
See [recorded results](validation/moveit/execute_latency/validation.md).
These are software/emulator timings, not a physical startup-time guarantee.
