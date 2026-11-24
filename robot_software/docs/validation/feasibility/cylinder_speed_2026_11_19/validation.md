# Cylinder timing and physical-recording analysis

November 19, 2026. See the [findings and limitations](../../../cylinder_speed_investigation.md).
No execute action, motor command, driver restart or parameter write was used.
This is diagnosis, not a new physical or full-coverage qualification.

- `large_branch.npz`: saved example-07 geometric branch from the earlier passing
  whole-stroke search; input to the exact polynomial timing replay.
- `rates.json`, `cylinder_peak_rates.py`: all-six-joint velocity/acceleration
  extrema at 7–18 mm/s, using shared speed-law, spline, polynomial-extrema and
  trajectory validation functions. Branch bounds remain the saved bounds.
- `small_branch.npz`, `small_rates.json`, `small_peak_rates.py`: newly selected
  complete numerical example-04 branch, with the same rate analysis. Its
  selection intentionally omits collision service checks: it is never executable.
- `small_smooth_axis_branch.npz`, `small_smooth_axis_screen.txt`: numerical
  interpolation comparison, not an installed replacement or execution result.
- `session_planning_events.log.gz`: snapshot of the existing physical-mode
  planner's requests/search rejections/dispatch log. Requested joint/TCP
  acceleration settings are not present in these events; replay settings are
  explicit in the JSON rate tables instead of inferred from those logs.
- `physical_14mm_s_recording.jsonl.gz`: selected existing controller recording
  around the 14 mm/s physical run, captured before rotation removed it.
- `live_14mm_s.npz`, `live_summary.json`: FK/Jacobian/encoder-derived analysis.
  Only the large 14 mm/s run remained in the available rotating-record snapshot.
- `halfway_tracking.png`: commanded speed and measured motion around 180°,
  alongside shoulder/elbow reversals. Smoothed encoder speed is an estimate.

Reproduce peak tables from `robot_software/`, with ROS and the workspace sourced:

```bash
OPENBLAS_NUM_THREADS=1 python3 docs/validation/feasibility/cylinder_speed_2026_11_19/cylinder_peak_rates.py
OPENBLAS_NUM_THREADS=1 python3 docs/validation/feasibility/cylinder_speed_2026_11_19/small_peak_rates.py
```

These scripts only calculate trajectories and rate reports; they create no ROS
node, execution client or serial connection. All peaks use actual JTC polynomial
extrema, rather than finite differences of sparse input waypoints. Reproduced
rejections agree with the shared validator; the live log independently identifies
wrist-roll velocity for the high-speed large-loop request. No global IK or
maximum-speed optimality is claimed, and the halfway pause's root cause remains
unresolved pending adapter/downstream tracking evidence.
