# Conditional operating-envelope calculation

November 22, 2026. Read [the assumptions and manufacturer sources](../../../estimated_operating_limits.md)
before interpreting these numbers. No ROS connection, motor command or physical
qualification is part of this calculation.

```bash
python3 tools/estimate_operating_envelope.py \
  --branch docs/validation/engineering/operating_envelope_2026_11_22/geometric_branch.npz \
  --output /tmp/operating-envelope.json
```

The saved `estimate.json` contains the CAD gravity, mass-matrix and Coriolis
absolute-sum bounds over 1,184 sampled states. `geometric_branch.npz` is the
additional cylinder branch; the other samples use the script's fixed seed.
`source_hashes.json` identifies the model, tool, contract, driver and profile.
`validation_results.json` records the applied pair and per-joint torque check.

The limiting shoulder gives a common relative increase of 1.1349276819 from
the previous 90 deg/s / 360 deg/s² pair: 102.14349 deg/s and 408.57397 deg/s².
Subtracting the owner's 10-unit margins and rounding down yields **92.14 deg/s
and 398.5 deg/s²**. Neither result falls below the old ceilings. All six
sample-based torque upper bounds at that pair pass the derated torque–speed
curve. Independent one-parameter maxima are not combined.

Profile comparison confirms only calibrated velocity ceilings and the operating
update audit changed: saved positions, zeros, directions and raw speed=1000 /
acceleration=64 remain. Shared contract, generated URDF and C++ acceptance
agree. Driver tests, including acceptance of this ceiling and rejection above
it, pass; related test logs are in the [TCP validation bundle](../../moveit/bounded_speed/validation.md).

Finite posture sampling, uncertain installed servo suffix, absent payload,
friction/rotor inertia, voltage sag, thermal duty and tracking measurements
prevent treating this as an assembled-arm safe-limit certification. The
sampled shoulder gravity alone is close to the manufacturer's rated torque.
Physical verification remains pending; software defaults remain 30/180.
