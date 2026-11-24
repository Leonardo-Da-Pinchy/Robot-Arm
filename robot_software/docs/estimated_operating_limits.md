# Nominal operating-envelope estimate

November 22, 2026. Owner instruction: estimate joint speed/acceleration limits,
subtract 10 in each corresponding unit, and retain the previous ceiling if the
result would be lower. The pre-task ceilings were 90 deg/s and 360 deg/s².

The resulting **software ceilings are 92.14 deg/s and 398.5 deg/s²**. Selected
defaults remain 30 deg/s and 180 deg/s². No physical motion was commanded to
obtain or apply these estimates. They are conditional model-based operating
settings, **not physically established safe limits**.

## Sources and assumptions

The manufacturer's [12 V ST-3215-C018 specification](https://cdn.robotshop.com/media/F/Fit/RB-Fit-155/pdf/feetech_12v_30kg_cm_magnetic_encoding_servo_sts321_specification_pdf.pdf)
lists 45 rpm no-load speed, 30 kg·cm stall torque and 10 kg·cm rated torque.
The [related 12 V ST-3215-C047 specification](https://files.seeedstudio.com/products/Feetech/108090003_FEETECH_ST-3215-C047-Datasheet.pdf)
shows the approximately linear torque–speed curve on page 6. The installed
12 V STS3215 family is recorded in the inventory; its exact product suffix is
not confirmed. The graph is a family performance reference, not a serial-number
specific calibration. The 7.4 V variant's graph must not be substituted.

The calculation uses the shared CAD masses, centres of mass and inertia tensors:
about 0.518 kg of moving bodies, without additional pen/payload. It uses 1,003
seeded joint-limit samples plus 181 previously selected geometric-path states.
The modeled base is upright, voltage is nominally 12 V, and all joints may move
simultaneously. Friction, transmission/motor rotor inertia, added payload,
compliance, voltage sag, tracking error and thermal duty are not identified.
Finite sampling is not a global safety proof.

## Coupled estimate

For each sampled posture, compute the mass matrix M, gravity g and Coriolis
coefficients Γ from the six-body CAD model. Conservative absolute-value sums
bound simultaneous demand as:

`|torque_i| <= G_i + A_i * alpha + C_i * omega²`

Here G is maximum absolute gravity torque, A the maximum absolute mass-matrix
row sum, and C the maximum absolute Coriolis-coefficient sum over the samples.
Angular units in this calculation are radians. Taking separate maxima makes
the bound conservative over the sampled set, including configurations that
collision planning may later exclude.

Use the lower 10%-tolerance values of stall torque and no-load speed for the
approximately linear motor-performance envelope. Increase speed and acceleration
**together by one common factor**, relative to 90/360, until the limiting joint's
modeled demand reaches that curve. Independent maximum speed and maximum
acceleration cannot simply be combined: both consume the same torque budget.
The shoulder-lift bound limits the common factor to about **1.13493**.

| Quantity | Joint-compatible estimate | Minus owner margin | Applied ceiling |
| --- | --- | --- | --- |
| Joint speed | 102.14349 deg/s | 92.14349 deg/s | **92.14 deg/s** |
| Joint acceleration | 408.57397 deg/s² | 398.57397 deg/s² | **398.5 deg/s²** |

The results are rounded down to panel precision, then compared with the old
90/360 floors. Both exceed those floors. Every modeled torque upper bound at
the applied pair is below the derated performance curve. Larger independent
single-parameter bounds are recorded for analysis but are deliberately not
combined into an operating pair.

This is a transient performance-envelope estimate, not a continuous-duty rating.
For example, sampled shoulder gravity alone approaches 0.987 N·m, near the
specified 0.981 N·m rated torque. Thermal duty, actual payload and physical
tracking therefore remain verification work; the graph cannot certify one
universally safe speed/acceleration pair for every posture and duration.

## Applied layers

The shared control contract, model generator, six-joint URDF, saved operating
profile, driver velocity validation and panel bounds agree on the new ceilings.
The previous profile is backed up as
`calibration/backups/arm-2026-11-22-before-estimated-caps.json`, and the profile
contains the requested policy/method in its operating-update audit.
Measured zeros, directions, tick/radian position bounds and historical captures
are retained. The raw motor profile remains speed=1000, acceleration=64; the
existing timing margin keeps the new planned velocity peak within that profile.
Startup's 150/8 raw caps and 0.225 rad/s zeroing ramp are unchanged, as are
thermal confirmation, stale-feedback checks and powered Stop/fault recovery.

Use the next normal startup to load hardware/model changes. Nothing restarts a
powered arm automatically. Current software limits are not a claim that all
motors have been physically exercised at those rates.

## Reproduce

With the workspace's NumPy/SciPy available, this read-only tool produces a report
and never connects to ROS or a motor:

```bash
python3 tools/estimate_operating_envelope.py \
  --branch docs/validation/engineering/operating_envelope_2026_11_22/geometric_branch.npz \
  --output /tmp/operating-envelope.json
```

It reproduces this decision using the recorded pre-task floors, not automatic
future self-adjustment. Inputs, hashes and numerical results are in the
[validation bundle](validation/engineering/operating_envelope_2026_11_22/validation.md).
Physical operating-envelope qualification remains in roadmap verification.
