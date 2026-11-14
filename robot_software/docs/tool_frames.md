# Active end-effector point

The temporary active TCP is **`pointer_tip`**, the end of the long pointed part
already displayed in RViz. The owner authorized this model reference on
November 13 while only the single servo-mounted attachment is installed.
The endpoint of that physical attachment has not been independently identified
or measured. This is an explicit RViz proxy, not a claim of calibrated physical
accuracy, and it does not represent an empty sleeve opening.

## Frame definition

| Frame | Meaning |
| --- | --- |
| `base_link` | Existing robot model reference; unchanged by this work. |
| `tool0` | Sixth-axis mounting origin, coincident with `pen_holder_link`; not the tip. |
| `pen_tip` | Preserved CAD nominal point: 149 mm along the tool, assuming the source pen at 5 mm sleeve compression. |
| **`pointer_tip`** | Active temporary TCP, coincident with `pen_tip`. Use this frame for the next MoveIt/Cartesian work. |

`pen_holder_link → pointer_tip` uses translation **(0, −0.149, 0.0188) m**
and RPY **(π/2, 0, 0) rad**, with R = Rz(yaw) Ry(pitch) Rx(roll).
The TCP's **+z points outward toward the endpoint** (holder −y); +x follows
holder +x. A cyan dot of 2 mm radius marks the TCP in RViz; it is a visual aid,
not added collision geometry. The six actuated joints, their zeros, ranges and
calibration are unchanged.

At all model joint angles zero, the TCP is approximately
**(0.4656345, −0.001160747, 0.254546079) m** in `base_link`.
These are model coordinates, not independent physical measurements.

[Active tool configuration](../src/pinchy_description/config/active_tool.json)
records the chosen frame, its attachment and provenance, and
`physical_tip_verified: false`. The [model generator](../tools/build_arm_model.py)
reads this file when generating the runtime URDF. It is installed under
`share/pinchy_description/config/active_tool.json` for future planning code.
The profile is currently metadata/model input; MoveIt is not configured yet.

## Inspect without moving physical motors

From a prepared ROS terminal:

```bash
ros2 launch pinchy_bringup mock_control.launch.py rviz:=true sliders:=true
```

In a second prepared terminal on the same ROS domain:

```bash
ros2 run tf2_ros tf2_echo base_link pointer_tip
```

The cyan dot follows the model's pointer end. TF axes can be inspected by
expanding RViz's TF display and enabling frame names. Normal physical startup
also publishes this frame from measured joint states after loading the updated
model; an already running publisher retains its previous description until
restarted. This addition itself issues no motor commands.

## Validation and remaining physical checks

Software validation passed:

- Nine model tests, including endpoint coincidence with the rendered nib
  cylinder, outward-axis agreement, preserved source hashes/joint definitions,
  and all original link transforms plus TCP agreement at 107 poses.
- Live TF publication through both mock and emulated hardware ROS launches.
- `pinchy_description` build/install, including the active tool profile.

Step 7's temporary software TCP is implemented and ready for MoveIt setup.
Physical/model agreement remains a step-8 measurement: compare the real
attachment and modeled endpoint at several independently measured poses before
claiming real-tip positioning accuracy. If the physical tip differs, supply its
transform or dimensions and update the active tool definition/geometry together.
Final pen length, axis, contact/compression calibration and physical collision
geometry are step 13. Do not infer a real pen offset from this proxy.

The pending base wings/camera mounting transforms are separate; see the
[roadmap](implementation_roadmap.md#pending-base-redesign--owner-follow-up).
