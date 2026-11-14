# CAD 4 runtime model

The supplied `CAD 4 - Six-joint URDF and transforms` directory is now
[`src/pinchy_description/cad/cad4/`](../src/pinchy_description/cad/cad4/).
Its URDF, CSV, README, JSON report and ZIP were preserved unchanged. The ZIP's
matching validation program and provenance-hashed source files were unpacked
alongside them.

> Current active TCP: **`pointer_tip`**, the owner-approved RViz pointer proxy.
> See [tool frames](tool_frames.md) for its definition and physical limitations.

## Runtime integration

`tools/build_arm_model.py` generates `urdf/pinchy_arm.urdf` from the supplied
transform-only URDF. All six joint origins, axes, CAD position ranges and the
fixed nominal `pen_tip` attachment are retained. The runtime chain is:

```text
base_link → shoulder_link → upper_arm_link → forearm_link
          → wrist_pitch_link → wrist_roll_link → pen_holder_link → pen_tip → pointer_tip
```

The six joint names, in order, are `shoulder_pan`, `shoulder_lift`, `elbow_flex`,
`wrist_flex`, `wrist_roll`, **`pen_pitch`**. The last name replaces the earlier
`tool_rotation`; controllers, sliders, CLI, mock interfaces, calibration template
and tests use the new name. The original stock reference remains unchanged.
`tool0` remains a fixed alias at the sixth-axis origin; it is not the pen tip.

The export contains no geometry or inertias. Those are merged from its bundled,
provenance-matched `source/cad4_model.xml`:

- Stock arm STL meshes use the source body-local poses.
- The NoClaw follower is reconstructed from the stock mesh using the source's
  prescribed native-part cut at z = 38 mm, capped as a closed solid. Its volume
  agrees with the saved CAD part volume within 2 mm³. It is a mesh proxy; the
  original CAD4 NoClaw analysis STL was not included in the new bundle.
- The pen holder, sleeve, pen and nib use the source model's simplified boxes
  and cylinders. They are not newly inferred placements of the earlier loose
  STEP components. The yoke boxes are also included as collision proxies.
- The source capsule collisions are represented by cylinders and spherical
  caps, since URDF has no capsule primitive. Base collision meshes are included.
- Link inertias are copied from the source analysis model. They are model
  estimates, not new mass or load measurements on the assembled robot.

The supplied effort and velocity values are zero placeholders. Runtime velocity
is explicitly set to **30 deg/s (approximately 0.524 rad/s)** for position-control software; it is not a
motor rating. Effort stays unspecified at zero because this adapter commands
positions, not joint torque. Hardware calibration may narrow the CAD position
ranges and supplies its tested velocity bounds.

## Joint limits and self-collision

| Joint | CAD minimum | CAD maximum |
| --- | ---: | ---: |
| Shoulder pan | −110° | +110° |
| Shoulder lift | −100° | +100° |
| Elbow flex | −96.83° | +96.83° |
| Wrist flex | −95° | +95° |
| Wrist roll | −157.21° | +162.79° |
| Pen pitch | −3° | +100° |

**The slider and numeric input both clamp to these loaded URDF ranges.** Their
0.001 rad resolution rounds endpoints inward, so their accessible range is
slightly narrower. Tests attempt to exceed every minimum/maximum and verify
both controls stay inside the URDF limits. In hardware mode the displayed URDF
is additionally narrowed by the reviewed calibration file.

**Joint ranges do not prevent self-collision.** A combination of valid angles
can put one link through another. The current mock hardware mirrors commanded
positions; it does not simulate contact. RViz displays geometry and transforms,
and does not enforce collision avoidance. Its Collision Enabled checkbox only
changes what geometry is displayed.

No arbitrary angle restrictions were added to hide intersections. A later
MoveIt configuration must check the robot and environment along the complete
planned path, with collision exclusions for intended contacts between adjacent
parts. The direct slider workflow currently has no such path checker. The GUI
now states this explicitly; collision geometry alone does not make a movement
collision-safe.

## Nominal tip and remaining measurements

`pen_tip` is 149 mm from joint six at **5 mm sleeve compression**, at
`(0, -0.149, 0.0188)` metres in `pen_holder_link`. Tip +z points toward the nib.
At all joint angles zero its base position is approximately
`(0.4656345, -0.001160747, 0.254546079)` metres.

The spring travel is not an actuated seventh joint. The source describes a tip
radius of 154 mm at the free stop and 144 mm at full compression. The current
fixed TF cannot infer actual spring compression. Installed pen length,
compression/contact behavior, joint encoder zeros/signs and physical stops
still require measurement. In particular, review/calibrate motor six against
the CAD `pen_pitch` zero before physical use; old calibration is not migrated.
The source's assembly-to-base registration is retained as provenance, not
published as a guessed world/camera or physical calibration transform.

## Use and validation

Stop any existing control launch and separately opened old slider window, then:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Robot-Arm/robot_software
source install/setup.bash
ros2 launch pinchy_bringup mock_control.launch.py rviz:=true sliders:=true
```

The normal workspace has been rebuilt. A running publisher retains the old
description until restarted; sourcing the workspace alone does not reload it.
No physical motor was moved or EEPROM setting changed during this update.

Reproduction tools:

```bash
python3 tools/build_arm_model.py
python3 tools/test_arm_model.py
python3 tools/test_ros_control.py
```

The optional CAD validation environment requires
`src/pinchy_description/cad/cad4/validation_requirements.txt`. In that environment:

```bash
python3 tools/build_noclaw_mesh.py
python3 tools/build_arm_model.py
python3 tools/validate_cad4_model.py
```

The source validator was repeated on a temporary copy, preserving the supplied
exports. The integrated runtime URDF was also independently compared against
MuJoCo at **107 poses** and the saved SolidWorks pen-tip readings at 0°, 45° and
100°. Maximum FK position difference was below 4e-15 m; CAD tip readback
differences were below 0.00005 mm. These demonstrate numerical model consistency,
not physical robot accuracy. Geometry loading and closed-mesh checks are saved
with the other [validation records](validation/cad4/).

Final software regressions: **103 test cases passed** (67 existing bench, 7 model,
17 hardware/calibration and 12 ROS/GUI integration tests). Both mock and
emulated-hardware GUI checks exercise every minimum and maximum in the sliders
and numeric inputs. The normal workspace build, full visual/collision geometry
loading and URDF parsing also passed. No physical servo was accessed.
