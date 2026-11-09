ITERATION 2 / CAD 4 ARM - SIX-JOINT TRANSFORMS

Start with iteration_2_cad_4_arm.urdf.
This single file has no mesh dependencies. It describes the six rotary joints
and a fixed pen_tip frame for forward kinematics and ROS transforms.
It has no visual meshes, collision geometry, inertia or motor-control setup.

REVISION AND SOURCE
SO101_PenArm_J6.SLDASM, CAD 4 NoClaw follower, October 5, 2026 review fixes.
Nominal pen position matches the Q6_0_Set assembly configuration (and the
Q6_45_Set configuration when pen_pitch = pi/4).

Converted from analysis/joint6/so101_joint6.xml, the existing CAD 4 model,
using its parent-body origins, normalized quaternions, local joint axes,
joint ranges and pen_tip site. This is not a new SolidWorks URDF exporter run.
CAD4 parameters and saved SolidWorks measurement reports establish provenance.
Source files and SHA-256 hashes are in source/provenance.json.

JOINT ORDER AND FRAME CHAIN
base_link
  -> shoulder_pan  -> shoulder_link
  -> shoulder_lift -> upper_arm_link
  -> elbow_flex    -> forearm_link
  -> wrist_flex    -> wrist_pitch_link
  -> wrist_roll    -> wrist_roll_link
  -> pen_pitch     -> pen_holder_link
  -> pen_tip_fixed -> pen_tip

The first six joints are revolute. pen_tip_fixed is a fixed frame attachment,
not a seventh actuated joint. The old model calls joint 6 "gripper"; map that
motor's calibrated angle to "pen_pitch" when publishing JointState messages.

UNITS AND TRANSFORM CONVENTION
All URDF positions are metres. Joint angles and origin RPY are radians.
RPY uses Rz(yaw) * Ry(pitch) * Rx(roll). Every rotary axis is expressed in its
own joint frame, not the base frame. Positive motion follows the right-hand rule.
The first five joint names and their zero conventions follow the SO-101 model.

base_link preserves the stock model's base frame. It is NOT the SolidWorks
assembly origin or the design report's platform/world origin. The existing
CAD registration matrix assembly_from_base_link_m is in transform_validation.json.
That registration comes from the saved CAD alignment, not hardware calibration.

T_base_tip maps a point expressed in pen_tip coordinates to base_link:
    p_base = T_base_tip * p_tip
Multiply each parent-to-child joint origin by its axis rotation, then multiply
the six resulting joint transforms and the final pen_tip_fixed transform.
joint_transforms.csv lists the exact origins, axes and joint limits.

PEN TIP AND SPRING TRAVEL
The fixed pen_tip origin in pen_holder_link is (0, -0.149, 0.0188) metres.
Its rotation is roll = pi/2, pitch = 0, yaw = 0.
Tip +z points outward along the pen toward the nib; +x matches holder +x.
At q6 = 0 the pen points along wrist_roll_link -z.
Positive pen_pitch swings the pen toward wrist_roll_link +x.

The tip is nominally 149 mm radially from joint 6 at 5 mm sleeve compression.
It is 154 mm at the free/front stop and 144 mm at the back stop.
For a known compression c_mm, change the fixed origin's y value to:
    -(154 - c_mm) / 1000
Keep x, z and RPY unchanged. Compression range is 0 to 10 mm. The URDF keeps
one fixed spring position; it cannot infer actual compression during drawing.

HOW TO GET THE TRANSFORM
With Python and yourdfpy installed:
    from yourdfpy import URDF
    arm = URDF.load("iteration_2_cad_4_arm.urdf", load_meshes=False)
    arm.update_cfg({"shoulder_pan": 0, "shoulder_lift": 0, "elbow_flex": 0,
                    "wrist_flex": 0, "wrist_roll": 0, "pen_pitch": 0})
    print(arm.get_transform("pen_tip", "base_link"))

Replace those zeros with calibrated joint angles in radians. With ROS,
load the URDF as robot_description in robot_state_publisher and publish the
six named angles on /joint_states; request the base_link <- pen_tip transform.
The all-zero tip position is approximately (0.4656345, -0.001160747, 0.254546079) m
in base_link. The complete matrix is in transform_validation.json.

VALIDATION
Parsed with yourdfpy and compared every link and the tip against MuJoCo FK at
107 configurations, including zero, limit endpoints, joint-6 checks and 100
deterministic random poses. Maximum position difference: below 4e-15 metres.
Compared against saved SolidWorks pen-tip readback at q6 = 0, 45 and 100 degrees:
maximum difference below 0.00005 mm. Compared all six zero-pose joint axes with
the saved CAD registration: maximum origin difference below 0.002 mm.
These numbers describe model consistency, not physical robot accuracy.

To rebuild and repeat the checks from the bundled source:
    python -m pip install -r validation_requirements.txt
    python build_and_validate.py
This rewrites the URDF, CSV and validation report in this folder.

CALIBRATION STILL REQUIRED
Motor encoder zeros, signs and the new joint-6 zero must match this convention.
Joint 6's nominal soft range is -3 to +100 degrees; verify stops on the arm.
Other joint ranges are inherited from the existing model. Effort and velocity
are zero placeholders required by the URDF schema, not motor ratings or usable
controller settings. This file is for transforms, not direct motion control.
Measure the installed pen offset/compression and camera-to-base transform on
the physical assembly. No camera or fixture transform is guessed in this file.

FORMAT REFERENCES
https://mujoco.readthedocs.io/en/latest/modeling.html
https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/Building-a-Movable-Robot-Model-with-URDF.html
