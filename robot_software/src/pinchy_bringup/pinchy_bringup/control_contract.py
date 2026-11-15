"""Shared joint order, controller endpoints, timing and powered-hold goal."""
import math
import xml.etree.ElementTree as ET
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'pen_pitch']
TRAJECTORY_ACTION = '/arm_controller/follow_joint_trajectory'
CONTROLLER_STATE = '/arm_controller/controller_state'
SAFETY_STATUS = '/arm_safety/status'
APPLIED_TARGETS = '/arm_safety/held_positions'
SCALING_INPUT = '/arm_controller/speed_scaling_input'
SAFETY_RESET = '/arm_safety/reset'
DEFAULT_SPEED = math.radians(30)
DEFAULT_ACCELERATION = math.radians(180)
MAX_SPEED = math.radians(60)
MAX_ACCELERATION = math.radians(360)
MAX_TCP_SPEED = 0.4  # m/s, selectable ceiling, not a qualified drawing speed
MAX_TCP_ACCELERATION = 2.0  # m/s²
ARRIVAL_TOLERANCE = 0.03
# Match the adapter's existing encoder feedback allowance; never a command allowance.
FEEDBACK_BOUND_TOLERANCE = 10 * 2 * math.pi / 4096


def motion_duration(distances, velocities, acceleration):
    """Bounded-peak quintic timing, shared with the commissioning sliders."""
    if (not distances or len(distances) != len(velocities) or
            not math.isfinite(acceleration) or acceleration <= 0 or
            any(not math.isfinite(d) or d < 0 for d in distances) or
            any(not math.isfinite(v) or v <= 0 for v in velocities)):
        raise ValueError('Require finite distances, positive speeds and positive acceleration')
    return 1.05*max(0.5, max(1.875*d/v for d, v in zip(distances, velocities)),
                    max(math.sqrt((10/math.sqrt(3))*d/acceleration) for d in distances))


def hold_goal(positions):
    if len(positions) != len(JOINTS) or not all(math.isfinite(p) for p in positions):
        raise ValueError('Hold requires six finite joint positions')
    goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
    start = JointTrajectoryPoint(positions=list(positions), velocities=[0.0]*6)
    end = JointTrajectoryPoint(positions=list(positions), velocities=[0.0]*6)
    end.time_from_start.nanosec = 250_000_000
    goal.trajectory.points = [start, end]
    goal.goal_tolerance = [JointTolerance(name=name, position=-1.0) for name in JOINTS]
    return goal


def model_limits(model):
    root = ET.fromstring(model) if isinstance(model, str) else model
    raw = root.find("./ros2_control/hardware/param[@name='acceleration']")
    acceleration = min(MAX_ACCELERATION, math.radians(0.8*8.7*int(raw.text))) if raw is not None else MAX_ACCELERATION
    result = {}
    for name in JOINTS:
        joint = root.find(f"joint[@name='{name}']"); limit = joint.find('limit')
        result[name] = dict(lower=float(limit.get('lower')), upper=float(limit.get('upper')),
            velocity=min(MAX_SPEED, float(limit.get('velocity'))), acceleration=acceleration)
        for side in ('lower','upper'):
            nominal=root.find(f"./ros2_control/joint[@name='{name}']/param[@name='nominal_model_{side}']")
            result[name]['nominal_model_'+side]=float(nominal.text) if nominal is not None else result[name][side]
    return result

