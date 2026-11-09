"""Explicit Feetech bringup. Requires a reviewed external calibration file."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
          'wrist_flex', 'wrist_roll', 'pen_pitch']


def setup(context):
    calibration_path = Path(LaunchConfiguration('calibration').perform(context)).expanduser()
    config = yaml.safe_load(calibration_path.read_text())
    enabled = LaunchConfiguration('motion_enabled').perform(context) == 'true'
    adopt = LaunchConfiguration('adopt_zero_hold').perform(context) == 'true'
    handoff = config.get('profile_kind') == 'zero_hold_handoff'
    if adopt != handoff or (adopt and not config.get('handoff_token')):
        raise RuntimeError('Zero-hold adoption requires the session profile from the guided startup')
    if adopt and config.get('capture_backend') != 'hardware':
        raise RuntimeError('Zero-hold adoption requires a hardware startup session')
    bench = config.get('profile_kind') == 'unloaded_ros_bench'
    bench_selected = LaunchConfiguration('unloaded_bench').perform(context) == 'true'
    if bench != bench_selected:
        raise RuntimeError('An unloaded bench profile requires unloaded_bench:=true; '
                           'assembled-arm calibration must use the normal launch mode.')
    if bench and config.get('capture_backend') != 'hardware':
        raise RuntimeError('Mock profile capture cannot enable physical hardware')
    if not enabled or config.get('calibration_confirmed') is not True:
        raise RuntimeError('Review calibration, set calibration_confirmed: true, '
                           'and explicitly pass motion_enabled:=true')
    if 'pen_pitch' not in config.get('joints', {}):
        raise ValueError('CAD 4 names joint six pen_pitch. Review its pen-forward zero and '
                         'update the calibration mapping; old tool_rotation/gripper calibration '
                         'is not migrated automatically.')
    description = Path(get_package_share_directory('pinchy_description'))
    bringup = Path(get_package_share_directory('pinchy_bringup'))
    root = ET.parse(description / 'urdf/pinchy_arm.urdf').getroot()
    root.set('name', 'pinchy_bench_hardware' if bench else 'pinchy_hardware')
    control = ET.SubElement(root, 'ros2_control', name='FeetechArm', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'pinchy_hardware/FeetechSystem'
    params = {key: config[key] for key in ('port', 'timeout_ms', 'speed', 'acceleration',
                                         'torque_limit', 'max_feedback_age')}
    params.update(calibration_confirmed='true', motion_enabled='true')
    if adopt:
        params.update(adopt_zero_hold='true', handoff_token=config['handoff_token'])
    if bench:
        params['profile_kind'] = 'unloaded_ros_bench'
    for key, value in params.items():
        ET.SubElement(hardware, 'param', name=key).text = str(value)
    for name in JOINTS:
        values = config['joints'][name]
        if bench and not (-0.1 <= values['min_radians'] <= 0 <= values['max_radians'] <= 0.1
                          and values['max_velocity'] <= 0.1 and values['direction'] == 1
                          and values['zero_tick'] - values['min_tick'] <= 64
                          and values['max_tick'] - values['zero_tick'] <= 64):
            raise ValueError(f'{name}: unloaded bench profile exceeds its small-motion limits')
        # A calibration file may narrow, but cannot expand the model's provisional limits.
        limits = root.find(f"joint[@name='{name}']/limit")
        if not float(limits.get('lower')) <= values['min_radians'] < values['max_radians'] <= float(limits.get('upper')):
            raise ValueError(f'{name}: calibration exceeds model joint limits')
        limits.set('lower', str(values['min_radians']))
        limits.set('upper', str(values['max_radians']))
        limits.set('velocity', str(values['max_velocity']))
        joint = ET.SubElement(control, 'joint', name=name)
        ET.SubElement(joint, 'command_interface', name='position')
        ET.SubElement(joint, 'state_interface', name='position')
        for key, value in values.items():
            ET.SubElement(joint, 'param', name=key).text = str(value)
    model = ET.tostring(root, encoding='unicode')
    manager = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[str(bringup / 'config/arm_controllers.yaml'),
                               {'update_rate': 50}], output='screen')
    spawner = Node(package='controller_manager', executable='spawner',
                   arguments=['joint_state_broadcaster', 'arm_controller',
                              '--controller-manager-timeout', '30'], output='screen')
    def spawner_finished(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Controller startup failed'))]
        return []
    return [
        RegisterEventHandler(OnProcessExit(target_action=manager,
            on_exit=[EmitEvent(event=Shutdown(reason='Controller manager exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=spawner_finished)),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': model}], output='screen'),
        manager, spawner,
        Node(package='pinchy_bringup', executable='motion_monitor.py', output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration', description='Path to reviewed servo calibration YAML'),
        DeclareLaunchArgument('motion_enabled', default_value='false'),
        DeclareLaunchArgument('adopt_zero_hold', default_value='false'),
        DeclareLaunchArgument('unloaded_bench', default_value='false',
                              description='Use only for a captured profile of loose, unloaded motors'),
        OpaqueFunction(function=setup),
    ])
