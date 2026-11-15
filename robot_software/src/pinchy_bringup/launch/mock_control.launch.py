"""Six-joint integration fixture. Always mock; no serial hardware option."""

from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
from pinchy_bringup.control_contract import JOINTS


def launch_setup(context):
    description = Path(get_package_share_directory('pinchy_description'))
    bringup = Path(get_package_share_directory('pinchy_bringup'))
    model = xacro.process_file(str(description / 'urdf/pinchy_mock.urdf.xacro')).toxml()
    calibration=LaunchConfiguration('calibration').perform(context)
    if calibration:
        config=yaml.safe_load(Path(calibration).expanduser().read_text());root=ET.fromstring(model)
        if set(config['joints'])!=set(JOINTS):raise ValueError('Mock calibration must contain exactly the six shared joints')
        for name,values in config['joints'].items():
            limit=root.find(f"joint[@name='{name}']/limit")
            low,high,velocity=values['min_radians'],values['max_radians'],values['max_velocity']
            if limit is None or not all(math.isfinite(v) for v in (low,high,velocity)):
                raise ValueError('Invalid mock calibration joint/limits: '+name)
            if not (float(limit.get('lower'))<=low<=0<=high<=float(limit.get('upper')) and low<high and 0<velocity<=float(limit.get('velocity'))):
                raise ValueError('Mock calibration exceeds the shared CAD limits: '+name)
            control_joint=root.find(f"./ros2_control/joint[@name='{name}']")
            ET.SubElement(control_joint,'param',name='nominal_model_lower').text=limit.get('lower')
            ET.SubElement(control_joint,'param',name='nominal_model_upper').text=limit.get('upper')
            limit.set('lower',str(low));limit.set('upper',str(high));limit.set('velocity',str(velocity))
        model=ET.tostring(root,encoding='unicode')
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': model}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[str(bringup / 'config/arm_controllers.yaml')],
             remappings=[('~/robot_description', '/robot_description')], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'arm_controller',
                        '--controller-manager-timeout', '30'], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(bringup / 'config/arm.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
        Node(package='pinchy_bringup', executable='joint_sliders.py',
             condition=IfCondition(LaunchConfiguration('sliders')), output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('sliders',default_value='false'),
        DeclareLaunchArgument('calibration',default_value='',description='Optional saved profile: apply its joint bounds/velocity to mock geometry only; never opens its port'),
        OpaqueFunction(function=launch_setup)])
