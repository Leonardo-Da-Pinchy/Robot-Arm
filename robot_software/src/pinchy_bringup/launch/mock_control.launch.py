"""Six-joint integration fixture. Always mock; no serial hardware option."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    description = Path(get_package_share_directory('pinchy_description'))
    bringup = Path(get_package_share_directory('pinchy_bringup'))
    model = xacro.process_file(str(description / 'urdf/pinchy_mock.urdf.xacro')).toxml()
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('sliders', default_value='false'),
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
    ])
