"""Fixed CAD zero-pose display. Publishes synthetic states; no motor interface."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description = Path(get_package_share_directory('pinchy_description'))
    bringup = Path(get_package_share_directory('pinchy_bringup'))
    model = (description / 'urdf/pinchy_arm.urdf').read_text()
    zeros = {name: 0.0 for name in ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
                                   'wrist_flex', 'wrist_roll', 'pen_pitch']}
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': model}], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='cad_zero_reference', parameters=[{'rate': 10, 'zeros': zeros}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(bringup / 'config/arm.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
