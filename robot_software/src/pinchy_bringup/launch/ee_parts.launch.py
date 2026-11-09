"""Review converted EE source parts in a gallery; no control or motor access."""
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
    model = xacro.process_file(str(description / 'urdf/ee_parts_preview.urdf.xacro')).toxml()
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': model}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(bringup / 'config/arm.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
