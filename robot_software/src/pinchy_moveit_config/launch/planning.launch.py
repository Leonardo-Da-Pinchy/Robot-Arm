"""Planning/application layer only; attaches to existing bringup without owning motors."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hardware=ParameterValue(LaunchConfiguration('hardware'),value_type=bool)
    rviz=ParameterValue(LaunchConfiguration('rviz'),value_type=bool)
    scene=Path(get_package_share_directory('pinchy_moveit_config'))/'config/scene.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('hardware',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('panel',default_value='true'),
        DeclareLaunchArgument('scene',default_value=str(scene)),
        Node(package='pinchy_moveit_config',executable='planning_stack.py',
             parameters=[{'hardware':hardware,'rviz':rviz}],output='screen'),
        Node(package='pinchy_motion',executable='motion_server',
             parameters=[{'hardware':hardware,'scene_file':LaunchConfiguration('scene')}],output='screen'),
        Node(package='pinchy_motion',executable='pose_panel',parameters=[{'hardware':hardware}],
             condition=IfCondition(LaunchConfiguration('panel')),output='screen')])
