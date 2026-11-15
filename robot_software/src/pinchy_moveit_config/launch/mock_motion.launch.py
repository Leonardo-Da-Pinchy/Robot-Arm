"""Complete mock planning workflow; never opens a serial port."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup=Path(get_package_share_directory('pinchy_bringup'))
    config=Path(get_package_share_directory('pinchy_moveit_config'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('panel',default_value='true'),
        DeclareLaunchArgument('calibration',default_value=''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/'launch/mock_control.launch.py')),
            launch_arguments={'rviz':'false','sliders':'false','calibration':LaunchConfiguration('calibration')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(config/'launch/planning.launch.py')),
            launch_arguments={'hardware':'false','rviz':LaunchConfiguration('rviz'),
                              'panel':LaunchConfiguration('panel')}.items())])
