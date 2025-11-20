from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gazebo_pkg = get_package_share_directory('ackermann')
    nav2_pkg = get_package_share_directory('ackermann')

    gazebo_launch = os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
    mppi_smac_launch = os.path.join(nav2_pkg, 'launch', 'mppi_smac_2d.launch.py')

    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch)
    )

    mppi_smac_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mppi_smac_launch)
    )

    return LaunchDescription([
        gazebo_launch_include,
        mppi_smac_launch_include,
    ])
