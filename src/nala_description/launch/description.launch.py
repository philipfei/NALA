"""Robot model: robot_state_publisher with the URDF (base_footprint -> base_link -> laser).

Included by the app launch files with robot_config:=<path to config/robot.yaml>.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('nala_description'), 'urdf', 'nala.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' robot_config:=', LaunchConfiguration('robot_config')]),
        value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('robot_config', description='Path to config/robot.yaml'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
    ])
