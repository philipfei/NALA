"""App 1 (Pi): base driver. The PC runs the keyboard teleop.

ros2 launch nala_bringup app1_teleop_pi.launch.py [log_level:=debug]
log_level debug also shows the sent frames and the measured wheel speeds.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('nala_bringup'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='nala_base',
            executable='base_node',
            parameters=[os.path.join(config_dir, 'base.yaml')],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen',
        ),
    ])
