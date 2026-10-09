"""App 1 (Pi): base driver and Xbox controller. The PC runs the keyboard teleop.

ros2 launch nala_bringup app1_teleop_pi.launch.py [log_level:=debug]
log_level debug also shows the sent frames and the measured wheel speeds.
odom_tf:=false: base_node does not publish TF odom -> base_footprint (app 2: the EKF does).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('nala_bringup'), 'config')
    teleop_config = os.path.join(config_dir, 'teleop.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('odom_tf', default_value='true'),
        Node(
            package='nala_base',
            executable='base_node',
            parameters=[os.path.join(config_dir, 'base.yaml'),
                        {'publish_tf': ParameterValue(LaunchConfiguration('odom_tf'), value_type=bool)}],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen',
        ),
        # Xbox controller: hold LB and use the sticks (config/teleop.yaml).
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[teleop_config],
            output='screen',
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[teleop_config],
            output='screen',
        ),
    ])
