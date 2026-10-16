"""App 3 (PC): RViz to watch the coverage mission and set or check the start pose.

ros2 launch nala_bringup app3_coverage_pc.launch.py
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('nala_bringup')) / 'rviz' / 'coverage.rviz'
    return LaunchDescription([Node(
        package='rviz2', executable='rviz2', name='nala_coverage_rviz', arguments=['-d', str(config)],
        output='screen', sigterm_timeout='6', sigkill_timeout='2')])
