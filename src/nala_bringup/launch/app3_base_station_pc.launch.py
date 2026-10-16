"""App 3 (PC): show a saved map and click the base station with "2D Goal Pose" in RViz.

ros2 launch nala_bringup app3_base_station_pc.launch.py map:=maps/house1/map.yaml
Saves maps/house1/base_station.yaml (next to the map). Click again to replace it.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(package, executable, name, **kwargs):
    return Node(package=package, executable=executable, name=name, output='screen',
                sigterm_timeout='6', sigkill_timeout='2', **kwargs)


def _setup(context):
    share = Path(get_package_share_directory('nala_bringup'))
    map_yaml = LaunchConfiguration('map').perform(context)
    if not map_yaml or not Path(map_yaml).is_file():
        raise RuntimeError('app3_base_station_pc.launch.py requires map:=PATH_TO_MAP_YAML')
    map_yaml = str(Path(map_yaml).resolve())
    return [
        _node('nav2_map_server', 'map_server', 'map_server',
              parameters=[{'yaml_filename': map_yaml, 'frame_id': 'map', 'topic_name': 'map'}]),
        _node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_map',
              parameters=[{'autostart': True, 'node_names': ['map_server']}]),
        # coverage.rviz's "2D Goal Pose" tool publishes /coverage/test_goal.
        _node('nala_coverage', 'base_station_picker', 'base_station_picker',
              parameters=[{'config_dir': str(share / 'config'), 'map_yaml': map_yaml}],
              remappings=[('/goal_pose', '/coverage/test_goal')]),
        _node('rviz2', 'rviz2', 'nala_base_station_rviz', arguments=['-d', str(share / 'rviz' / 'coverage.rviz')]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map', default_value=''), OpaqueFunction(function=_setup)])
