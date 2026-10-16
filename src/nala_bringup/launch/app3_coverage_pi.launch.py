"""App 3 (Pi): base driver + robot model + LiDAR + localization + Nav2 + coverage mission. Never starts motion.

ros2 launch nala_bringup app3_coverage_pi.launch.py map:=$HOME/NALA/maps/house1/map.yaml \
    plan:=$HOME/NALA/maps/house1/plan.yaml output_dir:=$HOME/NALA/output

The base station is maps/<name>/base_station.yaml (next to the map; base_station:= to override).
AMCL starts there (the supervisor sends it as the initial pose). Start the mission from the PC with the /coverage/* services (README).
The Xbox controller is not started: only the velocity gate may publish /cmd_vel while coverage runs.
hardware:=false leaves out base_node and the LiDAR driver (simulation, tools/sim).
"""
from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nala_coverage import base_station
from nala_coverage.geometry import Grid
from nala_coverage.mode_guard import preflight
from nala_coverage.params import load_settings

CONTROLLER_REMAPS = [('cmd_vel', '/cmd_vel_nav'), ('/cmd_vel', '/cmd_vel_nav')]


def _node(package, executable, name, **kwargs):
    return Node(package=package, executable=executable, name=name, output='screen',
                sigterm_timeout='6', sigkill_timeout='2', **kwargs)


def _setup(context):
    config_dir = Path(LaunchConfiguration('config_dir').perform(context)).resolve()
    map_yaml = LaunchConfiguration('map').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    plan = LaunchConfiguration('plan').perform(context)
    station_file = LaunchConfiguration('base_station').perform(context)
    hardware = LaunchConfiguration('hardware').perform(context).lower() in ('1', 'true', 'yes')
    stamped = LaunchConfiguration('nav2_stamped_supported').perform(context).lower() in ('1', 'true', 'yes')
    if not map_yaml:
        raise RuntimeError('app3_coverage_pi.launch.py requires map:=PATH_TO_MAP_YAML')
    if not output_dir:
        raise RuntimeError('app3_coverage_pi.launch.py requires output_dir:=PATH')
    if not plan:
        raise RuntimeError('app3_coverage_pi.launch.py requires plan:=PATH_TO_COVERAGE_TOOL_EXPORT')
    if not Path(plan).is_file():
        raise RuntimeError('app3_coverage_pi.launch.py plan:= file does not exist: ' + plan)
    station_file = str(Path(station_file or base_station.default_path(map_yaml)).resolve())
    settings = load_settings(config_dir, {'map_yaml': map_yaml, 'output_dir': output_dir,
                                          'base_station_yaml': station_file})
    try:
        station = base_station.load(station_file, settings['runtime']['map_frame'])
        base_station.check(Grid.load(map_yaml), station, settings.collision)
    except ValueError as error:
        raise RuntimeError(str(error)) from None
    nav_dir = Path(tempfile.mkdtemp(prefix='nala-nav2-'))
    nav_file = settings.render_nav2(map_yaml, nav_dir, stamped_supported=stamped)
    shared = {'config_dir': str(config_dir), 'map_yaml': str(Path(map_yaml).resolve()),
              'base_station_yaml': station_file, 'output_dir': str(Path(output_dir).resolve())}
    geometry = settings['geometry']
    # Robot model from the robot repo: base_footprint -> base_link -> laser (LiDAR facing backward).
    nodes = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('nala_description')) / 'launch/description.launch.py')),
        launch_arguments={'robot_config': str(config_dir / 'robot.yaml')}.items())]
    # The coverage sensor point, for RViz and tools; the coverage meter applies the same offset itself.
    nodes.append(_node(
        'tf2_ros', 'static_transform_publisher', 'scintillator_static_transform',
        arguments=['--x', str(geometry['sensor_offset_m']), '--y', '0', '--z', '0',
                   '--frame-id', settings['runtime']['base_frame'], '--child-frame-id', 'scintillator_link']))
    if hardware:
        nodes += [
            # Robot repo driver: /cmd_vel (from the velocity gate only) -> MCU; wheel speeds -> /odom and TF.
            _node('nala_base', 'base_node', 'base_node', parameters=[str(config_dir / 'base.yaml')]),
            _node('rplidar_ros', 'rplidar_composition', 'rplidar_node',
                  parameters=[str(config_dir / 'rplidar.yaml')]),
        ]
    nodes += [
        _node('nav2_map_server', 'map_server', 'map_server', parameters=[str(nav_file)]),
        _node('nav2_amcl', 'amcl', 'amcl', parameters=[str(nav_file)]),
        _node('nav2_planner', 'planner_server', 'planner_server', parameters=[str(nav_file)]),
        _node('nav2_controller', 'controller_server', 'controller_server', parameters=[str(nav_file)],
              remappings=CONTROLLER_REMAPS),
        _node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_localization',
              parameters=[{'autostart': True, 'use_sim_time': False, 'node_names': ['map_server', 'amcl']}]),
        _node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation',
              parameters=[{'autostart': True, 'use_sim_time': False,
                           'node_names': ['planner_server', 'controller_server']}]),
        _node('nala_coverage', 'coverage_supervisor', 'coverage_supervisor',
              parameters=[{**shared, 'plan_file': str(Path(plan).resolve())}]),
        _node('nala_coverage', 'coverage_meter', 'coverage_meter', parameters=[shared]),
        _node('nala_coverage', 'velocity_gate', 'velocity_safety_gate',
              parameters=[{**shared, 'mapping_mode': False}]),
    ]
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('nala_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value=str(share / 'config')),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('plan', default_value='', description='coverage_tool export to drive'),
        DeclareLaunchArgument('base_station', default_value='',
                              description='base station file (default: base_station.yaml next to the map)'),
        DeclareLaunchArgument('hardware', default_value='true',
                              description='false: no base_node and no LiDAR driver (simulation)'),
        DeclareLaunchArgument('nav2_stamped_supported', default_value='true'),
        OpaqueFunction(function=preflight),
        OpaqueFunction(function=_setup),
    ])
