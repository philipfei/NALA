"""App 2 (Pi): app 1 (base driver, Xbox controller) + robot model + LiDAR + IMU + EKF + SLAM.

The EKF (robot_localization) publishes TF odom -> base_footprint from the wheel speeds
(vx, vy) and the IMU turn rate, so base_node does not publish it here (odom_tf:=false).

ros2 launch nala_bringup app2_slam_pi.launch.py [log_level:=debug]
The PC runs RViz (app2_slam_pc.launch.py) and the keyboard teleop.
Save the map with scripts/save_map.sh while this runs.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('nala_bringup')
    config_dir = os.path.join(bringup_dir, 'config')
    return LaunchDescription([
        # Robot model first: it publishes imu_link on /tf_static, which the EKF needs.
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('nala_description'),
                         'launch', 'description.launch.py'),
            launch_arguments={'robot_config': os.path.join(config_dir, 'robot.yaml')}.items()),
        IncludeLaunchDescription(
            os.path.join(bringup_dir, 'launch', 'app1_teleop_pi.launch.py'),
            launch_arguments={'odom_tf': 'false'}.items()),
        Node(
            package='nala_base',
            executable='imu_node',
            parameters=[os.path.join(config_dir, 'imu.yaml')],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(config_dir, 'ekf.yaml')],
            output='screen',
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[os.path.join(config_dir, 'rplidar.yaml')],
            output='screen',
        ),
        # slam_toolbox is a lifecycle node. Its own launch file configures and activates it.
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py'),
            launch_arguments={
                'slam_params_file': os.path.join(config_dir, 'slam_mapping.yaml'),
                'use_sim_time': 'false',
            }.items()),
    ])
