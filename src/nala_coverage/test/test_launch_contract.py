"""Static launch-contract tests that run without ROS hardware or Nav2 imports."""
import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / 'nala_bringup' / 'launch'
APP3 = ('app3_coverage_pi.launch.py', 'app3_coverage_pc.launch.py', 'app3_base_station_pc.launch.py')


def source(name):
    path = LAUNCH / name
    text = path.read_text(encoding='utf-8')
    ast.parse(text, filename=str(path))
    return text


def test_coverage_launch_has_exact_nav2_process_contract_and_remap():
    text = source('app3_coverage_pi.launch.py')
    expected = {
        "_node('nav2_map_server', 'map_server'",
        "_node('nav2_amcl', 'amcl'",
        "_node('nav2_planner', 'planner_server'",
        "_node('nav2_controller', 'controller_server'",
        "_node('nav2_lifecycle_manager', 'lifecycle_manager'",
    }
    assert all(token in text for token in expected)
    assert "CONTROLLER_REMAPS = [('cmd_vel', '/cmd_vel_nav'), ('/cmd_vel', '/cmd_vel_nav')]" in text
    assert "'plan_file'" in text and text.count("'plan_file'")==1
    # An exported plan is followed by the supervisor with the planner and controller only.
    for forbidden in ('bt_navigator', 'waypoint_follower', 'smoother_server', 'velocity_smoother',
                      'behavior_server', 'nav2_behaviors', 'collision_monitor'):
        assert forbidden not in text


def test_coverage_launch_uses_the_robot_repo_driver_model_and_lidar():
    text = source('app3_coverage_pi.launch.py')
    # The robot repo's base driver, URDF (LiDAR facing backward) and LiDAR settings; no own copies.
    assert "_node('nala_base', 'base_node'" in text and "'base.yaml'" in text
    assert "'nala_description'" in text and "'robot.yaml'" in text
    assert "'rplidar.yaml'" in text
    # The velocity gate is the only /cmd_vel publisher: no joystick teleop, no own odom TF or laser TF.
    for forbidden in ('teleop_twist_joy', 'joy_node', 'odom_tf', 'laser_static_transform', 'slam_toolbox'):
        assert forbidden not in text, forbidden
    assert text.count("'velocity_gate'") == 1 and "'/cmd_vel_remote'" not in text


def test_launches_have_bounded_shutdown_and_no_respawn():
    for name in APP3:
        text = source(name)
        assert "sigterm_timeout='6'" in text
        assert "sigkill_timeout='2'" in text
        assert 'respawn=True' not in text


def test_no_create3_nodes_or_tf_relay_and_forward_only_autodrive():
    for name in APP3:
        text = source(name).lower()
        for forbidden in ('irobot', 'create3', 'tf_relay', 'undock', 'explor'):
            assert forbidden not in text, (name, forbidden)
    # No Nav2 behavior server: no BackUp recovery can drive NALA backwards in autodrive.
    assert 'behavior_server' not in source('app3_coverage_pi.launch.py')
