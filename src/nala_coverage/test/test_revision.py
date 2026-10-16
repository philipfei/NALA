"""Behavioral regression tests for the shared configuration and the rendered Nav2 parameters."""
import math
import shutil
import pytest
import yaml
from nala_coverage.params import load_settings, resource


def copy_config(tmp_path):
    shutil.copytree(resource('.'), tmp_path/'config', dirs_exist_ok=True)
    return tmp_path/'config'


def edit(path, section, key, value):
    raw = yaml.safe_load(path.read_text())
    params = next(iter(raw.values()))['ros__parameters']
    params[section][key] = value
    path.write_text(yaml.safe_dump(raw))


def test_config_hash_ignores_comments_order_and_numeric_spelling(tmp_path):
    config = copy_config(tmp_path)
    base = load_settings(); path = config/'coverage.yaml'
    raw = yaml.safe_load(path.read_text())
    raw = dict(reversed(list(raw.items())))
    path.write_text('# Commentary only\n'+yaml.safe_dump(raw, sort_keys=False))
    assert load_settings(config).hash == base.hash
    edit(config/'coverage.yaml', 'motion', 'linear_m_s', .11)
    changed = load_settings(config)
    assert changed.hash != base.hash
    rendered = yaml.safe_load(changed.render_nav2('/tmp/map.yaml', tmp_path/'rendered').read_text())
    assert rendered['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] == .11


def test_outline_is_the_robot_repo_box_and_collision_radius_its_turning_circle():
    s = load_settings()
    robot = yaml.safe_load(resource('robot.yaml').read_text())
    hx, hy = robot['body_length']/2, robot['body_width']/2
    assert s['geometry']['footprint_m'] == [[hx, hy], [hx, -hy], [-hx, -hy], [-hx, hy]]
    corner = max(math.hypot(x, y) for x, y in s['geometry']['footprint_m'])
    assert s.circumscribed == pytest.approx(corner) == pytest.approx(math.hypot(hx, hy))
    assert s.collision == pytest.approx(corner + s['geometry']['planning_padding_m'])


def test_outline_is_not_configured_twice(tmp_path):
    config = copy_config(tmp_path)
    edit(config/'coverage.yaml', 'geometry', 'footprint_m', [[.3, .2], [.3, -.2], [-.2, -.2], [-.2, .2]])
    with pytest.raises(ValueError, match='unknown'):
        load_settings(config)


@pytest.mark.parametrize('key,value', [('body_length', 0.0), ('body_width', -0.3), ('body_length', None)])
def test_invalid_robot_body_rejected(tmp_path, key, value):
    config = copy_config(tmp_path)
    robot = yaml.safe_load((config/'robot.yaml').read_text()); robot[key] = value
    (config/'robot.yaml').write_text(yaml.safe_dump(robot))
    with pytest.raises(ValueError):
        load_settings(config)


def test_rendered_localization_uses_robot_repo_frames(tmp_path):
    s = load_settings()
    rendered = yaml.safe_load(s.render_nav2('/tmp/map.yaml', tmp_path).read_text())
    amcl = rendered['amcl']['ros__parameters']
    # The initial pose comes from the supervisor (base station with a spread), not from a zero-spread parameter.
    assert amcl['set_initial_pose'] is False
    assert amcl['base_frame_id'] == 'base_footprint' and rendered['map_server']['ros__parameters']['yaml_filename'] == '/tmp/map.yaml'


def test_speeds_never_round_to_zero_in_the_mcu_protocol():
    # The MCU protocol sends speeds in 0.02 m/s steps; below 0.01 m/s a command becomes 0 (docs/mcu_protocol.md).
    rpp = load_settings()['nav2']['controller_server']['ros__parameters']['FollowPath']
    assert rpp['min_approach_linear_velocity'] >= .04 and rpp['regulated_linear_scaling_min_speed'] >= .04


def test_rendered_nav2_uses_turning_circle_for_planning_and_box_for_the_controller(tmp_path):
    s = load_settings()
    rendered = yaml.safe_load(s.render_nav2('/tmp/map.yaml', tmp_path).read_text())
    glob = rendered['global_costmap']['global_costmap']['ros__parameters']
    local = rendered['local_costmap']['local_costmap']['ros__parameters']
    # Half a cell more than the collision radius: NavFn measures from cell centres, the supervisor from cell edges.
    assert glob['robot_radius'] == pytest.approx(s.collision + glob['resolution'] / 2) and 'footprint' not in glob
    assert 'robot_radius' not in local
    assert yaml.safe_load(local['footprint']) == s['geometry']['footprint_m']
    # No padding in the controller's check: the plan's margin to walls is about one 5 cm cell already.
    assert local['footprint_padding'] == 0.0 and local['resolution'] <= .025
    assert glob['robot_base_frame'] == local['robot_base_frame'] == 'base_footprint'
    for name in ('global_costmap', 'local_costmap'):
        params = rendered[name][name]['ros__parameters']
        assert params['inflation_layer']['inflation_radius'] >= s.collision
    rpp = rendered['controller_server']['ros__parameters']['FollowPath']
    # Autodrive is forward only: no reversing; a goal behind the robot is turned towards on the spot.
    assert rpp['allow_reversing'] is False and rpp['use_rotate_to_heading'] is True
    assert rpp['use_regulated_linear_velocity_scaling'] is True
    assert rpp['regulated_linear_scaling_min_radius'] == s['motion']['linear_m_s']/s['motion']['angular_rad_s']
    for absent in ('bt_navigator', 'behavior_server'):
        assert absent not in rendered


@pytest.mark.parametrize('section,key,value', [
    ('geometry', 'sensor_offset_m', 0.7),                                    # beyond the coverage disk
    ('geometry', 'sensor_offset_m', -0.1),
    ('geometry', 'coverage_disk_radius_m', 0.0),
])
def test_invalid_robot_settings_rejected(tmp_path, section, key, value):
    config = copy_config(tmp_path)
    edit(config/'coverage.yaml', section, key, value)
    with pytest.raises(ValueError):
        load_settings(config)


def test_battery_is_optional_but_its_thresholds_must_be_ordered(tmp_path):
    config = copy_config(tmp_path)
    assert load_settings(config)['battery']['enabled'] is False
    edit(config/'coverage.yaml', 'battery', 'critical_ratio', .5)
    with pytest.raises(ValueError):
        load_settings(config)


def test_no_create3_settings_remain():
    s = load_settings()
    assert 'create3' not in s.values and 'dock_file' not in s['runtime']
    text = ''.join(resource(n).read_text() for n in ('coverage.yaml', 'nav2_params.yaml', 'localization.yaml'))
    for word in ('create3', 'dock', 'hazard', 'bump'):
        assert word not in text.lower(), word


def test_costmaps_mark_lidar_points_at_the_lidar_height():
    # Nav2 jazzy defaults each observation source's max_obstacle_height to 0.0 m, which drops every LiDAR
    # point above the floor: the costmaps would only clear, never mark an obstacle.
    lidar = yaml.safe_load(resource('robot.yaml').read_text())['laser']   # robot repo model
    nav2 = yaml.safe_load(resource('nav2_params.yaml').read_text())
    for name in ('local_costmap', 'global_costmap'):
        layer = nav2[name][name]['ros__parameters']['obstacle_layer']
        for source in layer['observation_sources'].split():
            s = layer[source]
            assert s['min_obstacle_height'] <= lidar['z'] < s['max_obstacle_height'], (name, source)
            assert s['marking'] is True
