"""A small map library for the panel tests: the example room map with its base station and plan."""
from pathlib import Path
import shutil
import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
EXAMPLES = REPO / 'coverage_tool' / 'examples'


@pytest.fixture
def maps_dir(tmp_path):
    """maps/room/ as app 2 and the panel make it: map.yaml + map.pgm, base_station.yaml, plans/."""
    room = tmp_path / 'maps' / 'room'
    (room / 'plans').mkdir(parents=True)
    meta = yaml.safe_load((EXAMPLES / 'map_ME_room1v4.yaml').read_text())
    shutil.copy(EXAMPLES / meta['image'], room / 'map.pgm')
    meta['image'] = 'map.pgm'
    (room / 'map.yaml').write_text(yaml.safe_dump(meta))
    shutil.copy(EXAMPLES / 'map_ME_room1v4_base_station.yaml', room / 'base_station.yaml')
    shutil.copy(EXAMPLES / 'map_ME_room1v4_nala.yaml', room / 'plans' / '2026-10-08_120000.yaml')
    (tmp_path / 'maps' / 'not_a_map').mkdir()
    return tmp_path / 'maps'


# The example room, its base station and plan were made for this outline (the robot until 2026-10-16, centred
# 410 x 360 mm). The tests use it, so they do not depend on the real robot's outline in config/robot.yaml.
TEST_OUTLINE = {'front': 0.205, 'back': 0.205, 'left': 0.18, 'right': 0.18}


@pytest.fixture
def config_dir(tmp_path):
    """A copy of the repo config/ with the test robot outline."""
    config = tmp_path / 'config'
    shutil.copytree(REPO / 'config', config)
    robot = yaml.safe_load((config / 'robot.yaml').read_text())
    robot['outline'] = dict(TEST_OUTLINE)
    (config / 'robot.yaml').write_text(yaml.safe_dump(robot))
    return config


@pytest.fixture
def settings(config_dir):
    from nala_coverage.params import load_settings
    return load_settings(config_dir)
