"""Base station file: save, load and the turn-on-the-spot check (no ROS)."""
import numpy as np
import pytest
from nala_coverage import base_station
from nala_coverage.geometry import Grid


def room():
    cells = np.zeros((80, 80), np.int8); cells[[0, -1], :] = 100; cells[:, [0, -1]] = 100
    return Grid.from_cells(cells, .05, (0., 0., 0.))


def test_save_and_load_round_trip(tmp_path):
    path = tmp_path / 'base_station.yaml'
    base_station.save(path, (1.5, 2.25, 7.0), 'map')
    x, y, yaw = base_station.load(path, 'map')
    assert (x, y) == (1.5, 2.25) and yaw == pytest.approx(7.0 - 2 * np.pi, abs=1e-4)


def test_default_path_is_next_to_the_map(tmp_path):
    assert base_station.default_path(tmp_path / 'house1' / 'map.yaml') == tmp_path / 'house1' / 'base_station.yaml'


@pytest.mark.parametrize('text,match', [
    ('frame_id: odom\nx: 1\ny: 1\nyaw: 0\n', 'frame_id'),
    ('frame_id: map\nx: 1\nyaw: 0\n', 'x, y and yaw'),
    ('frame_id: map\nx: .nan\ny: 1\nyaw: 0\n', 'non-finite'),
])
def test_invalid_files_are_refused(tmp_path, text, match):
    path = tmp_path / 'base_station.yaml'; path.write_text(text)
    with pytest.raises(ValueError, match=match):
        base_station.load(path, 'map')


def test_missing_file_names_the_picker(tmp_path):
    with pytest.raises(ValueError, match='BASE_STATION_MISSING'):
        base_station.load(tmp_path / 'base_station.yaml', 'map')


def test_base_station_needs_room_to_turn():
    grid = room()
    base_station.check(grid, (2., 2., 0.), .3)
    with pytest.raises(ValueError, match='BASE_STATION_TOO_CLOSE'):
        base_station.check(grid, (.25, 2., 0.), .3)
