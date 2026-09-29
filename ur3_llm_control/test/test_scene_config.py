"""Tests for the YAML-backed Gazebo and MoveIt scene configuration."""

from pathlib import Path
from xml.etree import ElementTree

import pytest

from ur3_llm_control.scene_config import box_sdf, gazebo_boxes, load_scene


SCENE_FILE = Path(__file__).parents[1] / 'config' / 'scene.yaml'


@pytest.fixture
def scene():
    """Load the repository's default scene."""
    return load_scene(str(SCENE_FILE))


def test_required_entities_are_present(scene):
    """The assignment's three cubes and three target zones must exist."""
    assert set(scene.objects) == {'red_cube', 'yellow_cube', 'blue_cube'}
    assert set(scene.zones) == {'zone_a', 'zone_b', 'zone_c'}
    assert all(spec.collision for spec in scene.objects.values())
    assert all(not spec.collision for spec in scene.zones.values())


def test_gazebo_sdf_uses_yaml_pose_and_size(scene):
    """Every generated model must preserve its configured pose and size."""
    for spec in gazebo_boxes(scene):
        model = ElementTree.fromstring(box_sdf(spec)).find('model')
        assert model is not None
        assert model.attrib['name'] == spec.name
        pose = [float(value) for value in model.findtext('pose').split()]
        size = [
            float(value)
            for value in model.find('link/visual/geometry/box/size').text.split()
        ]
        assert pose == pytest.approx(spec.pose)
        assert size == pytest.approx(spec.size)


def test_cubes_rest_on_table_surface(scene):
    """Configured cube bottoms must coincide with the table surface."""
    table_surface = scene.table.pose[2] + scene.table.size[2] / 2.0
    for cube in scene.objects.values():
        cube_bottom = cube.pose[2] - cube.size[2] / 2.0
        assert cube_bottom == pytest.approx(table_surface)
    for zone in scene.zones.values():
        zone_top = zone.pose[2] + zone.size[2] / 2.0
        assert zone_top == pytest.approx(table_surface)


def test_target_zones_form_an_even_row(scene):
    """The three visual targets must stay aligned and evenly spaced."""
    zone_a = scene.zones['zone_a'].pose
    zone_b = scene.zones['zone_b'].pose
    zone_c = scene.zones['zone_c'].pose
    assert zone_a[0] == pytest.approx(zone_b[0])
    assert zone_b[0] == pytest.approx(zone_c[0])
    assert zone_a[1] - zone_b[1] == pytest.approx(0.12)
    assert zone_b[1] - zone_c[1] == pytest.approx(0.12)


def test_each_cube_faces_its_matching_zone(scene):
    """Each coloured cube must stay opposite its matching coloured zone."""
    pairs = (
        ('red_cube', 'zone_a'),
        ('yellow_cube', 'zone_b'),
        ('blue_cube', 'zone_c'),
    )
    row_x = None
    separation = None
    for cube_name, zone_name in pairs:
        cube = scene.objects[cube_name].pose
        zone = scene.zones[zone_name].pose
        if row_x is None:
            row_x = cube[0]
            separation = zone[0] - cube[0]
        assert cube[0] == pytest.approx(row_x)
        assert cube[1] == pytest.approx(zone[1])
        assert zone[0] - cube[0] == pytest.approx(separation)
        assert separation > 0.0
