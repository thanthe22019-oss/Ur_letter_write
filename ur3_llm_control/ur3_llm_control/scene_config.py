"""Load the shared scene configuration and generate simple Gazebo models."""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List

import yaml


REQUIRED_OBJECTS = {'red_cube', 'yellow_cube', 'blue_cube'}
REQUIRED_ZONES = {'zone_a', 'zone_b', 'zone_c'}


@dataclass(frozen=True)
class BoxSpec:
    """A configured box used by Gazebo, MoveIt, or RViz."""

    name: str
    pose: List[float]
    size: List[float]
    color: List[float]
    static: bool
    collision: bool


@dataclass(frozen=True)
class SceneConfig:
    """Validated scene data loaded from scene.yaml."""

    frame_id: str
    world_name: str
    floor: BoxSpec
    table: BoxSpec
    objects: Dict[str, BoxSpec]
    zones: Dict[str, BoxSpec]


def _number_list(raw: object, length: int, field: str) -> List[float]:
    if not isinstance(raw, list) or len(raw) != length:
        raise ValueError(f'{field} must contain exactly {length} numbers')
    if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in raw):
        raise ValueError(f'{field} must contain only numbers')
    return [float(value) for value in raw]


def _box(
    name: str,
    raw: object,
    *,
    static: bool,
    collision: bool,
) -> BoxSpec:
    if not isinstance(raw, dict):
        raise ValueError(f"scene entry '{name}' must be a mapping")
    pose = _number_list(raw.get('pose'), 6, f'{name}.pose')
    size = _number_list(raw.get('size'), 3, f'{name}.size')
    color = _number_list(raw.get('color'), 4, f'{name}.color')
    if any(value <= 0.0 for value in size):
        raise ValueError(f'{name}.size values must be positive')
    if any(value < 0.0 or value > 1.0 for value in color):
        raise ValueError(f'{name}.color values must be in [0, 1]')
    return BoxSpec(name, pose, size, color, static, collision)


def load_scene(path: str) -> SceneConfig:
    """Read and validate a scene YAML file."""
    scene_path = Path(path)
    if not scene_path.is_file():
        raise ValueError(f'scene file does not exist: {scene_path}')
    with scene_path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or not isinstance(document.get('scene'), dict):
        raise ValueError("scene.yaml must contain a top-level 'scene' mapping")

    raw_scene = document['scene']
    frame_id = raw_scene.get('frame_id')
    world_name = raw_scene.get('world_name')
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError('scene.frame_id must be a non-empty string')
    if not isinstance(world_name, str) or not world_name:
        raise ValueError('scene.world_name must be a non-empty string')

    raw_objects = raw_scene.get('objects')
    raw_zones = raw_scene.get('zones')
    if not isinstance(raw_objects, dict):
        raise ValueError('scene.objects must be a mapping')
    if not isinstance(raw_zones, dict):
        raise ValueError('scene.zones must be a mapping')
    missing_objects = REQUIRED_OBJECTS.difference(raw_objects)
    missing_zones = REQUIRED_ZONES.difference(raw_zones)
    if missing_objects:
        raise ValueError(f'missing required objects: {sorted(missing_objects)}')
    if missing_zones:
        raise ValueError(f'missing required zones: {sorted(missing_zones)}')

    objects = {
        name: _box(name, raw, static=False, collision=True)
        for name, raw in raw_objects.items()
    }
    zones = {
        name: _box(name, raw, static=True, collision=False)
        for name, raw in raw_zones.items()
    }
    return SceneConfig(
        frame_id=frame_id,
        world_name=world_name,
        floor=_box('floor', raw_scene.get('floor'), static=True, collision=True),
        table=_box('table', raw_scene.get('table'), static=True, collision=True),
        objects=objects,
        zones=zones,
    )


def gazebo_boxes(scene: SceneConfig) -> Iterable[BoxSpec]:
    """Return entities that must be spawned in Gazebo."""
    yield scene.table
    yield from scene.objects.values()
    yield from scene.zones.values()


def box_sdf(box: BoxSpec) -> str:
    """Generate an SDF model for a configured box."""
    pose = ' '.join(f'{value:.9g}' for value in box.pose)
    size = ' '.join(f'{value:.9g}' for value in box.size)
    color = ' '.join(f'{value:.9g}' for value in box.color)
    collision_xml = ''
    inertial_xml = ''
    if box.collision:
        collision_xml = f"""
        <collision name="collision">
          <geometry><box><size>{size}</size></box></geometry>
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>"""
    if not box.static:
        mass = 0.05
        x_size, y_size, z_size = box.size
        ixx = mass * (y_size**2 + z_size**2) / 12.0
        iyy = mass * (x_size**2 + z_size**2) / 12.0
        izz = mass * (x_size**2 + y_size**2) / 12.0
        inertial_xml = f"""
        <inertial>
          <mass>{mass}</mass>
          <inertia>
            <ixx>{ixx:.9g}</ixx><iyy>{iyy:.9g}</iyy><izz>{izz:.9g}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>"""

    return f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{box.name}">
    <static>{str(box.static).lower()}</static>
    <pose>{pose}</pose>
    <link name="body">{inertial_xml}{collision_xml}
      <visual name="visual">
        <geometry><box><size>{size}</size></box></geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
        <transparency>{1.0 - box.color[3]:.9g}</transparency>
      </visual>
    </link>
  </model>
</sdf>"""
