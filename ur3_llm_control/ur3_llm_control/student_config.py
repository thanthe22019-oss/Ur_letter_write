"""Load the real student ID and derive the assignment's object-zone mapping."""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple

import yaml


OBJECT_ORDERS: Tuple[Tuple[str, str, str], ...] = (
    ('red_cube', 'yellow_cube', 'blue_cube'),
    ('red_cube', 'blue_cube', 'yellow_cube'),
    ('yellow_cube', 'red_cube', 'blue_cube'),
    ('yellow_cube', 'blue_cube', 'red_cube'),
    ('blue_cube', 'red_cube', 'yellow_cube'),
    ('blue_cube', 'yellow_cube', 'red_cube'),
)
ZONES = ('zone_a', 'zone_b', 'zone_c')


@dataclass(frozen=True)
class StudentConfig:
    """Validated student identity and its deterministic assignment mapping."""

    name: str
    student_id: str
    permutation_index: int
    zone_to_object: Dict[str, str]


def mapping_for_student_id(student_id: str) -> Tuple[int, Dict[str, str]]:
    """Calculate P and the zone-to-object mapping from the final two digits."""
    if not isinstance(student_id, str) or len(student_id) < 2 or not student_id.isdigit():
        raise ValueError('student.id must be a digit string containing at least two digits')
    permutation_index = int(student_id[-2:]) % len(OBJECT_ORDERS)
    mapping = dict(zip(ZONES, OBJECT_ORDERS[permutation_index]))
    return permutation_index, mapping


def load_student_config(path: str) -> StudentConfig:
    """Read and validate student_config.yaml."""
    config_path = Path(path)
    if not config_path.is_file():
        raise ValueError(f'student config does not exist: {config_path}')
    with config_path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or not isinstance(document.get('student'), dict):
        raise ValueError("student_config.yaml must contain a top-level 'student' mapping")

    student = document['student']
    name = student.get('name')
    student_id = student.get('id')
    if not isinstance(name, str) or not name.strip():
        raise ValueError('student.name must be a non-empty string')
    permutation_index, mapping = mapping_for_student_id(student_id)
    return StudentConfig(name.strip(), student_id, permutation_index, mapping)


def single_pick_place_plan(
    config: StudentConfig,
    object_name: str,
) -> Dict[str, object]:
    """Build one pick/place/home plan from the student's computed mapping."""
    matching_zones = [
        zone
        for zone, mapped_object in config.zone_to_object.items()
        if mapped_object == object_name
    ]
    if len(matching_zones) != 1:
        raise ValueError(
            f"object '{object_name}' is not uniquely mapped for "
            f"student {config.student_id}"
        )
    zone_name = matching_zones[0]
    return {
        'plan': [
            {'skill': 'pick', 'object': object_name},
            {
                'skill': 'place',
                'object': object_name,
                'zone': zone_name,
            },
            {'skill': 'home'},
        ],
    }


def student_arrangement_plan(config: StudentConfig) -> Dict[str, object]:
    """Build the complete arrangement, returning home after every placement."""
    steps = []
    for zone_name in ZONES:
        object_name = config.zone_to_object[zone_name]
        steps.extend([
            {'skill': 'pick', 'object': object_name},
            {
                'skill': 'place',
                'object': object_name,
                'zone': zone_name,
            },
            {'skill': 'home'},
        ])
    return {'plan': steps}
