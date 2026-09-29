"""Tests for student-ID personalization required by the assignment."""

from pathlib import Path

import pytest

from ur3_llm_control.student_config import (
    OBJECT_ORDERS,
    ZONES,
    load_student_config,
    mapping_for_student_id,
)


STUDENT_FILE = Path(__file__).parents[1] / 'config' / 'student_config.yaml'


def test_real_student_id_mapping():
    """MSSV 23020730 must select permutation zero from the assignment."""
    config = load_student_config(str(STUDENT_FILE))
    assert config.student_id == '23020730'
    assert config.permutation_index == 0
    assert config.zone_to_object == {
        'zone_a': 'red_cube',
        'zone_b': 'yellow_cube',
        'zone_c': 'blue_cube',
    }


@pytest.mark.parametrize('suffix', range(6))
def test_all_six_assignment_permutations(suffix):
    """Each remainder must use the corresponding mapping from the brief."""
    permutation_index, mapping = mapping_for_student_id(f'230207{suffix:02d}')
    assert permutation_index == suffix
    assert tuple(mapping[zone] for zone in ZONES) == OBJECT_ORDERS[suffix]


@pytest.mark.parametrize('student_id', ['', '7', '2302073A', 23020730, None])
def test_invalid_student_ids_are_rejected(student_id):
    """IDs must stay strings so leading zeroes are never discarded."""
    with pytest.raises(ValueError):
        mapping_for_student_id(student_id)
