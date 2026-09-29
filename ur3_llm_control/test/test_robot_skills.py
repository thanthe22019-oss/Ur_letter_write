"""Tests for the safe PlanStep-to-action request boundary."""

import pytest

from ur3_llm_control.robot_skills import (
    RobotSkillRequest,
    request_from_plan_step,
)
from ur3_llm_control.task_validator import PlanStep


@pytest.mark.parametrize(
    ('step', 'expected'),
    [
        (
            PlanStep(skill='pick', object_name='red_cube'),
            RobotSkillRequest(skill='pick', object_name='red_cube'),
        ),
        (
            PlanStep(
                skill='place',
                object_name='yellow_cube',
                zone_name='zone_b',
            ),
            RobotSkillRequest(
                skill='place',
                object_name='yellow_cube',
                zone_name='zone_b',
            ),
        ),
        (
            PlanStep(skill='home'),
            RobotSkillRequest(skill='home'),
        ),
    ],
)
def test_plan_step_maps_to_action_request(step, expected):
    assert request_from_plan_step(step) == expected


def test_unvalidated_dictionary_cannot_cross_robot_skill_boundary():
    with pytest.raises(TypeError):
        request_from_plan_step({
            'skill': 'joint_trajectory',
            'joint_positions': [0.0],
        })
