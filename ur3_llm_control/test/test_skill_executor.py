"""Tests for the Milestone 5 sequential skill executor."""

import json

import pytest

from ur3_llm_control.skill_executor import (
    SkillResult,
    execute_validated_plan,
    validate_and_execute_plan,
)
from ur3_llm_control.task_validator import (
    PlanStep,
    PlanValidationError,
    parse_and_validate_plan,
)


VALID_PLAN = json.dumps({
    'plan': [
        {'skill': 'pick', 'object': 'red_cube'},
        {
            'skill': 'place',
            'object': 'red_cube',
            'zone': 'zone_a',
        },
        {'skill': 'home'},
    ],
})


def successful_runner(calls):
    def run(step):
        calls.append(step.label())
        return SkillResult(0, 'SUCCESS', f'{step.label()} completed')

    return run


def test_valid_plan_runs_every_step_in_order():
    calls = []
    report = validate_and_execute_plan(VALID_PLAN, successful_runner(calls))

    assert report.succeeded
    assert calls == [
        'pick(red_cube)',
        'place(red_cube, zone_a)',
        'home()',
    ]
    assert len(report.completed) == report.total_steps == 3
    assert report.failed_step is None


def test_executor_stops_immediately_after_skill_failure():
    calls = []

    def run(step):
        calls.append(step.label())
        if step.skill == 'place':
            return SkillResult(4, 'PLANNING_FAILED', 'no collision-free path')
        return SkillResult(0, 'SUCCESS')

    plan = parse_and_validate_plan(VALID_PLAN)
    report = execute_validated_plan(plan, run)

    assert not report.succeeded
    assert calls == ['pick(red_cube)', 'place(red_cube, zone_a)']
    assert report.failed_step.step.skill == 'place'
    assert report.failed_step.result.status == 'PLANNING_FAILED'


@pytest.mark.parametrize(
    ('code', 'status'),
    [
        (1, 'FAILED'),
        (4, 'PLANNING_FAILED'),
        (5, 'EXECUTION_FAILED'),
        (-1, 'RESULT_TIMEOUT'),
    ],
)
def test_every_non_success_result_stops_the_plan(code, status):
    calls = []

    def run(step):
        calls.append(step.skill)
        return SkillResult(code, status)

    report = validate_and_execute_plan(VALID_PLAN, run)
    assert not report.succeeded
    assert calls == ['pick']


def test_invalid_plan_never_invokes_the_skill_runner():
    calls = []
    invalid_plan = json.dumps({
        'plan': [{'skill': 'pick', 'object': 'green_cube'}],
    })

    with pytest.raises(PlanValidationError):
        validate_and_execute_plan(invalid_plan, successful_runner(calls))
    assert calls == []


def test_executor_converts_runner_exception_to_failure_and_stops():
    plan = parse_and_validate_plan(VALID_PLAN)

    def run(_step):
        raise RuntimeError('action connection lost')

    report = execute_validated_plan(plan, run)
    assert not report.succeeded
    assert len(report.completed) == 1
    assert report.failed_step.result.status == 'EXECUTOR_ERROR'
    assert report.failed_step.result.message == 'action connection lost'


def test_step_callback_receives_each_completed_step():
    observed = []
    plan = parse_and_validate_plan(VALID_PLAN)

    execute_validated_plan(
        plan,
        lambda _step: SkillResult(0, 'SUCCESS'),
        observed.append,
    )
    assert [item.step.skill for item in observed] == ['pick', 'place', 'home']


def test_executor_rejects_unvalidated_values():
    with pytest.raises(TypeError):
        execute_validated_plan([{'skill': 'home'}], lambda _step: None)


def test_executor_rejects_empty_plan():
    with pytest.raises(ValueError):
        execute_validated_plan([], lambda _step: None)


def test_success_requires_both_success_code_and_status():
    assert SkillResult(0, 'SUCCESS').succeeded
    assert not SkillResult(0, 'FAILED').succeeded
    assert not SkillResult(1, 'SUCCESS').succeeded


def test_plan_step_type_is_the_validator_canonical_type():
    plan = parse_and_validate_plan(VALID_PLAN)
    assert all(isinstance(step, PlanStep) for step in plan)
