"""Offline tests for the OpenAI Responses API planner boundary."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from ur3_llm_control.llm_planner import MockLLMPlanner, PlannerError
from ur3_llm_control.openai_planner import OpenAIPlanner
from ur3_llm_control.student_config import load_student_config
from ur3_llm_control.task_validator import parse_and_validate_plan


PACKAGE_DIR = Path(__file__).parents[1]
STUDENT_FILE = PACKAGE_DIR / 'config' / 'student_config.yaml'


class FakeResponses:
    def __init__(self, output_text=None, error=None):
        self.output_text = output_text
        self.error = error
        self.calls = []

    def create(self, **arguments):
        self.calls.append(arguments)
        if self.error is not None:
            raise self.error
        return SimpleNamespace(output_text=self.output_text)


class FakeClient:
    def __init__(self, output_text=None, error=None):
        self.responses = FakeResponses(output_text, error)


def test_openai_response_is_canonicalized_and_validated():
    response = json.dumps(
        {
            'plan': [
                {'skill': 'pick', 'object': 'red_cube', 'zone': None},
                {
                    'skill': 'place',
                    'object': 'red_cube',
                    'zone': 'zone_b',
                },
                {'skill': 'home', 'object': None, 'zone': None},
            ]
        }
    )
    client = FakeClient(response)
    planner = OpenAIPlanner(client=client, prompt='safe prompt')

    raw_json = planner.generate_plan('Đưa khối đỏ sang vùng B.')
    steps = parse_and_validate_plan(raw_json)

    assert [step.as_dict() for step in steps] == [
        {'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'},
        {'skill': 'home'},
    ]
    request = client.responses.calls[0]
    assert request['model'] == 'gpt-4o-mini'
    assert request['input'] == 'Đưa khối đỏ sang vùng B.'
    response_format = request['text']['format']
    assert response_format['type'] == 'json_schema'
    assert response_format['strict'] is True
    assert response_format['schema']['additionalProperties'] is False


def test_invalid_model_plan_is_rejected_before_robot_execution():
    response = json.dumps(
        {
            'plan': [
                {'skill': 'pick', 'object': 'red_cube', 'zone': None},
                {'skill': 'home', 'object': None, 'zone': None},
            ]
        }
    )
    planner = OpenAIPlanner(client=FakeClient(response), prompt='safe prompt')

    with pytest.raises(ValueError, match='home is unsafe'):
        planner.generate_plan('unsafe response')


def test_api_failure_is_closed_by_default():
    planner = OpenAIPlanner(
        client=FakeClient(error=TimeoutError('network timeout')),
        prompt='safe prompt',
    )

    with pytest.raises(PlannerError, match='OpenAI API request failed'):
        planner.generate_plan('Move the red cube to zone A.')


def test_explicit_mock_fallback_handles_api_timeout():
    fallback = MockLLMPlanner(load_student_config(str(STUDENT_FILE)))
    planner = OpenAIPlanner(
        client=FakeClient(error=TimeoutError('network timeout')),
        prompt='safe prompt',
        fallback=fallback,
    )

    steps = parse_and_validate_plan(
        planner.generate_plan('Move the blue cube to zone C.')
    )
    assert [step.as_dict() for step in steps] == [
        {'skill': 'pick', 'object': 'blue_cube'},
        {'skill': 'place', 'object': 'blue_cube', 'zone': 'zone_c'},
        {'skill': 'home'},
    ]


def test_empty_command_never_calls_openai():
    client = FakeClient('{}')
    planner = OpenAIPlanner(client=client, prompt='safe prompt')

    with pytest.raises(PlannerError, match='non-empty'):
        planner.generate_plan('   ')
    assert client.responses.calls == []


def test_mock_fallback_also_handles_missing_api_key(monkeypatch):
    monkeypatch.delenv('OPENAI_API_KEY', raising=False)
    fallback = MockLLMPlanner(load_student_config(str(STUDENT_FILE)))
    planner = OpenAIPlanner(prompt='safe prompt', fallback=fallback)

    steps = parse_and_validate_plan(
        planner.generate_plan('Move the red cube to zone A.')
    )
    assert [step.as_dict() for step in steps] == [
        {'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_a'},
        {'skill': 'home'},
    ]
