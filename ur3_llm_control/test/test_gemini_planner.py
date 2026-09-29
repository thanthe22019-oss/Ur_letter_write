"""Offline tests for the Google Gemini planner boundary."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from ur3_llm_control.gemini_planner import GeminiPlanner
from ur3_llm_control.llm_planner import MockLLMPlanner, PlannerError
from ur3_llm_control.student_config import load_student_config
from ur3_llm_control.task_validator import parse_and_validate_plan


PACKAGE_DIR = Path(__file__).parents[1]
STUDENT_FILE = PACKAGE_DIR / 'config' / 'student_config.yaml'


class FakeModels:
    def __init__(self, response_text=None, error=None):
        self.response_text = response_text
        self.error = error
        self.calls = []

    def generate_content(self, **arguments):
        self.calls.append(arguments)
        if self.error is not None:
            raise self.error
        return SimpleNamespace(text=self.response_text)


class FakeClient:
    def __init__(self, response_text=None, error=None):
        self.models = FakeModels(response_text, error)


def test_gemini_response_is_canonicalized_and_validated():
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
    planner = GeminiPlanner(client=client, prompt='safe prompt')

    raw_json = planner.generate_plan('Đưa khối đỏ sang vùng B.')
    steps = parse_and_validate_plan(raw_json)

    assert [step.as_dict() for step in steps] == [
        {'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'},
        {'skill': 'home'},
    ]
    request = client.models.calls[0]
    assert request['model'] == 'gemini-3.5-flash-lite'
    assert request['contents'] == 'Đưa khối đỏ sang vùng B.'
    config = request['config']
    assert config['system_instruction'] == 'safe prompt'
    assert config['response_mime_type'] == 'application/json'
    assert config['temperature'] == 0.0
    assert config['response_json_schema']['additionalProperties'] is False


def test_invalid_gemini_plan_is_rejected_before_robot_execution():
    response = json.dumps(
        {
            'plan': [
                {'skill': 'pick', 'object': 'red_cube', 'zone': None},
                {'skill': 'home', 'object': None, 'zone': None},
            ]
        }
    )
    planner = GeminiPlanner(client=FakeClient(response), prompt='safe prompt')

    with pytest.raises(ValueError, match='home is unsafe'):
        planner.generate_plan('unsafe response')


def test_gemini_api_failure_is_closed_by_default():
    planner = GeminiPlanner(
        client=FakeClient(error=TimeoutError('network timeout')),
        prompt='safe prompt',
    )

    with pytest.raises(PlannerError, match='Gemini API request failed'):
        planner.generate_plan('Move the red cube to zone A.')


def test_explicit_mock_fallback_handles_gemini_timeout():
    fallback = MockLLMPlanner(load_student_config(str(STUDENT_FILE)))
    planner = GeminiPlanner(
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


def test_empty_command_never_calls_gemini():
    client = FakeClient('{}')
    planner = GeminiPlanner(client=client, prompt='safe prompt')

    with pytest.raises(PlannerError, match='non-empty'):
        planner.generate_plan('   ')
    assert client.models.calls == []


def test_mock_fallback_also_handles_missing_gemini_key(monkeypatch):
    monkeypatch.delenv('GEMINI_API_KEY', raising=False)
    fallback = MockLLMPlanner(load_student_config(str(STUDENT_FILE)))
    planner = GeminiPlanner(prompt='safe prompt', fallback=fallback)

    steps = parse_and_validate_plan(
        planner.generate_plan('Move the red cube to zone A.')
    )
    assert [step.as_dict() for step in steps] == [
        {'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_a'},
        {'skill': 'home'},
    ]
