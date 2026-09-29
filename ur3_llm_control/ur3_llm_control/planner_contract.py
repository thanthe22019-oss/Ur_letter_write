"""Shared structured-output contract for external LLM planners."""

import json
from pathlib import Path

from ur3_llm_control.llm_planner import PlannerError
from ur3_llm_control.task_validator import parse_and_validate_plan


# External providers receive the same closed schema. Schema-only null values
# are removed before the plan reaches the independent Plan Validator.
PLAN_RESPONSE_SCHEMA = {
    'type': 'object',
    'properties': {
        'plan': {
            'type': 'array',
            'minItems': 1,
            'maxItems': 30,
            'items': {
                'type': 'object',
                'properties': {
                    'skill': {
                        'type': 'string',
                        'enum': ['pick', 'place', 'home'],
                    },
                    'object': {
                        'type': ['string', 'null'],
                        'enum': [
                            'red_cube',
                            'yellow_cube',
                            'blue_cube',
                            None,
                        ],
                    },
                    'zone': {
                        'type': ['string', 'null'],
                        'enum': ['zone_a', 'zone_b', 'zone_c', None],
                    },
                },
                'required': ['skill', 'object', 'zone'],
                'additionalProperties': False,
            },
        },
    },
    'required': ['plan'],
    'additionalProperties': False,
}


def default_prompt() -> Path:
    """Return the installed prompt path, or the source-tree fallback."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return (
            Path(get_package_share_directory('ur3_llm_control'))
            / 'prompts'
            / 'planner_prompt.txt'
        )
    except Exception:
        return Path(__file__).parents[1] / 'prompts' / 'planner_prompt.txt'


def canonical_json(raw_output: str, provider: str) -> str:
    """Remove schema-only nulls and validate the canonical robot plan."""
    if not isinstance(raw_output, str) or not raw_output.strip():
        raise PlannerError(f'{provider} returned an empty response')
    try:
        document = json.loads(raw_output)
    except json.JSONDecodeError as error:
        raise PlannerError(
            f'{provider} returned invalid JSON: {error.msg}'
        ) from error

    if not isinstance(document, dict) or not isinstance(
        document.get('plan'), list
    ):
        raise PlannerError(f'{provider} response does not contain a plan list')

    canonical_steps = []
    for raw_step in document['plan']:
        if not isinstance(raw_step, dict):
            raise PlannerError(f'{provider} returned a non-object plan step')
        canonical_steps.append(
            {key: value for key, value in raw_step.items() if value is not None}
        )

    canonical = json.dumps(
        {'plan': canonical_steps},
        separators=(',', ':'),
    )
    parse_and_validate_plan(canonical)
    return canonical
