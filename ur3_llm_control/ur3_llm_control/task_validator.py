"""Validate LLM-style JSON plans before any robot skill can run."""

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple


VALID_SKILLS = frozenset({'pick', 'place', 'home'})
VALID_OBJECTS = frozenset({'red_cube', 'yellow_cube', 'blue_cube'})
VALID_ZONES = frozenset({'zone_a', 'zone_b', 'zone_c'})
FORBIDDEN_CONTROL_TOKENS = (
    'joint',
    'trajectory',
    'velocity',
    'torque',
    'effort',
    'controller',
    'command',
)


class PlanValidationError(ValueError):
    """A stable validation code plus a human-readable explanation."""

    def __init__(
        self,
        code: str,
        message: str,
        step_index: Optional[int] = None,
    ) -> None:
        self.code = code
        self.message = message
        self.step_index = step_index
        location = '' if step_index is None else f'step {step_index}: '
        super().__init__(f'{code}: {location}{message}')


@dataclass(frozen=True)
class PlanStep:
    """Canonical skill call accepted by the validator."""

    skill: str
    object_name: Optional[str] = None
    zone_name: Optional[str] = None

    def as_dict(self) -> Dict[str, str]:
        """Return the JSON-compatible representation used by the executor."""
        result = {'skill': self.skill}
        if self.object_name is not None:
            result['object'] = self.object_name
        if self.zone_name is not None:
            result['zone'] = self.zone_name
        return result

    def label(self) -> str:
        """Format a concise terminal representation."""
        arguments = [
            value
            for value in (self.object_name, self.zone_name)
            if value is not None
        ]
        return f"{self.skill}({', '.join(arguments)})"


def _normalise_key(key: str) -> str:
    return key.lower().replace('-', '_').replace(' ', '_')


def _reject_forbidden_controls(value: Any, path: str = '$') -> None:
    if isinstance(value, dict):
        for key, nested in value.items():
            normalised = _normalise_key(str(key))
            if any(token in normalised for token in FORBIDDEN_CONTROL_TOKENS):
                raise PlanValidationError(
                    'FORBIDDEN_CONTROL',
                    f"field '{path}.{key}' may not control robot motion directly",
                )
            _reject_forbidden_controls(nested, f'{path}.{key}')
    elif isinstance(value, list):
        for index, nested in enumerate(value):
            _reject_forbidden_controls(nested, f'{path}[{index}]')


def _require_string(
    step: Dict[str, Any],
    field: str,
    step_index: int,
) -> str:
    value = step[field]
    if not isinstance(value, str) or not value:
        raise PlanValidationError(
            'INVALID_PARAMETER',
            f"'{field}' must be a non-empty string",
            step_index,
        )
    return value


def _validate_step(step: Any, step_index: int) -> PlanStep:
    if not isinstance(step, dict):
        raise PlanValidationError(
            'INVALID_STEP',
            'each plan step must be an object',
            step_index,
        )
    if 'skill' not in step:
        raise PlanValidationError(
            'MISSING_FIELD',
            "required field 'skill' is missing",
            step_index,
        )

    skill = _require_string(step, 'skill', step_index)
    if skill not in VALID_SKILLS:
        raise PlanValidationError(
            'INVALID_SKILL',
            f"unsupported skill '{skill}'",
            step_index,
        )

    expected_fields = {
        'home': {'skill'},
        'pick': {'skill', 'object'},
        'place': {'skill', 'object', 'zone'},
    }[skill]
    missing = expected_fields - set(step)
    if missing:
        field = sorted(missing)[0]
        raise PlanValidationError(
            'MISSING_FIELD',
            f"required field '{field}' is missing for {skill}",
            step_index,
        )
    unexpected = set(step) - expected_fields
    if unexpected:
        fields = ', '.join(sorted(unexpected))
        raise PlanValidationError(
            'UNEXPECTED_FIELD',
            f'unexpected field(s) for {skill}: {fields}',
            step_index,
        )

    if skill == 'home':
        return PlanStep(skill='home')

    object_name = _require_string(step, 'object', step_index)
    if object_name not in VALID_OBJECTS:
        raise PlanValidationError(
            'INVALID_OBJECT',
            f"unknown object '{object_name}'",
            step_index,
        )
    if skill == 'pick':
        return PlanStep(skill='pick', object_name=object_name)

    zone_name = _require_string(step, 'zone', step_index)
    if zone_name not in VALID_ZONES:
        raise PlanValidationError(
            'INVALID_ZONE',
            f"unknown zone '{zone_name}'",
            step_index,
        )
    return PlanStep(
        skill='place',
        object_name=object_name,
        zone_name=zone_name,
    )


def _validate_sequence(steps: Sequence[PlanStep]) -> None:
    held_object = None
    home_required = False
    for index, step in enumerate(steps):
        if home_required and step.skill != 'home':
            raise PlanValidationError(
                'INVALID_SEQUENCE',
                'home is required after place before the next task',
                index,
            )

        if step.skill == 'pick':
            if held_object is not None:
                raise PlanValidationError(
                    'INVALID_SEQUENCE',
                    f"cannot pick '{step.object_name}' while holding "
                    f"'{held_object}'",
                    index,
                )
            held_object = step.object_name
        elif step.skill == 'place':
            if held_object is None:
                raise PlanValidationError(
                    'INVALID_SEQUENCE',
                    'place requires a previously picked object',
                    index,
                )
            if step.object_name != held_object:
                raise PlanValidationError(
                    'INVALID_SEQUENCE',
                    f"cannot place '{step.object_name}' while holding "
                    f"'{held_object}'",
                    index,
                )
            held_object = None
            home_required = True
        elif held_object is not None:
            raise PlanValidationError(
                'INVALID_SEQUENCE',
                f"home is unsafe while holding '{held_object}'",
                index,
            )
        else:
            home_required = False

    if held_object is not None:
        raise PlanValidationError(
            'INVALID_SEQUENCE',
            f"plan ends while holding '{held_object}'",
            len(steps) - 1,
        )
    if home_required:
        raise PlanValidationError(
            'INVALID_SEQUENCE',
            'plan must return home after the final place',
            len(steps) - 1,
        )


def validate_plan_document(document: Any) -> Tuple[PlanStep, ...]:
    """Validate a decoded JSON document and return canonical immutable steps."""
    _reject_forbidden_controls(document)
    if not isinstance(document, dict):
        raise PlanValidationError(
            'INVALID_ROOT',
            'the JSON root must be an object',
        )
    if 'plan' not in document:
        raise PlanValidationError(
            'MISSING_PLAN',
            "top-level field 'plan' is required",
        )
    unexpected = set(document) - {'plan'}
    if unexpected:
        fields = ', '.join(sorted(unexpected))
        raise PlanValidationError(
            'UNEXPECTED_FIELD',
            f'unexpected top-level field(s): {fields}',
        )

    raw_plan = document['plan']
    if not isinstance(raw_plan, list):
        raise PlanValidationError(
            'INVALID_PLAN',
            "'plan' must be a list",
        )
    if not raw_plan:
        raise PlanValidationError(
            'EMPTY_PLAN',
            "'plan' must contain at least one step",
        )

    steps = tuple(
        _validate_step(step, index)
        for index, step in enumerate(raw_plan)
    )
    _validate_sequence(steps)
    return steps


def parse_and_validate_plan(raw_json: str) -> Tuple[PlanStep, ...]:
    """Parse raw JSON, reject malformed input, then validate the whole plan."""
    if not isinstance(raw_json, str):
        raise PlanValidationError(
            'INVALID_JSON',
            'plan input must be a JSON string',
        )
    try:
        document = json.loads(raw_json)
    except json.JSONDecodeError as error:
        raise PlanValidationError(
            'INVALID_JSON',
            f'{error.msg} at line {error.lineno}, column {error.colno}',
        ) from error
    return validate_plan_document(document)


def main() -> int:
    """Validate JSON from the command line without contacting the robot."""
    parser = argparse.ArgumentParser(
        description='Validate a UR3 skill plan without executing it.'
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--json', help='raw JSON plan')
    source.add_argument('--file', type=Path, help='path to a JSON plan')
    arguments = parser.parse_args()

    raw_json = (
        arguments.file.read_text(encoding='utf-8')
        if arguments.file is not None
        else arguments.json
    )
    try:
        plan = parse_and_validate_plan(raw_json)
    except (OSError, PlanValidationError) as error:
        print('VALIDATION: REJECTED')
        print(error)
        return 2

    print('VALIDATION: SUCCESS')
    for index, step in enumerate(plan, start=1):
        print(f'{index}. {step.label()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
