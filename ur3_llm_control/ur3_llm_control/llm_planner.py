"""Mock natural-language planner for Milestone 6."""

import argparse
import json
from pathlib import Path
import re
import sys
import unicodedata

from ur3_llm_control.student_config import (
    StudentConfig,
    load_student_config,
    student_arrangement_plan,
)
from ur3_llm_control.task_validator import parse_and_validate_plan


OBJECT_ALIASES = {
    'red_cube': ('red', 'do'),
    'yellow_cube': ('yellow', 'vang'),
    'blue_cube': ('blue', 'xanh'),
}
ZONE_PATTERN = re.compile(r'\b(?:zone|vung|khu|o)\s*[_-]?\s*([abc])\b')
STUDENT_MARKERS = (
    'student id',
    'student number',
    'mssv',
    'ma sinh vien',
)


class PlannerError(ValueError):
    """Natural-language input could not be converted into a safe plan."""


def normalise_command(command: str) -> str:
    """Lowercase Vietnamese/English text and remove accents for matching."""
    if not isinstance(command, str) or not command.strip():
        raise PlannerError('user command must be a non-empty string')
    decomposed = unicodedata.normalize(
        'NFKD', command.lower().replace('đ', 'd')
    )
    without_accents = ''.join(
        character
        for character in decomposed
        if not unicodedata.combining(character)
    )
    return re.sub(r'\s+', ' ', without_accents).strip()


def _contains_alias(command: str, alias: str) -> bool:
    return re.search(rf'\b{re.escape(alias)}\b', command) is not None


def _detect_object(command: str) -> str:
    matches = [
        object_name
        for object_name, aliases in OBJECT_ALIASES.items()
        if any(_contains_alias(command, alias) for alias in aliases)
    ]
    if len(matches) != 1:
        if not matches:
            raise PlannerError('command does not identify a valid cube colour')
        raise PlannerError('command identifies more than one cube')
    return matches[0]


def _detect_zone(command: str) -> str:
    matches = set(ZONE_PATTERN.findall(command))
    if len(matches) != 1:
        if not matches:
            raise PlannerError('command does not identify zone A, B, or C')
        raise PlannerError('command identifies more than one zone')
    return f'zone_{matches.pop()}'


def _single_move_document(command: str):
    object_name = _detect_object(command)
    zone_name = _detect_zone(command)
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


class MockLLMPlanner:
    """Deterministic stand-in for 9Router while preserving the LLM boundary."""

    def __init__(self, student_config: StudentConfig):
        self._student_config = student_config

    def generate_plan(self, user_command: str) -> str:
        """Return raw JSON only, after validating the generated document."""
        command = normalise_command(user_command)
        if any(marker in command for marker in STUDENT_MARKERS):
            document = student_arrangement_plan(self._student_config)
        else:
            document = _single_move_document(command)

        raw_json = json.dumps(document, separators=(',', ':'))
        parse_and_validate_plan(raw_json)
        return raw_json


def _default_student_config() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return (
            Path(get_package_share_directory('ur3_llm_control'))
            / 'config'
            / 'student_config.yaml'
        )
    except Exception:
        return Path(__file__).parents[1] / 'config' / 'student_config.yaml'


def main(arguments=None) -> int:
    """Print one validated mock LLM response as raw JSON."""
    parser = argparse.ArgumentParser(
        description='Generate a validated mock JSON plan from a user command.'
    )
    parser.add_argument('command', nargs='+', help='natural-language command')
    parser.add_argument(
        '--student-config',
        type=Path,
        default=_default_student_config(),
    )
    options = parser.parse_args(arguments)

    try:
        config = load_student_config(str(options.student_config))
        raw_json = MockLLMPlanner(config).generate_plan(' '.join(options.command))
    except (OSError, ValueError) as error:
        print(f'PLANNER REJECTED: {error}', file=sys.stderr)
        return 2

    print(raw_json)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
