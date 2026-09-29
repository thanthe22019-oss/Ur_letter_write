"""OpenAI Responses API planner for safe UR3 skill plans."""

import argparse
import os
from pathlib import Path
import sys
from typing import Any, Optional

from ur3_llm_control.llm_planner import (
    MockLLMPlanner,
    PlannerError,
    _default_student_config,
)
from ur3_llm_control.planner_contract import (
    PLAN_RESPONSE_SCHEMA,
    canonical_json,
    default_prompt,
)
from ur3_llm_control.student_config import load_student_config


DEFAULT_MODEL = 'gpt-4o-mini'
DEFAULT_TIMEOUT_SECONDS = 30.0


def _default_prompt() -> Path:
    """Compatibility wrapper for the shared prompt location."""
    return default_prompt()


def _canonical_json(raw_output: str) -> str:
    """Compatibility wrapper for OpenAI-specific error messages."""
    return canonical_json(raw_output, 'OpenAI')


class OpenAIPlanner:
    """Generate a plan through OpenAI and validate it before returning."""

    def __init__(
        self,
        model: str = DEFAULT_MODEL,
        prompt: Optional[str] = None,
        prompt_path: Optional[Path] = None,
        timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS,
        fallback: Optional[MockLLMPlanner] = None,
        client: Optional[Any] = None,
    ) -> None:
        if not isinstance(model, str) or not model.strip():
            raise PlannerError('OpenAI model must be a non-empty string')
        if timeout_seconds <= 0.0:
            raise PlannerError('OpenAI timeout must be positive')
        if prompt is not None and prompt_path is not None:
            raise PlannerError('provide prompt or prompt_path, not both')

        self._model = model.strip()
        self._prompt = (
            prompt
            if prompt is not None
            else (prompt_path or _default_prompt()).read_text(encoding='utf-8')
        )
        self._fallback = fallback
        if client is not None:
            self._client = client
        else:
            try:
                self._client = self._create_client(timeout_seconds)
            except PlannerError:
                if fallback is None:
                    raise
                self._client = None

    @staticmethod
    def _create_client(timeout_seconds: float):
        if not os.environ.get('OPENAI_API_KEY'):
            raise PlannerError(
                'OPENAI_API_KEY is not set; export it before using OpenAI'
            )
        try:
            from openai import OpenAI
        except ImportError as error:
            raise PlannerError(
                'OpenAI Python SDK is missing; run: '
                'python3 -m pip install --user openai'
            ) from error
        return OpenAI(timeout=timeout_seconds, max_retries=2)

    def generate_plan(self, user_command: str) -> str:
        """Call OpenAI, canonicalize its JSON, then run Plan Validator."""
        if not isinstance(user_command, str) or not user_command.strip():
            raise PlannerError('user command must be a non-empty string')

        if self._client is None:
            return self._fallback.generate_plan(user_command)

        try:
            response = self._client.responses.create(
                model=self._model,
                instructions=self._prompt,
                input=user_command.strip(),
                text={
                    'format': {
                        'type': 'json_schema',
                        'name': 'ur3_skill_plan',
                        'strict': True,
                        'schema': PLAN_RESPONSE_SCHEMA,
                    },
                },
            )
        except Exception as error:
            if self._fallback is not None:
                return self._fallback.generate_plan(user_command)
            raise PlannerError(
                f'OpenAI API request failed: {type(error).__name__}: {error}'
            ) from error

        return _canonical_json(getattr(response, 'output_text', ''))


def main(arguments=None) -> int:
    """Generate and print one validated plan through the OpenAI API."""
    parser = argparse.ArgumentParser(
        description='Generate a validated UR3 JSON plan with OpenAI.'
    )
    parser.add_argument('command', nargs='+', help='natural-language command')
    parser.add_argument(
        '--model',
        default=os.environ.get('OPENAI_MODEL', DEFAULT_MODEL),
        help='OpenAI model (default: OPENAI_MODEL or gpt-4o-mini)',
    )
    parser.add_argument(
        '--timeout-seconds',
        type=float,
        default=DEFAULT_TIMEOUT_SECONDS,
    )
    parser.add_argument(
        '--prompt-file',
        type=Path,
        default=_default_prompt(),
    )
    parser.add_argument(
        '--student-config',
        type=Path,
        default=_default_student_config(),
    )
    parser.add_argument(
        '--fallback-to-mock',
        action='store_true',
        help='use the deterministic mock only when the API request fails',
    )
    options = parser.parse_args(arguments)

    try:
        fallback = None
        if options.fallback_to_mock:
            fallback = MockLLMPlanner(
                load_student_config(str(options.student_config))
            )
        planner = OpenAIPlanner(
            model=options.model,
            prompt_path=options.prompt_file,
            timeout_seconds=options.timeout_seconds,
            fallback=fallback,
        )
        raw_json = planner.generate_plan(' '.join(options.command))
    except (OSError, ValueError) as error:
        print(f'PLANNER REJECTED: {error}', file=sys.stderr)
        return 2

    print(raw_json)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
