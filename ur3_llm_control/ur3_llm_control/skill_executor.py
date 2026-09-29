"""Execute a validated robot-skill plan sequentially and stop on failure."""

from dataclasses import dataclass
from typing import Callable, Optional, Sequence, Tuple

from ur3_llm_control.task_validator import PlanStep, parse_and_validate_plan


@dataclass(frozen=True)
class SkillResult:
    """Result returned by one robot skill invocation."""

    code: int
    status: str
    message: str = ''

    @property
    def succeeded(self) -> bool:
        """Return true only for the action contract's SUCCESS result."""
        return self.code == 0 and self.status == 'SUCCESS'


@dataclass(frozen=True)
class StepExecution:
    """A plan step paired with the result returned by the skill server."""

    step: PlanStep
    result: SkillResult


@dataclass(frozen=True)
class ExecutionReport:
    """Final, immutable report for one attempted task plan."""

    succeeded: bool
    completed: Tuple[StepExecution, ...]
    total_steps: int

    @property
    def failed_step(self) -> Optional[StepExecution]:
        """Return the first failed step, if the task stopped on one."""
        if self.succeeded or not self.completed:
            return None
        return self.completed[-1]


SkillRunner = Callable[[PlanStep], SkillResult]
StepCallback = Callable[[StepExecution], None]


def execute_validated_plan(
    plan: Sequence[PlanStep],
    run_skill: SkillRunner,
    on_step_complete: Optional[StepCallback] = None,
) -> ExecutionReport:
    """Run canonical PlanStep values in order and stop at the first failure."""
    if not plan:
        raise ValueError('validated plan must contain at least one step')
    if any(not isinstance(step, PlanStep) for step in plan):
        raise TypeError('executor accepts only PlanStep values from the validator')

    completed = []
    for step in plan:
        try:
            result = run_skill(step)
            if not isinstance(result, SkillResult):
                raise TypeError('skill runner must return SkillResult')
        except Exception as error:  # Keep an executor fault from advancing the plan.
            result = SkillResult(
                code=-1,
                status='EXECUTOR_ERROR',
                message=str(error),
            )

        execution = StepExecution(step=step, result=result)
        completed.append(execution)
        if on_step_complete is not None:
            on_step_complete(execution)
        if not result.succeeded:
            return ExecutionReport(
                succeeded=False,
                completed=tuple(completed),
                total_steps=len(plan),
            )

    return ExecutionReport(
        succeeded=True,
        completed=tuple(completed),
        total_steps=len(plan),
    )


def validate_and_execute_plan(
    raw_json: str,
    run_skill: SkillRunner,
    on_step_complete: Optional[StepCallback] = None,
) -> ExecutionReport:
    """Validate the complete JSON document before invoking any robot skill."""
    plan = parse_and_validate_plan(raw_json)
    return execute_validated_plan(plan, run_skill, on_step_complete)
