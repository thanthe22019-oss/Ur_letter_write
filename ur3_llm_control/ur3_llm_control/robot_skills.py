"""Safe request boundary between validated plan steps and robot skills."""

from dataclasses import dataclass

from ur3_llm_control.task_validator import PlanStep


@dataclass(frozen=True)
class RobotSkillRequest:
    """Fields allowed to cross from the executor to ExecuteSkill.action."""

    skill: str
    object_name: str = ''
    zone_name: str = ''


def request_from_plan_step(step: PlanStep) -> RobotSkillRequest:
    """Map one canonical validator output to the robot action contract."""
    if not isinstance(step, PlanStep):
        raise TypeError('robot skills accept only PlanStep values')
    return RobotSkillRequest(
        skill=step.skill,
        object_name=step.object_name or '',
        zone_name=step.zone_name or '',
    )
