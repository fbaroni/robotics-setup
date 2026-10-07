"""The three required failure scenarios, end-to-end through the agent loop."""

from __future__ import annotations

from collections.abc import Sequence

from conftest import AgentFactory

from robot_agent import Outcome, Position, ScriptedPlanner, SimRobot, ToolCall
from robot_agent.models import Action, StepRecord


def test_unknown_tool_is_blocked_and_never_executed(
    make_agent: AgentFactory, robot: SimRobot
) -> None:
    agent = make_agent(ScriptedPlanner([ToolCall("self_destruct"), ToolCall("look")]))

    result = agent.run("do something")

    blocked, after = result.steps
    assert not blocked.decision.approved
    assert "unknown tool 'self_destruct'" in blocked.decision.reason
    assert blocked.observation is None
    assert after.decision.approved  # loop keeps going after a non-terminal block
    assert result.outcome is Outcome.FINISHED
    assert robot.world.robot == Position(0, 0)


def test_sensor_timeout_is_handled_without_crashing(
    make_agent: AgentFactory, robot: SimRobot
) -> None:
    robot.sensor_timeout = True
    agent = make_agent(
        ScriptedPlanner([ToolCall("read_sensor"), ToolCall("move", {"direction": "south"})])
    )

    result = agent.run("check surroundings, then move")

    sensor_step, move_step = result.steps
    # Tool call itself: approved by the gate, fails in hardware, error recorded.
    assert sensor_step.decision.approved
    assert sensor_step.error is not None and "SensorTimeoutError" in sensor_step.error
    # Motion without a valid clearance reading: fail-closed.
    assert not move_step.decision.approved
    assert "sensor unavailable" in move_step.decision.reason
    assert robot.world.robot == Position(0, 0)
    assert result.outcome is Outcome.FINISHED


class LookForeverPlanner:
    """Stuck LLM: keeps looking and never advances."""

    def propose(self, goal: str, history: Sequence[StepRecord]) -> Action:
        return ToolCall("look")


def test_infinite_look_loop_is_cut_by_step_limit(make_agent: AgentFactory) -> None:
    agent = make_agent(LookForeverPlanner())

    result = agent.run("go to the red ball")

    assert result.outcome is Outcome.HALTED
    assert "step limit" in result.reason
    assert len(result.steps) == 21  # 20 executed + 1 terminal block
    assert all(s.decision.approved for s in result.steps[:20])
    last = result.steps[-1]
    assert not last.decision.approved and last.decision.terminal
