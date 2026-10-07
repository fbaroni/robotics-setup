"""Unit tests for the deterministic gate rules (no agent loop, no I/O)."""

from __future__ import annotations

import io

import pytest
from conftest import AgentFactory

from robot_agent import (
    Agent,
    Direction,
    GateContext,
    Position,
    PrintTracer,
    SafetyGate,
    ScriptedPlanner,
    ToolCall,
    build_toolbox,
)
from robot_agent.tools import MOVE

ALL_CLEAR = {d: 3 for d in Direction}


def ctx(steps: int = 0, clearance: dict[Direction, int] | None = ALL_CLEAR) -> GateContext:
    return GateContext(steps_taken=steps, clearance=clearance)


def test_valid_move_is_approved(gate: SafetyGate) -> None:
    assert gate.check(ToolCall("move", {"direction": "north"}), ctx()).approved


@pytest.mark.parametrize("direction", list(Direction))
def test_move_into_obstacle_is_blocked(gate: SafetyGate, direction: Direction) -> None:
    clearance = {**ALL_CLEAR, direction: 0}
    decision = gate.check(ToolCall("move", {"direction": direction.value}), ctx(clearance=clearance))
    assert not decision.approved
    assert "wall or boundary" in decision.reason


@pytest.mark.parametrize(
    "args",
    [{}, {"direction": "up"}, {"direction": 3}, {"direction": "east", "speed": 9}],
)
def test_invalid_args_are_blocked(gate: SafetyGate, args: dict[str, object]) -> None:
    decision = gate.check(ToolCall("move", args), ctx())
    assert not decision.approved
    assert decision.reason.startswith("invalid args")


def test_step_limit_is_terminal(gate: SafetyGate) -> None:
    decision = gate.check(ToolCall("look"), ctx(steps=20))
    assert not decision.approved and decision.terminal


def test_gate_fails_closed_on_internal_error() -> None:
    class Boom(dict[Direction, int]):
        def get(self, *_: object) -> int:  # type: ignore[override]
            raise RuntimeError("corrupt reading")

    gate = SafetyGate({"move": MOVE})
    decision = gate.check(ToolCall("move", {"direction": "east"}), ctx(clearance=Boom()))
    assert not decision.approved and decision.terminal


def test_real_world_wall_and_border(make_agent: AgentFactory, world) -> None:
    agent = make_agent(
        ScriptedPlanner(
            [
                ToolCall("move", {"direction": "north"}),  # border
                ToolCall("move", {"direction": "east"}),
                ToolCall("move", {"direction": "east"}),  # wall at (0,2)
            ]
        )
    )
    result = agent.run("probe")
    assert [s.decision.approved for s in result.steps] == [False, True, False]
    assert world.robot == Position(0, 1)


def test_trace_shows_proposal_decision_and_reason(make_agent: AgentFactory, robot) -> None:
    out = io.StringIO()
    toolbox = build_toolbox(robot)
    agent = Agent(
        ScriptedPlanner([ToolCall("fly")]), robot, toolbox, SafetyGate(toolbox.specs), PrintTracer(out)
    )
    agent.run("fly away")
    text = out.getvalue()
    assert "propose: fly()" in text
    assert "BLOCK - unknown tool 'fly'" in text
