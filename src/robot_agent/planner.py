"""Planner interface (the "LLM proposes" half).

A real LLM planner implements `Planner.propose`: send the goal, the tool schemas
(`ToolSpec.json_schema()`) and the step history to the model, then map its
tool-use response to a `ToolCall` (or `Finish` when it stops calling tools).
The planner never executes anything; it only proposes.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from typing import Protocol

from robot_agent.models import Action, Finish, StepRecord, ToolCall


class Planner(Protocol):
    def propose(self, goal: str, history: Sequence[StepRecord]) -> Action: ...


class ScriptedPlanner:
    """Mock LLM: returns a fixed sequence of actions, then `Finish`."""

    def __init__(self, actions: Iterable[ToolCall]) -> None:
        self._actions = list(actions)
        self._index = 0

    def propose(self, goal: str, history: Sequence[StepRecord]) -> Action:
        if self._index >= len(self._actions):
            return Finish("script exhausted")
        action = self._actions[self._index]
        self._index += 1
        return action
