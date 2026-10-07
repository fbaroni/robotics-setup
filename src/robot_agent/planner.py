"""Planners: the "LLM proposes" half.

A planner only proposes. It never touches the robot, and everything it returns is
validated by the gate before execution.

- `ScriptedPlanner`: fixed action list. Useful for tests and reproducible demos.
- `GreedyPlanner`: deterministic stand-in for an LLM. It reads the goal, looks for
  the named object, and walks toward it, learning from blocked moves. It is a pure
  function of (goal, history), which makes it trivial to test.

To plug in a real LLM, implement `Planner.propose`: send the goal, the tool schemas
(`ToolSpec.json_schema()`) and the step history to the model, then map its tool-use
response to a `ToolCall`, or `Finish` when it stops calling tools.
"""

from __future__ import annotations

import re
from collections.abc import Iterable, Sequence
from typing import Protocol

from robot_agent.models import (
    Action,
    Direction,
    Finish,
    LookResult,
    MoveResult,
    Position,
    StepRecord,
    ToolCall,
)
from robot_agent.tools import LOOK, MOVE


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


def parse_target(goal: str, known_objects: Iterable[str]) -> str | None:
    """Pick which known object the goal refers to, e.g. "go to the red ball" -> "red_ball".

    Matches on words: every word of the object name must appear in the goal.
    """
    words = set(re.findall(r"[a-z0-9]+", goal.lower()))
    for name in known_objects:
        if set(name.lower().split("_")) <= words:
            return name
    return None


class GreedyPlanner:
    """Walk toward the object named in the goal, one Manhattan step at a time.

    Stateless by design: position, target, visited cells and blocked directions are
    all recovered from `history` on every call, exactly as an LLM re-reads its context.
    """

    def propose(self, goal: str, history: Sequence[StepRecord]) -> Action:
        seen = _last_look(history)
        if seen is None:
            return ToolCall(LOOK.name)

        target_name = parse_target(goal, (o.name for o in seen.objects))
        if target_name is None:
            return Finish(f"no visible object matches goal {goal!r}")
        target = next(o.position for o in seen.objects if o.name == target_name)

        visited = _visited_cells(history)
        here = visited[-1]
        if here == target:
            return Finish(f"reached {target_name} at {target}")

        blocked = _blocked_directions(history)
        for direction in _rank_directions(here, target, set(visited)):
            if direction not in blocked:
                return ToolCall(MOVE.name, {"direction": direction})
        return Finish(f"stuck at {here}: every direction is blocked")


def _last_look(history: Sequence[StepRecord]) -> LookResult | None:
    for record in reversed(history):
        if isinstance(record.observation, LookResult):
            return record.observation
    return None


def _visited_cells(history: Sequence[StepRecord]) -> list[Position]:
    """Every cell the robot has reported being in, in order. The last one is "here"."""
    cells: list[Position] = []
    for record in history:
        if isinstance(record.observation, MoveResult):
            cells.append(record.observation.position)
        elif isinstance(record.observation, LookResult):
            cells.append(record.observation.robot)
    return cells


def _blocked_directions(history: Sequence[StepRecord]) -> set[Direction]:
    """Directions refused by the gate (or the hardware) from the current cell.

    Only steps since the last successful move count: a block elsewhere says nothing
    about this cell.
    """
    blocked: set[Direction] = set()
    for record in reversed(history):
        if isinstance(record.observation, MoveResult):
            break
        if record.action.name == MOVE.name and (not record.decision.approved or record.error):
            blocked.add(Direction(record.action.args["direction"]))
    return blocked


def _rank_directions(here: Position, target: Position, visited: set[Position]) -> list[Direction]:
    """Candidate directions, best first: unvisited cells before visited ones, then by
    how much the move shrinks the Manhattan distance (ties keep enum order)."""
    gain = {
        Direction.SOUTH: target.row - here.row,
        Direction.NORTH: here.row - target.row,
        Direction.EAST: target.col - here.col,
        Direction.WEST: here.col - target.col,
    }
    return sorted(Direction, key=lambda d: (here.moved(d) in visited, -gain[d]))
