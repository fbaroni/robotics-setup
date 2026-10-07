"""Deterministic safety gate. The planner proposes; only the gate can approve.

`SafetyGate.check` is a pure function of (proposed call, context snapshot): no I/O,
no randomness, no hidden state. Any unexpected condition or internal error results
in BLOCK (fail-closed). Rules run in order and the first BLOCK wins.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass

from robot_agent.models import Direction, GateDecision, ToolCall
from robot_agent.tools import MOVE, ToolSpec, ToolValidationError

DEFAULT_MAX_STEPS = 20


@dataclass(frozen=True)
class GateContext:
    """World snapshot the gate decides on. Built by the agent loop before each check.

    `clearance` is a fresh sensor reading (free cells per direction). It is `None`
    when the reading is unavailable, so motion rules fail closed.
    """

    steps_taken: int
    clearance: Mapping[Direction, int] | None = None


Rule = Callable[[ToolCall, GateContext], GateDecision | None]
"""A rule returns a BLOCK decision, or `None` to let the next rule run."""


class SafetyGate:
    def __init__(
        self, specs: Mapping[str, ToolSpec], *, max_steps: int = DEFAULT_MAX_STEPS
    ) -> None:
        if max_steps <= 0:
            raise ValueError("max_steps must be positive")
        self._specs = dict(specs)
        self.max_steps = max_steps
        self._rules: tuple[Rule, ...] = (
            self._rule_step_limit,
            self._rule_tool_exists,
            self._rule_args_valid,
            self._rule_move_is_clear,
        )

    def check(self, call: ToolCall, ctx: GateContext) -> GateDecision:
        try:
            for rule in self._rules:
                decision = rule(call, ctx)
                if decision is not None:
                    return decision
            return GateDecision.approve()
        except Exception as exc:
            return GateDecision.block(f"gate internal error: {exc!r}", terminal=True)

    def _rule_step_limit(self, call: ToolCall, ctx: GateContext) -> GateDecision | None:
        if ctx.steps_taken >= self.max_steps:
            return GateDecision.block(
                f"step limit reached ({ctx.steps_taken}/{self.max_steps})", terminal=True
            )
        return None

    def _rule_tool_exists(self, call: ToolCall, ctx: GateContext) -> GateDecision | None:
        if call.name not in self._specs:
            return GateDecision.block(f"unknown tool {call.name!r}; allowed: {sorted(self._specs)}")
        return None

    def _rule_args_valid(self, call: ToolCall, ctx: GateContext) -> GateDecision | None:
        try:
            self._specs[call.name].bind(call.args)
        except ToolValidationError as exc:
            return GateDecision.block(f"invalid args: {exc}")
        return None

    def _rule_move_is_clear(self, call: ToolCall, ctx: GateContext) -> GateDecision | None:
        if call.name != MOVE.name:
            return None
        direction = Direction(call.args["direction"])
        if ctx.clearance is None:
            return GateDecision.block("clearance unknown (sensor unavailable); fail-closed")
        if ctx.clearance.get(direction, 0) <= 0:
            return GateDecision.block(f"wall or boundary {direction.value} (clearance 0)")
        return None
