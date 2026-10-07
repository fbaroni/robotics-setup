"""Deterministic safety gate. The planner proposes; only the gate can approve.

`SafetyGate.check` is a pure function of (proposed call, context snapshot): no I/O,
no randomness. Any unexpected condition or internal error results in BLOCK
(fail-closed).
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass

from robot_agent.models import Direction, GateDecision, ToolCall
from robot_agent.tools import ToolSpec, ToolValidationError

DEFAULT_MAX_STEPS = 20


@dataclass(frozen=True)
class GateContext:
    """World snapshot the gate decides on. Built by the agent loop before each check."""

    steps_taken: int
    clearance: Mapping[Direction, int] | None  # None => sensor unavailable


Rule = Callable[[ToolCall, GateContext], "GateDecision | None"]


class SafetyGate:
    def __init__(self, specs: Mapping[str, ToolSpec], *, max_steps: int = DEFAULT_MAX_STEPS):
        if max_steps <= 0:
            raise ValueError("max_steps must be positive")
        self._specs = dict(specs)
        self.max_steps = max_steps
        # Order matters: first BLOCK wins.
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
        except Exception as exc:  # fail-closed on any gate bug
            return GateDecision.block(f"gate internal error: {exc!r}", terminal=True)

    # --- rules: return a BLOCK decision, or None to pass --------------------------------

    def _rule_step_limit(self, call: ToolCall, ctx: GateContext) -> GateDecision | None:
        if ctx.steps_taken >= self.max_steps:
            return GateDecision.block(
                f"step limit reached ({ctx.steps_taken}/{self.max_steps})", terminal=True
            )
        return None

    def _rule_tool_exists(self, call: ToolCall, ctx: GateContext) -> GateDecision | None:
        if call.name not in self._specs:
            return GateDecision.block(
                f"unknown tool {call.name!r}; allowed: {sorted(self._specs)}"
            )
        return None

    def _rule_args_valid(self, call: ToolCall, ctx: GateContext) -> GateDecision | None:
        try:
            self._specs[call.name].bind(call.args)
        except ToolValidationError as exc:
            return GateDecision.block(f"invalid args: {exc}")
        return None

    def _rule_move_is_clear(self, call: ToolCall, ctx: GateContext) -> GateDecision | None:
        if call.name != "move":
            return None
        direction = Direction(call.args["direction"])
        if ctx.clearance is None:
            return GateDecision.block("clearance unknown (sensor unavailable); fail-closed")
        if ctx.clearance.get(direction, 0) <= 0:
            return GateDecision.block(f"wall or boundary {direction.value} (clearance 0)")
        return None
