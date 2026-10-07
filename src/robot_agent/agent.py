"""Agent loop: planner proposes -> gate validates -> toolbox executes -> trace."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum

from robot_agent.gate import GateContext, SafetyGate
from robot_agent.hal import HardwareError, Robot
from robot_agent.models import Direction, Finish, GateDecision, StepRecord, ToolCall
from robot_agent.planner import Planner
from robot_agent.tools import Toolbox
from robot_agent.trace import NullTracer, Tracer


class Outcome(str, Enum):
    FINISHED = "finished"
    HALTED = "halted"  # gate issued a terminal block (e.g. step limit)
    PLANNER_ERROR = "planner_error"


@dataclass(frozen=True)
class AgentResult:
    goal: str
    outcome: Outcome
    reason: str
    steps: list[StepRecord] = field(default_factory=list)


class Agent:
    def __init__(
        self,
        planner: Planner,
        robot: Robot,
        toolbox: Toolbox,
        gate: SafetyGate,
        tracer: Tracer | None = None,
    ) -> None:
        self._planner = planner
        self._robot = robot
        self._toolbox = toolbox
        self._gate = gate
        self._tracer = tracer or NullTracer()

    def run(self, goal: str) -> AgentResult:
        self._tracer.on_start(goal)
        history: list[StepRecord] = []
        while True:
            try:
                action = self._planner.propose(goal, tuple(history))
            except Exception as exc:
                return self._end(goal, Outcome.PLANNER_ERROR, f"planner failed: {exc!r}", history)

            if isinstance(action, Finish):
                return self._end(goal, Outcome.FINISHED, action.reason, history)

            record = self._step(len(history) + 1, action, len(history))
            history.append(record)
            self._tracer.on_step(record)

            if record.decision.terminal:
                return self._end(goal, Outcome.HALTED, record.decision.reason, history)

    def _step(self, step: int, call: ToolCall, steps_taken: int) -> StepRecord:
        ctx = GateContext(steps_taken=steps_taken, clearance=self._clearance_for(call))
        decision = self._gate.check(call, ctx)
        if not decision.approved:
            return StepRecord(step, call, decision)
        return self._execute(step, call, decision)

    def _execute(self, step: int, call: ToolCall, decision: GateDecision) -> StepRecord:
        try:
            observation = self._toolbox.execute(call)
        except HardwareError as exc:
            return StepRecord(step, call, decision, error=f"{type(exc).__name__}: {exc}")
        return StepRecord(step, call, decision, observation=observation)

    def _clearance_for(self, call: ToolCall) -> Mapping[Direction, int] | None:
        """Fresh sensor reading for motion commands only; None if the sensor fails."""
        if call.name != "move":
            return {}
        try:
            return self._robot.read_sensor().clearance
        except HardwareError:
            return None

    def _end(
        self, goal: str, outcome: Outcome, reason: str, history: list[StepRecord]
    ) -> AgentResult:
        result = AgentResult(goal=goal, outcome=outcome, reason=reason, steps=history)
        self._tracer.on_end(result)
        return result
