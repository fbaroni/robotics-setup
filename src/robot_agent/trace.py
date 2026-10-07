"""Observable trace: what was proposed, what the gate decided, and why."""

from __future__ import annotations

import sys
from typing import TYPE_CHECKING, Protocol, TextIO

from robot_agent.models import StepRecord

if TYPE_CHECKING:
    from robot_agent.agent import AgentResult


class Tracer(Protocol):
    def on_start(self, goal: str) -> None: ...

    def on_step(self, record: StepRecord) -> None: ...

    def on_end(self, result: AgentResult) -> None: ...


class PrintTracer:
    def __init__(self, stream: TextIO | None = None) -> None:
        self._stream = stream or sys.stdout

    def _print(self, line: str) -> None:
        print(line, file=self._stream)

    def on_start(self, goal: str) -> None:
        self._print(f"GOAL: {goal}")

    def on_step(self, record: StepRecord) -> None:
        verdict = "APPROVE" if record.decision.approved else "BLOCK"
        self._print(f"[step {record.step:02d}] propose: {record.action}")
        self._print(f"          gate:    {verdict} - {record.decision.reason}")
        if record.error is not None:
            self._print(f"          error:   {record.error}")
        elif record.observation is not None:
            self._print(f"          result:  {record.observation}")

    def on_end(self, result: AgentResult) -> None:
        self._print(f"END: {result.outcome.value} - {result.reason} ({len(result.steps)} steps)")


class NullTracer:
    def on_start(self, goal: str) -> None:
        pass

    def on_step(self, record: StepRecord) -> None:
        pass

    def on_end(self, result: AgentResult) -> None:
        pass
