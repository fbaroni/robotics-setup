"""Core value types shared by every layer. Immutable and hardware-agnostic."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType


class Direction(str, Enum):
    NORTH = "north"
    SOUTH = "south"
    EAST = "east"
    WEST = "west"

    @property
    def delta(self) -> tuple[int, int]:
        """(row, col) offset. Row 0 is the top of the grid."""
        return _DELTAS[self]


_DELTAS: dict[Direction, tuple[int, int]] = {
    Direction.NORTH: (-1, 0),
    Direction.SOUTH: (1, 0),
    Direction.EAST: (0, 1),
    Direction.WEST: (0, -1),
}


@dataclass(frozen=True)
class Position:
    row: int
    col: int

    def moved(self, direction: Direction) -> Position:
        d_row, d_col = direction.delta
        return Position(self.row + d_row, self.col + d_col)

    def __str__(self) -> str:
        return f"({self.row},{self.col})"


# --- Actions proposed by the planner -------------------------------------------------


@dataclass(frozen=True)
class ToolCall:
    """A tool invocation proposed by the planner. Args are untrusted until the gate approves."""

    name: str
    args: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "args", MappingProxyType(dict(self.args)))

    def __str__(self) -> str:
        rendered = ", ".join(f"{k}={_render(v)}" for k, v in self.args.items())
        return f"{self.name}({rendered})"


@dataclass(frozen=True)
class Finish:
    """The planner declares the task done (or gives up)."""

    reason: str


Action = ToolCall | Finish


def _render(value: object) -> str:
    if isinstance(value, Enum):
        return str(value.value)
    return value if isinstance(value, str) else repr(value)


# --- Observations returned by tools --------------------------------------------------


@dataclass(frozen=True)
class SeenObject:
    name: str
    position: Position


@dataclass(frozen=True)
class LookResult:
    robot: Position
    objects: tuple[SeenObject, ...]

    def __str__(self) -> str:
        seen = ", ".join(f"{o.name}@{o.position}" for o in self.objects) or "nothing"
        return f"robot@{self.robot}; sees {seen}"


@dataclass(frozen=True)
class SensorReading:
    """Free cells before the nearest obstacle (wall or boundary), per direction."""

    clearance: Mapping[Direction, int]

    def __str__(self) -> str:
        return ", ".join(f"{d.value}={n}" for d, n in self.clearance.items())


@dataclass(frozen=True)
class MoveResult:
    position: Position

    def __str__(self) -> str:
        return f"robot now @{self.position}"


Observation = LookResult | SensorReading | MoveResult


# --- Gate + trace --------------------------------------------------------------------


@dataclass(frozen=True)
class GateDecision:
    approved: bool
    reason: str
    terminal: bool = False  # True => the agent loop must stop

    @classmethod
    def approve(cls, reason: str = "all rules passed") -> GateDecision:
        return cls(approved=True, reason=reason)

    @classmethod
    def block(cls, reason: str, *, terminal: bool = False) -> GateDecision:
        return cls(approved=False, reason=reason, terminal=terminal)


@dataclass(frozen=True)
class StepRecord:
    step: int
    action: ToolCall
    decision: GateDecision
    observation: Observation | None = None
    error: str | None = None
