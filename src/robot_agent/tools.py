"""Typed tools exposed to the planner.

Each tool has a `ToolSpec` (name, description, typed params) used by the gate for
validation and by an LLM adapter for its tool schema. The `Toolbox` binds specs to
a concrete `Robot`.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import Enum
from typing import Any

from robot_agent.hal import Robot
from robot_agent.models import Direction, MoveResult, Observation, ToolCall


class ToolValidationError(ValueError):
    """The proposed call does not match the tool's signature."""


@dataclass(frozen=True)
class ToolSpec:
    name: str
    description: str
    params: Mapping[str, type]

    def bind(self, args: Mapping[str, object]) -> dict[str, Any]:
        """Validate and coerce raw args (e.g. LLM strings) into typed values."""
        unknown = set(args) - set(self.params)
        if unknown:
            raise ToolValidationError(f"unexpected args: {sorted(unknown)}")
        missing = set(self.params) - set(args)
        if missing:
            raise ToolValidationError(f"missing args: {sorted(missing)}")
        return {name: _coerce(name, args[name], typ) for name, typ in self.params.items()}

    def json_schema(self) -> dict[str, Any]:
        """Schema in the shape most LLM tool-use APIs expect."""
        properties: dict[str, Any] = {}
        for name, typ in self.params.items():
            if issubclass(typ, Enum):
                properties[name] = {"type": "string", "enum": [m.value for m in typ]}
            else:
                properties[name] = {"type": _JSON_TYPES.get(typ, "string")}
        return {
            "name": self.name,
            "description": self.description,
            "input_schema": {
                "type": "object",
                "properties": properties,
                "required": list(self.params),
            },
        }


_JSON_TYPES: dict[type, str] = {str: "string", int: "integer", float: "number", bool: "boolean"}


def _coerce(name: str, value: object, typ: type) -> Any:
    if isinstance(value, typ):
        return value
    if issubclass(typ, Enum):
        try:
            return typ(value)
        except ValueError:
            allowed = [m.value for m in typ]
            raise ToolValidationError(f"{name}={value!r} not in {allowed}") from None
    raise ToolValidationError(f"{name} must be {typ.__name__}, got {type(value).__name__}")


LOOK = ToolSpec("look", "Return the robot position and visible objects with positions.", {})
MOVE = ToolSpec("move", "Move the robot one cell in the given direction.", {"direction": Direction})
READ_SENSOR = ToolSpec(
    "read_sensor", "Return free distance (cells) to the nearest obstacle per direction.", {}
)


class Toolbox:
    def __init__(self, robot: Robot) -> None:
        self._handlers: dict[str, tuple[ToolSpec, Callable[..., Observation]]] = {
            LOOK.name: (LOOK, robot.look),
            MOVE.name: (MOVE, lambda direction: MoveResult(robot.move(direction))),
            READ_SENSOR.name: (READ_SENSOR, robot.read_sensor),
        }

    @property
    def specs(self) -> Mapping[str, ToolSpec]:
        return {name: spec for name, (spec, _) in self._handlers.items()}

    def execute(self, call: ToolCall) -> Observation:
        """Run an already-approved call. May raise `HardwareError`."""
        spec, handler = self._handlers[call.name]
        return handler(**spec.bind(call.args))


def build_toolbox(robot: Robot) -> Toolbox:
    return Toolbox(robot)
