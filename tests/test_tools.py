"""ToolSpec validation/coercion and the JSON schema handed to an LLM."""

from __future__ import annotations

import pytest

from robot_agent import Direction, SimRobot, ToolCall, build_toolbox
from robot_agent.models import LookResult, MoveResult, SensorReading
from robot_agent.tools import LOOK, MOVE, ToolValidationError


def test_bind_coerces_strings_to_enum() -> None:
    assert MOVE.bind({"direction": "north"}) == {"direction": Direction.NORTH}
    assert MOVE.bind({"direction": Direction.EAST}) == {"direction": Direction.EAST}


@pytest.mark.parametrize(
    ("args", "message"),
    [
        ({}, "missing args"),
        ({"direction": "up"}, "not in"),
        ({"direction": 1}, "not in"),
        ({"direction": "east", "speed": 2}, "unexpected args"),
    ],
)
def test_bind_rejects_bad_args(args: dict[str, object], message: str) -> None:
    with pytest.raises(ToolValidationError, match=message):
        MOVE.bind(args)


def test_bind_rejects_extra_args_on_zero_arity_tool() -> None:
    with pytest.raises(ToolValidationError, match="unexpected"):
        LOOK.bind({"x": 1})


def test_json_schema_lists_enum_values() -> None:
    schema = MOVE.json_schema()
    assert schema["name"] == "move"
    assert schema["input_schema"]["required"] == ["direction"]
    assert schema["input_schema"]["properties"]["direction"]["enum"] == [
        "north",
        "south",
        "east",
        "west",
    ]


def test_toolbox_dispatches_to_robot(robot: SimRobot) -> None:
    toolbox = build_toolbox(robot)

    assert set(toolbox.specs) == {"look", "move", "read_sensor"}
    assert isinstance(toolbox.execute(ToolCall("look")), LookResult)
    assert isinstance(toolbox.execute(ToolCall("read_sensor")), SensorReading)
    moved = toolbox.execute(ToolCall("move", {"direction": "south"}))
    assert isinstance(moved, MoveResult)
    assert moved.position.row == 1
