"""GridWorld geometry and the simulated robot's own collision guard."""

from __future__ import annotations

import pytest

from robot_agent import CollisionError, Direction, GridWorld, Position, SimRobot


def test_clearance_counts_free_cells_to_wall_or_border(world: GridWorld) -> None:
    # Robot at (0,0); wall at (0,2).
    assert world.clearance(Direction.NORTH) == 0
    assert world.clearance(Direction.WEST) == 0
    assert world.clearance(Direction.EAST) == 1
    assert world.clearance(Direction.SOUTH) == 4


def test_render_marks_robot_walls_and_objects(world: GridWorld) -> None:
    assert world.render().splitlines() == [
        "@ . # . .",
        ". . . . .",
        ". . R . .",
        ". . . . .",
        ". . . . .",
    ]


@pytest.mark.parametrize("bad", [Position(0, 2), Position(-1, 0), Position(5, 5)])
def test_world_rejects_robot_on_wall_or_outside(bad: Position) -> None:
    with pytest.raises(ValueError, match="not a free cell"):
        GridWorld(robot=bad, walls=frozenset({Position(0, 2)}))


def test_sim_robot_refuses_collisions_even_without_gate(robot: SimRobot) -> None:
    with pytest.raises(CollisionError):
        robot.move(Direction.NORTH)
    assert robot.world.robot == Position(0, 0)
