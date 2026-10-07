"""GreedyPlanner: navigates from observations alone and learns from blocked moves."""

from __future__ import annotations

import pytest

from conftest import AgentFactory
from robot_agent import (
    Agent,
    AgentResult,
    GreedyPlanner,
    GridWorld,
    Outcome,
    Position,
    SafetyGate,
    SimRobot,
    build_toolbox,
    parse_target,
)


def run_greedy(world: GridWorld, goal: str) -> AgentResult:
    toolbox = build_toolbox(SimRobot(world))
    return Agent(GreedyPlanner(), SimRobot(world), toolbox, SafetyGate(toolbox.specs)).run(goal)


@pytest.mark.parametrize(
    ("goal", "expected"),
    [
        ("go to the red ball", "red_ball"),
        ("Andá hasta la BLUE box, por favor", "blue_box"),
        ("find the ball", None),  # "red" missing: ambiguous, no match
        ("", None),
    ],
)
def test_parse_target_matches_every_word_of_the_object_name(goal, expected):
    assert parse_target(goal, ["red_ball", "blue_box"]) == expected


def test_greedy_reaches_target_around_walls(make_agent: AgentFactory, world: GridWorld):
    result = make_agent(GreedyPlanner()).run("go to the red ball")

    assert result.outcome is Outcome.FINISHED
    assert result.reason.startswith("reached red_ball")
    assert world.robot == Position(2, 2)
    assert result.steps[0].action.name == "look"


def test_greedy_routes_around_a_wall_it_bumps_into():
    # Wall column at col 1 forces a detour: the first eastward move gets blocked.
    world = GridWorld(
        robot=Position(0, 0),
        walls=frozenset({Position(0, 1), Position(1, 1), Position(2, 1)}),
        objects={"red_ball": Position(0, 4)},
    )

    result = run_greedy(world, "red ball")

    assert result.outcome is Outcome.FINISHED
    assert world.robot == Position(0, 4)
    assert result.blocked >= 1  # at least one move was refused by the gate
    assert len(result.steps) < 20


def test_greedy_gives_up_when_target_is_not_visible(make_agent: AgentFactory):
    result = make_agent(GreedyPlanner()).run("go to the green cube")

    assert result.outcome is Outcome.FINISHED
    assert "no visible object" in result.reason
    assert len(result.steps) == 1  # one look, then stop


def test_greedy_gives_up_when_boxed_in():
    world = GridWorld(
        robot=Position(0, 0),
        walls=frozenset({Position(0, 1), Position(1, 0)}),
        objects={"red_ball": Position(4, 4)},
    )

    result = run_greedy(world, "red ball")

    assert result.outcome is Outcome.FINISHED
    assert result.reason.startswith("stuck")
    assert world.robot == Position(0, 0)
