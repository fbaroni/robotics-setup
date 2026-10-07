from __future__ import annotations

from collections.abc import Callable

import pytest

from robot_agent import (
    Agent,
    GridWorld,
    NullTracer,
    Position,
    SafetyGate,
    SimRobot,
    build_toolbox,
)
from robot_agent.planner import Planner


@pytest.fixture
def world() -> GridWorld:
    return GridWorld(
        robot=Position(0, 0),
        walls=frozenset({Position(0, 2)}),
        objects={"red_ball": Position(2, 2)},
    )


@pytest.fixture
def robot(world: GridWorld) -> SimRobot:
    return SimRobot(world)


@pytest.fixture
def gate(robot: SimRobot) -> SafetyGate:
    return SafetyGate(build_toolbox(robot).specs, max_steps=20)


AgentFactory = Callable[[Planner], Agent]


@pytest.fixture
def make_agent(robot: SimRobot) -> AgentFactory:
    def factory(planner: Planner) -> Agent:
        toolbox = build_toolbox(robot)
        return Agent(planner, robot, toolbox, SafetyGate(toolbox.specs), NullTracer())

    return factory
