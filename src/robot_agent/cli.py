"""Command-line demo: `robot-agent "go to the red ball"` (or `python -m robot_agent`)."""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence

from robot_agent.agent import Agent, AgentResult, Outcome
from robot_agent.gate import DEFAULT_MAX_STEPS, SafetyGate
from robot_agent.models import Position, ToolCall
from robot_agent.planner import GreedyPlanner, Planner, ScriptedPlanner
from robot_agent.sim import SimRobot
from robot_agent.tools import build_toolbox
from robot_agent.trace import PrintTracer
from robot_agent.world import GridWorld

DEFAULT_GOAL = "go to the red ball"


def build_world() -> GridWorld:
    """Fixed 5x5 scenario: a wall column splits the top rows, objects on both sides."""
    return GridWorld(
        robot=Position(0, 0),
        walls=frozenset({Position(0, 2), Position(1, 2), Position(3, 1)}),
        objects={"red_ball": Position(2, 2), "blue_box": Position(4, 4)},
    )


def demo_script() -> list[ToolCall]:
    """A deliberately flawed plan, to show the gate blocking a wall and a bogus tool."""
    return [
        ToolCall("look"),
        ToolCall("read_sensor"),
        ToolCall("move", {"direction": "east"}),
        ToolCall("move", {"direction": "east"}),  # wall at (0,2): gate blocks
        ToolCall("fly", {"height": 3}),  # hallucinated tool: gate blocks
        ToolCall("move", {"direction": "south"}),
        ToolCall("move", {"direction": "south"}),
        ToolCall("move", {"direction": "east"}),
        ToolCall("look"),
    ]


def make_planner(name: str) -> Planner:
    if name == "greedy":
        return GreedyPlanner()
    return ScriptedPlanner(demo_script())


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="robot-agent",
        description="Drive a simulated grid robot: the planner proposes, the gate executes.",
    )
    parser.add_argument("goal", nargs="?", default=DEFAULT_GOAL, help="natural-language order")
    parser.add_argument(
        "--planner",
        choices=("greedy", "scripted"),
        default="greedy",
        help="greedy: navigate from observations; scripted: fixed flawed plan (default: greedy)",
    )
    parser.add_argument("--max-steps", type=int, default=DEFAULT_MAX_STEPS)
    parser.add_argument(
        "--fail-sensor", action="store_true", help="make read_sensor() time out (fault injection)"
    )
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> AgentResult:
    world = build_world()
    robot = SimRobot(world, sensor_timeout=args.fail_sensor)
    toolbox = build_toolbox(robot)
    agent = Agent(
        planner=make_planner(args.planner),
        robot=robot,
        toolbox=toolbox,
        gate=SafetyGate(toolbox.specs, max_steps=args.max_steps),
        tracer=PrintTracer(),
    )
    print(world.render(), end="\n\n")
    result = agent.run(args.goal)
    print()
    print(world.render())
    return result


def main(argv: Sequence[str] | None = None) -> int:
    result = run(parse_args(argv))
    return 0 if result.outcome is Outcome.FINISHED and "reached" in result.reason else 1


if __name__ == "__main__":
    sys.exit(main())
