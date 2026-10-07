"""Demo: `python -m robot_agent` — drive the robot to the red ball on a 5x5 grid."""

from __future__ import annotations

from robot_agent.agent import Agent
from robot_agent.gate import SafetyGate
from robot_agent.models import Position, ToolCall
from robot_agent.planner import ScriptedPlanner
from robot_agent.sim import SimRobot
from robot_agent.tools import build_toolbox
from robot_agent.trace import PrintTracer
from robot_agent.world import GridWorld


def build_world() -> GridWorld:
    return GridWorld(
        robot=Position(0, 0),
        walls=frozenset({Position(0, 2), Position(1, 2), Position(3, 1)}),
        objects={"red_ball": Position(2, 2), "blue_box": Position(4, 4)},
    )


def demo_script() -> list[ToolCall]:
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


def main() -> None:
    world = build_world()
    robot = SimRobot(world)
    toolbox = build_toolbox(robot)
    agent = Agent(
        planner=ScriptedPlanner(demo_script()),
        robot=robot,
        toolbox=toolbox,
        gate=SafetyGate(toolbox.specs, max_steps=20),
        tracer=PrintTracer(),
    )
    print(world.render(), end="\n\n")
    agent.run("go to the red ball")
    print()
    print(world.render())
    reached = world.robot == world.objects["red_ball"]
    print(f"\nreached red_ball: {reached}")


if __name__ == "__main__":
    main()
