"""Robot agent skeleton: the LLM proposes, the gate executes."""

from robot_agent.agent import Agent, AgentResult, Outcome
from robot_agent.gate import GateContext, SafetyGate
from robot_agent.hal import CollisionError, HardwareError, Robot, SensorTimeoutError
from robot_agent.models import Direction, Finish, Position, ToolCall
from robot_agent.planner import Planner, ScriptedPlanner
from robot_agent.sim import SimRobot
from robot_agent.tools import Toolbox, ToolSpec, build_toolbox
from robot_agent.trace import NullTracer, PrintTracer, Tracer
from robot_agent.world import GridWorld

__all__ = [
    "Agent",
    "AgentResult",
    "CollisionError",
    "Direction",
    "Finish",
    "GateContext",
    "GridWorld",
    "HardwareError",
    "NullTracer",
    "Outcome",
    "Planner",
    "Position",
    "PrintTracer",
    "Robot",
    "SafetyGate",
    "ScriptedPlanner",
    "SensorTimeoutError",
    "SimRobot",
    "ToolCall",
    "ToolSpec",
    "Toolbox",
    "Tracer",
    "build_toolbox",
]
