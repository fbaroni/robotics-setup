"""Hardware Abstraction Layer.

The agent loop, gate and toolbox depend only on the `Robot` protocol below.
To move from simulation to real hardware, implement this protocol (e.g. a
`SerialRobot` talking to a microcontroller) and pass it to `build_toolbox`.
Nothing else changes.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from robot_agent.models import Direction, LookResult, Position, SensorReading


class HardwareError(Exception):
    """Base class for any failure coming from the robot (sim or real)."""


class SensorTimeoutError(HardwareError, TimeoutError):
    """A sensor did not answer in time."""


class CollisionError(HardwareError):
    """The robot was commanded into an obstacle. The gate should make this unreachable."""


@runtime_checkable
class Robot(Protocol):
    def look(self) -> LookResult: ...

    def move(self, direction: Direction) -> Position: ...

    def read_sensor(self) -> SensorReading: ...
