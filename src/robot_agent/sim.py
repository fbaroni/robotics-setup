"""Simulated robot: implements `hal.Robot` on top of a `GridWorld`."""

from __future__ import annotations

from robot_agent.hal import CollisionError, SensorTimeoutError
from robot_agent.models import Direction, LookResult, Position, SeenObject, SensorReading
from robot_agent.world import GridWorld


class SimRobot:
    def __init__(self, world: GridWorld, *, sensor_timeout: bool = False) -> None:
        self.world = world
        self.sensor_timeout = sensor_timeout  # fault injection for tests

    def look(self) -> LookResult:
        objects = tuple(SeenObject(name, pos) for name, pos in sorted(self.world.objects.items()))
        return LookResult(robot=self.world.robot, objects=objects)

    def move(self, direction: Direction) -> Position:
        target = self.world.robot.moved(direction)
        if not self.world.is_free(target):
            raise CollisionError(f"cannot move {direction.value} into {target}")
        self.world.robot = target
        return target

    def read_sensor(self) -> SensorReading:
        if self.sensor_timeout:
            raise SensorTimeoutError("distance sensor did not respond")
        return SensorReading({d: self.world.clearance(d) for d in Direction})
