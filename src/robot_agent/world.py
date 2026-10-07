"""In-memory grid world used by the simulator."""

from __future__ import annotations

from dataclasses import dataclass, field

from robot_agent.models import Direction, Position


@dataclass
class GridWorld:
    robot: Position
    width: int = 5
    height: int = 5
    walls: frozenset[Position] = frozenset()
    objects: dict[str, Position] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.is_free(self.robot):
            raise ValueError(f"robot start {self.robot} is not a free cell")
        for name, pos in self.objects.items():
            if not self.is_free(pos):
                raise ValueError(f"object {name!r} at {pos} is not a free cell")

    def in_bounds(self, pos: Position) -> bool:
        return 0 <= pos.row < self.height and 0 <= pos.col < self.width

    def is_free(self, pos: Position) -> bool:
        return self.in_bounds(pos) and pos not in self.walls

    def clearance(self, direction: Direction) -> int:
        """Number of free cells the robot can advance in `direction`."""
        count = 0
        pos = self.robot.moved(direction)
        while self.is_free(pos):
            count += 1
            pos = pos.moved(direction)
        return count

    def render(self) -> str:
        symbols = {pos: name[0].upper() for name, pos in self.objects.items()}
        rows = []
        for r in range(self.height):
            cells = []
            for c in range(self.width):
                pos = Position(r, c)
                if pos == self.robot:
                    cells.append("@")
                elif pos in self.walls:
                    cells.append("#")
                else:
                    cells.append(symbols.get(pos, "."))
            rows.append(" ".join(cells))
        return "\n".join(rows)
