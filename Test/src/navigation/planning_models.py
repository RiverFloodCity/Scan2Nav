"""Dataclasses for global grid path planning."""

from dataclasses import dataclass


@dataclass(frozen=True)
class GridCell:
    """A grid cell addressed by image row and column."""

    row: int
    col: int


@dataclass(frozen=True)
class WorldPoint:
    """A 2D point in Isaac Sim world coordinates."""

    x: float
    y: float


@dataclass(frozen=True)
class PlannerConfig:
    """Configuration for obstacle inflation and A* search."""

    inflation_radius_m: float
    treat_unknown_as_occupied: bool
    allow_diagonal: bool
    prevent_corner_cutting: bool
    max_expanded_nodes: int


@dataclass(frozen=True)
class PlanningGrid:
    """Read-only planning mask derived from an OccupancyGridMap."""

    traversable: tuple[tuple[bool, ...], ...]
    original_blocked: tuple[tuple[bool, ...], ...]
    inflated_blocked: tuple[tuple[bool, ...], ...]
    inflation_radius_cells: int
    inflation_radius_m: float
    treat_unknown_as_occupied: bool

    @property
    def height(self) -> int:
        return len(self.traversable)

    @property
    def width(self) -> int:
        return len(self.traversable[0]) if self.traversable else 0

    def is_inside(self, cell: GridCell) -> bool:
        return 0 <= cell.row < self.height and 0 <= cell.col < self.width

    def is_traversable(self, cell: GridCell) -> bool:
        return self.is_inside(cell) and self.traversable[cell.row][cell.col]

    def is_original_blocked(self, cell: GridCell) -> bool:
        return self.is_inside(cell) and self.original_blocked[cell.row][cell.col]

    def is_inflated_blocked(self, cell: GridCell) -> bool:
        return self.is_inside(cell) and self.inflated_blocked[cell.row][cell.col]


@dataclass(frozen=True)
class PlannedPath:
    """Raw A* path in grid cells and world coordinates."""

    target_id: str
    pose_id: str
    start_cell: GridCell
    goal_cell: GridCell
    grid_path: tuple[GridCell, ...]
    world_path: tuple[WorldPoint, ...]
    goal_yaw: float
    grid_cost: float
    length_m: float
    expanded_nodes: int
