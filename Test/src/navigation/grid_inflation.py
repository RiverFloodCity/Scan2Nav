"""Generate inflated planning grids from occupancy maps."""

from __future__ import annotations

import math

from .models import GoalPose
from .occupancy_grid import OccupancyGridMap
from .planning_models import GridCell, PlannerConfig, PlanningGrid


def inflation_radius_cells(
    inflation_radius_m: float,
    resolution: float,
) -> int:
    """Convert an inflation radius in meters to grid cells."""
    if inflation_radius_m <= 0.0:
        return 0
    return int(math.ceil(inflation_radius_m / resolution))


def circular_offsets(radius_cells: int) -> tuple[tuple[int, int], ...]:
    """Return row/col offsets inside a Euclidean radius."""
    offsets: list[tuple[int, int]] = []
    radius_squared = radius_cells * radius_cells
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            if dr * dr + dc * dc <= radius_squared:
                offsets.append((dr, dc))
    return tuple(offsets)


def build_planning_grid(
    occupancy_map: OccupancyGridMap,
    config: PlannerConfig,
) -> PlanningGrid:
    """Build a read-only traversability mask for planning."""
    height = occupancy_map.metadata.height
    width = occupancy_map.metadata.width
    radius_cells = inflation_radius_cells(
        config.inflation_radius_m,
        occupancy_map.metadata.resolution,
    )
    offsets = circular_offsets(radius_cells)

    original_blocked_lists = [
        [False for _ in range(width)]
        for _ in range(height)
    ]
    inflated_blocked_lists = [
        [False for _ in range(width)]
        for _ in range(height)
    ]

    for row in range(height):
        for col in range(width):
            if occupancy_map.is_occupied(row, col):
                original_blocked_lists[row][col] = True
            elif config.treat_unknown_as_occupied and occupancy_map.is_unknown(row, col):
                original_blocked_lists[row][col] = True

    for row in range(height):
        for col in range(width):
            if not original_blocked_lists[row][col]:
                continue
            for dr, dc in offsets:
                inflated_row = row + dr
                inflated_col = col + dc
                if 0 <= inflated_row < height and 0 <= inflated_col < width:
                    inflated_blocked_lists[inflated_row][inflated_col] = True

    if radius_cells > 0:
        for row in range(height):
            for col in range(width):
                if (
                    row < radius_cells
                    or col < radius_cells
                    or row >= height - radius_cells
                    or col >= width - radius_cells
                ):
                    inflated_blocked_lists[row][col] = True

    traversable = tuple(
        tuple(not inflated_blocked_lists[row][col] for col in range(width))
        for row in range(height)
    )
    original_blocked = tuple(tuple(row) for row in original_blocked_lists)
    inflated_blocked = tuple(tuple(row) for row in inflated_blocked_lists)

    return PlanningGrid(
        traversable=traversable,
        original_blocked=original_blocked,
        inflated_blocked=inflated_blocked,
        inflation_radius_cells=radius_cells,
        inflation_radius_m=config.inflation_radius_m,
        treat_unknown_as_occupied=config.treat_unknown_as_occupied,
    )


def goal_pose_to_cell(
    occupancy_map: OccupancyGridMap,
    goal_pose: GoalPose,
) -> GridCell:
    """Convert a GoalPose x/y into a GridCell using the map transform."""
    row, col = occupancy_map.world_to_grid(goal_pose.x, goal_pose.y)
    return GridCell(row=row, col=col)
