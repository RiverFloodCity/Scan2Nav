"""Deterministic line-of-sight simplification for raw A* paths."""

from __future__ import annotations

import math

from .exceptions import (
    EmptyPathError,
    InvalidPathCellError,
    InvalidSimplifiedSegmentError,
    PathSimplificationError,
)
from .occupancy_grid import OccupancyGridMap
from .planning_models import GridCell, PlannedPath, WorldPoint
from .simplification_models import SimplifiedPath


TraversableMask = tuple[tuple[bool, ...], ...]
LENGTH_EPSILON_M = 1e-9


class PathSimplifier:
    """Greedy string-pulling simplifier over an inflated traversable mask."""

    def __init__(
        self,
        *,
        traversable_mask: TraversableMask,
        prevent_corner_cutting: bool = True,
    ):
        self.traversable_mask = _validate_traversable_mask(traversable_mask)
        self.prevent_corner_cutting = bool(prevent_corner_cutting)
        self.height = len(self.traversable_mask)
        self.width = len(self.traversable_mask[0])

    def has_line_of_sight(self, start: GridCell, end: GridCell) -> bool:
        """Return True when a segment touches only traversable cells."""
        return self._line_of_sight_failure(start, end) is None

    def simplify(
        self,
        planned_path: PlannedPath,
        *,
        occupancy_map: OccupancyGridMap,
    ) -> SimplifiedPath:
        """Simplify a raw A* path without changing its endpoints."""
        original_grid_path = tuple(planned_path.grid_path)
        original_world_path = tuple(planned_path.world_path)

        if not original_grid_path:
            raise EmptyPathError("Cannot simplify an empty grid path.")
        if not original_world_path:
            raise PathSimplificationError(
                "Cannot simplify a path with an empty world_path."
            )
        if len(original_grid_path) != len(original_world_path):
            raise PathSimplificationError(
                "grid_path and world_path length mismatch: "
                f"grid={len(original_grid_path)}, "
                f"world={len(original_world_path)}."
            )

        self._validate_original_path(original_grid_path)

        if len(original_grid_path) <= 2:
            simplified_grid_path = original_grid_path
            visibility_checks = 0
        else:
            simplified: list[GridCell] = [original_grid_path[0]]
            anchor_index = 0
            visibility_checks = 0

            while anchor_index < len(original_grid_path) - 1:
                next_index = None
                for candidate_index in range(
                    len(original_grid_path) - 1,
                    anchor_index,
                    -1,
                ):
                    visibility_checks += 1
                    if self.has_line_of_sight(
                        original_grid_path[anchor_index],
                        original_grid_path[candidate_index],
                    ):
                        next_index = candidate_index
                        break

                if next_index is None:
                    raise PathSimplificationError(
                        "Could not find any visible next path node from "
                        f"index={anchor_index}, "
                        f"cell={original_grid_path[anchor_index]}."
                    )

                simplified.append(original_grid_path[next_index])
                anchor_index = next_index

            simplified_grid_path = tuple(simplified)

        self.validate_simplified_path(simplified_grid_path)

        simplified_world_path = _build_simplified_world_path(
            simplified_grid_path=simplified_grid_path,
            planned_path=planned_path,
            occupancy_map=occupancy_map,
        )
        simplified_length_m = _path_length(simplified_world_path)
        original_length_m = float(planned_path.length_m)

        if not math.isfinite(original_length_m):
            raise PathSimplificationError(
                f"Original path length is not finite: {original_length_m!r}."
            )
        if simplified_length_m > original_length_m + LENGTH_EPSILON_M:
            raise PathSimplificationError(
                "Simplified path is longer than original path: "
                f"simplified_length_m={simplified_length_m}, "
                f"original_length_m={original_length_m}."
            )

        return SimplifiedPath(
            target_id=planned_path.target_id,
            pose_id=planned_path.pose_id,
            original_grid_path=original_grid_path,
            simplified_grid_path=simplified_grid_path,
            original_world_path=original_world_path,
            simplified_world_path=simplified_world_path,
            goal_yaw=planned_path.goal_yaw,
            original_length_m=original_length_m,
            simplified_length_m=simplified_length_m,
            original_node_count=len(original_grid_path),
            simplified_node_count=len(simplified_grid_path),
            visibility_checks=visibility_checks,
        )

    def validate_simplified_path(
        self,
        simplified_grid_path: tuple[GridCell, ...],
    ) -> None:
        """Recheck every simplified segment with line-of-sight."""
        if not simplified_grid_path:
            raise EmptyPathError("Cannot validate an empty simplified path.")

        for index, cell in enumerate(simplified_grid_path):
            failure = self._cell_failure(cell)
            if failure is not None:
                raise InvalidPathCellError(
                    f"Simplified path node index={index}, cell={cell} "
                    f"is invalid: {failure}."
                )

        for segment_index, (start, end) in enumerate(
            zip(simplified_grid_path, simplified_grid_path[1:])
        ):
            failure = self._line_of_sight_failure(start, end)
            if failure is not None:
                raise InvalidSimplifiedSegmentError(
                    "Simplified segment failed line-of-sight validation: "
                    f"segment_index={segment_index}, start={start}, "
                    f"end={end}, reason={failure}."
                )

    def _validate_original_path(
        self,
        original_grid_path: tuple[GridCell, ...],
    ) -> None:
        for index, cell in enumerate(original_grid_path):
            failure = self._cell_failure(cell)
            if failure is not None:
                raise InvalidPathCellError(
                    f"Original path node index={index}, cell={cell} "
                    f"is invalid: {failure}."
                )

    def _line_of_sight_failure(self, start: GridCell, end: GridCell) -> str | None:
        for label, cell in (("start", start), ("end", end)):
            failure = self._cell_failure(cell)
            if failure is not None:
                return f"{label} cell {cell} is invalid: {failure}"

        trace = _trace_grid_line(start, end)
        for cell in trace.cells:
            failure = self._cell_failure(cell)
            if failure is not None:
                return f"line touches cell {cell}: {failure}"

        if self.prevent_corner_cutting:
            for first, second in trace.corner_checks:
                first_failure = self._cell_failure(first)
                second_failure = self._cell_failure(second)
                if first_failure is not None or second_failure is not None:
                    return (
                        "corner cutting violation: "
                        f"adjacent cells {first} ({first_failure or 'ok'}), "
                        f"{second} ({second_failure or 'ok'}) "
                        "must both be traversable"
                    )

        return None

    def _cell_failure(self, cell: GridCell) -> str | None:
        if not isinstance(cell, GridCell):
            return f"not a GridCell: {cell!r}"
        if not self._is_inside(cell):
            return (
                f"outside map bounds height={self.height}, width={self.width}"
            )
        if not self.traversable_mask[cell.row][cell.col]:
            return "blocked by inflated traversable mask"
        return None

    def _is_inside(self, cell: GridCell) -> bool:
        return 0 <= cell.row < self.height and 0 <= cell.col < self.width


class _LineTrace:
    def __init__(
        self,
        *,
        cells: tuple[GridCell, ...],
        corner_checks: tuple[tuple[GridCell, GridCell], ...],
    ):
        self.cells = cells
        self.corner_checks = corner_checks


def _trace_grid_line(start: GridCell, end: GridCell) -> _LineTrace:
    """Trace grid cells crossed by a segment between cell centers."""
    if start == end:
        return _LineTrace(cells=(start,), corner_checks=())

    row = start.row
    col = start.col
    delta_row = end.row - start.row
    delta_col = end.col - start.col
    step_row = _sign(delta_row)
    step_col = _sign(delta_col)

    t_delta_row = math.inf if delta_row == 0 else 1.0 / abs(delta_row)
    t_delta_col = math.inf if delta_col == 0 else 1.0 / abs(delta_col)
    t_max_row = math.inf if delta_row == 0 else 0.5 / abs(delta_row)
    t_max_col = math.inf if delta_col == 0 else 0.5 / abs(delta_col)

    cells = [start]
    corner_checks: list[tuple[GridCell, GridCell]] = []

    while row != end.row or col != end.col:
        if t_max_col < t_max_row - 1e-12:
            col += step_col
            t_max_col += t_delta_col
        elif t_max_row < t_max_col - 1e-12:
            row += step_row
            t_max_row += t_delta_row
        else:
            corner_checks.append(
                (
                    GridCell(row=row, col=col + step_col),
                    GridCell(row=row + step_row, col=col),
                )
            )
            col += step_col
            row += step_row
            t_max_col += t_delta_col
            t_max_row += t_delta_row

        cells.append(GridCell(row=row, col=col))

    return _LineTrace(
        cells=tuple(cells),
        corner_checks=tuple(corner_checks),
    )


def _build_simplified_world_path(
    *,
    simplified_grid_path: tuple[GridCell, ...],
    planned_path: PlannedPath,
    occupancy_map: OccupancyGridMap,
) -> tuple[WorldPoint, ...]:
    world_points = [
        WorldPoint(*occupancy_map.grid_to_world(cell.row, cell.col))
        for cell in simplified_grid_path
    ]

    world_points[0] = planned_path.world_path[0]
    if len(world_points) > 1:
        world_points[-1] = planned_path.world_path[-1]
    return tuple(world_points)


def _path_length(world_path: tuple[WorldPoint, ...]) -> float:
    total = 0.0
    for first, second in zip(world_path, world_path[1:]):
        total += math.hypot(second.x - first.x, second.y - first.y)
    if not math.isfinite(total):
        raise PathSimplificationError(
            f"Simplified path length is not finite: {total!r}."
        )
    return total


def _validate_traversable_mask(mask: TraversableMask) -> TraversableMask:
    if not mask or not mask[0]:
        raise PathSimplificationError("traversable_mask must not be empty.")
    width = len(mask[0])
    normalized_rows = []
    for row_index, row in enumerate(mask):
        if len(row) != width:
            raise PathSimplificationError(
                "traversable_mask must be rectangular: "
                f"row 0 width={width}, row {row_index} width={len(row)}."
            )
        normalized_rows.append(tuple(bool(value) for value in row))
    return tuple(normalized_rows)


def _sign(value: int) -> int:
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0
