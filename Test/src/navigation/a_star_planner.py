"""Deterministic 8-neighborhood A* global grid planner."""

from __future__ import annotations

import heapq
import json
import math
from pathlib import Path
from typing import Any

from .exceptions import (
    GoalCellBlockedError,
    GoalOutsideMapError,
    NavigationPlannerConfigError,
    NoPathFoundError,
    PlanningLimitExceededError,
    StartCellBlockedError,
    StartOutsideMapError,
)
from .grid_inflation import build_planning_grid
from .occupancy_grid import OccupancyGridMap
from .planning_models import (
    GridCell,
    PlannedPath,
    PlannerConfig,
    PlanningGrid,
    WorldPoint,
)


PROJECT_ROOT = Path("/home/ps/ScanEverything/Test")
DEFAULT_PLANNER_CONFIG_PATH = PROJECT_ROOT / "config" / "navigation_planner.json"
SUPPORTED_SCHEMA_VERSION = 1
SQRT_2 = math.sqrt(2.0)

ORTHOGONAL_NEIGHBORS = (
    (-1, 0),
    (0, 1),
    (1, 0),
    (0, -1),
)
DIAGONAL_NEIGHBORS = (
    (-1, 1),
    (1, 1),
    (1, -1),
    (-1, -1),
)


def load_planner_config(
    config_path: str | Path = DEFAULT_PLANNER_CONFIG_PATH,
) -> PlannerConfig:
    """Load and strictly validate planner configuration JSON."""
    config_path = Path(config_path)
    if not config_path.is_file():
        raise NavigationPlannerConfigError(
            f"Planner config file not found: {config_path}"
        )

    try:
        with config_path.open("r", encoding="utf-8") as file:
            data = json.load(file, object_pairs_hook=_reject_duplicate_keys)
    except json.JSONDecodeError as error:
        raise NavigationPlannerConfigError(
            f"Planner config is not valid JSON: {config_path}. {error}"
        ) from error

    if not isinstance(data, dict):
        raise NavigationPlannerConfigError(
            "Planner config root must be a JSON object."
        )

    schema_version = data.get("schema_version")
    if schema_version != SUPPORTED_SCHEMA_VERSION:
        raise NavigationPlannerConfigError(
            "Unsupported planner schema_version: "
            f"{schema_version!r}. Supported version: "
            f"{SUPPORTED_SCHEMA_VERSION}."
        )

    return PlannerConfig(
        inflation_radius_m=_required_non_negative_float(
            data,
            "inflation_radius_m",
        ),
        treat_unknown_as_occupied=_required_bool(
            data,
            "treat_unknown_as_occupied",
        ),
        allow_diagonal=_required_bool(data, "allow_diagonal"),
        prevent_corner_cutting=_required_bool(data, "prevent_corner_cutting"),
        max_expanded_nodes=_required_positive_int(data, "max_expanded_nodes"),
    )


class AStarPlanner:
    """A deterministic A* planner over an inflated occupancy grid."""

    def __init__(
        self,
        occupancy_map: OccupancyGridMap,
        config: PlannerConfig,
    ):
        self.occupancy_map = occupancy_map
        self.config = config
        self.planning_grid = build_planning_grid(occupancy_map, config)

    def plan(
        self,
        *,
        start_cell: GridCell,
        goal_cell: GridCell,
        target_id: str,
        pose_id: str,
        exact_start_world: tuple[float, float],
        exact_goal_world: tuple[float, float],
        goal_yaw: float,
    ) -> PlannedPath:
        """Plan a raw A* path and return grid/world path data."""
        self._validate_endpoint(
            label="start",
            cell=start_cell,
            world_xy=exact_start_world,
        )
        self._validate_endpoint(
            label="goal",
            cell=goal_cell,
            world_xy=exact_goal_world,
        )

        if start_cell == goal_cell:
            world_path = (
                WorldPoint(
                    x=float(exact_start_world[0]),
                    y=float(exact_start_world[1]),
                ),
            )
            return PlannedPath(
                target_id=target_id,
                pose_id=pose_id,
                start_cell=start_cell,
                goal_cell=goal_cell,
                grid_path=(start_cell,),
                world_path=world_path,
                goal_yaw=float(goal_yaw),
                grid_cost=0.0,
                length_m=0.0,
                expanded_nodes=1,
            )

        came_from: dict[GridCell, GridCell] = {}
        g_score: dict[GridCell, float] = {start_cell: 0.0}
        open_heap: list[tuple[float, float, int, GridCell]] = []
        counter = 0
        heapq.heappush(
            open_heap,
            (
                self._octile_distance(start_cell, goal_cell),
                self._octile_distance(start_cell, goal_cell),
                counter,
                start_cell,
            ),
        )
        closed: set[GridCell] = set()
        expanded_nodes = 0

        while open_heap:
            _, _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)
            expanded_nodes += 1

            if expanded_nodes > self.config.max_expanded_nodes:
                raise PlanningLimitExceededError(
                    "A* exceeded max_expanded_nodes="
                    f"{self.config.max_expanded_nodes}. "
                    f"start={start_cell}, goal={goal_cell}, "
                    f"inflation_radius_m={self.config.inflation_radius_m}."
                )

            if current == goal_cell:
                return self._build_result(
                    came_from=came_from,
                    g_score=g_score,
                    start_cell=start_cell,
                    goal_cell=goal_cell,
                    target_id=target_id,
                    pose_id=pose_id,
                    exact_start_world=exact_start_world,
                    exact_goal_world=exact_goal_world,
                    goal_yaw=goal_yaw,
                    expanded_nodes=expanded_nodes,
                )

            for neighbor, move_cost in self._neighbors(current):
                if neighbor in closed:
                    continue
                tentative_g = g_score[current] + move_cost
                if tentative_g >= g_score.get(neighbor, math.inf):
                    continue

                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                heuristic = self._octile_distance(neighbor, goal_cell)
                counter += 1
                heapq.heappush(
                    open_heap,
                    (
                        tentative_g + heuristic,
                        heuristic,
                        counter,
                        neighbor,
                    ),
                )

        raise NoPathFoundError(
            "No A* path found. "
            f"start={start_cell}, goal={goal_cell}, "
            f"start_status={self._cell_status(start_cell)}, "
            f"goal_status={self._cell_status(goal_cell)}, "
            f"inflation_radius_m={self.config.inflation_radius_m}, "
            f"expanded_nodes={expanded_nodes}."
        )

    def _neighbors(self, cell: GridCell) -> tuple[tuple[GridCell, float], ...]:
        offsets = ORTHOGONAL_NEIGHBORS
        if self.config.allow_diagonal:
            offsets = offsets + DIAGONAL_NEIGHBORS

        neighbors: list[tuple[GridCell, float]] = []
        for dr, dc in offsets:
            neighbor = GridCell(
                row=cell.row + dr,
                col=cell.col + dc,
            )
            if not self.planning_grid.is_traversable(neighbor):
                continue
            if (
                dr != 0
                and dc != 0
                and self.config.prevent_corner_cutting
                and (
                    not self.planning_grid.is_traversable(
                        GridCell(row=cell.row + dr, col=cell.col)
                    )
                    or not self.planning_grid.is_traversable(
                        GridCell(row=cell.row, col=cell.col + dc)
                    )
                )
            ):
                continue
            neighbors.append((neighbor, SQRT_2 if dr != 0 and dc != 0 else 1.0))
        return tuple(neighbors)

    def _validate_endpoint(
        self,
        *,
        label: str,
        cell: GridCell,
        world_xy: tuple[float, float],
    ) -> None:
        if not self.occupancy_map.is_inside(cell.row, cell.col):
            error_cls = StartOutsideMapError if label == "start" else GoalOutsideMapError
            raise error_cls(
                f"{label} is outside map. "
                f"world={world_xy}, grid={cell}, "
                f"raw_status=outside, inflated_blocked=True, "
                f"inflation_radius_m={self.config.inflation_radius_m}."
            )

        if not self.planning_grid.is_traversable(cell):
            error_cls = StartCellBlockedError if label == "start" else GoalCellBlockedError
            raw_status = self._cell_status(cell)
            message = (
                f"{label} cell is blocked. world={world_xy}, grid={cell}, "
                f"raw_status={raw_status}, "
                f"inflated_blocked={self.planning_grid.is_inflated_blocked(cell)}, "
                f"inflation_radius_m={self.config.inflation_radius_m}."
            )
            if raw_status == "free" and self.planning_grid.is_inflated_blocked(cell):
                if label == "goal":
                    message += (
                        " Target is free in the raw map but blocked after "
                        "inflation; the goal pose is too close to an obstacle. "
                        "Recalibrate the goal pose or adjust inflation_radius_m."
                    )
                else:
                    message += (
                        " Start is free in the raw map but blocked after "
                        "inflation; move the robot farther from obstacles or "
                        "adjust inflation_radius_m."
                    )
            raise error_cls(message)

    def _cell_status(self, cell: GridCell) -> str:
        if not self.occupancy_map.is_inside(cell.row, cell.col):
            return "outside"
        return self.occupancy_map.status_at(cell.row, cell.col)

    def _build_result(
        self,
        *,
        came_from: dict[GridCell, GridCell],
        g_score: dict[GridCell, float],
        start_cell: GridCell,
        goal_cell: GridCell,
        target_id: str,
        pose_id: str,
        exact_start_world: tuple[float, float],
        exact_goal_world: tuple[float, float],
        goal_yaw: float,
        expanded_nodes: int,
    ) -> PlannedPath:
        path = [goal_cell]
        while path[-1] != start_cell:
            path.append(came_from[path[-1]])
        path.reverse()
        grid_path = tuple(path)

        world_points = [
            WorldPoint(*self.occupancy_map.grid_to_world(cell.row, cell.col))
            for cell in grid_path
        ]
        world_points[0] = WorldPoint(
            x=float(exact_start_world[0]),
            y=float(exact_start_world[1]),
        )
        world_points[-1] = WorldPoint(
            x=float(exact_goal_world[0]),
            y=float(exact_goal_world[1]),
        )
        world_path = tuple(world_points)

        return PlannedPath(
            target_id=target_id,
            pose_id=pose_id,
            start_cell=start_cell,
            goal_cell=goal_cell,
            grid_path=grid_path,
            world_path=world_path,
            goal_yaw=float(goal_yaw),
            grid_cost=g_score[goal_cell],
            length_m=_path_length(world_path),
            expanded_nodes=expanded_nodes,
        )

    @staticmethod
    def _octile_distance(first: GridCell, second: GridCell) -> float:
        dx = abs(first.col - second.col)
        dy = abs(first.row - second.row)
        return max(dx, dy) + (SQRT_2 - 1.0) * min(dx, dy)


def _path_length(world_path: tuple[WorldPoint, ...]) -> float:
    total = 0.0
    for first, second in zip(world_path, world_path[1:]):
        total += math.hypot(second.x - first.x, second.y - first.y)
    return total


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    data: dict[str, Any] = {}
    for key, value in pairs:
        if key in data:
            raise NavigationPlannerConfigError(
                f"Duplicate planner config key is not allowed: {key!r}."
            )
        data[key] = value
    return data


def _required_bool(data: dict[str, Any], key: str) -> bool:
    value = data.get(key)
    if not isinstance(value, bool):
        raise NavigationPlannerConfigError(
            f"Planner config field {key!r} must be a boolean."
        )
    return value


def _required_positive_int(data: dict[str, Any], key: str) -> int:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise NavigationPlannerConfigError(
            f"Planner config field {key!r} must be a positive integer."
        )
    return value


def _required_non_negative_float(data: dict[str, Any], key: str) -> float:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise NavigationPlannerConfigError(
            f"Planner config field {key!r} must be a number."
        )
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise NavigationPlannerConfigError(
            f"Planner config field {key!r} must be a finite non-negative number."
        )
    return result
