"""Save path planning debug PNG and JSON artifacts."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import struct
import zlib

from .occupancy_grid import OccupancyGridMap, _read_png_rgba
from .planning_models import GridCell, PlannedPath, PlanningGrid
from .simplification_models import SimplifiedPath


PROJECT_ROOT = Path("/home/ps/ScanEverything/Test")
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "outputs" / "navigation_plans"

COLOR_ORIGINAL_OBSTACLE = (20, 20, 20, 255)
COLOR_INFLATED = (255, 170, 0, 255)
COLOR_PATH = (0, 120, 255, 255)
COLOR_SIMPLIFIED_PATH = (220, 0, 255, 255)
COLOR_SIMPLIFIED_NODE = (255, 230, 0, 255)
COLOR_START = (0, 220, 80, 255)
COLOR_GOAL = (255, 40, 40, 255)


@dataclass(frozen=True)
class PathDebugArtifacts:
    """Paths written for one planning debug result."""

    png_path: Path
    scaled_png_path: Path
    json_path: Path


def save_path_debug_artifacts(
    *,
    occupancy_map: OccupancyGridMap,
    planning_grid: PlanningGrid,
    planned_path: PlannedPath,
    start_world: tuple[float, float, float],
    goal_world: tuple[float, float, float],
    output_dir: str | Path = DEFAULT_OUTPUT_DIR,
) -> PathDebugArtifacts:
    """Write base-scale PNG, 4x PNG, and JSON debug artifacts."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"{planned_path.target_id}_{planned_path.pose_id}_{timestamp}"
    png_path = output_dir / f"{stem}.png"
    scaled_png_path = output_dir / f"{stem}_x4.png"
    json_path = output_dir / f"{stem}.json"

    image = _render_path_overlay(
        occupancy_map=occupancy_map,
        planning_grid=planning_grid,
        planned_path=planned_path,
    )
    _write_png_rgba(png_path, image)
    _write_png_rgba(scaled_png_path, _scale_nearest(image, scale=4))
    _write_json(
        json_path=json_path,
        planned_path=planned_path,
        start_world=start_world,
        goal_world=goal_world,
        planning_grid=planning_grid,
    )

    return PathDebugArtifacts(
        png_path=png_path,
        scaled_png_path=scaled_png_path,
        json_path=json_path,
    )


def save_simplified_path_debug_artifacts(
    *,
    occupancy_map: OccupancyGridMap,
    planning_grid: PlanningGrid,
    simplified_path: SimplifiedPath,
    start_world: tuple[float, float, float],
    goal_world: tuple[float, float, float],
    output_dir: str | Path = DEFAULT_OUTPUT_DIR,
) -> PathDebugArtifacts:
    """Write PNG, 4x PNG, and JSON comparing raw and simplified paths."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = (
        f"{simplified_path.target_id}_{simplified_path.pose_id}_"
        f"{timestamp}_simplified"
    )
    png_path = output_dir / f"{stem}.png"
    scaled_png_path = output_dir / f"{stem}_4x.png"
    json_path = output_dir / f"{stem}.json"

    image = _render_simplified_path_overlay(
        occupancy_map=occupancy_map,
        planning_grid=planning_grid,
        simplified_path=simplified_path,
    )
    _write_png_rgba(png_path, image)
    _write_png_rgba(scaled_png_path, _scale_nearest(image, scale=4))
    _write_simplified_json(
        json_path=json_path,
        simplified_path=simplified_path,
        start_world=start_world,
        goal_world=goal_world,
        planning_grid=planning_grid,
    )

    return PathDebugArtifacts(
        png_path=png_path,
        scaled_png_path=scaled_png_path,
        json_path=json_path,
    )


def _render_path_overlay(
    *,
    occupancy_map: OccupancyGridMap,
    planning_grid: PlanningGrid,
    planned_path: PlannedPath,
) -> list[list[tuple[int, int, int, int]]]:
    _, _, base_pixels = _read_png_rgba(occupancy_map.metadata.image_path)
    image = [
        [pixel for pixel in row]
        for row in base_pixels
    ]

    for row in range(planning_grid.height):
        for col in range(planning_grid.width):
            cell = GridCell(row=row, col=col)
            if planning_grid.is_original_blocked(cell):
                image[row][col] = COLOR_ORIGINAL_OBSTACLE
            elif planning_grid.is_inflated_blocked(cell):
                image[row][col] = COLOR_INFLATED

    for cell in planned_path.grid_path:
        image[cell.row][cell.col] = COLOR_PATH

    _draw_square(image, planned_path.start_cell.row, planned_path.start_cell.col, COLOR_START)
    _draw_square(image, planned_path.goal_cell.row, planned_path.goal_cell.col, COLOR_GOAL)
    return image


def _render_simplified_path_overlay(
    *,
    occupancy_map: OccupancyGridMap,
    planning_grid: PlanningGrid,
    simplified_path: SimplifiedPath,
) -> list[list[tuple[int, int, int, int]]]:
    image = _render_planning_base(
        occupancy_map=occupancy_map,
        planning_grid=planning_grid,
    )

    for cell in simplified_path.original_grid_path:
        image[cell.row][cell.col] = COLOR_PATH

    for start, end in zip(
        simplified_path.simplified_grid_path,
        simplified_path.simplified_grid_path[1:],
    ):
        for cell in _trace_render_line(start, end):
            _draw_square(
                image,
                cell.row,
                cell.col,
                COLOR_SIMPLIFIED_PATH,
                radius=1,
            )

    for cell in simplified_path.simplified_grid_path:
        _draw_square(
            image,
            cell.row,
            cell.col,
            COLOR_SIMPLIFIED_NODE,
            radius=1,
        )

    start_cell = simplified_path.simplified_grid_path[0]
    goal_cell = simplified_path.simplified_grid_path[-1]
    _draw_square(image, start_cell.row, start_cell.col, COLOR_START)
    _draw_square(image, goal_cell.row, goal_cell.col, COLOR_GOAL)
    return image


def _render_planning_base(
    *,
    occupancy_map: OccupancyGridMap,
    planning_grid: PlanningGrid,
) -> list[list[tuple[int, int, int, int]]]:
    _, _, base_pixels = _read_png_rgba(occupancy_map.metadata.image_path)
    image = [
        [pixel for pixel in row]
        for row in base_pixels
    ]

    for row in range(planning_grid.height):
        for col in range(planning_grid.width):
            cell = GridCell(row=row, col=col)
            if planning_grid.is_original_blocked(cell):
                image[row][col] = COLOR_ORIGINAL_OBSTACLE
            elif planning_grid.is_inflated_blocked(cell):
                image[row][col] = COLOR_INFLATED

    return image


def _draw_square(
    image: list[list[tuple[int, int, int, int]]],
    row: int,
    col: int,
    color: tuple[int, int, int, int],
    radius: int = 2,
) -> None:
    height = len(image)
    width = len(image[0]) if image else 0
    for draw_row in range(row - radius, row + radius + 1):
        for draw_col in range(col - radius, col + radius + 1):
            if 0 <= draw_row < height and 0 <= draw_col < width:
                image[draw_row][draw_col] = color


def _trace_render_line(start: GridCell, end: GridCell) -> tuple[GridCell, ...]:
    if start == end:
        return (start,)

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

    while row != end.row or col != end.col:
        if t_max_col < t_max_row - 1e-12:
            col += step_col
            t_max_col += t_delta_col
        elif t_max_row < t_max_col - 1e-12:
            row += step_row
            t_max_row += t_delta_row
        else:
            col += step_col
            row += step_row
            t_max_col += t_delta_col
            t_max_row += t_delta_row
        cells.append(GridCell(row=row, col=col))

    return tuple(cells)


def _scale_nearest(
    image: list[list[tuple[int, int, int, int]]],
    *,
    scale: int,
) -> list[list[tuple[int, int, int, int]]]:
    scaled: list[list[tuple[int, int, int, int]]] = []
    for row in image:
        scaled_row: list[tuple[int, int, int, int]] = []
        for pixel in row:
            scaled_row.extend([pixel] * scale)
        for _ in range(scale):
            scaled.append(list(scaled_row))
    return scaled


def _write_json(
    *,
    json_path: Path,
    planned_path: PlannedPath,
    start_world: tuple[float, float, float],
    goal_world: tuple[float, float, float],
    planning_grid: PlanningGrid,
) -> None:
    data = {
        "target_id": planned_path.target_id,
        "pose_id": planned_path.pose_id,
        "start_world": {
            "x": start_world[0],
            "y": start_world[1],
            "yaw": start_world[2],
        },
        "goal_world": {
            "x": goal_world[0],
            "y": goal_world[1],
            "yaw": goal_world[2],
        },
        "start_grid": {
            "row": planned_path.start_cell.row,
            "col": planned_path.start_cell.col,
        },
        "goal_grid": {
            "row": planned_path.goal_cell.row,
            "col": planned_path.goal_cell.col,
        },
        "inflation_radius_m": planning_grid.inflation_radius_m,
        "grid_cost": planned_path.grid_cost,
        "length_m": planned_path.length_m,
        "expanded_nodes": planned_path.expanded_nodes,
        "grid_path": [
            {
                "row": cell.row,
                "col": cell.col,
            }
            for cell in planned_path.grid_path
        ],
        "world_path": [
            {
                "x": point.x,
                "y": point.y,
            }
            for point in planned_path.world_path
        ],
    }
    _ensure_finite_json(data)
    json_path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def _write_simplified_json(
    *,
    json_path: Path,
    simplified_path: SimplifiedPath,
    start_world: tuple[float, float, float],
    goal_world: tuple[float, float, float],
    planning_grid: PlanningGrid,
) -> None:
    data = {
        "path_type": "line_of_sight_simplified",
        "target_id": simplified_path.target_id,
        "pose_id": simplified_path.pose_id,
        "goal_yaw": simplified_path.goal_yaw,
        "start_world": {
            "x": start_world[0],
            "y": start_world[1],
            "yaw": start_world[2],
        },
        "goal_world": {
            "x": goal_world[0],
            "y": goal_world[1],
            "yaw": goal_world[2],
        },
        "inflation_radius_m": planning_grid.inflation_radius_m,
        "inflation_radius_cells": planning_grid.inflation_radius_cells,
        "treat_unknown_as_occupied": planning_grid.treat_unknown_as_occupied,
        "original_node_count": simplified_path.original_node_count,
        "simplified_node_count": simplified_path.simplified_node_count,
        "original_length_m": simplified_path.original_length_m,
        "simplified_length_m": simplified_path.simplified_length_m,
        "visibility_checks": simplified_path.visibility_checks,
        "original_grid_path": _grid_path_to_json(
            simplified_path.original_grid_path
        ),
        "simplified_grid_path": _grid_path_to_json(
            simplified_path.simplified_grid_path
        ),
        "original_world_path": _world_path_to_json(
            simplified_path.original_world_path
        ),
        "simplified_world_path": _world_path_to_json(
            simplified_path.simplified_world_path
        ),
    }
    _ensure_finite_json(data)
    json_path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def _grid_path_to_json(path: tuple[GridCell, ...]) -> list[dict[str, int]]:
    return [
        {
            "row": cell.row,
            "col": cell.col,
        }
        for cell in path
    ]


def _world_path_to_json(path) -> list[dict[str, float]]:
    return [
        {
            "x": point.x,
            "y": point.y,
        }
        for point in path
    ]


def _ensure_finite_json(value) -> None:
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("Debug JSON cannot contain NaN or Infinity.")
    elif isinstance(value, dict):
        for item in value.values():
            _ensure_finite_json(item)
    elif isinstance(value, list):
        for item in value:
            _ensure_finite_json(item)


def _write_png_rgba(
    path: Path,
    image: list[list[tuple[int, int, int, int]]],
) -> None:
    height = len(image)
    width = len(image[0]) if image else 0
    raw_rows = []
    for row in image:
        scanline = bytearray()
        for red, green, blue, alpha in row:
            scanline.extend((red, green, blue, alpha))
        raw_rows.append(b"\x00" + bytes(scanline))

    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(
            b"IHDR",
            struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0),
        )
        + _png_chunk(b"IDAT", zlib.compress(b"".join(raw_rows)))
        + _png_chunk(b"IEND", b"")
    )


def _png_chunk(chunk_type: bytes, data: bytes) -> bytes:
    crc = zlib.crc32(chunk_type + data) & 0xFFFFFFFF
    return (
        struct.pack(">I", len(data))
        + chunk_type
        + data
        + struct.pack(">I", crc)
    )


def _sign(value: int) -> int:
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


DEBUG_LEGEND = (
    "Legend: original obstacle=black, inflated blocked=orange, "
    "path=blue, start=green, goal=red."
)

DEBUG_SIMPLIFIED_LEGEND = (
    "Legend: original obstacle=black, inflated blocked=orange, "
    "raw A*=blue thin line, simplified path=magenta thick line, "
    "simplified key nodes=yellow, start=green, goal=red."
)
