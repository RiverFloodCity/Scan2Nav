"""Dataclasses for deterministic path simplification."""

from dataclasses import dataclass

from .planning_models import GridCell, WorldPoint


@dataclass(frozen=True)
class SimplifiedPath:
    """Raw and line-of-sight simplified path data."""

    target_id: str
    pose_id: str
    original_grid_path: tuple[GridCell, ...]
    simplified_grid_path: tuple[GridCell, ...]
    original_world_path: tuple[WorldPoint, ...]
    simplified_world_path: tuple[WorldPoint, ...]
    goal_yaw: float
    original_length_m: float
    simplified_length_m: float
    original_node_count: int
    simplified_node_count: int
    visibility_checks: int
