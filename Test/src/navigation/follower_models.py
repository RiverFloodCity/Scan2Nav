"""Dataclasses and enums for path following."""

from dataclasses import dataclass
from enum import Enum

from .planning_models import WorldPoint


class NavigationState(str, Enum):
    """High-level automatic navigation state."""

    IDLE = "IDLE"
    PLANNING = "PLANNING"
    ALIGN_TO_PATH = "ALIGN_TO_PATH"
    TRACK_PATH = "TRACK_PATH"
    ALIGN_GOAL_YAW = "ALIGN_GOAL_YAW"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class FollowerConfig:
    """Configuration for pure-pursuit path following."""

    max_linear_speed: float
    min_linear_speed: float
    max_angular_speed: float
    lookahead_distance: float
    align_enter_angle: float
    align_exit_angle: float
    goal_position_tolerance: float
    goal_yaw_tolerance: float
    slowdown_distance: float
    max_path_deviation: float
    stop_on_inflated_blocked_cell: bool
    stop_on_raw_occupied_cell: bool
    warn_on_blocked_cell: bool
    progress_timeout_seconds: float
    minimum_progress_distance: float
    rotate_gain: float
    goal_yaw_gain: float
    control_period_seconds: float


@dataclass(frozen=True)
class FollowerOutput:
    """One path follower control result."""

    vx: float
    wz: float
    state: NavigationState
    message: str
    distance_to_goal: float
    yaw_error: float
    path_deviation: float
    current_segment_index: int
    lookahead_point: WorldPoint


PathFollowerConfig = FollowerConfig
PathFollowerOutput = FollowerOutput


@dataclass(frozen=True)
class NavigationStatus:
    """Snapshot of automatic navigation execution status."""

    state: NavigationState
    target_id: str | None = None
    pose_id: str | None = None
    message: str = ""
    current_x: float | None = None
    current_y: float | None = None
    current_yaw: float | None = None
    goal_x: float | None = None
    goal_y: float | None = None
    goal_yaw: float | None = None
    distance_to_goal: float | None = None
    yaw_error: float | None = None
    path_deviation: float | None = None
    current_segment_index: int = 0
    last_vx: float = 0.0
    last_wz: float = 0.0
    elapsed_seconds: float = 0.0
