"""Pure-math path follower for simplified world-coordinate paths."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from .exceptions import (
    NavigationExecutionError,
    NavigationFollowerConfigError,
)
from .follower_models import FollowerConfig, FollowerOutput, NavigationState
from .planning_models import WorldPoint


PROJECT_ROOT = Path("/home/ps/ScanEverything/Test")
DEFAULT_FOLLOWER_CONFIG_PATH = PROJECT_ROOT / "config" / "navigation_follower.json"
SUPPORTED_SCHEMA_VERSION = 1
EPSILON = 1e-9


def load_follower_config(
    config_path: str | Path = DEFAULT_FOLLOWER_CONFIG_PATH,
) -> FollowerConfig:
    """Load and strictly validate path follower configuration JSON."""
    config_path = Path(config_path)
    if not config_path.is_file():
        raise NavigationFollowerConfigError(
            f"Follower config file not found: {config_path}"
        )

    try:
        with config_path.open("r", encoding="utf-8") as file:
            data = json.load(file, object_pairs_hook=_reject_duplicate_keys)
    except json.JSONDecodeError as error:
        raise NavigationFollowerConfigError(
            f"Follower config is not valid JSON: {config_path}. {error}"
        ) from error

    if not isinstance(data, dict):
        raise NavigationFollowerConfigError(
            "Follower config root must be a JSON object."
        )

    schema_version = data.get("schema_version")
    if schema_version != SUPPORTED_SCHEMA_VERSION:
        raise NavigationFollowerConfigError(
            "Unsupported follower schema_version: "
            f"{schema_version!r}. Supported version: "
            f"{SUPPORTED_SCHEMA_VERSION}."
        )

    config = FollowerConfig(
        max_linear_speed=_required_positive_float(data, "max_linear_speed"),
        min_linear_speed=_required_non_negative_float(data, "min_linear_speed"),
        max_angular_speed=_required_positive_float(data, "max_angular_speed"),
        lookahead_distance=_required_positive_float(data, "lookahead_distance"),
        align_enter_angle=_required_positive_float(data, "align_enter_angle"),
        align_exit_angle=_required_positive_float(data, "align_exit_angle"),
        goal_position_tolerance=_required_positive_float(
            data,
            "goal_position_tolerance",
        ),
        goal_yaw_tolerance=_required_positive_float(data, "goal_yaw_tolerance"),
        slowdown_distance=_required_positive_float(data, "slowdown_distance"),
        max_path_deviation=_required_positive_float(data, "max_path_deviation"),
        stop_on_inflated_blocked_cell=_required_bool(
            data,
            "stop_on_inflated_blocked_cell",
        ),
        stop_on_raw_occupied_cell=_required_bool(
            data,
            "stop_on_raw_occupied_cell",
        ),
        warn_on_blocked_cell=_required_bool(data, "warn_on_blocked_cell"),
        progress_timeout_seconds=_required_positive_float(
            data,
            "progress_timeout_seconds",
        ),
        minimum_progress_distance=_required_positive_float(
            data,
            "minimum_progress_distance",
        ),
        rotate_gain=_required_positive_float(data, "rotate_gain"),
        goal_yaw_gain=_required_positive_float(data, "goal_yaw_gain"),
        control_period_seconds=_required_positive_float(
            data,
            "control_period_seconds",
        ),
    )

    if config.min_linear_speed > config.max_linear_speed:
        raise NavigationFollowerConfigError(
            "min_linear_speed must be <= max_linear_speed."
        )
    if config.align_exit_angle >= config.align_enter_angle:
        raise NavigationFollowerConfigError(
            "align_exit_angle must be smaller than align_enter_angle."
        )
    if config.goal_position_tolerance >= config.slowdown_distance:
        raise NavigationFollowerConfigError(
            "goal_position_tolerance should be smaller than slowdown_distance."
        )

    return config


class PathFollower:
    """Stateful pure-pursuit follower for a simplified polyline."""

    def __init__(
        self,
        *,
        world_path: tuple[WorldPoint, ...],
        goal_yaw: float,
        config: FollowerConfig,
        initial_state: NavigationState = NavigationState.ALIGN_TO_PATH,
    ):
        if not world_path:
            raise NavigationExecutionError("world_path must not be empty.")
        if not all(isinstance(point, WorldPoint) for point in world_path):
            raise NavigationExecutionError(
                "world_path must contain only WorldPoint objects."
            )
        self.world_path = tuple(world_path)
        self.goal_yaw = float(goal_yaw)
        if not math.isfinite(self.goal_yaw):
            raise NavigationExecutionError("goal_yaw must be finite.")
        self.config = config
        self.state = initial_state
        self.current_segment_index = 0

    def step(self, *, x: float, y: float, yaw: float) -> FollowerOutput:
        """Compute one conservative vx/wz command."""
        x = _finite_float(x, "x")
        y = _finite_float(y, "y")
        yaw = normalize_angle(_finite_float(yaw, "yaw"))

        goal = self.world_path[-1]
        distance_to_goal = distance_xy(x, y, goal.x, goal.y)
        yaw_error = normalize_angle(self.goal_yaw - yaw)
        path_deviation = distance_to_polyline(x, y, self.world_path)

        if self.state in (
            NavigationState.SUCCEEDED,
            NavigationState.FAILED,
            NavigationState.CANCELLED,
        ):
            return self._output(
                vx=0.0,
                wz=0.0,
                state=self.state,
                message=f"Follower is already {self.state.value}.",
                x=x,
                y=y,
                yaw_error=yaw_error,
                distance_to_goal=distance_to_goal,
                path_deviation=path_deviation,
                lookahead_point=goal,
            )

        if path_deviation > self.config.max_path_deviation:
            self.state = NavigationState.FAILED
            return self._output(
                vx=0.0,
                wz=0.0,
                state=self.state,
                message=(
                    "Path deviation exceeded max_path_deviation: "
                    f"{path_deviation:.3f} m > "
                    f"{self.config.max_path_deviation:.3f} m."
                ),
                x=x,
                y=y,
                yaw_error=yaw_error,
                distance_to_goal=distance_to_goal,
                path_deviation=path_deviation,
                lookahead_point=goal,
            )

        if distance_to_goal <= self.config.goal_position_tolerance:
            self.state = NavigationState.ALIGN_GOAL_YAW

        if self.state == NavigationState.ALIGN_GOAL_YAW:
            if abs(yaw_error) <= self.config.goal_yaw_tolerance:
                self.state = NavigationState.SUCCEEDED
                return self._output(
                    vx=0.0,
                    wz=0.0,
                    state=self.state,
                    message="Goal position and yaw reached.",
                    x=x,
                    y=y,
                    yaw_error=yaw_error,
                    distance_to_goal=distance_to_goal,
                    path_deviation=path_deviation,
                    lookahead_point=goal,
                )

            return self._output(
                vx=0.0,
                wz=clamp(
                    self.config.goal_yaw_gain * yaw_error,
                    -self.config.max_angular_speed,
                    self.config.max_angular_speed,
                ),
                state=self.state,
                message="Aligning to final goal yaw.",
                x=x,
                y=y,
                yaw_error=yaw_error,
                distance_to_goal=distance_to_goal,
                path_deviation=path_deviation,
                lookahead_point=goal,
            )

        lookahead_point = self._select_lookahead_point(x, y)
        heading_error = normalize_angle(
            math.atan2(lookahead_point.y - y, lookahead_point.x - x) - yaw
        )

        if self.state == NavigationState.ALIGN_TO_PATH:
            if abs(heading_error) > self.config.align_exit_angle:
                return self._output(
                    vx=0.0,
                    wz=clamp(
                        self.config.rotate_gain * heading_error,
                        -self.config.max_angular_speed,
                        self.config.max_angular_speed,
                    ),
                    state=self.state,
                    message="Aligning to path heading.",
                    x=x,
                    y=y,
                    yaw_error=heading_error,
                    distance_to_goal=distance_to_goal,
                    path_deviation=path_deviation,
                    lookahead_point=lookahead_point,
                )
            self.state = NavigationState.TRACK_PATH

        if abs(heading_error) > self.config.align_enter_angle:
            self.state = NavigationState.ALIGN_TO_PATH
            return self._output(
                vx=0.0,
                wz=clamp(
                    self.config.rotate_gain * heading_error,
                    -self.config.max_angular_speed,
                    self.config.max_angular_speed,
                ),
                state=self.state,
                message="Heading error too large; re-aligning to path.",
                x=x,
                y=y,
                yaw_error=heading_error,
                distance_to_goal=distance_to_goal,
                path_deviation=path_deviation,
                lookahead_point=lookahead_point,
            )

        actual_lookahead = max(
            distance_xy(x, y, lookahead_point.x, lookahead_point.y),
            EPSILON,
        )
        curvature = 2.0 * math.sin(heading_error) / actual_lookahead
        turn_scale = max(
            0.2,
            1.0 - abs(heading_error) / self.config.align_enter_angle,
        )
        goal_scale = clamp(
            distance_to_goal / self.config.slowdown_distance,
            0.2,
            1.0,
        )
        vx = self.config.max_linear_speed * min(turn_scale, goal_scale)
        if distance_to_goal > self.config.goal_position_tolerance:
            vx = max(vx, self.config.min_linear_speed)
        vx = clamp(vx, 0.0, self.config.max_linear_speed)
        wz = clamp(
            vx * curvature,
            -self.config.max_angular_speed,
            self.config.max_angular_speed,
        )

        self.state = NavigationState.TRACK_PATH
        return self._output(
            vx=vx,
            wz=wz,
            state=self.state,
            message="Tracking simplified path.",
            x=x,
            y=y,
            yaw_error=heading_error,
            distance_to_goal=distance_to_goal,
            path_deviation=path_deviation,
            lookahead_point=lookahead_point,
        )

    def _select_lookahead_point(self, x: float, y: float) -> WorldPoint:
        if len(self.world_path) == 1:
            self.current_segment_index = 0
            return self.world_path[0]

        closest = closest_segment_projection(
            x,
            y,
            self.world_path,
            start_segment_index=self.current_segment_index,
        )
        self.current_segment_index = max(
            self.current_segment_index,
            closest.segment_index,
        )

        remaining = self.config.lookahead_distance
        segment_index = self.current_segment_index
        segment_start_t = closest.t if segment_index == closest.segment_index else 0.0

        while segment_index < len(self.world_path) - 1:
            first = self.world_path[segment_index]
            second = self.world_path[segment_index + 1]
            segment_length = distance_xy(first.x, first.y, second.x, second.y)
            if segment_length <= EPSILON:
                segment_index += 1
                segment_start_t = 0.0
                continue

            available = segment_length * (1.0 - segment_start_t)
            if remaining <= available + EPSILON:
                t = segment_start_t + remaining / segment_length
                return interpolate(first, second, clamp(t, 0.0, 1.0))

            remaining -= available
            segment_index += 1
            segment_start_t = 0.0

        return self.world_path[-1]

    def _output(
        self,
        *,
        vx: float,
        wz: float,
        state: NavigationState,
        message: str,
        x: float,
        y: float,
        yaw_error: float,
        distance_to_goal: float,
        path_deviation: float,
        lookahead_point: WorldPoint,
    ) -> FollowerOutput:
        return FollowerOutput(
            vx=clamp(vx, 0.0, self.config.max_linear_speed),
            wz=clamp(
                wz,
                -self.config.max_angular_speed,
                self.config.max_angular_speed,
            ),
            state=state,
            message=message,
            distance_to_goal=distance_to_goal,
            yaw_error=yaw_error,
            path_deviation=path_deviation,
            current_segment_index=self.current_segment_index,
            lookahead_point=lookahead_point,
        )


class SegmentProjection:
    """Projection of a point onto one path segment."""

    def __init__(
        self,
        *,
        segment_index: int,
        t: float,
        point: WorldPoint,
        distance: float,
    ):
        self.segment_index = segment_index
        self.t = t
        self.point = point
        self.distance = distance


def closest_segment_projection(
    x: float,
    y: float,
    world_path: tuple[WorldPoint, ...],
    *,
    start_segment_index: int = 0,
) -> SegmentProjection:
    """Return the closest projection on a polyline from start_segment_index."""
    if len(world_path) == 1:
        point = world_path[0]
        return SegmentProjection(
            segment_index=0,
            t=0.0,
            point=point,
            distance=distance_xy(x, y, point.x, point.y),
        )

    max_segment_index = len(world_path) - 2
    start_segment_index = max(0, min(start_segment_index, max_segment_index))
    best: SegmentProjection | None = None

    for segment_index in range(start_segment_index, max_segment_index + 1):
        projection = project_point_to_segment(
            x,
            y,
            world_path[segment_index],
            world_path[segment_index + 1],
            segment_index=segment_index,
        )
        if best is None or projection.distance < best.distance:
            best = projection

    assert best is not None
    return best


def distance_to_polyline(
    x: float,
    y: float,
    world_path: tuple[WorldPoint, ...],
) -> float:
    """Return shortest distance from a point to the whole polyline."""
    if len(world_path) == 1:
        point = world_path[0]
        return distance_xy(x, y, point.x, point.y)

    return min(
        project_point_to_segment(
            x,
            y,
            world_path[index],
            world_path[index + 1],
            segment_index=index,
        ).distance
        for index in range(len(world_path) - 1)
    )


def project_point_to_segment(
    x: float,
    y: float,
    first: WorldPoint,
    second: WorldPoint,
    *,
    segment_index: int,
) -> SegmentProjection:
    dx = second.x - first.x
    dy = second.y - first.y
    length_squared = dx * dx + dy * dy
    if length_squared <= EPSILON:
        t = 0.0
    else:
        t = clamp(
            ((x - first.x) * dx + (y - first.y) * dy) / length_squared,
            0.0,
            1.0,
        )
    point = interpolate(first, second, t)
    return SegmentProjection(
        segment_index=segment_index,
        t=t,
        point=point,
        distance=distance_xy(x, y, point.x, point.y),
    )


def interpolate(first: WorldPoint, second: WorldPoint, t: float) -> WorldPoint:
    return WorldPoint(
        x=first.x + (second.x - first.x) * t,
        y=first.y + (second.y - first.y) * t,
    )


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def distance_xy(first_x: float, first_y: float, second_x: float, second_y: float) -> float:
    return math.hypot(second_x - first_x, second_y - first_y)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, float(value)))


def _finite_float(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise NavigationExecutionError(f"{name} must be finite.")
    return result


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    data: dict[str, Any] = {}
    for key, value in pairs:
        if key in data:
            raise NavigationFollowerConfigError(
                f"Duplicate follower config key is not allowed: {key!r}."
            )
        data[key] = value
    return data


def _required_positive_float(data: dict[str, Any], key: str) -> float:
    value = _required_number(data, key)
    if value <= 0.0:
        raise NavigationFollowerConfigError(
            f"Follower config field {key!r} must be positive."
        )
    return value


def _required_non_negative_float(data: dict[str, Any], key: str) -> float:
    value = _required_number(data, key)
    if value < 0.0:
        raise NavigationFollowerConfigError(
            f"Follower config field {key!r} must be non-negative."
        )
    return value


def _required_number(data: dict[str, Any], key: str) -> float:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise NavigationFollowerConfigError(
            f"Follower config field {key!r} must be a number."
        )
    result = float(value)
    if not math.isfinite(result):
        raise NavigationFollowerConfigError(
            f"Follower config field {key!r} must be finite."
        )
    return result


def _required_bool(data: dict[str, Any], key: str) -> bool:
    value = data.get(key)
    if not isinstance(value, bool):
        raise NavigationFollowerConfigError(
            f"Follower config field {key!r} must be a boolean."
        )
    return value
