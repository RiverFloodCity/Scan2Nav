"""Load and query the strict semantic navigation target registry."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
from typing import Any

from .exceptions import (
    InvalidTargetIdError,
    NavigationConfigError,
    UnknownNavigationTargetError,
)
from .models import GoalPose, NavigationTarget


PROJECT_ROOT = Path("/home/ps/ScanEverything/Test")
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config" / "navigation_targets.json"
SUPPORTED_SCHEMA_VERSION = 1
TARGET_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")


def validate_target_id_format(target_id: object) -> str:
    """Return a valid target_id or raise InvalidTargetIdError."""
    if not isinstance(target_id, str) or not target_id:
        raise InvalidTargetIdError(
            "target_id must be a non-empty string matching "
            "^[a-z][a-z0-9_]*$."
        )

    if TARGET_ID_PATTERN.fullmatch(target_id) is None:
        raise InvalidTargetIdError(
            "Invalid target_id: "
            f"{target_id!r}. target_id must match ^[a-z][a-z0-9_]*$."
        )

    return target_id


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    data: dict[str, Any] = {}
    for key, value in pairs:
        if key in data:
            raise NavigationConfigError(
                f"Duplicate JSON key is not allowed: {key!r}."
            )
        data[key] = value
    return data


def _available_target_ids_message(target_ids: tuple[str, ...]) -> str:
    lines = ["Available target_ids:"]
    lines.extend(f"- {target_id}" for target_id in target_ids)
    return "\n".join(lines)


class TargetRegistry:
    """Strict registry for semantic navigation targets."""

    def __init__(self, config_path: str | Path | None = None):
        self.config_path = (
            Path(config_path) if config_path is not None else DEFAULT_CONFIG_PATH
        )
        self._targets: dict[str, NavigationTarget] = {}
        self.reload()

    def reload(self) -> None:
        """Reload targets from the JSON configuration file."""
        raw_config = self._load_json(self.config_path)
        self._targets = self._parse_config(raw_config)

    def list_target_ids(self) -> tuple[str, ...]:
        """Return registered target ids in stable sorted order."""
        return tuple(sorted(self._targets))

    def list_targets(self) -> tuple[NavigationTarget, ...]:
        """Return registered targets in stable target_id order."""
        return tuple(self._targets[target_id] for target_id in self.list_target_ids())

    def has_target(self, target_id: str) -> bool:
        """Return True only for exact registered target_id matches."""
        return target_id in self._targets

    def get_target(self, target_id: str) -> NavigationTarget:
        """Return a target by exact target_id match."""
        target_id = validate_target_id_format(target_id)

        try:
            return self._targets[target_id]
        except KeyError as error:
            raise UnknownNavigationTargetError(
                "Unknown navigation target_id: "
                f"{target_id!r}.\n"
                f"{_available_target_ids_message(self.list_target_ids())}"
            ) from error

    @staticmethod
    def _load_json(config_path: Path) -> dict[str, Any]:
        if not config_path.is_file():
            raise NavigationConfigError(
                f"Navigation target config file not found: {config_path}"
            )

        try:
            with config_path.open("r", encoding="utf-8") as file:
                data = json.load(file, object_pairs_hook=_reject_duplicate_keys)
        except json.JSONDecodeError as error:
            raise NavigationConfigError(
                "Navigation target config is not valid JSON: "
                f"{config_path}. {error}"
            ) from error

        if not isinstance(data, dict):
            raise NavigationConfigError(
                "Navigation target config root must be a JSON object."
            )

        return data

    def _parse_config(self, data: dict[str, Any]) -> dict[str, NavigationTarget]:
        schema_version = data.get("schema_version")
        if schema_version != SUPPORTED_SCHEMA_VERSION:
            raise NavigationConfigError(
                "Unsupported navigation target schema_version: "
                f"{schema_version!r}. Supported version: "
                f"{SUPPORTED_SCHEMA_VERSION}."
            )

        targets_data = data.get("targets")
        if not isinstance(targets_data, dict):
            raise NavigationConfigError(
                "Navigation target config field 'targets' must be an object."
            )

        targets: dict[str, NavigationTarget] = {}
        for target_id in sorted(targets_data):
            parsed_id = validate_target_id_format(target_id)
            target_data = targets_data[target_id]
            if not isinstance(target_data, dict):
                raise NavigationConfigError(
                    f"Target {target_id!r} must be a JSON object."
                )
            targets[parsed_id] = self._parse_target(parsed_id, target_data)

        return targets

    def _parse_target(
        self,
        target_id: str,
        data: dict[str, Any],
    ) -> NavigationTarget:
        display_name = self._required_string(data, "display_name", target_id)
        aliases = self._parse_aliases(data.get("aliases"), target_id)
        target_type = self._required_string(data, "type", target_id)
        description = self._required_string(data, "description", target_id)
        enabled = data.get("enabled")
        if not isinstance(enabled, bool):
            raise NavigationConfigError(
                f"Target {target_id!r} field 'enabled' must be a boolean."
            )

        goal_poses_data = data.get("goal_poses")
        if not isinstance(goal_poses_data, list):
            raise NavigationConfigError(
                f"Target {target_id!r} field 'goal_poses' must be an array."
            )

        goal_poses = self._parse_goal_poses(target_id, goal_poses_data)

        return NavigationTarget(
            target_id=target_id,
            display_name=display_name,
            aliases=aliases,
            target_type=target_type,
            description=description,
            enabled=enabled,
            goal_poses=goal_poses,
        )

    @staticmethod
    def _required_string(
        data: dict[str, Any],
        field_name: str,
        target_id: str,
    ) -> str:
        value = data.get(field_name)
        if not isinstance(value, str) or not value:
            raise NavigationConfigError(
                f"Target {target_id!r} field {field_name!r} "
                "must be a non-empty string."
            )
        return value

    @staticmethod
    def _parse_aliases(value: Any, target_id: str) -> tuple[str, ...]:
        if not isinstance(value, list):
            raise NavigationConfigError(
                f"Target {target_id!r} field 'aliases' must be an array."
            )

        aliases: list[str] = []
        for alias in value:
            if not isinstance(alias, str):
                raise NavigationConfigError(
                    f"Target {target_id!r} aliases must contain only strings."
                )
            aliases.append(alias)

        return tuple(aliases)

    def _parse_goal_poses(
        self,
        target_id: str,
        goal_poses_data: list[Any],
    ) -> tuple[GoalPose, ...]:
        poses: list[GoalPose] = []
        seen_pose_ids: set[str] = set()

        for index, pose_data in enumerate(goal_poses_data):
            if not isinstance(pose_data, dict):
                raise NavigationConfigError(
                    f"Target {target_id!r} goal_poses[{index}] "
                    "must be an object."
                )

            pose_id = self._required_pose_id(target_id, index, pose_data)
            if pose_id in seen_pose_ids:
                raise NavigationConfigError(
                    f"Target {target_id!r} has duplicate pose_id: "
                    f"{pose_id!r}."
                )
            seen_pose_ids.add(pose_id)

            poses.append(
                GoalPose(
                    pose_id=pose_id,
                    x=self._required_number(target_id, pose_id, pose_data, "x"),
                    y=self._required_number(target_id, pose_id, pose_data, "y"),
                    yaw=self._required_number(target_id, pose_id, pose_data, "yaw"),
                    priority=self._parse_priority(target_id, pose_id, pose_data),
                )
            )

        return tuple(poses)

    @staticmethod
    def _required_pose_id(
        target_id: str,
        index: int,
        data: dict[str, Any],
    ) -> str:
        pose_id = data.get("pose_id")
        if not isinstance(pose_id, str) or not pose_id:
            raise NavigationConfigError(
                f"Target {target_id!r} goal_poses[{index}] field 'pose_id' "
                "must be a non-empty string."
            )
        return pose_id

    @staticmethod
    def _required_number(
        target_id: str,
        pose_id: str,
        data: dict[str, Any],
        field_name: str,
    ) -> float:
        value = data.get(field_name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise NavigationConfigError(
                f"Target {target_id!r} pose {pose_id!r} field "
                f"{field_name!r} must be a number."
            )

        numeric_value = float(value)
        if not math.isfinite(numeric_value):
            raise NavigationConfigError(
                f"Target {target_id!r} pose {pose_id!r} field "
                f"{field_name!r} must be finite."
            )

        return numeric_value

    @staticmethod
    def _parse_priority(
        target_id: str,
        pose_id: str,
        data: dict[str, Any],
    ) -> int:
        value = data.get("priority", 0)
        if isinstance(value, bool) or not isinstance(value, int):
            raise NavigationConfigError(
                f"Target {target_id!r} pose {pose_id!r} field 'priority' "
                "must be an integer."
            )
        return value
