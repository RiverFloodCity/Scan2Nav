"""Validate strict structured navigation commands."""

from __future__ import annotations

from .exceptions import (
    DisabledNavigationTargetError,
    InvalidNavigationCommandError,
    NavigationTargetNotReadyError,
    UnsupportedNavigationActionError,
)
from .models import NavigationCommand, NavigationTarget
from .target_registry import TargetRegistry, validate_target_id_format


SUPPORTED_ACTION = "navigate"


class CommandValidator:
    """Validate NavigationCommand objects against a TargetRegistry."""

    def __init__(self, registry: TargetRegistry | None = None):
        self.registry = registry if registry is not None else TargetRegistry()

    def validate(self, command: NavigationCommand) -> NavigationTarget:
        """Validate a command and return the resolved target."""
        if not isinstance(command, NavigationCommand):
            raise InvalidNavigationCommandError(
                "command must be an instance of NavigationCommand."
            )

        if command.action != SUPPORTED_ACTION:
            raise UnsupportedNavigationActionError(
                "Unsupported navigation action: "
                f"{command.action!r}. Supported action: {SUPPORTED_ACTION!r}."
            )

        target_id = validate_target_id_format(command.target_id)
        target = self.registry.get_target(target_id)

        if not target.enabled:
            raise DisabledNavigationTargetError(
                "Navigation target is registered but disabled: "
                f"{target.target_id!r}. Fill real goal_poses and set "
                "'enabled' to true before executing navigation."
            )

        if not target.goal_poses:
            raise NavigationTargetNotReadyError(
                "Navigation target is enabled but has no calibrated "
                f"goal_poses: {target.target_id!r}."
            )

        return target
