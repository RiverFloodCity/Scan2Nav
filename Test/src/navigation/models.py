"""Dataclasses for the strict navigation command interface."""

from dataclasses import dataclass


@dataclass(frozen=True)
class GoalPose:
    """A calibrated candidate docking pose in Isaac Sim world coordinates."""

    pose_id: str
    x: float
    y: float
    yaw: float
    priority: int = 0


@dataclass(frozen=True)
class NavigationTarget:
    """A registered semantic navigation target."""

    target_id: str
    display_name: str
    aliases: tuple[str, ...]
    target_type: str
    description: str
    enabled: bool
    goal_poses: tuple[GoalPose, ...]


@dataclass(frozen=True)
class NavigationCommand:
    """A structured request to navigate to a registered target_id."""

    action: str
    target_id: str
    request_id: str | None = None
