"""Navigation command and target registry exceptions."""


class NavigationError(Exception):
    """Base class for navigation interface errors."""


class NavigationConfigError(NavigationError):
    """Raised when the navigation target configuration is invalid."""


class InvalidNavigationCommandError(NavigationError):
    """Raised when a command object has the wrong type."""


class UnsupportedNavigationActionError(NavigationError):
    """Raised when a command action is not supported."""


class InvalidTargetIdError(NavigationError):
    """Raised when a target_id is not a stable machine-readable id."""


class UnknownNavigationTargetError(NavigationError):
    """Raised when a target_id is well-formed but not registered."""


class DisabledNavigationTargetError(NavigationError):
    """Raised when a target exists but is disabled."""


class NavigationTargetNotReadyError(NavigationError):
    """Raised when a target is enabled but has no calibrated goal pose."""


class NavigationPlannerConfigError(NavigationError):
    """Raised when the navigation planner configuration is invalid."""


class NavigationFollowerConfigError(NavigationError):
    """Raised when the navigation follower configuration is invalid."""


class StartOutsideMapError(NavigationError):
    """Raised when the requested start is outside the map."""


class GoalOutsideMapError(NavigationError):
    """Raised when the requested goal is outside the map."""


class StartCellBlockedError(NavigationError):
    """Raised when the requested start cell is blocked."""


class GoalCellBlockedError(NavigationError):
    """Raised when the requested goal cell is blocked."""


class NoPathFoundError(NavigationError):
    """Raised when A* cannot find a path."""


class PlanningLimitExceededError(NavigationError):
    """Raised when A* exceeds the configured expansion limit."""


class NavigationExecutionError(NavigationError):
    """Raised when automatic navigation execution cannot continue."""


class PathSimplificationError(NavigationError):
    """Raised when raw A* path simplification fails."""


class EmptyPathError(PathSimplificationError):
    """Raised when a path has no grid cells."""


class InvalidPathCellError(PathSimplificationError):
    """Raised when a raw path contains a blocked or outside cell."""


class InvalidSimplifiedSegmentError(PathSimplificationError):
    """Raised when a simplified segment fails line-of-sight validation."""
