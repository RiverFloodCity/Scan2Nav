"""Pose math helpers shared by Script Editor tools and tests."""

import math


def normalize_yaw(yaw: float) -> float:
    """Normalize yaw to the range [-pi, pi)."""
    yaw = float(yaw)
    if not math.isfinite(yaw):
        raise ValueError("yaw must be a finite number.")

    normalized = (yaw + math.pi) % (2.0 * math.pi) - math.pi
    if normalized >= math.pi:
        normalized -= 2.0 * math.pi
    return normalized


def quaternion_wxyz_to_yaw(
    w: float,
    x: float,
    y: float,
    z: float,
) -> float:
    """Convert a wxyz quaternion to normalized world-Z yaw in radians."""
    w = float(w)
    x = float(x)
    y = float(y)
    z = float(z)
    if not all(math.isfinite(value) for value in (w, x, y, z)):
        raise ValueError("quaternion values must be finite numbers.")

    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return normalize_yaw(yaw)
