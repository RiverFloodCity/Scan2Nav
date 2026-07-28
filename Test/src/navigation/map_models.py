"""Data models for 2D occupancy grid maps."""

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class MapMetadata:
    """Metadata loaded from a ROS-style occupancy map YAML file."""

    image_path: Path
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    width: int
    height: int
    negate: int
    occupied_thresh: float
    free_thresh: float
