"""ROS-style occupancy grid map loading and coordinate conversion."""

from __future__ import annotations

import ast
import math
from pathlib import Path
import struct
from typing import Any
import zlib

from .exceptions import NavigationConfigError
from .map_models import MapMetadata


PROJECT_ROOT = Path("/home/ps/ScanEverything/Test")
DEFAULT_MAP_YAML_PATH = (
    PROJECT_ROOT / "config" / "maps" / "final_scene_occupancy.yaml"
)
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


class OccupancyGridMap:
    """A ROS map_server style 2D occupancy map."""

    def __init__(
        self,
        metadata: MapMetadata,
        occupancy_probabilities: tuple[tuple[float, ...], ...],
        alpha_unknown: tuple[tuple[bool, ...], ...],
    ):
        self.metadata = metadata
        self._occupancy_probabilities = occupancy_probabilities
        self._alpha_unknown = alpha_unknown

    @classmethod
    def from_yaml(cls, yaml_path: str | Path = DEFAULT_MAP_YAML_PATH):
        """Load an occupancy grid map from a ROS-style YAML file."""
        yaml_path = Path(yaml_path)
        yaml_data = _load_simple_yaml(yaml_path)

        image_value = _required_string(yaml_data, "image")
        image_path = Path(image_value)
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        image_path = image_path.resolve()

        resolution = _required_positive_float(yaml_data, "resolution")
        origin = _required_origin(yaml_data)
        negate = _required_negate(yaml_data)
        occupied_thresh = _required_threshold(yaml_data, "occupied_thresh")
        free_thresh = _required_threshold(yaml_data, "free_thresh")
        if not free_thresh < occupied_thresh:
            raise NavigationConfigError(
                "Map free_thresh must be smaller than occupied_thresh."
            )

        width, height, pixels = _read_png_rgba(image_path)
        probabilities, alpha_unknown = _build_occupancy_tables(
            pixels=pixels,
            negate=negate,
        )

        metadata = MapMetadata(
            image_path=image_path,
            resolution=resolution,
            origin_x=origin[0],
            origin_y=origin[1],
            origin_yaw=origin[2],
            width=width,
            height=height,
            negate=negate,
            occupied_thresh=occupied_thresh,
            free_thresh=free_thresh,
        )
        return cls(metadata, probabilities, alpha_unknown)

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert world x/y meters to image row/col indices."""
        x = float(x)
        y = float(y)
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("World coordinates must be finite numbers.")

        dx = x - self.metadata.origin_x
        dy = y - self.metadata.origin_y
        cos_yaw = math.cos(self.metadata.origin_yaw)
        sin_yaw = math.sin(self.metadata.origin_yaw)

        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy

        map_width = self.metadata.width * self.metadata.resolution
        map_height = self.metadata.height * self.metadata.resolution
        epsilon = 1e-9
        if (
            local_x < -epsilon
            or local_y < -epsilon
            or local_x >= map_width - epsilon
            or local_y >= map_height - epsilon
        ):
            raise ValueError(
                "World point is outside occupancy map bounds: "
                f"x={x}, y={y}."
            )

        local_x = max(0.0, local_x)
        local_y = max(0.0, local_y)
        col = int(math.floor(local_x / self.metadata.resolution))
        bottom_row = int(math.floor(local_y / self.metadata.resolution))
        row = self.metadata.height - 1 - bottom_row

        if not self.is_inside(row, col):
            raise ValueError(
                "World point converted outside occupancy map indices: "
                f"x={x}, y={y}, row={row}, col={col}."
            )

        return row, col

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        """Convert image row/col indices to world x/y at the cell center."""
        self._require_inside(row, col)

        local_x = (col + 0.5) * self.metadata.resolution
        local_y = (
            self.metadata.height - row - 0.5
        ) * self.metadata.resolution

        cos_yaw = math.cos(self.metadata.origin_yaw)
        sin_yaw = math.sin(self.metadata.origin_yaw)
        world_x = self.metadata.origin_x + cos_yaw * local_x - sin_yaw * local_y
        world_y = self.metadata.origin_y + sin_yaw * local_x + cos_yaw * local_y
        return world_x, world_y

    def is_inside(self, row: int, col: int) -> bool:
        """Return True when row/col are inside the map."""
        return (
            isinstance(row, int)
            and isinstance(col, int)
            and 0 <= row < self.metadata.height
            and 0 <= col < self.metadata.width
        )

    def occupancy_value(self, row: int, col: int) -> float | None:
        """Return occupancy probability, or None for alpha-marked unknown."""
        self._require_inside(row, col)
        if self._alpha_unknown[row][col]:
            return None
        return self._occupancy_probabilities[row][col]

    def is_occupied(self, row: int, col: int) -> bool:
        """Return True when occupancy probability exceeds occupied_thresh."""
        value = self.occupancy_value(row, col)
        return (
            value is not None
            and value > self.metadata.occupied_thresh
        )

    def is_free(self, row: int, col: int) -> bool:
        """Return True when occupancy probability is below free_thresh."""
        value = self.occupancy_value(row, col)
        return value is not None and value < self.metadata.free_thresh

    def is_unknown(self, row: int, col: int) -> bool:
        """Return True when a cell is neither free nor occupied."""
        self._require_inside(row, col)
        return not self.is_free(row, col) and not self.is_occupied(row, col)

    def status_at(self, row: int, col: int) -> str:
        """Return a printable status for a valid cell."""
        if self.is_free(row, col):
            return "free"
        if self.is_occupied(row, col):
            return "occupied"
        return "unknown"

    def world_corners(self) -> tuple[tuple[float, float], ...]:
        """Return world coordinates for the four map outer corners."""
        width = self.metadata.width * self.metadata.resolution
        height = self.metadata.height * self.metadata.resolution
        corners = (
            (0.0, 0.0),
            (width, 0.0),
            (width, height),
            (0.0, height),
        )
        cos_yaw = math.cos(self.metadata.origin_yaw)
        sin_yaw = math.sin(self.metadata.origin_yaw)
        return tuple(
            (
                self.metadata.origin_x + cos_yaw * x - sin_yaw * y,
                self.metadata.origin_y + sin_yaw * x + cos_yaw * y,
            )
            for x, y in corners
        )

    def world_bounds(self) -> tuple[float, float, float, float]:
        """Return axis-aligned world bounds as min_x, min_y, max_x, max_y."""
        corners = self.world_corners()
        xs = [corner[0] for corner in corners]
        ys = [corner[1] for corner in corners]
        return min(xs), min(ys), max(xs), max(ys)

    def _require_inside(self, row: int, col: int) -> None:
        if not self.is_inside(row, col):
            raise IndexError(
                "Grid index is outside occupancy map: "
                f"row={row}, col={col}, "
                f"height={self.metadata.height}, width={self.metadata.width}."
            )


def _load_simple_yaml(yaml_path: Path) -> dict[str, Any]:
    if not yaml_path.is_file():
        raise NavigationConfigError(f"Map YAML file not found: {yaml_path}")

    data: dict[str, Any] = {}
    for line_number, raw_line in enumerate(
        yaml_path.read_text(encoding="utf-8").splitlines(),
        start=1,
    ):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        if ":" not in line:
            raise NavigationConfigError(
                f"Invalid YAML line {line_number}: {raw_line!r}."
            )
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        if not key:
            raise NavigationConfigError(
                f"Invalid YAML key at line {line_number}."
            )
        if key in data:
            raise NavigationConfigError(f"Duplicate YAML key: {key!r}.")
        data[key] = _parse_yaml_scalar(value)

    return data


def _parse_yaml_scalar(value: str) -> Any:
    if value == "":
        return ""
    if value[0] in "[{\"'" or value in ("true", "false", "True", "False"):
        try:
            return ast.literal_eval(value)
        except (SyntaxError, ValueError):
            if value.lower() == "true":
                return True
            if value.lower() == "false":
                return False
            raise
    try:
        if any(marker in value for marker in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value.strip("\"'")


def _required_string(data: dict[str, Any], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value:
        raise NavigationConfigError(f"Map YAML field {key!r} must be a string.")
    return value


def _required_positive_float(data: dict[str, Any], key: str) -> float:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise NavigationConfigError(f"Map YAML field {key!r} must be a number.")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise NavigationConfigError(
            f"Map YAML field {key!r} must be a positive finite number."
        )
    return result


def _required_threshold(data: dict[str, Any], key: str) -> float:
    value = data.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise NavigationConfigError(f"Map YAML field {key!r} must be a number.")
    result = float(value)
    if not 0.0 <= result <= 1.0:
        raise NavigationConfigError(
            f"Map YAML field {key!r} must be between 0.0 and 1.0."
        )
    return result


def _required_origin(data: dict[str, Any]) -> tuple[float, float, float]:
    value = data.get("origin")
    if not isinstance(value, list) or len(value) != 3:
        raise NavigationConfigError(
            "Map YAML field 'origin' must be a list of [x, y, yaw]."
        )

    origin: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise NavigationConfigError(
                "Map YAML origin values must be numeric."
            )
        numeric_value = float(item)
        if not math.isfinite(numeric_value):
            raise NavigationConfigError(
                "Map YAML origin values must be finite."
            )
        origin.append(numeric_value)

    return origin[0], origin[1], origin[2]


def _required_negate(data: dict[str, Any]) -> int:
    value = data.get("negate")
    if isinstance(value, bool):
        return int(value)
    if value not in (0, 1):
        raise NavigationConfigError("Map YAML field 'negate' must be 0 or 1.")
    return int(value)


def _read_png_rgba(
    image_path: Path,
) -> tuple[int, int, tuple[tuple[tuple[int, int, int, int], ...], ...]]:
    if not image_path.is_file():
        raise NavigationConfigError(f"Map image file not found: {image_path}")

    data = image_path.read_bytes()
    if not data.startswith(PNG_SIGNATURE):
        raise NavigationConfigError(f"Map image is not a PNG file: {image_path}")

    offset = len(PNG_SIGNATURE)
    width = height = bit_depth = color_type = interlace = None
    idat_parts: list[bytes] = []
    palette: list[tuple[int, int, int]] = []
    palette_alpha: list[int] = []

    while offset < len(data):
        if offset + 8 > len(data):
            raise NavigationConfigError(f"Truncated PNG chunk in {image_path}.")
        chunk_length = struct.unpack(">I", data[offset:offset + 4])[0]
        chunk_type = data[offset + 4:offset + 8]
        chunk_start = offset + 8
        chunk_end = chunk_start + chunk_length
        if chunk_end + 4 > len(data):
            raise NavigationConfigError(f"Truncated PNG chunk in {image_path}.")
        chunk_data = data[chunk_start:chunk_end]
        offset = chunk_end + 4

        if chunk_type == b"IHDR":
            width, height, bit_depth, color_type, _, _, interlace = struct.unpack(
                ">IIBBBBB",
                chunk_data,
            )
        elif chunk_type == b"PLTE":
            palette = [
                tuple(chunk_data[index:index + 3])
                for index in range(0, len(chunk_data), 3)
            ]
        elif chunk_type == b"tRNS":
            palette_alpha = list(chunk_data)
        elif chunk_type == b"IDAT":
            idat_parts.append(chunk_data)
        elif chunk_type == b"IEND":
            break

    if width is None or height is None or bit_depth is None or color_type is None:
        raise NavigationConfigError(f"PNG is missing IHDR: {image_path}")
    if bit_depth != 8:
        raise NavigationConfigError(
            f"Only 8-bit PNG maps are supported, got bit_depth={bit_depth}."
        )
    if interlace != 0:
        raise NavigationConfigError("Interlaced PNG maps are not supported.")

    channels_by_type = {
        0: 1,
        2: 3,
        3: 1,
        4: 2,
        6: 4,
    }
    if color_type not in channels_by_type:
        raise NavigationConfigError(
            f"Unsupported PNG color_type for map: {color_type}."
        )

    channels = channels_by_type[color_type]
    row_bytes = width * channels
    raw = zlib.decompress(b"".join(idat_parts))
    expected = height * (1 + row_bytes)
    if len(raw) != expected:
        raise NavigationConfigError(
            "Unexpected PNG decompressed size: "
            f"expected {expected}, got {len(raw)}."
        )

    rows: list[bytes] = []
    previous = bytes(row_bytes)
    cursor = 0
    for _ in range(height):
        filter_type = raw[cursor]
        cursor += 1
        scanline = raw[cursor:cursor + row_bytes]
        cursor += row_bytes
        unfiltered = _unfilter_scanline(
            filter_type=filter_type,
            scanline=scanline,
            previous=previous,
            bytes_per_pixel=channels,
        )
        rows.append(unfiltered)
        previous = unfiltered

    rgba_rows = tuple(
        tuple(
            _pixel_to_rgba(
                row=row,
                col=col,
                channels=channels,
                color_type=color_type,
                palette=palette,
                palette_alpha=palette_alpha,
            )
            for col in range(width)
        )
        for row in rows
    )

    return width, height, rgba_rows


def _unfilter_scanline(
    *,
    filter_type: int,
    scanline: bytes,
    previous: bytes,
    bytes_per_pixel: int,
) -> bytes:
    result = bytearray(scanline)
    for index, value in enumerate(scanline):
        left = result[index - bytes_per_pixel] if index >= bytes_per_pixel else 0
        up = previous[index]
        upper_left = (
            previous[index - bytes_per_pixel]
            if index >= bytes_per_pixel
            else 0
        )

        if filter_type == 0:
            reconstructed = value
        elif filter_type == 1:
            reconstructed = value + left
        elif filter_type == 2:
            reconstructed = value + up
        elif filter_type == 3:
            reconstructed = value + ((left + up) // 2)
        elif filter_type == 4:
            reconstructed = value + _paeth(left, up, upper_left)
        else:
            raise NavigationConfigError(
                f"Unsupported PNG filter type: {filter_type}."
            )
        result[index] = reconstructed & 0xFF
    return bytes(result)


def _paeth(left: int, up: int, upper_left: int) -> int:
    estimate = left + up - upper_left
    left_distance = abs(estimate - left)
    up_distance = abs(estimate - up)
    upper_left_distance = abs(estimate - upper_left)
    if left_distance <= up_distance and left_distance <= upper_left_distance:
        return left
    if up_distance <= upper_left_distance:
        return up
    return upper_left


def _pixel_to_rgba(
    *,
    row: bytes,
    col: int,
    channels: int,
    color_type: int,
    palette: list[tuple[int, int, int]],
    palette_alpha: list[int],
) -> tuple[int, int, int, int]:
    offset = col * channels
    if color_type == 0:
        gray = row[offset]
        return gray, gray, gray, 255
    if color_type == 2:
        return row[offset], row[offset + 1], row[offset + 2], 255
    if color_type == 3:
        palette_index = row[offset]
        try:
            red, green, blue = palette[palette_index]
        except IndexError as error:
            raise NavigationConfigError(
                f"PNG palette index out of range: {palette_index}."
            ) from error
        alpha = palette_alpha[palette_index] if palette_index < len(palette_alpha) else 255
        return red, green, blue, alpha
    if color_type == 4:
        gray = row[offset]
        alpha = row[offset + 1]
        return gray, gray, gray, alpha
    if color_type == 6:
        return row[offset], row[offset + 1], row[offset + 2], row[offset + 3]
    raise NavigationConfigError(f"Unsupported PNG color type: {color_type}.")


def _build_occupancy_tables(
    *,
    pixels: tuple[tuple[tuple[int, int, int, int], ...], ...],
    negate: int,
) -> tuple[tuple[tuple[float, ...], ...], tuple[tuple[bool, ...], ...]]:
    probability_rows: list[tuple[float, ...]] = []
    alpha_rows: list[tuple[bool, ...]] = []
    for row in pixels:
        probabilities: list[float] = []
        alpha_unknown: list[bool] = []
        for red, green, blue, alpha in row:
            color_average = (red + green + blue) / 3.0
            if negate:
                probability = color_average / 255.0
            else:
                probability = (255.0 - color_average) / 255.0
            probabilities.append(probability)
            alpha_unknown.append(alpha < 255)
        probability_rows.append(tuple(probabilities))
        alpha_rows.append(tuple(alpha_unknown))
    return tuple(probability_rows), tuple(alpha_rows)
