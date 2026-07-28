import math
from pathlib import Path
import struct
import tempfile
import unittest
import zlib

from src.navigation import OccupancyGridMap
from src.navigation.exceptions import NavigationConfigError


def write_png_rgba(path: Path, rows):
    height = len(rows)
    width = len(rows[0])
    raw_rows = []
    for row in rows:
        scanline = bytearray()
        for red, green, blue, alpha in row:
            scanline.extend((red, green, blue, alpha))
        raw_rows.append(b"\x00" + bytes(scanline))

    def chunk(chunk_type, data):
        crc = zlib.crc32(chunk_type + data) & 0xFFFFFFFF
        return (
            struct.pack(">I", len(data))
            + chunk_type
            + data
            + struct.pack(">I", crc)
        )

    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(
            b"IHDR",
            struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0),
        )
        + chunk(b"IDAT", zlib.compress(b"".join(raw_rows)))
        + chunk(b"IEND", b"")
    )
    path.write_bytes(png)


def write_map(directory, *, image_name="map.png", negate=0, origin=None, rows=None):
    if origin is None:
        origin = [0.0, 0.0, 0.0]
    if rows is None:
        rows = [
            [(255, 255, 255, 255), (0, 0, 0, 255)],
            [(128, 128, 128, 255), (255, 255, 255, 0)],
        ]

    directory = Path(directory)
    write_png_rgba(directory / image_name, rows)
    yaml_path = directory / "map.yaml"
    yaml_path.write_text(
        "\n".join(
            [
                f"image: {image_name}",
                "resolution: 1.0",
                f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]",
                f"negate: {negate}",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
            ]
        ),
        encoding="utf-8",
    )
    return yaml_path


class OccupancyGridMapTests(unittest.TestCase):
    def load_map(self, **kwargs):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        yaml_path = write_map(self.temp_dir.name, **kwargs)
        return OccupancyGridMap.from_yaml(yaml_path)

    def test_yaml_loads_metadata(self):
        grid_map = self.load_map()
        metadata = grid_map.metadata
        self.assertEqual(metadata.width, 2)
        self.assertEqual(metadata.height, 2)
        self.assertEqual(metadata.resolution, 1.0)
        self.assertEqual(metadata.origin_x, 0.0)
        self.assertEqual(metadata.origin_y, 0.0)
        self.assertEqual(metadata.origin_yaw, 0.0)

    def test_relative_image_path_is_resolved_from_yaml_directory(self):
        grid_map = self.load_map(image_name="relative_map.png")
        self.assertEqual(grid_map.metadata.image_path.name, "relative_map.png")
        self.assertTrue(grid_map.metadata.image_path.is_absolute())

    def test_world_to_grid(self):
        grid_map = self.load_map()
        self.assertEqual(grid_map.world_to_grid(0.5, 0.5), (1, 0))
        self.assertEqual(grid_map.world_to_grid(1.5, 1.5), (0, 1))

    def test_grid_to_world_uses_pixel_center(self):
        grid_map = self.load_map()
        self.assertEqual(grid_map.grid_to_world(1, 0), (0.5, 0.5))
        self.assertEqual(grid_map.grid_to_world(0, 1), (1.5, 1.5))

    def test_y_axis_flip_is_correct(self):
        grid_map = self.load_map()
        bottom_left = grid_map.grid_to_world(1, 0)
        top_left = grid_map.grid_to_world(0, 0)
        self.assertLess(bottom_left[1], top_left[1])

    def test_map_boundaries(self):
        grid_map = self.load_map()
        self.assertTrue(grid_map.is_inside(0, 0))
        self.assertTrue(grid_map.is_inside(1, 1))
        self.assertFalse(grid_map.is_inside(-1, 0))
        self.assertFalse(grid_map.is_inside(2, 0))
        self.assertFalse(grid_map.is_inside(0, 2))

    def test_world_point_outside_map_raises(self):
        grid_map = self.load_map()
        with self.assertRaises(ValueError):
            grid_map.world_to_grid(-0.1, 0.5)
        with self.assertRaises(ValueError):
            grid_map.world_to_grid(2.0, 0.5)

    def test_origin_nonzero(self):
        grid_map = self.load_map(origin=[10.0, 20.0, 0.0])
        self.assertEqual(grid_map.world_to_grid(10.5, 20.5), (1, 0))
        self.assertEqual(grid_map.grid_to_world(1, 0), (10.5, 20.5))

    def test_origin_yaw_nonzero(self):
        grid_map = self.load_map(origin=[0.0, 0.0, math.pi / 2.0])
        world = grid_map.grid_to_world(1, 0)
        self.assertAlmostEqual(world[0], -0.5)
        self.assertAlmostEqual(world[1], 0.5)
        self.assertEqual(grid_map.world_to_grid(*world), (1, 0))

    def test_negate_zero_classification(self):
        grid_map = self.load_map(negate=0)
        self.assertTrue(grid_map.is_free(0, 0))
        self.assertTrue(grid_map.is_occupied(0, 1))

    def test_negate_one_classification(self):
        grid_map = self.load_map(negate=1)
        self.assertTrue(grid_map.is_occupied(0, 0))
        self.assertTrue(grid_map.is_free(0, 1))

    def test_occupied_free_unknown_classification(self):
        grid_map = self.load_map(negate=0)
        self.assertTrue(grid_map.is_free(0, 0))
        self.assertTrue(grid_map.is_occupied(0, 1))
        self.assertTrue(grid_map.is_unknown(1, 0))
        self.assertTrue(grid_map.is_unknown(1, 1))
        self.assertIsNone(grid_map.occupancy_value(1, 1))

    def test_round_trip_error_within_half_pixel(self):
        grid_map = self.load_map(origin=[3.0, -2.0, 0.3])
        half_pixel = grid_map.metadata.resolution * 0.5
        for row in range(grid_map.metadata.height):
            for col in range(grid_map.metadata.width):
                world = grid_map.grid_to_world(row, col)
                round_trip = grid_map.world_to_grid(*world)
                self.assertEqual(round_trip, (row, col))
                center = grid_map.grid_to_world(*round_trip)
                error = math.hypot(world[0] - center[0], world[1] - center[1])
                self.assertLessEqual(error, half_pixel)

    def test_missing_relative_image_raises_config_error(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            yaml_path = Path(temp_dir) / "map.yaml"
            yaml_path.write_text(
                "\n".join(
                    [
                        "image: missing.png",
                        "resolution: 1.0",
                        "origin: [0.0, 0.0, 0.0]",
                        "negate: 0",
                        "occupied_thresh: 0.65",
                        "free_thresh: 0.196",
                    ]
                ),
                encoding="utf-8",
            )
            with self.assertRaises(NavigationConfigError):
                OccupancyGridMap.from_yaml(yaml_path)


if __name__ == "__main__":
    unittest.main()
