import math
from pathlib import Path
import unittest

from src.navigation import (
    EmptyPathError,
    GridCell,
    InvalidPathCellError,
    InvalidSimplifiedSegmentError,
    PathSimplifier,
    PlannedPath,
    PlannerConfig,
    WorldPoint,
)
from src.navigation.grid_inflation import build_planning_grid
from src.navigation.map_models import MapMetadata
from src.navigation.occupancy_grid import OccupancyGridMap


def make_map(rows: list[str], *, resolution: float = 1.0) -> OccupancyGridMap:
    probabilities = []
    alpha_unknown = []
    for row in rows:
        probability_row = []
        unknown_row = []
        for value in row:
            if value == ".":
                probability_row.append(0.0)
                unknown_row.append(False)
            elif value == "#":
                probability_row.append(1.0)
                unknown_row.append(False)
            elif value == "?":
                probability_row.append(0.5)
                unknown_row.append(False)
            else:
                raise ValueError(f"Unsupported map value: {value!r}")
        probabilities.append(tuple(probability_row))
        alpha_unknown.append(tuple(unknown_row))

    metadata = MapMetadata(
        image_path=Path("/tmp/test_map.png"),
        resolution=resolution,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        width=len(rows[0]),
        height=len(rows),
        negate=0,
        occupied_thresh=0.65,
        free_thresh=0.196,
    )
    return OccupancyGridMap(
        metadata=metadata,
        occupancy_probabilities=tuple(probabilities),
        alpha_unknown=tuple(alpha_unknown),
    )


def config(**overrides) -> PlannerConfig:
    values = {
        "inflation_radius_m": 0.0,
        "treat_unknown_as_occupied": True,
        "allow_diagonal": True,
        "prevent_corner_cutting": True,
        "max_expanded_nodes": 10000,
    }
    values.update(overrides)
    return PlannerConfig(**values)


def make_simplifier(rows: list[str], **config_overrides):
    occupancy_map = make_map(rows)
    planner_config = config(**config_overrides)
    planning_grid = build_planning_grid(occupancy_map, planner_config)
    simplifier = PathSimplifier(
        traversable_mask=planning_grid.traversable,
        prevent_corner_cutting=planner_config.prevent_corner_cutting,
    )
    return occupancy_map, simplifier


def make_planned_path(
    occupancy_map: OccupancyGridMap,
    cells: list[GridCell],
    *,
    exact_start_world: tuple[float, float] | None = None,
    exact_goal_world: tuple[float, float] | None = None,
) -> PlannedPath:
    grid_path = tuple(cells)
    world_points = [
        WorldPoint(*occupancy_map.grid_to_world(cell.row, cell.col))
        for cell in grid_path
    ]
    if exact_start_world is not None:
        world_points[0] = WorldPoint(*exact_start_world)
    if exact_goal_world is not None and len(world_points) > 1:
        world_points[-1] = WorldPoint(*exact_goal_world)
    world_path = tuple(world_points)
    return PlannedPath(
        target_id="target",
        pose_id="front",
        start_cell=grid_path[0] if grid_path else GridCell(0, 0),
        goal_cell=grid_path[-1] if grid_path else GridCell(0, 0),
        grid_path=grid_path,
        world_path=world_path,
        goal_yaw=1.25,
        grid_cost=0.0,
        length_m=path_length(world_path),
        expanded_nodes=1,
    )


def path_length(world_path: tuple[WorldPoint, ...]) -> float:
    total = 0.0
    for first, second in zip(world_path, world_path[1:]):
        total += math.hypot(second.x - first.x, second.y - first.y)
    return total


class PathSimplifierTests(unittest.TestCase):
    def test_single_node_path(self):
        occupancy_map, simplifier = make_simplifier(["."])
        planned = make_planned_path(occupancy_map, [GridCell(0, 0)])
        result = simplifier.simplify(planned, occupancy_map=occupancy_map)
        self.assertEqual(result.simplified_grid_path, (GridCell(0, 0),))
        self.assertEqual(result.simplified_node_count, 1)

    def test_two_node_path(self):
        occupancy_map, simplifier = make_simplifier([".."])
        planned = make_planned_path(
            occupancy_map,
            [GridCell(0, 0), GridCell(0, 1)],
        )
        result = simplifier.simplify(planned, occupancy_map=occupancy_map)
        self.assertEqual(result.simplified_grid_path, planned.grid_path)

    def test_straight_path_removes_middle_nodes(self):
        occupancy_map, simplifier = make_simplifier(["....."])
        cells = [GridCell(0, col) for col in range(5)]
        planned = make_planned_path(occupancy_map, cells)
        result = simplifier.simplify(planned, occupancy_map=occupancy_map)
        self.assertEqual(
            result.simplified_grid_path,
            (GridCell(0, 0), GridCell(0, 4)),
        )

    def test_open_space_polyline_simplifies_to_endpoints(self):
        occupancy_map, simplifier = make_simplifier(
            [
                ".....",
                ".....",
                ".....",
                ".....",
                ".....",
            ]
        )
        cells = [
            GridCell(4, 0),
            GridCell(4, 1),
            GridCell(3, 2),
            GridCell(2, 3),
            GridCell(1, 4),
            GridCell(0, 4),
        ]
        planned = make_planned_path(occupancy_map, cells)
        result = simplifier.simplify(planned, occupancy_map=occupancy_map)
        self.assertEqual(
            result.simplified_grid_path,
            (GridCell(4, 0), GridCell(0, 4)),
        )

    def test_obstacle_retains_needed_turn(self):
        occupancy_map, simplifier = make_simplifier(
            [
                ".....",
                "..#..",
                ".....",
            ]
        )
        cells = [
            GridCell(1, 0),
            GridCell(0, 1),
            GridCell(0, 2),
            GridCell(0, 3),
            GridCell(1, 4),
        ]
        planned = make_planned_path(occupancy_map, cells)
        result = simplifier.simplify(planned, occupancy_map=occupancy_map)
        self.assertGreater(result.simplified_node_count, 2)
        for start, end in zip(
            result.simplified_grid_path,
            result.simplified_grid_path[1:],
        ):
            self.assertTrue(simplifier.has_line_of_sight(start, end))

    def test_simplified_path_does_not_cross_obstacle(self):
        _, simplifier = make_simplifier(["..#.."])
        self.assertFalse(
            simplifier.has_line_of_sight(GridCell(0, 0), GridCell(0, 4))
        )

    def test_simplified_path_does_not_enter_inflated_region(self):
        occupancy_map, simplifier = make_simplifier(
            ["..#.."],
            inflation_radius_m=1.0,
        )
        planned = make_planned_path(
            occupancy_map,
            [GridCell(0, 0), GridCell(0, 1)],
        )
        with self.assertRaises(InvalidPathCellError):
            simplifier.simplify(planned, occupancy_map=occupancy_map)

    def test_unknown_is_not_visible(self):
        _, simplifier = make_simplifier(["..?.."])
        self.assertFalse(
            simplifier.has_line_of_sight(GridCell(0, 0), GridCell(0, 4))
        )

    def test_horizontal_line_of_sight(self):
        _, simplifier = make_simplifier(["...."])
        self.assertTrue(
            simplifier.has_line_of_sight(GridCell(0, 0), GridCell(0, 3))
        )

    def test_vertical_line_of_sight(self):
        _, simplifier = make_simplifier([".", ".", ".", "."])
        self.assertTrue(
            simplifier.has_line_of_sight(GridCell(0, 0), GridCell(3, 0))
        )

    def test_diagonal_line_of_sight(self):
        _, simplifier = make_simplifier(["...", "...", "..."])
        self.assertTrue(
            simplifier.has_line_of_sight(GridCell(2, 0), GridCell(0, 2))
        )

    def test_arbitrary_slope_line_of_sight(self):
        _, simplifier = make_simplifier(
            [
                "......",
                "......",
                "......",
                "......",
            ]
        )
        self.assertTrue(
            simplifier.has_line_of_sight(GridCell(3, 0), GridCell(0, 5))
        )

    def test_prevent_corner_cutting_blocks_diagonal_gap(self):
        _, simplifier = make_simplifier(["#.", ".#"])
        self.assertFalse(
            simplifier.has_line_of_sight(GridCell(1, 0), GridCell(0, 1))
        )

    def test_corner_cutting_allowed_when_disabled(self):
        _, simplifier = make_simplifier(
            ["#.", ".#"],
            prevent_corner_cutting=False,
        )
        self.assertTrue(
            simplifier.has_line_of_sight(GridCell(1, 0), GridCell(0, 1))
        )

    def test_outside_line_of_sight_returns_false(self):
        _, simplifier = make_simplifier([".."])
        self.assertFalse(
            simplifier.has_line_of_sight(GridCell(0, 0), GridCell(0, 2))
        )

    def test_start_blocked_returns_false(self):
        _, simplifier = make_simplifier(["#.", ".."])
        self.assertFalse(
            simplifier.has_line_of_sight(GridCell(0, 0), GridCell(1, 1))
        )

    def test_goal_blocked_returns_false(self):
        _, simplifier = make_simplifier([".#", ".."])
        self.assertFalse(
            simplifier.has_line_of_sight(GridCell(1, 0), GridCell(0, 1))
        )

    def test_middle_path_node_blocked_raises(self):
        occupancy_map, simplifier = make_simplifier([".#."])
        planned = make_planned_path(
            occupancy_map,
            [GridCell(0, 0), GridCell(0, 1), GridCell(0, 2)],
        )
        with self.assertRaises(InvalidPathCellError):
            simplifier.simplify(planned, occupancy_map=occupancy_map)

    def test_empty_path_raises(self):
        occupancy_map, simplifier = make_simplifier(["."])
        planned = PlannedPath(
            target_id="target",
            pose_id="front",
            start_cell=GridCell(0, 0),
            goal_cell=GridCell(0, 0),
            grid_path=(),
            world_path=(),
            goal_yaw=0.0,
            grid_cost=0.0,
            length_m=0.0,
            expanded_nodes=0,
        )
        with self.assertRaises(EmptyPathError):
            simplifier.simplify(planned, occupancy_map=occupancy_map)

    def test_deterministic_result(self):
        occupancy_map, simplifier = make_simplifier([".....", "....."])
        cells = [
            GridCell(1, 0),
            GridCell(1, 1),
            GridCell(0, 2),
            GridCell(0, 3),
            GridCell(0, 4),
        ]
        planned = make_planned_path(occupancy_map, cells)
        first = simplifier.simplify(planned, occupancy_map=occupancy_map)
        second = simplifier.simplify(planned, occupancy_map=occupancy_map)
        self.assertEqual(first, second)

    def test_endpoints_match_original_path(self):
        occupancy_map, simplifier = make_simplifier(["....."])
        cells = [GridCell(0, col) for col in range(5)]
        planned = make_planned_path(occupancy_map, cells)
        result = simplifier.simplify(planned, occupancy_map=occupancy_map)
        self.assertEqual(result.simplified_grid_path[0], planned.grid_path[0])
        self.assertEqual(result.simplified_grid_path[-1], planned.grid_path[-1])

    def test_exact_world_endpoints_are_preserved(self):
        occupancy_map, simplifier = make_simplifier(["....."])
        planned = make_planned_path(
            occupancy_map,
            [GridCell(0, col) for col in range(5)],
            exact_start_world=(0.1, 0.2),
            exact_goal_world=(4.8, 0.7),
        )
        result = simplifier.simplify(planned, occupancy_map=occupancy_map)
        self.assertEqual(result.simplified_world_path[0], WorldPoint(0.1, 0.2))
        self.assertEqual(result.simplified_world_path[-1], WorldPoint(4.8, 0.7))

    def test_simplified_length_calculation(self):
        occupancy_map, simplifier = make_simplifier(["...", "...", "..."])
        planned = make_planned_path(
            occupancy_map,
            [GridCell(2, 0), GridCell(1, 1), GridCell(0, 2)],
        )
        result = simplifier.simplify(planned, occupancy_map=occupancy_map)
        self.assertAlmostEqual(result.simplified_length_m, math.sqrt(8.0))

    def test_simplified_length_not_greater_than_original(self):
        occupancy_map, simplifier = make_simplifier([".....", "....."])
        planned = make_planned_path(
            occupancy_map,
            [
                GridCell(1, 0),
                GridCell(1, 1),
                GridCell(0, 2),
                GridCell(0, 3),
                GridCell(0, 4),
            ],
        )
        result = simplifier.simplify(planned, occupancy_map=occupancy_map)
        self.assertLessEqual(
            result.simplified_length_m,
            result.original_length_m + 1e-9,
        )

    def test_each_simplified_segment_validates(self):
        occupancy_map, simplifier = make_simplifier(
            [
                ".....",
                "..#..",
                ".....",
            ]
        )
        planned = make_planned_path(
            occupancy_map,
            [
                GridCell(1, 0),
                GridCell(0, 1),
                GridCell(0, 2),
                GridCell(0, 3),
                GridCell(1, 4),
            ],
        )
        result = simplifier.simplify(planned, occupancy_map=occupancy_map)
        simplifier.validate_simplified_path(result.simplified_grid_path)

    def test_invalid_segment_validation_raises(self):
        _, simplifier = make_simplifier(["..#.."])
        with self.assertRaises(InvalidSimplifiedSegmentError):
            simplifier.validate_simplified_path(
                (GridCell(0, 0), GridCell(0, 4))
            )

    def test_corner_supercover_is_conservative_when_enabled(self):
        _, simplifier = make_simplifier(
            [
                ".#.",
                "...",
                ".#.",
            ]
        )
        self.assertFalse(
            simplifier.has_line_of_sight(GridCell(2, 0), GridCell(0, 2))
        )


if __name__ == "__main__":
    unittest.main()
