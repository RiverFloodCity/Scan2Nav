import json
import math
from pathlib import Path
import tempfile
import unittest

from src.navigation import (
    AStarPlanner,
    GoalCellBlockedError,
    GoalOutsideMapError,
    GridCell,
    NavigationPlannerConfigError,
    NoPathFoundError,
    PlannerConfig,
    PlanningLimitExceededError,
    StartCellBlockedError,
    StartOutsideMapError,
    load_planner_config,
)
from src.navigation.grid_inflation import build_planning_grid
from src.navigation.map_models import MapMetadata
from src.navigation.occupancy_grid import OccupancyGridMap


def make_map(
    rows: list[str],
    *,
    resolution: float = 1.0,
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> OccupancyGridMap:
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

    height = len(rows)
    width = len(rows[0]) if rows else 0
    metadata = MapMetadata(
        image_path=Path("/tmp/test_map.png"),
        resolution=resolution,
        origin_x=origin[0],
        origin_y=origin[1],
        origin_yaw=origin[2],
        width=width,
        height=height,
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


def plan_cells(
    rows: list[str],
    start: GridCell,
    goal: GridCell,
    *,
    planner_config: PlannerConfig | None = None,
):
    occupancy_map = make_map(rows)
    planner = AStarPlanner(
        occupancy_map=occupancy_map,
        config=planner_config or config(),
    )
    start_world = occupancy_map.grid_to_world(start.row, start.col)
    goal_world = occupancy_map.grid_to_world(goal.row, goal.col)
    path = planner.plan(
        start_cell=start,
        goal_cell=goal,
        target_id="test_target",
        pose_id="front",
        exact_start_world=start_world,
        exact_goal_world=goal_world,
        goal_yaw=0.0,
    )
    return occupancy_map, planner, path


class AStarPlannerTests(unittest.TestCase):
    def test_empty_map_straight_path(self):
        _, _, path = plan_cells(
            ["....."],
            GridCell(0, 0),
            GridCell(0, 4),
            planner_config=config(allow_diagonal=False),
        )
        self.assertEqual(path.grid_path[0], GridCell(0, 0))
        self.assertEqual(path.grid_path[-1], GridCell(0, 4))
        self.assertEqual(len(path.grid_path), 5)
        self.assertAlmostEqual(path.grid_cost, 4.0)

    def test_obstacle_detour(self):
        _, _, path = plan_cells(
            [
                ".....",
                ".###.",
                ".....",
            ],
            GridCell(1, 0),
            GridCell(1, 4),
            planner_config=config(allow_diagonal=False),
        )
        self.assertGreater(len(path.grid_path), 5)
        self.assertNotIn(GridCell(1, 2), path.grid_path)

    def test_no_path_found(self):
        with self.assertRaises(NoPathFoundError):
            plan_cells(
                [
                    ".#.",
                    ".#.",
                    ".#.",
                ],
                GridCell(1, 0),
                GridCell(1, 2),
                planner_config=config(allow_diagonal=False),
            )

    def test_start_equals_goal(self):
        _, _, path = plan_cells(
            ["."],
            GridCell(0, 0),
            GridCell(0, 0),
        )
        self.assertEqual(path.grid_path, (GridCell(0, 0),))
        self.assertEqual(path.grid_cost, 0.0)
        self.assertEqual(path.length_m, 0.0)

    def test_start_blocked(self):
        with self.assertRaises(StartCellBlockedError):
            plan_cells(["#."], GridCell(0, 0), GridCell(0, 1))

    def test_goal_blocked(self):
        with self.assertRaises(GoalCellBlockedError):
            plan_cells([".#"], GridCell(0, 0), GridCell(0, 1))

    def test_start_outside(self):
        occupancy_map = make_map([".."])
        planner = AStarPlanner(occupancy_map, config())
        with self.assertRaises(StartOutsideMapError):
            planner.plan(
                start_cell=GridCell(-1, 0),
                goal_cell=GridCell(0, 1),
                target_id="target",
                pose_id="front",
                exact_start_world=(-1.0, 0.5),
                exact_goal_world=occupancy_map.grid_to_world(0, 1),
                goal_yaw=0.0,
            )

    def test_goal_outside(self):
        occupancy_map = make_map([".."])
        planner = AStarPlanner(occupancy_map, config())
        with self.assertRaises(GoalOutsideMapError):
            planner.plan(
                start_cell=GridCell(0, 0),
                goal_cell=GridCell(0, 2),
                target_id="target",
                pose_id="front",
                exact_start_world=occupancy_map.grid_to_world(0, 0),
                exact_goal_world=(2.5, 0.5),
                goal_yaw=0.0,
            )

    def test_diagonal_path_when_allowed(self):
        _, _, path = plan_cells(
            [
                "...",
                "...",
                "...",
            ],
            GridCell(2, 0),
            GridCell(0, 2),
        )
        self.assertEqual(
            path.grid_path,
            (
                GridCell(2, 0),
                GridCell(1, 1),
                GridCell(0, 2),
            ),
        )
        self.assertAlmostEqual(path.grid_cost, 2.0 * math.sqrt(2.0))

    def test_prevent_corner_cutting(self):
        with self.assertRaises(NoPathFoundError):
            plan_cells(
                [
                    "#.",
                    ".#",
                ],
                GridCell(1, 0),
                GridCell(0, 1),
            )

    def test_corner_cutting_allowed_when_disabled(self):
        _, _, path = plan_cells(
            [
                "#.",
                ".#",
            ],
            GridCell(1, 0),
            GridCell(0, 1),
            planner_config=config(prevent_corner_cutting=False),
        )
        self.assertEqual(path.grid_path, (GridCell(1, 0), GridCell(0, 1)))

    def test_unknown_is_blocked_by_default(self):
        with self.assertRaises(NoPathFoundError):
            plan_cells(
                ["...?..."],
                GridCell(0, 0),
                GridCell(0, 6),
                planner_config=config(allow_diagonal=False),
            )

    def test_inflation_expands_obstacle(self):
        occupancy_map = make_map(["..#.."])
        planning_grid = build_planning_grid(
            occupancy_map,
            config(inflation_radius_m=1.0),
        )
        self.assertTrue(planning_grid.is_inflated_blocked(GridCell(0, 1)))
        self.assertTrue(planning_grid.is_inflated_blocked(GridCell(0, 2)))
        self.assertTrue(planning_grid.is_inflated_blocked(GridCell(0, 3)))

    def test_inflation_closes_narrow_channel(self):
        with self.assertRaises(StartCellBlockedError):
            plan_cells(
                [
                    "#####",
                    "#...#",
                    "#####",
                ],
                GridCell(1, 1),
                GridCell(1, 3),
                planner_config=config(inflation_radius_m=1.0),
            )

    def test_map_boundary_blocks_cells_with_inflation_radius(self):
        occupancy_map = make_map(
            [
                "...",
                "...",
                "...",
            ]
        )
        planning_grid = build_planning_grid(
            occupancy_map,
            config(inflation_radius_m=1.0),
        )
        self.assertFalse(planning_grid.is_traversable(GridCell(0, 1)))
        self.assertFalse(planning_grid.is_traversable(GridCell(1, 0)))
        self.assertTrue(planning_grid.is_traversable(GridCell(1, 1)))

    def test_world_path_preserves_exact_start_and_goal(self):
        occupancy_map = make_map(["..."])
        planner = AStarPlanner(occupancy_map, config(allow_diagonal=False))
        path = planner.plan(
            start_cell=GridCell(0, 0),
            goal_cell=GridCell(0, 2),
            target_id="target",
            pose_id="front",
            exact_start_world=(0.25, 0.75),
            exact_goal_world=(2.25, 0.75),
            goal_yaw=1.0,
        )
        self.assertEqual(path.world_path[0].x, 0.25)
        self.assertEqual(path.world_path[0].y, 0.75)
        self.assertEqual(path.world_path[-1].x, 2.25)
        self.assertEqual(path.world_path[-1].y, 0.75)

    def test_length_m(self):
        _, _, path = plan_cells(
            ["..."],
            GridCell(0, 0),
            GridCell(0, 2),
            planner_config=config(allow_diagonal=False),
        )
        self.assertAlmostEqual(path.length_m, 2.0)

    def test_deterministic_path(self):
        _, _, first = plan_cells(
            [
                ".....",
                "..#..",
                ".....",
            ],
            GridCell(2, 0),
            GridCell(0, 4),
        )
        _, _, second = plan_cells(
            [
                ".....",
                "..#..",
                ".....",
            ],
            GridCell(2, 0),
            GridCell(0, 4),
        )
        self.assertEqual(first.grid_path, second.grid_path)

    def test_max_expanded_nodes_limit(self):
        with self.assertRaises(PlanningLimitExceededError):
            plan_cells(
                [
                    "..........",
                    "..........",
                    "..........",
                    "..........",
                ],
                GridCell(3, 0),
                GridCell(0, 9),
                planner_config=config(max_expanded_nodes=1),
            )

    def test_invalid_config_raises(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "navigation_planner.json"
            path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "inflation_radius_m": -1.0,
                        "treat_unknown_as_occupied": True,
                        "allow_diagonal": True,
                        "prevent_corner_cutting": True,
                        "max_expanded_nodes": 100,
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaises(NavigationPlannerConfigError):
                load_planner_config(path)


if __name__ == "__main__":
    unittest.main()
