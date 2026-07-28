import json
from pathlib import Path
import tempfile
import unittest

from src.navigation import (
    FollowerConfig,
    NavigationExecutionError,
    NavigationExecutor,
    NavigationState,
    PlannerConfig,
    TargetRegistry,
)
from src.navigation.map_models import MapMetadata
from src.navigation.occupancy_grid import OccupancyGridMap


TEMP_DIRS = []


class FakeBaseController:
    def __init__(self):
        self.running = True
        self.commands = []
        self.soft_stops = 0
        self.hard_stops = 0

    def set_cmd(self, vx, wz):
        self.commands.append((float(vx), float(wz)))

    def soft_stop(self):
        self.soft_stops += 1

    def hard_stop(self):
        self.hard_stops += 1


class FakePoseProvider:
    def __init__(self, poses):
        self.poses = list(poses)
        self.index = 0

    def get_pose(self):
        if self.index >= len(self.poses):
            return self.poses[-1]
        pose = self.poses[self.index]
        self.index += 1
        return pose


class FakeCallbackRegistry:
    def __init__(self):
        self.callback = None
        self.deregistered = []

    def register(self, callback):
        self.callback = callback
        return "callback-id"

    def deregister(self, callback_id):
        self.deregistered.append(callback_id)


class FakeClock:
    def __init__(self, now=10.0):
        self.now = float(now)

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += float(seconds)


def make_map(rows: list[str]) -> OccupancyGridMap:
    probabilities = []
    alpha_unknown = []
    for row in rows:
        probability_row = []
        unknown_row = []
        for value in row:
            probability_row.append(1.0 if value == "#" else 0.0)
            unknown_row.append(False)
        probabilities.append(tuple(probability_row))
        alpha_unknown.append(tuple(unknown_row))

    metadata = MapMetadata(
        image_path=Path("/tmp/test_map.png"),
        resolution=1.0,
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


def planner_config() -> PlannerConfig:
    return PlannerConfig(
        inflation_radius_m=0.0,
        treat_unknown_as_occupied=True,
        allow_diagonal=True,
        prevent_corner_cutting=True,
        max_expanded_nodes=10000,
    )


def inflated_planner_config() -> PlannerConfig:
    return PlannerConfig(
        inflation_radius_m=1.0,
        treat_unknown_as_occupied=True,
        allow_diagonal=True,
        prevent_corner_cutting=True,
        max_expanded_nodes=10000,
    )


def follower_config(**overrides) -> FollowerConfig:
    values = {
        "max_linear_speed": 0.15,
        "min_linear_speed": 0.03,
        "max_angular_speed": 0.45,
        "lookahead_distance": 0.40,
        "align_enter_angle": 0.70,
        "align_exit_angle": 0.25,
        "goal_position_tolerance": 0.15,
        "goal_yaw_tolerance": 0.08,
        "slowdown_distance": 0.80,
        "max_path_deviation": 0.60,
        "stop_on_inflated_blocked_cell": False,
        "stop_on_raw_occupied_cell": False,
        "warn_on_blocked_cell": True,
        "progress_timeout_seconds": 5.0,
        "minimum_progress_distance": 0.05,
        "rotate_gain": 1.2,
        "goal_yaw_gain": 1.0,
        "control_period_seconds": 0.02,
    }
    values.update(overrides)
    return FollowerConfig(**values)


def make_registry(goal_x=4.5, goal_y=0.5, goal_yaw=0.0) -> TargetRegistry:
    temp_dir = tempfile.TemporaryDirectory()
    TEMP_DIRS.append(temp_dir)
    path = Path(temp_dir.name) / "navigation_targets.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "targets": {
                    "hf_dc_power_screen": {
                        "display_name": "高频开关直流电源屏",
                        "aliases": ["直流屏"],
                        "type": "equipment",
                        "description": "test",
                        "enabled": True,
                        "goal_poses": [
                            {
                                "pose_id": "front",
                                "x": goal_x,
                                "y": goal_y,
                                "yaw": goal_yaw,
                                "priority": 0,
                            }
                        ],
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    registry = TargetRegistry(path)
    registry._temp_dir = temp_dir
    return registry


class NavigationExecutorTests(unittest.TestCase):
    def tearDown(self):
        if hasattr(self, "executor"):
            self.executor.cancel()
        while TEMP_DIRS:
            TEMP_DIRS.pop().cleanup()

    def make_executor(
        self,
        *,
        poses,
        goal_x=4.5,
        goal_y=0.5,
        goal_yaw=0.0,
        occupancy_map=None,
        planner=None,
        follower=None,
        clock=None,
        warning_sink=None,
    ):
        self.base = FakeBaseController()
        self.callbacks = FakeCallbackRegistry()
        self.clock = clock or FakeClock()
        self.executor = NavigationExecutor(
            robot_path="/World/Robot/Geometry/base",
            target_registry=make_registry(goal_x, goal_y, goal_yaw),
            occupancy_map=occupancy_map or make_map(["....."]),
            planner_config=planner or planner_config(),
            follower_config=follower or follower_config(),
            pose_provider=FakePoseProvider(poses),
            base_controller=self.base,
            callback_registry=self.callbacks,
            time_provider=self.clock,
            warning_sink=warning_sink or (lambda message: None),
        )
        return self.executor

    def test_start_registers_callback_and_sets_initial_status(self):
        executor = self.make_executor(poses=[(0.5, 0.5, 0.0, 0.0)])
        path = executor.start("hf_dc_power_screen")
        self.assertIsNotNone(self.callbacks.callback)
        self.assertEqual(executor.status().state, NavigationState.ALIGN_TO_PATH)
        self.assertEqual(path.target_id, "hf_dc_power_screen")

    def test_missing_base_controller_does_not_start(self):
        executor = NavigationExecutor(
            robot_path="/World/Robot/Geometry/base",
            target_registry=make_registry(),
            occupancy_map=make_map(["....."]),
            planner_config=planner_config(),
            follower_config=follower_config(),
            pose_provider=FakePoseProvider([(0.5, 0.5, 0.0, 0.0)]),
            base_controller=None,
            callback_registry=FakeCallbackRegistry(),
            warning_sink=lambda message: None,
        )
        with self.assertRaises(NavigationExecutionError):
            executor.start("hf_dc_power_screen")
        self.assertEqual(executor.status().state, NavigationState.FAILED)

    def test_callback_sends_base_command(self):
        executor = self.make_executor(
            poses=[
                (0.5, 0.5, 0.0, 0.0),
                (0.5, 0.5, 0.0, 0.0),
            ]
        )
        executor.start("hf_dc_power_screen")
        self.callbacks.callback(0.02, None)
        self.assertTrue(self.base.commands)
        self.assertGreaterEqual(self.base.commands[-1][0], 0.0)

    def test_success_stops_and_deregisters(self):
        executor = self.make_executor(
            poses=[
                (0.5, 0.5, 0.0, 0.0),
                (4.5, 0.5, 0.0, 0.0),
            ]
        )
        executor.start("hf_dc_power_screen")
        self.callbacks.callback(0.02, None)
        self.assertEqual(executor.status().state, NavigationState.SUCCEEDED)
        self.assertIn("callback-id", self.callbacks.deregistered)
        self.assertGreaterEqual(self.base.soft_stops, 1)

    def test_cancel_stops_and_deregisters(self):
        executor = self.make_executor(poses=[(0.5, 0.5, 0.0, 0.0)])
        executor.start("hf_dc_power_screen")
        status = executor.cancel()
        self.assertEqual(status.state, NavigationState.CANCELLED)
        self.assertIn((0.0, 0.0), self.base.commands)
        self.assertIn("callback-id", self.callbacks.deregistered)

    def test_path_deviation_failure_hard_stops(self):
        self.base = FakeBaseController()
        self.callbacks = FakeCallbackRegistry()
        self.executor = NavigationExecutor(
            robot_path="/World/Robot/Geometry/base",
            target_registry=make_registry(goal_x=4.5, goal_y=0.5),
            occupancy_map=make_map([".....", ".....", "....."]),
            planner_config=planner_config(),
            follower_config=follower_config(max_path_deviation=0.25),
            pose_provider=FakePoseProvider(
                [
                    (0.5, 0.5, 0.0, 0.0),
                    (0.5, 1.5, 0.0, 0.0),
                ]
            ),
            base_controller=self.base,
            callback_registry=self.callbacks,
            time_provider=lambda: 10.0,
            warning_sink=lambda message: None,
        )
        self.executor.start("hf_dc_power_screen")
        self.callbacks.callback(0.02, None)
        self.assertEqual(self.executor.status().state, NavigationState.FAILED)
        self.assertGreaterEqual(self.base.hard_stops, 1)
        self.assertIn("callback-id", self.callbacks.deregistered)

    def test_inflated_blocked_raw_free_continues_when_switch_off(self):
        warnings = []
        executor = self.make_executor(
            poses=[
                (1.5, 1.5, 0.0, 0.0),
                (3.5, 3.5, 0.0, 0.0),
            ],
            goal_x=5.5,
            goal_y=1.5,
            occupancy_map=make_map(
                [
                    ".......",
                    ".......",
                    "...#...",
                    ".......",
                    ".......",
                    ".......",
                    ".......",
                ]
            ),
            planner=inflated_planner_config(),
            follower=follower_config(max_path_deviation=10.0),
            warning_sink=warnings.append,
        )
        executor.start("hf_dc_power_screen")
        self.callbacks.callback(0.02, None)
        self.assertNotEqual(executor.status().state, NavigationState.FAILED)
        self.assertTrue(self.base.commands)
        self.assertEqual(len(warnings), 1)

    def test_inflated_blocked_stops_when_switch_on(self):
        executor = self.make_executor(
            poses=[
                (1.5, 1.5, 0.0, 0.0),
                (3.5, 3.5, 0.0, 0.0),
            ],
            goal_x=5.5,
            goal_y=1.5,
            occupancy_map=make_map(
                [
                    ".......",
                    ".......",
                    "...#...",
                    ".......",
                    ".......",
                    ".......",
                    ".......",
                ]
            ),
            planner=inflated_planner_config(),
            follower=follower_config(
                max_path_deviation=10.0,
                stop_on_inflated_blocked_cell=True,
            ),
        )
        executor.start("hf_dc_power_screen")
        self.callbacks.callback(0.02, None)
        self.assertEqual(executor.status().state, NavigationState.FAILED)
        self.assertGreaterEqual(self.base.hard_stops, 1)

    def test_raw_occupied_continues_when_switch_off(self):
        executor = self.make_executor(
            poses=[
                (0.5, 0.5, 0.0, 0.0),
                (3.5, 4.5, 0.0, 0.0),
            ],
            goal_x=6.5,
            goal_y=0.5,
            occupancy_map=make_map(
                [
                    ".......",
                    "...#...",
                    ".......",
                    ".......",
                    ".......",
                    ".......",
                ]
            ),
            planner=planner_config(),
            follower=follower_config(max_path_deviation=10.0),
        )
        executor.start("hf_dc_power_screen")
        self.callbacks.callback(0.02, None)
        self.assertNotEqual(executor.status().state, NavigationState.FAILED)
        self.assertTrue(self.base.commands)

    def test_raw_occupied_stops_when_switch_on(self):
        executor = self.make_executor(
            poses=[
                (0.5, 0.5, 0.0, 0.0),
                (3.5, 4.5, 0.0, 0.0),
            ],
            goal_x=6.5,
            goal_y=0.5,
            occupancy_map=make_map(
                [
                    ".......",
                    "...#...",
                    ".......",
                    ".......",
                    ".......",
                    ".......",
                ]
            ),
            planner=planner_config(),
            follower=follower_config(
                max_path_deviation=10.0,
                stop_on_raw_occupied_cell=True,
            ),
        )
        executor.start("hf_dc_power_screen")
        self.callbacks.callback(0.02, None)
        self.assertEqual(executor.status().state, NavigationState.FAILED)
        self.assertGreaterEqual(self.base.hard_stops, 1)

    def test_outside_map_always_stops_without_rethrow(self):
        executor = self.make_executor(
            poses=[
                (0.5, 0.5, 0.0, 0.0),
                (-1.0, 0.5, 0.0, 0.0),
            ]
        )
        executor.start("hf_dc_power_screen")
        self.callbacks.callback(0.02, None)
        self.assertEqual(executor.status().state, NavigationState.FAILED)
        self.assertGreaterEqual(self.base.hard_stops, 1)
        self.assertIn("callback-id", self.callbacks.deregistered)

    def test_blocked_warning_is_rate_limited(self):
        warnings = []
        clock = FakeClock()
        executor = self.make_executor(
            poses=[
                (1.5, 1.5, 0.0, 0.0),
                (3.5, 3.5, 0.0, 0.0),
                (3.5, 3.5, 0.0, 0.0),
                (3.5, 3.5, 0.0, 0.0),
            ],
            goal_x=5.5,
            goal_y=1.5,
            occupancy_map=make_map(
                [
                    ".......",
                    ".......",
                    "...#...",
                    ".......",
                    ".......",
                    ".......",
                    ".......",
                ]
            ),
            planner=inflated_planner_config(),
            follower=follower_config(
                max_path_deviation=10.0,
                progress_timeout_seconds=100.0,
            ),
            clock=clock,
            warning_sink=warnings.append,
        )
        executor.start("hf_dc_power_screen")
        self.callbacks.callback(0.02, None)
        clock.advance(0.5)
        self.callbacks.callback(0.02, None)
        clock.advance(0.6)
        self.callbacks.callback(0.02, None)
        self.assertEqual(len(warnings), 2)

    def test_stall_failure_after_no_progress_timeout(self):
        clock = FakeClock()
        executor = self.make_executor(
            poses=[
                (0.5, 0.5, 0.0, 0.0),
                (0.5, 0.5, 0.0, 0.0),
                (0.5, 0.5, 0.0, 0.0),
            ],
            follower=follower_config(
                progress_timeout_seconds=1.0,
                minimum_progress_distance=0.05,
            ),
            clock=clock,
        )
        executor.start("hf_dc_power_screen")
        self.callbacks.callback(0.02, None)
        clock.advance(1.1)
        self.callbacks.callback(0.02, None)
        self.assertEqual(executor.status().state, NavigationState.FAILED)
        self.assertIn("no progress", executor.status().message)
        self.assertGreaterEqual(self.base.hard_stops, 1)

    def test_normal_motion_does_not_trigger_stall(self):
        clock = FakeClock()
        executor = self.make_executor(
            poses=[
                (0.5, 0.5, 0.0, 0.0),
                (0.5, 0.5, 0.0, 0.0),
                (0.6, 0.5, 0.0, 0.0),
            ],
            follower=follower_config(
                progress_timeout_seconds=1.0,
                minimum_progress_distance=0.05,
            ),
            clock=clock,
        )
        executor.start("hf_dc_power_screen")
        self.callbacks.callback(0.02, None)
        clock.advance(1.1)
        self.callbacks.callback(0.02, None)
        self.assertNotEqual(executor.status().state, NavigationState.FAILED)


if __name__ == "__main__":
    unittest.main()
