import json
import math
from pathlib import Path
import tempfile
import unittest

from src.navigation import (
    FollowerConfig,
    NavigationFollowerConfigError,
    NavigationState,
    PathFollower,
    WorldPoint,
    load_follower_config,
)
from src.navigation.path_follower import (
    distance_to_polyline,
    normalize_angle,
    project_point_to_segment,
)


def config(**overrides) -> FollowerConfig:
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


def follower(
    path=None,
    *,
    goal_yaw=0.0,
    initial_state=NavigationState.TRACK_PATH,
    follower_config=None,
) -> PathFollower:
    if path is None:
        path = (WorldPoint(0.0, 0.0), WorldPoint(2.0, 0.0))
    return PathFollower(
        world_path=tuple(path),
        goal_yaw=goal_yaw,
        config=follower_config or config(),
        initial_state=initial_state,
    )


class PathFollowerTests(unittest.TestCase):
    def test_aligned_path_outputs_positive_vx_and_near_zero_wz(self):
        output = follower().step(x=0.0, y=0.0, yaw=0.0)
        self.assertGreater(output.vx, 0.0)
        self.assertAlmostEqual(output.wz, 0.0)

    def test_path_left_outputs_positive_wz(self):
        output = follower(
            (WorldPoint(0.0, 0.0), WorldPoint(2.0, 0.5))
        ).step(x=0.0, y=0.0, yaw=0.0)
        self.assertGreater(output.wz, 0.0)

    def test_path_right_outputs_negative_wz(self):
        output = follower(
            (WorldPoint(0.0, 0.0), WorldPoint(2.0, -0.5))
        ).step(x=0.0, y=0.0, yaw=0.0)
        self.assertLess(output.wz, 0.0)

    def test_large_heading_error_enters_align_to_path(self):
        output = follower().step(x=0.0, y=0.0, yaw=math.pi / 2.0)
        self.assertEqual(output.state, NavigationState.ALIGN_TO_PATH)

    def test_align_to_path_outputs_zero_vx(self):
        output = follower(
            initial_state=NavigationState.ALIGN_TO_PATH
        ).step(x=0.0, y=0.0, yaw=math.pi / 2.0)
        self.assertEqual(output.vx, 0.0)
        self.assertNotEqual(output.wz, 0.0)

    def test_align_error_decrease_enters_track_path(self):
        path_follower = follower(initial_state=NavigationState.ALIGN_TO_PATH)
        output = path_follower.step(x=0.0, y=0.0, yaw=0.0)
        self.assertEqual(output.state, NavigationState.TRACK_PATH)
        self.assertGreater(output.vx, 0.0)

    def test_near_goal_slows_down(self):
        output = follower().step(x=1.5, y=0.0, yaw=0.0)
        self.assertGreater(output.vx, 0.0)
        self.assertLess(output.vx, config().max_linear_speed)

    def test_goal_position_reached_enters_align_goal_yaw(self):
        output = follower(goal_yaw=0.5).step(x=2.0, y=0.0, yaw=0.0)
        self.assertEqual(output.state, NavigationState.ALIGN_GOAL_YAW)
        self.assertEqual(output.vx, 0.0)

    def test_goal_yaw_left_outputs_positive_wz(self):
        output = follower(goal_yaw=0.5).step(x=2.0, y=0.0, yaw=0.0)
        self.assertGreater(output.wz, 0.0)

    def test_goal_yaw_right_outputs_negative_wz(self):
        output = follower(goal_yaw=-0.5).step(x=2.0, y=0.0, yaw=0.0)
        self.assertLess(output.wz, 0.0)

    def test_goal_position_and_yaw_succeed(self):
        output = follower(goal_yaw=0.02).step(x=2.0, y=0.0, yaw=0.0)
        self.assertEqual(output.state, NavigationState.SUCCEEDED)
        self.assertEqual(output.vx, 0.0)
        self.assertEqual(output.wz, 0.0)

    def test_velocity_limits_are_respected(self):
        limited = config(max_linear_speed=0.1, max_angular_speed=0.2)
        output = follower(
            (WorldPoint(0.0, 0.0), WorldPoint(2.0, 0.5)),
            follower_config=limited,
        ).step(x=0.0, y=0.0, yaw=0.0)
        self.assertLessEqual(output.vx, limited.max_linear_speed)
        self.assertLessEqual(abs(output.wz), limited.max_angular_speed)

    def test_lookahead_segment_index_does_not_go_backward(self):
        path_follower = follower(
            (
                WorldPoint(0.0, 0.0),
                WorldPoint(1.0, 0.0),
                WorldPoint(2.0, 0.0),
            )
        )
        first = path_follower.step(x=1.5, y=0.0, yaw=0.0)
        second = path_follower.step(x=0.2, y=0.0, yaw=0.0)
        self.assertGreaterEqual(
            second.current_segment_index,
            first.current_segment_index,
        )

    def test_two_point_path_is_supported(self):
        output = follower().step(x=0.2, y=0.0, yaw=0.0)
        self.assertEqual(output.state, NavigationState.TRACK_PATH)

    def test_multi_segment_polyline_is_supported(self):
        output = follower(
            (
                WorldPoint(0.0, 0.0),
                WorldPoint(1.0, 0.0),
                WorldPoint(1.0, 1.0),
            )
        ).step(x=0.2, y=0.0, yaw=0.0)
        self.assertGreater(output.vx, 0.0)

    def test_path_deviation_failure(self):
        output = follower().step(x=0.0, y=1.0, yaw=0.0)
        self.assertEqual(output.state, NavigationState.FAILED)
        self.assertEqual(output.vx, 0.0)
        self.assertEqual(output.wz, 0.0)

    def test_wrap_angle_pi_boundary(self):
        self.assertAlmostEqual(
            normalize_angle(math.pi + 0.1),
            -math.pi + 0.1,
        )
        self.assertAlmostEqual(
            normalize_angle(-math.pi - 0.1),
            math.pi - 0.1,
        )

    def test_segment_distance_calculation(self):
        projection = project_point_to_segment(
            0.5,
            1.0,
            WorldPoint(0.0, 0.0),
            WorldPoint(1.0, 0.0),
            segment_index=0,
        )
        self.assertAlmostEqual(projection.distance, 1.0)
        self.assertAlmostEqual(
            distance_to_polyline(
                0.5,
                1.0,
                (WorldPoint(0.0, 0.0), WorldPoint(1.0, 0.0)),
            ),
            1.0,
        )

    def test_same_input_is_deterministic(self):
        path_follower = follower()
        first = path_follower.step(x=0.0, y=0.0, yaw=0.0)
        path_follower = follower()
        second = path_follower.step(x=0.0, y=0.0, yaw=0.0)
        self.assertEqual(first, second)

    def test_invalid_config_raises(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "navigation_follower.json"
            path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "max_linear_speed": 0.1,
                        "min_linear_speed": 0.2,
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
                ),
                encoding="utf-8",
            )
            with self.assertRaises(NavigationFollowerConfigError):
                load_follower_config(path)


if __name__ == "__main__":
    unittest.main()
