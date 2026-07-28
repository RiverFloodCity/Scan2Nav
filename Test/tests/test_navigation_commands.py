import json
from pathlib import Path
import tempfile
import unittest

from src.navigation import (
    CommandValidator,
    DisabledNavigationTargetError,
    InvalidTargetIdError,
    NavigationCommand,
    NavigationConfigError,
    NavigationTargetNotReadyError,
    TargetRegistry,
    UnknownNavigationTargetError,
    UnsupportedNavigationActionError,
)


def write_config(directory: str, data: dict) -> Path:
    path = Path(directory) / "navigation_targets.json"
    path.write_text(
        json.dumps(data, ensure_ascii=False),
        encoding="utf-8",
    )
    return path


def legal_config(
    *,
    enabled: bool = True,
    goal_poses: list[dict] | None = None,
) -> dict:
    if goal_poses is None:
        goal_poses = [
            {
                "pose_id": "front",
                "x": 1.0,
                "y": 2.0,
                "yaw": 0.5,
                "priority": 0,
            }
        ]

    return {
        "schema_version": 1,
        "targets": {
            "hf_dc_power_screen": {
                "display_name": "高频开关直流电源屏",
                "aliases": [
                    "直流电源屏",
                    "直流屏",
                ],
                "type": "equipment",
                "description": "高频开关直流电源屏的导航停靠目标",
                "enabled": enabled,
                "goal_poses": goal_poses,
            },
            "line_maintenance_area": {
                "display_name": "线路检修区",
                "aliases": [
                    "检修区",
                ],
                "type": "region",
                "description": "线路检修区域的导航目标",
                "enabled": False,
                "goal_poses": [],
            },
        },
    }


class NavigationCommandTests(unittest.TestCase):
    def load_registry(self, data: dict) -> TargetRegistry:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        path = write_config(self.temp_dir.name, data)
        return TargetRegistry(config_path=path)

    def test_loads_legal_config(self):
        registry = self.load_registry(legal_config())
        target = registry.get_target("hf_dc_power_screen")
        self.assertEqual(target.display_name, "高频开关直流电源屏")
        self.assertTrue(target.enabled)
        self.assertEqual(len(target.goal_poses), 1)

    def test_lists_target_ids_in_stable_order(self):
        registry = self.load_registry(legal_config())
        self.assertEqual(
            registry.list_target_ids(),
            (
                "hf_dc_power_screen",
                "line_maintenance_area",
            ),
        )

    def test_valid_enabled_command_with_goal_pose_passes(self):
        registry = self.load_registry(legal_config())
        validator = CommandValidator(registry)
        command = NavigationCommand(
            action="navigate",
            target_id="hf_dc_power_screen",
        )
        target = validator.validate(command)
        self.assertEqual(target.target_id, "hf_dc_power_screen")

    def test_chinese_display_name_as_target_id_is_rejected(self):
        registry = self.load_registry(legal_config())
        validator = CommandValidator(registry)
        command = NavigationCommand(
            action="navigate",
            target_id="高频开关直流电源屏",
        )
        with self.assertRaises(InvalidTargetIdError):
            validator.validate(command)

    def test_alias_as_target_id_is_rejected(self):
        registry = self.load_registry(legal_config())
        validator = CommandValidator(registry)
        command = NavigationCommand(
            action="navigate",
            target_id="直流屏",
        )
        with self.assertRaises(InvalidTargetIdError):
            validator.validate(command)

    def test_unknown_target_id_is_rejected(self):
        registry = self.load_registry(legal_config())
        validator = CommandValidator(registry)
        command = NavigationCommand(
            action="navigate",
            target_id="dc_screen",
        )
        with self.assertRaises(UnknownNavigationTargetError):
            validator.validate(command)

    def test_disabled_target_is_rejected(self):
        registry = self.load_registry(legal_config(enabled=False))
        validator = CommandValidator(registry)
        command = NavigationCommand(
            action="navigate",
            target_id="hf_dc_power_screen",
        )
        with self.assertRaises(DisabledNavigationTargetError):
            validator.validate(command)

    def test_enabled_target_without_goal_pose_is_rejected(self):
        registry = self.load_registry(
            legal_config(
                enabled=True,
                goal_poses=[],
            )
        )
        validator = CommandValidator(registry)
        command = NavigationCommand(
            action="navigate",
            target_id="hf_dc_power_screen",
        )
        with self.assertRaises(NavigationTargetNotReadyError):
            validator.validate(command)

    def test_non_navigate_action_is_rejected(self):
        registry = self.load_registry(legal_config())
        validator = CommandValidator(registry)
        command = NavigationCommand(
            action="inspect",
            target_id="hf_dc_power_screen",
        )
        with self.assertRaises(UnsupportedNavigationActionError):
            validator.validate(command)

    def test_invalid_target_id_format_is_rejected(self):
        registry = self.load_registry(legal_config())
        validator = CommandValidator(registry)
        command = NavigationCommand(
            action="navigate",
            target_id="dc-screen",
        )
        with self.assertRaises(InvalidTargetIdError):
            validator.validate(command)

    def test_invalid_json_raises_config_error(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "navigation_targets.json"
            path.write_text("{not valid json", encoding="utf-8")
            with self.assertRaises(NavigationConfigError):
                TargetRegistry(config_path=path)

    def test_invalid_field_type_raises_config_error(self):
        data = legal_config()
        data["targets"] = []
        with tempfile.TemporaryDirectory() as temp_dir:
            path = write_config(temp_dir, data)
            with self.assertRaises(NavigationConfigError):
                TargetRegistry(config_path=path)

    def test_multiple_goal_poses_are_parsed(self):
        registry = self.load_registry(
            legal_config(
                goal_poses=[
                    {
                        "pose_id": "front",
                        "x": 1.0,
                        "y": 2.0,
                        "yaw": 0.5,
                        "priority": 0,
                    },
                    {
                        "pose_id": "side",
                        "x": 3.0,
                        "y": 4.0,
                        "yaw": 1.0,
                        "priority": 1,
                    },
                ]
            )
        )
        target = registry.get_target("hf_dc_power_screen")
        self.assertEqual(len(target.goal_poses), 2)
        self.assertEqual(target.goal_poses[1].pose_id, "side")

    def test_duplicate_pose_id_is_rejected(self):
        data = legal_config(
            goal_poses=[
                {
                    "pose_id": "front",
                    "x": 1.0,
                    "y": 2.0,
                    "yaw": 0.5,
                },
                {
                    "pose_id": "front",
                    "x": 3.0,
                    "y": 4.0,
                    "yaw": 1.0,
                },
            ]
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            path = write_config(temp_dir, data)
            with self.assertRaises(NavigationConfigError):
                TargetRegistry(config_path=path)


if __name__ == "__main__":
    unittest.main()
