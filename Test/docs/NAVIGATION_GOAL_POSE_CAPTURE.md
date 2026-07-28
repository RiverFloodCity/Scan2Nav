# Navigation Goal Pose Capture

本文档说明如何人工采集导航目标停靠位姿。本流程只读取机器人当前世界姿态并打印 JSON 片段，不会自动修改配置，也不会移动机器人。

## 使用流程

1. 打开正式场景 `Assets/Scene/final_scene.usd`。
2. 点击 Play。
3. 运行 `Scripts/start_base_controller.py`。
4. 使用现有动作脚本将机器人移动到目标停靠位置。
5. 使用 `90_soft_stop.py` 或 `91_hard_stop.py` 停稳。
6. 确认机器人面向目标设备。
7. 修改 `Scripts/33_capture_navigation_goal_pose.py` 顶部的 `TARGET_ID` 和 `POSE_ID`。
8. 运行 `Scripts/33_capture_navigation_goal_pose.py`。
9. 检查地图状态必须为 `free`。
10. 将打印出的 JSON 人工复制到 `config/navigation_targets.json` 对应目标的 `goal_poses` 数组中。
11. 在至少有一个有效 `goal_pose` 后，将该目标 `enabled` 改为 `true`。
12. 再运行 `Scripts/31_validate_navigation_command.py` 验证命令是否通过。

## 脚本配置

脚本顶部提供以下常量：

```python
PROJECT_ROOT = "/home/ps/ScanEverything/Test"
TARGET_ID = "hf_dc_power_screen"
POSE_ID = "front"
PRIORITY = 0
MAP_YAML_PATH = (
    "/home/ps/ScanEverything/Test/"
    "config/maps/final_scene_occupancy.yaml"
)
```

`TARGET_ID` 必须已经存在于 `config/navigation_targets.json`。目标当前可以是 `enabled=false` 且 `goal_poses=[]`，这不影响采集。

## 输出内容

脚本会打印：

- `TARGET_ID`
- `display_name`
- 机器人世界位置 `x, y, z`
- `yaw radians`
- `yaw degrees`
- 地图 `row, col`
- occupancy status
- 可复制的 JSON 片段

只有当前位置状态为 `free` 时，脚本才会打印可写入配置的正式 JSON 片段。

如果状态是 `occupied`，说明当前位置位于障碍栅格，不建议写入目标配置。

如果状态是 `unknown`，说明当前位置位于未知区域，不建议写入目标配置。

如果状态是 `outside`，说明当前位置超出地图范围。

脚本不会自动修正或吸附到最近自由栅格。

## JSON 片段

free 时输出的 JSON 片段形如：

```json
{
  "pose_id": "front",
  "x": -3.699494767425088,
  "y": 0.602986736098746,
  "yaw": 1.5707963267948966,
  "priority": 0
}
```

数值来自当前机器人世界姿态，不是固定示例。`yaw` 单位是弧度，并规范化到 `[-pi, pi)`。

## 当前边界

本脚本只读取和打印：

- 不自动修改 `config/navigation_targets.json`；
- 不自动移动机器人；
- 不调用 `builtins.TIANYI_BASE_CTRL.set_cmd()`；
- 不进行路径规划；
- 不实现 A*、地图膨胀、路径跟踪、ROS 2、LLM 或 VLA。
