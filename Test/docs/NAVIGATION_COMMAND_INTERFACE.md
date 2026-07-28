# Navigation Command Interface

本文档说明当前项目中的严格结构化导航目标接口。该接口只负责目标与命令层，不包含实际路径规划或机器人运动。

## 当前边界

当前只实现：

- 语义导航目标配置；
- `NavigationCommand` 数据结构；
- 目标注册表加载与查询；
- 命令校验；
- Script Editor 演示脚本；
- 纯 Python 单元测试。

当前不实现：

- Occupancy Map；
- A*；
- Pure Pursuit；
- 实际导航运动；
- ROS 2 / Nav2；
- LLM；
- aliases 模糊匹配；
- 多控制源仲裁；
- watchdog 或命令超时。

## 严格 target_id

导航模块内部只接受稳定英文 `target_id`，例如：

```python
NavigationCommand(
    action="navigate",
    target_id="hf_dc_power_screen",
)
```

当前会拒绝以下形式：

- 中文名称，例如 `高频开关直流电源屏`；
- alias，例如 `直流屏`；
- 自然语言，例如 `去那个电源柜`；
- 未注册 ID，例如 `dc_screen`。

`display_name` 和 `aliases` 仅作为元数据保存，供人阅读或未来 LLM 使用。当前严格解析器不使用模糊匹配、编辑距离、子串匹配、embedding 或 LLM。

## 如何列出可用目标

在 Isaac Sim Script Editor 中运行：

```text
Scripts/30_list_navigation_targets.py
```

该脚本会加载 `config/navigation_targets.json` 并打印：

- `target_id`
- `display_name`
- `enabled`
- `goal pose count`

该脚本不访问 `builtins.TIANYI_BASE_CTRL`，也不会移动机器人。

## 如何构造 NavigationCommand

Python 代码示例：

```python
from src.navigation import (
    CommandValidator,
    NavigationCommand,
    TargetRegistry,
)

registry = TargetRegistry()
validator = CommandValidator(registry)

command = NavigationCommand(
    action="navigate",
    target_id="hf_dc_power_screen",
)

target = validator.validate(command)
```

`NavigationCommand` 表达的是“要去哪个已注册目标”，不是底盘运动命令。因此它不包含 `vx`、`wz`、路径、坐标或控制输出。

## 如何填写实际 goal pose

配置文件：

```text
config/navigation_targets.json
```

目标未标定时：

```json
{
  "enabled": false,
  "goal_poses": []
}
```

目标完成标定后，可以填写一个或多个候选停靠位姿：

```json
{
  "pose_id": "front",
  "x": 0.0,
  "y": 0.0,
  "yaw": 0.0,
  "priority": 0
}
```

字段约定：

- `x`、`y`：Isaac Sim 世界坐标系下的米；
- `yaw`：弧度；
- `pose_id`：同一目标下唯一；
- `priority`：整数，供未来选择多个候选停靠点时使用。

当前项目还没有可靠真实目标坐标，所以正式配置中两个目标均为 `enabled: false` 且 `goal_poses: []`。

## enabled 的含义

- `enabled: false`：目标已注册，但当前不可执行导航。通常表示尚未标定位姿。
- `enabled: true` 且 `goal_poses` 非空：命令可以通过当前校验层。
- `enabled: true` 但 `goal_poses` 为空：配置不完整，校验器会拒绝该命令。

## 错误类型区别

- `UnknownNavigationTargetError`：`target_id` 格式合法，但目标未注册。
- `InvalidTargetIdError`：`target_id` 不是稳定英文 ID，例如中文、空字符串或包含连字符。
- `DisabledNavigationTargetError`：目标已注册，但 `enabled` 为 `false`。
- `NavigationTargetNotReadyError`：目标已启用，但没有任何 `goal_poses`。
- `UnsupportedNavigationActionError`：`action` 不是严格的 `navigate`。
- `NavigationConfigError`：JSON 配置文件结构或字段类型错误。

## Script Editor 校验演示

在 Isaac Sim Script Editor 中运行：

```text
Scripts/31_validate_navigation_command.py
```

脚本顶部可以修改：

```python
TARGET_ID = "hf_dc_power_screen"
```

由于当前两个目标尚未配置真实位姿且 `enabled=false`，默认演示应输出“目标存在但当前不可执行”的错误，而不是导航成功。

## 未来 LLM 接入位置

正确链路：

```text
自然语言
-> LLM 输出 NavigationCommand / TaskPlan
-> CommandValidator
-> Navigator
-> IdealDifferentialBaseController
```

LLM 未来只能输出结构化命令，例如 `NavigationCommand` 或更高层 `TaskPlan`。这些输出必须经过 `CommandValidator`。

明确禁止 LLM 直接：

- 输出目标坐标绕过目标表；
- 输出 `vx`、`wz`；
- 调用 `builtins.TIANYI_BASE_CTRL`；
- 直接控制 Articulation Root。

## 与底盘控制器的关系

本接口不会修改 `src/ideal_base_controller.py`，也不会改变 `Scripts/start_base_controller.py` 的启动流程。

后续真实导航模块应接收已经通过校验的 `NavigationTarget`，从其中选择一个 `GoalPose`，再由路径规划和跟踪模块计算运动控制输出。当前版本不实现这部分。
