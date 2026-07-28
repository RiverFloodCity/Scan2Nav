# Scan2Nav 自动导航执行器

本文档说明第一版自动导航执行器。该阶段已经会通过底盘控制器输出 `vx`、`wz`，但仍不包含 ROS 2、Nav2、动态避障、在线重规划、LLM 或 VLA。

## 标准启动顺序

1. 打开 `Assets/Scene/final_scene.usd`。
2. 点击 Play。
3. 等待一到两个物理步。
4. 在 Script Editor 中运行 `Scripts/start_base_controller.py`。
5. 第一次测试先运行 `Scripts/36_start_navigation.py`，并保持：

```python
TARGET_ID = "hf_dc_power_screen"
POSE_ID = None
```

6. 使用 `Scripts/37_navigation_status.py` 查看状态。
7. 测试期间保持 `Scripts/95_cancel_navigation.py` 随时可运行。

测试 `line_maintenance_area` 时，修改 `Scripts/36_start_navigation.py` 顶部：

```python
TARGET_ID = "line_maintenance_area"
POSE_ID = None
```

## 全局对象

底盘控制仍统一经过：

```python
builtins.TIANYI_BASE_CTRL.set_cmd(vx, wz)
```

自动导航执行器启动成功后注册为：

```python
builtins.SCAN2NAV_EXECUTOR
```

取消脚本 `Scripts/95_cancel_navigation.py` 会调用 `cancel()`，安全停止后删除 `builtins.SCAN2NAV_EXECUTOR`。这表示下一次启动会创建新的执行器对象。

## 状态机

`NavigationState` 包含：

- `IDLE`：执行器空闲；
- `PLANNING`：正在校验目标、读取位姿、运行 A* 和路径简化；
- `ALIGN_TO_PATH`：原地旋转，对准当前前视点方向；
- `TRACK_PATH`：Pure Pursuit 跟踪简化折线；
- `ALIGN_GOAL_YAW`：到达目标位置后，原地对准目标 `yaw`；
- `SUCCEEDED`：位置和目标朝向均满足容差；
- `FAILED`：发生规划失败、执行异常、跑出地图或路径偏离过大；
- `CANCELLED`：用户取消。

## Pure Pursuit

`PathFollower` 是纯数学模块。它输入当前 `x, y, yaw`、简化后的 `world_path`、`goal_yaw` 和 `FollowerConfig`，输出 `vx, wz` 与调试状态。

跟踪时会：

1. 找到机器人到当前简化路径折线的最近投影点；
2. 从投影点沿路径向前累计 `lookahead_distance`；
3. 计算前视点方向误差；
4. 使用曲率：

```text
curvature = 2 * sin(alpha) / Ld
wz = vx * curvature
```

当前不会倒车，`vx` 不为负。

当航向误差过大时，状态重新进入 `ALIGN_TO_PATH`，原地对准后再继续跟踪。

## 终点 yaw

A* 和路径简化只处理二维位置。`goal_yaw` 不参与规划和简化，只在机器人进入 `goal_position_tolerance` 后由 `ALIGN_GOAL_YAW` 状态单独对准。

## 配置

配置文件：

```text
config/navigation_follower.json
```

单位：

- 距离：米；
- 角度：弧度；
- 线速度：m/s；
- 角速度：rad/s。

第一版保守速度：

```json
{
  "max_linear_speed": 0.15,
  "max_angular_speed": 0.45
}
```

规划配置 `config/navigation_planner.json` 中：

```json
{
  "inflation_radius_m": 0.50
}
```

`0.50 m` 是 A* 和路径简化使用的规划安全膨胀半径。

关键容差：

- `goal_position_tolerance`：进入终点朝向对准的距离；
- `goal_yaw_tolerance`：目标朝向完成容差；
- `max_path_deviation`：机器人到简化路径折线的最大允许偏离；
- `lookahead_distance`：Pure Pursuit 前视距离。
- `stop_on_inflated_blocked_cell`：机器人中心进入规划膨胀 blocked cell 时是否立即失败；
- `stop_on_raw_occupied_cell`：机器人中心进入原始 Occupancy Map occupied cell 时是否立即失败；
- `warn_on_blocked_cell`：进入 raw occupied 或 inflated blocked 时是否低频警告；
- `progress_timeout_seconds` 和 `minimum_progress_distance`：TRACK_PATH 中正向命令长期没有实际位移时触发 no-progress 失败。

当前默认：

```json
{
  "stop_on_inflated_blocked_cell": false,
  "stop_on_raw_occupied_cell": false,
  "warn_on_blocked_cell": true,
  "progress_timeout_seconds": 5.0,
  "minimum_progress_distance": 0.05
}
```

这表示运行时允许机器人中心短暂进入规划膨胀区域。`planning blocked` 不等同于真实 Mesh 碰撞；它是给 A* 和路径简化使用的安全边界。本阶段允许 Isaac Sim Mesh Collider 处理实际接触，优先跑通确定性导航闭环。

如果机器人真正撞到 Mesh 后持续顶住障碍，no-progress 检测会在 `TRACK_PATH` 状态下发现一段时间内位移不足，并进入 `FAILED` 停止机器人。

## 安全行为

执行器保证：

- 不存在 `builtins.TIANYI_BASE_CTRL` 时不启动；
- 起点或终点阻塞时不启动；
- A* 或路径简化失败时不注册导航回调；
- 机器人跑出地图时进入 `FAILED` 并停止；
- 路径偏离超过阈值时进入 `FAILED` 并停止；
- no-progress / stall 检测触发时进入 `FAILED` 并停止；
- 成功、失败、取消都会发送停止命令并注销导航回调。

## 当前不做

当前不处理：

- 动态障碍物；
- 在线局部重规划；
- SLAM / AMCL；
- ROS 2 / Nav2；
- 样条曲线平滑；
- 多控制源仲裁；
- 自动修改地图或目标位姿。
