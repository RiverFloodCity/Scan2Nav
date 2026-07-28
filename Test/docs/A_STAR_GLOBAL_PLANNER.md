# A* Global Planner

本文档说明当前项目的第一版全局路径规划。该阶段只规划，不移动机器人。

## 原始地图与膨胀地图

`OccupancyGridMap` 读取的是原始 Occupancy Map。它只描述原始栅格的 `free`、`occupied` 和 `unknown` 状态。

A* 不直接在原始地图上搜索，而是在膨胀后的规划地图上搜索。膨胀会把障碍物周围一定半径内的栅格标记为不可通行。

## 为什么机器人不能被当成一个点

机器人有实际尺寸。如果只把机器人当作点，路径可能贴着障碍物边缘穿过，实际机器人会碰撞。`inflation_radius_m` 用来给障碍物扩张安全距离。

## inflation_radius_m

配置文件：

```text
config/navigation_planner.json
```

字段：

```json
{
  "inflation_radius_m": 0.50
}
```

单位是米。系统会根据地图 `resolution` 转成栅格半径：

```text
radius_cells = ceil(inflation_radius_m / resolution)
```

如果目标点在原始地图上是 free，但在膨胀地图中被 blocked，说明目标距离障碍物太近，需要重新标定目标位姿，或调整 `inflation_radius_m`。

当前 `0.50 m` 是规划安全膨胀半径，用于 A*、路径简化、起终点合法性检查和规划调试图。

## unknown 默认视为障碍

当前配置：

```json
{
  "treat_unknown_as_occupied": true
}
```

未知区域可能没有可靠碰撞或可通行信息，因此默认不可通行。

## 禁止 diagonal corner cutting

当 8 邻域允许斜向移动时，如果两个正交相邻格都是障碍，机器人不能从它们的夹角中斜穿过去。

当前配置：

```json
{
  "prevent_corner_cutting": true
}
```

## A* 设置

当前 A*：

- 使用 `heapq`；
- 支持 8 邻域；
- 正交移动代价为 `1`；
- 对角移动代价为 `sqrt(2)`；
- 启发函数为 Octile Distance；
- 使用固定邻居顺序和单调计数器保证确定性。

## goal yaw

目标 `yaw` 不参与 A* 搜索。A* 只规划二维栅格路径。`goal_yaw` 会保存到 `PlannedPath` 和 JSON 调试结果中，供后续终点朝向控制使用。

## 运行 Script Editor 脚本

在 Isaac Sim Script Editor 中运行：

```text
Scripts/34_plan_navigation_path.py
```

脚本顶部可以修改：

```python
TARGET_ID = "hf_dc_power_screen"
POSE_ID = None
```

如果 `POSE_ID = None`，脚本会按 `priority` 从小到大、`pose_id` 字典序选择第一个 goal pose。

脚本会打印：

- 机器人当前世界位姿；
- 目标世界位姿；
- 起点/终点栅格；
- 原始地图状态；
- 膨胀后状态；
- 路径节点数；
- 栅格代价；
- 世界路径长度；
- 扩展节点数；
- PNG、4x PNG、JSON 输出路径。

如果需要查看原始 A* 路径与视线简化路径的对比，请运行：

```text
Scripts/35_simplify_navigation_path.py
```

该脚本会重新读取当前机器人位姿并重新规划，然后生成 `_simplified.png`、`_simplified_4x.png` 和 `_simplified.json`。

输出目录：

```text
outputs/navigation_plans/
```

## PNG 图例

调试图颜色：

- 黑色：原始障碍物；
- 橙色：膨胀后新增不可通行区域；
- 蓝色：A* 路径；
- 绿色：起点；
- 红色：终点。

图像方向与原始 Occupancy Map 保持一致，直接使用 `row, col` 绘制。

## 当前边界

当前只实现：

- 起点和终点栅格转换；
- 障碍物膨胀；
- 8 邻域 A*；
- 栅格路径；
- 世界坐标路径；
- PNG / JSON 调试输出。
- 原始 A* 路径的确定性视线简化；
- 原始路径与简化路径对比 PNG / JSON 输出。

当前不会：

- 移动机器人；
- 输出 `vx`、`wz`；
- 调用底盘控制接口；
- 做路径跟踪。

后续阶段才会实现：

- 路径跟踪；
- `vx`、`wz`；
- `TIANYI_BASE_CTRL.set_cmd(vx, wz)`。
