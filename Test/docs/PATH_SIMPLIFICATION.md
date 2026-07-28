# A* 路径视线简化

本文档说明当前项目的第一版 A* 原始路径简化。该阶段只生成调试结果，不移动机器人。

## 为什么不直接追踪原始 A* 路径

A* 输出的是逐栅格路径。它通常包含大量相邻节点，形状会贴着栅格边界折返。后续如果逐节点追踪，会产生过密目标点和不必要的转向。

`PathSimplifier` 使用确定性的 Line-of-Sight / String Pulling，把连续可直达的 A* 节点合并成更少的折线关键点。

## 简化方式

当前算法：

1. 从原始路径起点作为锚点。
2. 从终点向前搜索，找到锚点能安全直达的最远原始路径节点。
3. 保留该节点作为下一个关键点。
4. 重复直到终点。

算法不使用随机数，相同输入会得到相同输出。

## 为什么在膨胀地图上检查

视线检查使用 `AStarPlanner` 已构建的膨胀后 `traversable` 掩码，而不是原始 Occupancy Map。

因此简化路径仍然不能穿过：

- 原始障碍；
- `unknown` 区域；
- 障碍膨胀区域；
- 地图外区域。

这保证简化不会降低 A* 阶段已经采用的安全边界。

## Supercover 与普通 Bresenham

普通 Bresenham 只会选取少量靠近线段中心的栅格，线段擦过障碍物角点时可能漏检。

当前 `PathSimplifier.has_line_of_sight()` 使用保守的栅格线段遍历，并在 `prevent_corner_cutting=true` 时额外检查对角穿越处的两个正交相邻格，避免从障碍物夹角中穿过。

## 输出内容

`SimplifiedPath` 保存：

- 原始栅格路径；
- 简化后栅格路径；
- 原始世界坐标路径；
- 简化后世界坐标路径；
- `goal_yaw`；
- 原始长度与简化长度；
- 原始节点数与简化节点数；
- 视线检查次数。

简化后的世界路径会保留 `PlannedPath.world_path` 中的精确起点和终点，不会重新量化为栅格中心。

`goal_yaw` 当前只原样保留，不参与路径简化。

## 运行检查脚本

在 Isaac Sim Script Editor 中运行：

```text
Scripts/35_simplify_navigation_path.py
```

脚本顶部可以修改：

```python
TARGET_ID = "hf_dc_power_screen"
POSE_ID = None
```

测试另一个目标时可改为：

```python
TARGET_ID = "line_maintenance_area"
POSE_ID = None
```

如果 `POSE_ID = None`，脚本会按 `priority` 从小到大、`pose_id` 字典序选择第一个 goal pose。

脚本会重新读取当前机器人世界位姿，重新运行 A*，再执行路径简化。它不会复用历史 JSON，因为机器人可能已经移动。

## 调试图

输出目录：

```text
outputs/navigation_plans/
```

文件名形如：

```text
{target_id}_{pose_id}_{timestamp}_simplified.png
{target_id}_{pose_id}_{timestamp}_simplified_4x.png
{target_id}_{pose_id}_{timestamp}_simplified.json
```

图例：

- 黑色：原始障碍；
- 橙色：膨胀后新增不可通行区域；
- 蓝色细线：原始 A* 路径；
- 洋红色粗线：简化路径；
- 黄色：简化路径关键节点；
- 绿色：起点；
- 红色：终点。

图像方向与原始 Occupancy Map 保持一致，直接用 `row, col` 绘制。

## 当前边界

当前已经实现：

- 原始 A* 路径；
- 确定性视线简化；
- 简化结果二次校验；
- 原始路径与简化路径 PNG 对比；
- 简化路径 JSON 输出。

当前仍然不会：

- 移动机器人；
- 输出 `vx`、`wz`；
- 调用 `TIANYI_BASE_CTRL.set_cmd(vx, wz)`；
- 做 Pure Pursuit；
- 做路径跟踪状态机；
- 做终点 yaw 对准；
- 做动态避障或重规划。

下一阶段才会实现路径跟踪状态机、简化 Pure Pursuit、速度命令和导航取消/状态查询。
