# Scan2Nav 项目技术记忆

> 本文档只维护 Scan2Nav 项目的技术背景、代码结构、已完成工作、当前状态与后续计划。  
> 不记录个人背景、研究兴趣、设备资产、日报偏好或其他项目内容。  
> 后续项目技术变更应更新本文件，而不是写入通用长期记忆摘要。

- 项目名称：Scan2Nav
- 项目根目录：`/home/ps/ScanEverything/Test`
- 主要平台：Isaac Sim 6.0.1
- 当前更新时间：2026-07-27

---

## 1. 项目目标

实现一条确定性的机器人任务链：

```text
真实场景扫描
→ 场景重建
→ Isaac Sim 场景加载
→ 语义目标解析
→ 二维地图导航
→ 到达指定设备或区域
→ 后续衔接机械臂操作或 VLA
```

当前重点是：

```text
场景 → 理解 → 导航 → 操作
```

当前已经进入“前往指定地点”的自动导航联调阶段，暂不研究真实底盘动力学。

---

## 2. 当前技术路线

### 2.1 场景构建

真实场景由 Q9000 扫描获得原始数据，再经留形科技在线平台处理为：

- Mesh：负责碰撞和几何结构；
- 3DGS：负责视觉显示；
- 原始点云：不再作为当前 Isaac Sim 运行场景的核心表示。

当前场景原则：

```text
Mesh Collider
→ 物理碰撞
→ Occupancy Map

3DGS
→ 视觉显示
→ 不参与碰撞和地图生成
```

主要场景文件：

```text
Assets/Scene/final_scene.usd
```

环境资产目录：

```text
Assets/Collected_environment_scene/
```

机器人资产目录：

```text
Assets/Robots/Tianyi2/
```

---

## 3. 理想化底盘控制

### 3.1 当前底盘假设

当前不模拟：

- 主动轮和从动轮；
- 轮地接触；
- 摩擦、打滑；
- 电机模型；
- 差速轮动力学；
- 底盘惯量与驱动力；
- 真实底盘控制器。

当前采用理想化底盘：

- 删除或忽略底盘 Mesh 摩擦影响；
- 直接让机器人所有相关部件整体平移和转向；
- 上层只生成统一的 `vx、wz`；
- 底层统一调用 `builtins.TIANYI_BASE_CTRL.set_cmd(vx, wz)`。

只有未来研究以下问题时，才考虑真实底盘建模：

- VLA 输出全身关节动作，包括底盘；
- 全身协调控制；
- 底盘与机械臂联合动力学；
- 底盘 Sim2Real；
- 打滑和运动误差。

### 3.2 控制器职责分离

#### `src/ideal_base_controller.py`

只负责：

- 机器人路径和参数配置；
- 速度、加速度限制；
- 辅助函数；
- `IdealDifferentialBaseController` 类；
- 全部底层运动控制逻辑。

import 该模块时不会：

- 创建控制器实例；
- 注册物理回调；
- 修改 `builtins`；
- 自动启动程序。

#### `Scripts/start_base_controller.py`

作为唯一启动入口，负责：

- 显式将 `/home/ps/ScanEverything/Test` 加入 `sys.path`；
- 导入控制器类与 `ROBOT_PATH`；
- 关闭并清理旧的 `builtins.TIANYI_BASE_CTRL`；
- 创建新实例；
- 调用 `start()`；
- 仅在成功后注册：

```python
builtins.TIANYI_BASE_CTRL = controller
```

启动失败时不会残留无效全局对象。

### 3.3 统一控制接口

现有和未来所有控制源都必须经过：

```python
import builtins
builtins.TIANYI_BASE_CTRL.set_cmd(vx, wz)
```

其他接口：

```python
builtins.TIANYI_BASE_CTRL.soft_stop()
builtins.TIANYI_BASE_CTRL.hard_stop()
builtins.TIANYI_BASE_CTRL.status()
builtins.TIANYI_BASE_CTRL.shutdown()
```

导航模块不直接操作 Isaac Sim Articulation Root。

当前仍未实现：

- 多控制源仲裁；
- 控制优先级；
- 通用命令超时；
- 键盘控制；
- VLA 控制。

---

## 4. 标准运行流程

### 4.1 底盘控制器

1. 打开：

```text
Assets/Scene/final_scene.usd
```

2. 点击 Isaac Sim 的 Play；
3. 等待物理场景初始化；
4. 在 Isaac Sim Script Editor 中运行：

```text
Scripts/start_base_controller.py
```

5. 确认：

```python
builtins.TIANYI_BASE_CTRL
```

创建成功。

### 4.2 自动导航

底盘控制器启动后，运行：

```text
Scripts/36_start_navigation.py
```

辅助脚本：

```text
Scripts/37_navigation_status.py
Scripts/95_cancel_navigation.py
```

其中：

- `36_start_navigation.py`：根据严格 `TARGET_ID` 启动一次导航；
- `37_navigation_status.py`：查询当前导航状态；
- `95_cancel_navigation.py`：取消导航并安全停止。

自动导航执行器全局对象：

```python
builtins.SCAN2NAV_EXECUTOR
```

### 4.3 环境限制

- 不使用普通终端 Python 启动 Isaac Sim 脚本；
- 不创建 standalone `SimulationApp`；
- 不创建 Conda 环境；
- 不使用 `__file__` 推导项目根目录；
- Script Editor 可能从 `/tmp/carb...` 临时路径执行代码；
- Linux 下严格区分 `Scripts` 和 `src` 大小写。

---

## 5. 当前导航架构

当前确定性导航链已经实现到自动执行阶段：

```text
严格结构化 target_id
→ 当前机器人真值位姿
→ 二维 Occupancy Grid
→ 障碍物膨胀
→ 8 邻域 A*
→ Line-of-Sight 路径简化
→ Pure Pursuit 路径跟踪
→ 输出 vx、wz
→ builtins.TIANYI_BASE_CTRL.set_cmd(vx, wz)
→ 终点位置与 yaw 对准
```

每次运行导航都会：

```text
重新读取机器人当前位置
→ 重新运行 A*
→ 重新运行路径简化
→ 跟踪本次新路径
```

运行时不依赖历史调试文件：

```text
outputs/navigation_plans/*.png
outputs/navigation_plans/*.json
```

这些文件只用于人工检查。

当前暂不接入：

- LLM 自然语言理解；
- ROS 2 / Nav2；
- VLA；
- SLAM / AMCL；
- 动态障碍物；
- 在线局部重规划；
- 多控制源仲裁；
- 真实底盘动力学。

未来 LLM 只作为可插拔上层：

```text
自然语言
→ LLM 输出结构化命令或 TaskPlan
→ 严格校验
→ 现有导航接口
```

LLM 不应直接：

- 输出地图坐标绕过目标表；
- 输出 `vx、wz`；
- 调用 `TIANYI_BASE_CTRL`；
- 直接控制 Articulation Root。

---

## 6. 严格结构化目标接口

已新增：

```text
config/navigation_targets.json
src/navigation/
Scripts/30_list_navigation_targets.py
Scripts/31_validate_navigation_command.py
docs/NAVIGATION_COMMAND_INTERFACE.md
tests/test_navigation_commands.py
```

内部只允许严格英文 `target_id`：

```text
hf_dc_power_screen
line_maintenance_area
```

合法命令：

```python
NavigationCommand(
    action="navigate",
    target_id="hf_dc_power_screen",
)
```

当前不允许：

- 中文名称直接作为 target_id；
- alias 自动映射；
- 模糊匹配；
- 编辑距离；
- 子串匹配；
- embedding；
- LLM 推断。

`display_name` 和 `aliases` 只作为元数据保存。

### 6.1 已实现组件

#### `TargetRegistry`

支持：

- JSON 配置读取；
- 严格字段校验；
- 按 `target_id` 完全匹配查询；
- 目标列表；
- reload。

#### `CommandValidator`

能区分：

- target_id 不存在；
- target_id 格式非法；
- 目标被禁用；
- 目标尚未配置 goal pose；
- action 不受支持；
- 命令合法。

`Scripts/30_list_navigation_targets.py` 和 `Scripts/31_validate_navigation_command.py` 已通过 Isaac Sim Script Editor 冒烟测试。

---

## 7. Occupancy Map

基于场景 Mesh Collider，使用 Isaac Sim Occupancy Map Generator 生成二维地图。

地图文件：

```text
config/maps/final_scene_occupancy.png
config/maps/final_scene_occupancy.yaml
```

地图参数：

```text
width: 219
height: 399
resolution: 0.05 m/cell
origin: [-6.975, -14.975, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

世界坐标覆盖范围约为：

```text
X: -6.975 ～ 3.975
Y: -14.975 ～ 4.975
```

Occupancy Map Generator 的 Lower/Upper Bound 是三维包围盒：

- X、Y：决定地图平面范围；
- Z：决定哪些高度范围内的碰撞体投影为二维障碍。

栏杆、悬空柜体等不接触地面的障碍，只要处于扫描 Z 范围内，也会被投影为二维障碍。

---

## 8. 地图读取与坐标转换

已实现二维地图加载模块，支持：

```python
world_to_grid(x, y)
grid_to_world(row, col)
is_inside(row, col)
occupancy_value(row, col)
is_free(row, col)
is_occupied(row, col)
is_unknown(row, col)
```

已正确处理：

- 图像 row 向下增加；
- 世界 Y 轴方向；
- ROS map origin；
- resolution；
- origin yaw；
- free / occupied / unknown 分类。

检查脚本：

```text
Scripts/32_check_navigation_map.py
```

机器人根路径：

```text
/World/Robot/Geometry/base
```

已验证样例：

```text
世界原点 (0, 0)
→ grid (99, 139)
→ free
```

```text
机器人位置约 (-3.6995, 0.6030)
→ grid (87, 65)
→ free
```

说明世界坐标与地图坐标基本对齐。

---

## 9. 目标停靠位姿标定

标定脚本：

```text
Scripts/33_capture_navigation_goal_pose.py
```

### 9.1 高频开关直流电源屏

```text
target_id: hf_dc_power_screen
```

```json
{
  "pose_id": "front",
  "x": 0.27405983984563465,
  "y": 0.3931921474194034,
  "yaw": -0.19059413159050775,
  "priority": 0
}
```

地图位置：

```text
row: 91
col: 144
status: free
```

状态：

- 已写入 `config/navigation_targets.json`；
- `enabled=true`；
- 已通过命令校验。

### 9.2 线路检修区

```text
target_id: line_maintenance_area
```

```json
{
  "pose_id": "default",
  "x": -0.977578414335095,
  "y": -8.602337557121256,
  "yaw": -1.0767840573832617,
  "priority": 0
}
```

地图位置：

```text
row: 271
col: 119
status: free
```

状态：

- 已写入 `config/navigation_targets.json`；
- `enabled=true`；
- 已通过命令校验。

---

## 10. A* 全局路径规划

已实现第一版确定性栅格 A*。

相关文件包括：

```text
config/navigation_planner.json
src/navigation/a_star_planner.py
src/navigation/grid_inflation.py
src/navigation/planning_models.py
src/navigation/path_debug.py
Scripts/34_plan_navigation_path.py
tests/test_a_star_planner.py
docs/A_STAR_GLOBAL_PLANNER.md
```

### 10.1 算法设置

- 8 邻域搜索；
- 横向和纵向代价为 `1`；
- 对角代价为 `sqrt(2)`；
- 使用 Octile Distance 启发函数；
- 禁止 diagonal corner cutting；
- unknown 默认视为不可通行；
- 地图边界按膨胀半径收缩；
- 固定邻居顺序和 tie breaker，保证结果确定；
- 输出栅格路径和世界坐标路径；
- 保留精确世界坐标起点和终点；
- 目标 `yaw` 不参与 A*，只保留给最终朝向控制。

规划调试输出：

```text
outputs/navigation_plans/
```

包括：

- 原始分辨率 PNG；
- 4 倍放大 PNG；
- JSON 规划结果。

### 10.2 规划验证

初始使用 `inflation_radius_m=0.40` 时，两类目标均成功生成无穿障路径：

- `hf_dc_power_screen`；
- `line_maintenance_area`。

当前规划膨胀半径已经改为：

```text
inflation_radius_m = 0.50 m
```

因此旧的 `0.40 m` 调试结果仅作历史参考；现行配置下应重新生成路径图进行验证。

---

## 11. 路径简化

已实现确定性的 Line-of-Sight / String Pulling 路径简化。

相关文件包括：

```text
src/navigation/path_simplifier.py
Scripts/35_simplify_navigation_path.py
tests/test_path_simplifier.py
```

简化原则：

- 在膨胀后的 planning mask 上进行视线检查；
- 使用保守 Supercover 或等价覆盖逻辑；
- 禁止从障碍角点斜穿；
- 不使用样条、Bezier 或其他曲线平滑；
- 结果仍为无碰撞折线；
- 保留精确世界坐标起点、终点和目标 `yaw`。

### 11.1 已验证结果

以下结果是在 `inflation_radius_m=0.40` 时取得：

#### `hf_dc_power_screen`

```text
原始节点：80
简化节点：3
原始长度：约 4.0590 m
简化长度：约 3.9827 m
```

#### `line_maintenance_area`

```text
原始节点：236
简化节点：6
原始长度：约 12.3722 m
简化长度：约 11.9043 m
```

两张简化路径对比图已经人工检查：

- 不穿过黑色原始障碍；
- 不进入橙色膨胀区域；
- 起点、终点正确；
- 节点数量显著降低。

现行膨胀半径已改为 `0.50 m`，需要重新运行 `Scripts/35_simplify_navigation_path.py` 验证新路径。

---

## 12. 自动导航执行器

已实现自动导航执行链。

相关文件包括：

```text
config/navigation_follower.json
src/navigation/follower_models.py
src/navigation/path_follower.py
src/navigation/navigation_executor.py
Scripts/36_start_navigation.py
Scripts/37_navigation_status.py
Scripts/95_cancel_navigation.py
tests/test_path_follower.py
tests/test_navigation_executor.py
docs/AUTOMATIC_NAVIGATION_EXECUTOR.md
```

### 12.1 状态机

当前状态机包括：

```text
IDLE
→ PLANNING
→ ALIGN_TO_PATH
→ TRACK_PATH
→ ALIGN_GOAL_YAW
→ SUCCEEDED
```

异常与取消状态：

```text
FAILED
CANCELLED
```

### 12.2 PathFollower

当前路径跟踪采用简化 Pure Pursuit：

- 读取当前机器人 `x、y、yaw`；
- 找到最近路径段；
- 沿路径向前选取前视点；
- 根据 heading error 和曲率生成 `vx、wz`；
- 大航向误差时原地对准；
- 转弯时降低线速度；
- 接近终点时减速；
- 到达目标位置后单独对准目标 `yaw`；
- 最终通过 `TIANYI_BASE_CTRL.set_cmd(vx,wz)` 驱动。

速度与跟踪参数统一在：

```text
config/navigation_follower.json
```

中维护。

初始基线曾设置为：

```text
max_linear_speed: 0.15 m/s
max_angular_speed: 0.45 rad/s
lookahead_distance: 0.40 m
```

后续已进行人工提速，实际运行观测线速度约可达到 `0.23 m/s`。当前精确参数以 `config/navigation_follower.json` 为准。

### 12.3 NavigationExecutor

`NavigationExecutor.start(target_id, pose_id=None)` 会：

1. 检查 `builtins.TIANYI_BASE_CTRL`；
2. 校验严格目标命令；
3. 读取机器人当前精确位姿；
4. 重新运行 A*；
5. 重新运行路径简化；
6. 创建 PathFollower；
7. 注册物理更新回调；
8. 连续输出 `vx、wz`。

成功、失败或取消时会：

- 输出零速度；
- 停止底盘；
- 清理导航回调；
- 保留最终状态供查询。

---

## 13. 规划膨胀与运行时碰撞策略

### 13.1 规划膨胀

当前：

```text
inflation_radius_m = 0.50 m
```

该参数用于：

- A*；
- 路径简化；
- 起终点合法性检查；
- 规划调试图。

### 13.2 planning blocked 与真实碰撞的区别

- 黑色：原始 Occupancy Map 障碍；
- 橙色：按膨胀半径扩展出的 planning blocked 区域；
- planning blocked 不等同于真实 Mesh 碰撞。

Pure Pursuit 在转角处可能切内角，使机器人中心短暂进入橙色膨胀区，即使规划折线本身仍位于白色区域。

### 13.3 当前运行时策略

`config/navigation_follower.json` 已增加：

```json
{
  "stop_on_inflated_blocked_cell": false,
  "stop_on_raw_occupied_cell": false,
  "warn_on_blocked_cell": true,
  "progress_timeout_seconds": 5.0,
  "minimum_progress_distance": 0.05
}
```

含义：

- 进入膨胀区域时不主动停止，只限频警告；
- 进入 raw occupied 时当前也不由导航程序主动停止，允许 Mesh Collider 处理实际接触；
- 超出 Occupancy Map 范围仍立即失败；
- 路径偏离过大仍立即失败；
- `TRACK_PATH` 中长时间无位移会触发 stall/no-progress 失败，避免持续顶住障碍。

当前阶段接受真实碰撞，目标是先跑通确定性导航闭环，而不是建立严格安全导航系统。

---

## 14. 已知运行时问题与规则

### 14.1 包级导出

若脚本使用：

```python
from src.navigation import SomeName
```

则必须同步确认：

- 名称在实际模块中存在；
- `src/navigation/__init__.py` 已真实 import；
- `__all__` 已包含；
- 不存在循环导入。

已处理过的公共导出包括：

- `OccupancyGridMap`；
- `MapMetadata`；
- `AStarPlanner`；
- `NavigationExecutor`；
- 自动导航脚本使用的其他公共模型与服务。

清理 `sys.modules` 只能刷新模块，不能解决未导出名称。

### 14.2 Isaac Sim 旧物理回调残留

修改 `navigation_executor.py` 等运行时模块后：

- 删除 `sys.modules` 不会注销已经注册的旧物理回调；
- 旧对象可能继续执行修改前的函数逻辑；
- traceback 可能显示新源码行，但实际异常来自旧函数对象；
- 可能出现新版回调和旧版孤儿回调同时运行。

因此，修改导航运行时回调逻辑后，推荐：

```text
停止 Play
→ 完全重启 Isaac Sim
→ 重新打开 final_scene.usd
→ 点击 Play
→ 重新启动底盘控制器
→ 重新启动导航
```

不要只依赖重复 import 或重新运行启动脚本来清理旧回调。

### 14.3 Script Editor 中文显示

Script Editor 中中文可能显示为问号，属于终端字体或编码显示问题。

不影响：

- 英文 `target_id`；
- JSON；
- 坐标；
- 规划；
- 导航控制。

---

## 15. 测试状态

最近一次代码调整后：

```text
全部 unittest：117 tests OK
Python 语法检查：通过
import src.navigation：通过
包级导入契约检查：通过
```

纯算法测试覆盖：

- 目标命令校验；
- Occupancy Grid；
- A*；
- 障碍物膨胀；
- corner cutting；
- 路径简化；
- Supercover Line-of-Sight；
- Pure Pursuit；
- 状态机；
- 运行时 blocked-cell 策略；
- no-progress/stall；
- 回调失败后的停止与清理。

---

## 16. 当前已完成状态

```text
Q9000 真实场景扫描
→ Mesh / 3DGS 场景构建
→ Isaac Sim 场景加载
→ Mesh Collider
→ Occupancy Map
→ 地图加载与坐标转换
→ 严格目标注册表与命令校验
→ 两个真实停靠位姿标定
→ 8 邻域 A*
→ 路径可视化
→ Line-of-Sight 路径简化
→ Pure Pursuit 路径跟踪
→ 自动导航执行器
→ vx、wz 输出
→ TIANYI_BASE_CTRL
```

自动导航已经能够：

- 从当前机器人位置重新规划；
- 输出有效 `vx、wz`；
- 驱动机器人沿路径运动；
- 查询状态；
- 取消并停止；
- 执行目标末端 yaw 对准逻辑。

当前仍在完成最终运行时联调：

- 新的 `0.50 m` 膨胀路径需重新可视化验证；
- 新的 blocked-cell 非硬停止策略需在完全重启 Isaac Sim 后重新验证；
- 需要确认从 `hf_dc_power_screen` 到 `line_maintenance_area` 能完整进入 `SUCCEEDED`。

---

## 17. 当前下一步

### 第一步：干净环境验证新配置

```text
完全重启 Isaac Sim
→ 打开 final_scene.usd
→ 点击 Play
→ start_base_controller.py
```

然后分别在 `inflation_radius_m=0.50 m` 下重新运行：

```text
Scripts/34_plan_navigation_path.py
Scripts/35_simplify_navigation_path.py
```

确认两个目标仍有合法路径。

### 第二步：完成长路径自动导航验收

从 `hf_dc_power_screen` 位置启动：

```text
TARGET_ID = "line_maintenance_area"
```

确认：

- 进入橙色膨胀区时只警告、不硬停止；
- 没有旧物理回调继续抛出旧版 blocked-cell 异常；
- 路径偏离和 stall 保护工作正常；
- 最终进入 `ALIGN_GOAL_YAW`；
- 最终状态为 `SUCCEEDED`；
- 到达后停止。

验收通过后，再考虑：

- 转角自适应前视距离；
- 转角减速；
- 更平滑的跟踪；
- 机械臂操作接口；
- 可插拔 LLM / VLA 上层任务模块。

---

## 18. 当前禁止事项

除非后续明确改变方案，否则不要：

- 绕过 `TIANYI_BASE_CTRL` 直接操作 Articulation Root；
- 使用普通终端运行 Isaac Sim 控制脚本；
- 创建 standalone `SimulationApp`；
- 使用 `__file__` 推导项目根目录；
- 创建 Conda 环境；
- 在当前阶段接入 ROS 2 / Nav2；
- 在当前阶段接入 LLM 或 VLA；
- 在当前阶段实现真实底盘动力学；
- 让 LLM 直接输出坐标或速度控制量；
- 把历史 PNG/JSON 调试文件当成运行时固定路径；
- 修改运行时回调模块后，仅通过清理 `sys.modules` 假设旧回调已经消失。

---

## 19. 文档维护规则

每次完成一个技术阶段后，更新以下内容：

1. 新增或修改的文件；
2. 关键接口；
3. 实际参数；
4. 已通过的测试；
5. 已知问题；
6. 当前阶段；
7. 下一步；
8. 废弃方案。

### 变更记录

- 2026-07-24：从通用记忆摘要中抽离 Scan2Nav 技术细节，建立独立项目技术记忆文档。
- 2026-07-27：完成 A*、路径简化、Pure Pursuit 路径跟踪和自动导航执行器；规划膨胀半径调整为 0.50 m；运行时允许进入 planning blocked 区域并增加 stall 检测；进入最终自动导航联调阶段。
