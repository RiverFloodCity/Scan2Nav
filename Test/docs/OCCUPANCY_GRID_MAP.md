# Occupancy Grid Map

本文档说明当前二维导航地图加载与坐标转换检查接口。本阶段只实现地图读取和检查，不实现地图膨胀、A*、路径跟踪或运动控制。

## 地图文件

当前正式地图文件：

```text
config/maps/final_scene_occupancy.yaml
config/maps/final_scene_occupancy.png
```

YAML 使用 ROS Occupancy Map 常见字段：

```yaml
image: final_scene_occupancy.png
resolution: 0.05
origin: [-6.975, -14.975, 0.0000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

`image` 可以是相对路径。相对路径以 YAML 文件所在目录为基准解析。

## row 与 col

本项目所有栅格索引都使用：

```text
(row, col)
```

- `row`：图像行号，从上到下增加；
- `col`：图像列号，从左到右增加；
- 左上角像素是 `(row=0, col=0)`；
- 右下角像素是 `(row=height-1, col=width-1)`。

不要把返回值理解成 `(x_pixel, y_pixel)`。

## ROS origin 的含义

ROS map `origin: [x, y, yaw]` 表示地图左下角在世界坐标系中的位姿。

图像坐标和世界坐标有一个重要差异：

- 图像 `row` 向下增加；
- 世界坐标中的地图局部 `Y` 向上增加。

因此图像最下面一行对应地图局部 `Y` 最小区域，图像最上面一行对应地图局部 `Y` 最大区域。

## 坐标转换

`grid_to_world(row, col)` 使用像素中心：

```text
local_x = (col + 0.5) * resolution
local_y = (height - row - 0.5) * resolution
```

然后根据 `origin_yaw` 旋转并平移到世界坐标。

`world_to_grid(x, y)` 会先把世界坐标用 `origin` 的逆变换转回地图局部坐标，再计算：

```text
col = floor(local_x / resolution)
bottom_row = floor(local_y / resolution)
row = height - 1 - bottom_row
```

如果世界点在地图外，函数会抛出明确异常，不会静默截断到地图边界。

## 占用概率

PNG 被读取为像素颜色，使用 RGB 平均亮度计算占用概率。透明像素会被视为 unknown。

ROS map_server 风格规则：

```text
negate = 0: occupancy = (255 - color_average) / 255
negate = 1: occupancy = color_average / 255
```

分类规则：

```text
occupancy > occupied_thresh -> occupied
occupancy < free_thresh     -> free
otherwise                   -> unknown
```

因此不要直接假定黑色一定是障碍、白色一定是自由、灰色一定是 unknown；必须结合 `negate`、`occupied_thresh` 和 `free_thresh` 判断。

## 检查脚本

在 Isaac Sim Script Editor 中运行：

```text
Scripts/32_check_navigation_map.py
```

该脚本会打印：

- YAML 绝对路径；
- PNG 绝对路径；
- `width`、`height`；
- `resolution`；
- `origin`；
- 地图覆盖的世界坐标范围；
- 世界坐标 `(0, 0)` 对应的 `row, col`；
- `ROBOT_PATH`；
- 机器人当前世界位置；
- 机器人当前栅格状态：`free` / `occupied` / `unknown` / `outside`。

脚本顶部可以扩展：

```python
CHECK_POINTS = [
    (0.0, 0.0),
]
```

该脚本不创建 `SimulationApp`，不调用 `set_cmd()`，也不会移动机器人。

## 当前边界

当前只实现：

- YAML 元数据读取；
- PNG 静态地图读取；
- 世界坐标与栅格坐标转换；
- 栅格占用状态查询；
- Script Editor 检查脚本。

当前尚未实现：

- 地图膨胀；
- A*；
- Pure Pursuit；
- 路径跟踪；
- 自动导航运动控制。
