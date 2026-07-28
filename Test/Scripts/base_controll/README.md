# Isaac Sim 底盘控制脚本

## 标准运行顺序

1. 在 Isaac Sim 中打开 `Assets/Scene/final_scene.usd`。
2. 手动点击 Play。
3. 等待一到两个物理步。
4. 在 Script Editor 中运行：
   `Scripts/start_base_controller.py`
5. 运行对应动作脚本。
6. 使用 `90_soft_stop.py` 或 `91_hard_stop.py` 停止。
7. 结束仿真前运行：
   `99_shutdown_controller.py`
8. 再点击 Isaac Sim 的 Stop。

## 脚本说明

- `Scripts/start_base_controller.py`：唯一的底盘控制器启动入口。
- `src/ideal_base_controller.py`：只定义控制器、常量和工具函数，不应直接运行。
- `builtins.TIANYI_BASE_CTRL`：当前各控制源共享的底层运动接口。
- `01_controller_status.py`：打印控制器状态。
- `10_move_forward.py`：前进。
- `11_move_backward.py`：后退。
- `12_turn_left.py`：原地左转。
- `13_turn_right.py`：原地右转。
- `14_arc_left.py`：向左圆弧前进。
- `15_arc_right.py`：向右圆弧前进。
- `20_custom_command.py`：手动修改 VX、WZ。
- `90_soft_stop.py`：平滑停止。
- `91_hard_stop.py`：立即停止。
- `99_shutdown_controller.py`：注销物理回调并删除控制器对象。

## 注意事项

每次执行 Stop → Play，或重新打开 USD 后，都必须重新运行：

`Scripts/start_base_controller.py`

不要使用普通终端中的：

`python3 Scripts/start_base_controller.py`

这些脚本必须在 Isaac Sim Script Editor 或连接到当前 Isaac Sim 实例的代码编辑器中运行。

当前暂不实现控制源仲裁和命令超时。
