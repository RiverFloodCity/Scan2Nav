import builtins


# 在这里修改自定义速度。
#
# vx > 0：前进
# vx < 0：后退
# wz > 0：左转
# wz < 0：右转
#
# 当前理想差速底盘不支持横移 vy。

VX = 0.10
WZ = 0.00


controller = getattr(
    builtins,
    "TIANYI_BASE_CTRL",
    None,
)

if controller is None:
    raise RuntimeError(
        "底盘控制器尚未创建。"
        "请先运行 00_start_base_controller.py。"
    )

controller.set_cmd(
    vx=VX,
    wz=WZ,
)
