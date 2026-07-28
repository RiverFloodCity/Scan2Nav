import builtins


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
    vx=0.0,
    wz=2.0,
)
