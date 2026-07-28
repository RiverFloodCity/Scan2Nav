import builtins


controller = getattr(
    builtins,
    "TIANYI_BASE_CTRL",
    None,
)

if controller is None:
    raise RuntimeError(
        "底盘控制器尚未创建。"
        "请先点击 Play，然后运行 "
        "00_start_base_controller.py。"
    )

controller.status()
