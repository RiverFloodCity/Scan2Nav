import builtins


controller = getattr(
    builtins,
    "TIANYI_BASE_CTRL",
    None,
)

if controller is None:
    raise RuntimeError(
        "底盘控制器尚未创建，无法执行减速停止。"
    )

controller.soft_stop()
