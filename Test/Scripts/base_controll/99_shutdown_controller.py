import builtins


controller = getattr(
    builtins,
    "TIANYI_BASE_CTRL",
    None,
)

if controller is None:
    print("底盘控制器当前不存在，无需关闭。")
else:
    controller.shutdown()

    try:
        delattr(
            builtins,
            "TIANYI_BASE_CTRL",
        )
    except AttributeError:
        pass

    print(
        "底盘控制器已经关闭，"
        "全局对象 TIANYI_BASE_CTRL 已删除。"
    )
