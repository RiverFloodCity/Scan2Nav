import builtins
import sys


# ============================================================
# 项目根目录
# ============================================================

PROJECT_ROOT = "/home/ps/ScanEverything/Test"
GLOBAL_CONTROLLER_NAME = "TIANYI_BASE_CTRL"

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.ideal_base_controller import (  # noqa: E402
    IdealDifferentialBaseController,
    ROBOT_PATH,
)


def shutdown_old_controller():
    """关闭并移除旧的全局控制器。"""
    old_controller = getattr(
        builtins,
        GLOBAL_CONTROLLER_NAME,
        None,
    )

    if old_controller is None:
        return

    print("检测到旧底盘控制器，正在尝试关闭。")

    try:
        old_controller.shutdown()
    except Exception as error:
        print("关闭旧底盘控制器时发生异常：", repr(error))

    try:
        delattr(
            builtins,
            GLOBAL_CONTROLLER_NAME,
        )
    except AttributeError:
        pass


def start_controller():
    """创建并启动新的底盘控制器。"""
    shutdown_old_controller()

    controller = IdealDifferentialBaseController(ROBOT_PATH)

    try:
        controller.start()
    except Exception as error:
        if hasattr(
            builtins,
            GLOBAL_CONTROLLER_NAME,
        ):
            delattr(
                builtins,
                GLOBAL_CONTROLLER_NAME,
            )

        print("底盘控制器启动失败：", repr(error))
        raise

    return controller


print("======================================")
print("正在启动理想底盘控制器")
print("项目目录：", PROJECT_ROOT)
print("机器人路径：", ROBOT_PATH)
print("======================================")

controller = start_controller()
builtins.TIANYI_BASE_CTRL = controller

print(
    "控制器全局名称：builtins.TIANYI_BASE_CTRL"
)
print("底盘控制器启动入口执行完成")
