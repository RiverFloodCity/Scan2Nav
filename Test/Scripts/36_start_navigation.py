import builtins
import importlib
import sys


PROJECT_ROOT = "/home/ps/ScanEverything/Test"
TARGET_ID = "hf_dc_power_screen"
POSE_ID = None

GLOBAL_EXECUTOR_NAME = "SCAN2NAV_EXECUTOR"
GLOBAL_BASE_CONTROLLER_NAME = "TIANYI_BASE_CTRL"

while PROJECT_ROOT in sys.path:
    sys.path.remove(PROJECT_ROOT)

sys.path.insert(0, PROJECT_ROOT)
importlib.invalidate_caches()

for module_name in list(sys.modules):
    if module_name == "src" or module_name.startswith("src."):
        del sys.modules[module_name]

from src.ideal_base_controller import ROBOT_PATH  # noqa: E402
from src.navigation import NavigationExecutor  # noqa: E402


def shutdown_old_executor():
    old_executor = getattr(builtins, GLOBAL_EXECUTOR_NAME, None)
    if old_executor is None:
        return

    print("检测到旧导航执行器，正在取消并清理。")
    try:
        old_executor.cancel()
    except Exception as error:
        print("取消旧导航执行器时发生异常：", repr(error))

    try:
        delattr(builtins, GLOBAL_EXECUTOR_NAME)
    except AttributeError:
        pass


def require_base_controller():
    controller = getattr(builtins, GLOBAL_BASE_CONTROLLER_NAME, None)
    if controller is None:
        raise RuntimeError(
            "builtins.TIANYI_BASE_CTRL 不存在。"
            "请先运行 Scripts/start_base_controller.py。"
        )
    return controller


print("======================================")
print("正在启动 Scan2Nav 自动导航")
print("项目目录：", PROJECT_ROOT)
print("target_id：", TARGET_ID)
print("pose_id：", POSE_ID)
print("机器人路径：", ROBOT_PATH)
print("======================================")

require_base_controller()
shutdown_old_executor()

executor = NavigationExecutor(robot_path=ROBOT_PATH)

try:
    simplified_path = executor.start(TARGET_ID, pose_id=POSE_ID)
except Exception as error:
    if hasattr(builtins, GLOBAL_EXECUTOR_NAME):
        delattr(builtins, GLOBAL_EXECUTOR_NAME)
    print("自动导航启动失败：", repr(error))
    raise

builtins.SCAN2NAV_EXECUTOR = executor

status = executor.status()
print("导航执行器全局名称：builtins.SCAN2NAV_EXECUTOR")
print("初始状态：", status.state.value)
print("目标：", status.target_id, status.pose_id)
print("起点：", (status.current_x, status.current_y, status.current_yaw))
print("终点：", (status.goal_x, status.goal_y, status.goal_yaw))
print("原始节点数：", simplified_path.original_node_count)
print("简化节点数：", simplified_path.simplified_node_count)
print("原始长度 m：", simplified_path.original_length_m)
print("简化长度 m：", simplified_path.simplified_length_m)
print("导航已经注册物理回调，将在 Play 状态下非阻塞执行。")
