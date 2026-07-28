import builtins
import importlib
import sys


PROJECT_ROOT = "/home/ps/ScanEverything/Test"
GLOBAL_EXECUTOR_NAME = "SCAN2NAV_EXECUTOR"

while PROJECT_ROOT in sys.path:
    sys.path.remove(PROJECT_ROOT)

sys.path.insert(0, PROJECT_ROOT)
importlib.invalidate_caches()


executor = getattr(builtins, GLOBAL_EXECUTOR_NAME, None)

if executor is None:
    print("当前没有 builtins.SCAN2NAV_EXECUTOR。")
else:
    status = executor.status()
    print("======================================")
    print("Scan2Nav navigation status")
    print("======================================")
    print("state：", status.state.value)
    print("target_id：", status.target_id)
    print("pose_id：", status.pose_id)
    print("message：", status.message)
    print("current x, y, yaw：", (status.current_x, status.current_y, status.current_yaw))
    print("goal x, y, yaw：", (status.goal_x, status.goal_y, status.goal_yaw))
    print("distance_to_goal：", status.distance_to_goal)
    print("yaw_error：", status.yaw_error)
    print("path_deviation：", status.path_deviation)
    print("current_segment_index：", status.current_segment_index)
    print("last vx, wz：", (status.last_vx, status.last_wz))
    print("elapsed_seconds：", status.elapsed_seconds)
    print("======================================")
