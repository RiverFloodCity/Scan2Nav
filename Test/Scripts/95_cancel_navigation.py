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
    print("当前没有 builtins.SCAN2NAV_EXECUTOR，无需取消。")
else:
    status = executor.cancel()
    print("======================================")
    print("Scan2Nav navigation cancelled")
    print("======================================")
    print("state：", status.state.value)
    print("message：", status.message)
    print("last vx, wz：", (status.last_vx, status.last_wz))
    print("elapsed_seconds：", status.elapsed_seconds)
    try:
        delattr(builtins, GLOBAL_EXECUTOR_NAME)
        print("已删除 builtins.SCAN2NAV_EXECUTOR。")
    except AttributeError:
        pass
