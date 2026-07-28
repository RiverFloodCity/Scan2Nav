import importlib
import sys


PROJECT_ROOT = "/home/ps/ScanEverything/Test"

while PROJECT_ROOT in sys.path:
    sys.path.remove(PROJECT_ROOT)

sys.path.insert(0, PROJECT_ROOT)
importlib.invalidate_caches()

for module_name in list(sys.modules):
    if module_name == "src" or module_name.startswith("src.navigation"):
        del sys.modules[module_name]

from src.navigation import TargetRegistry  # noqa: E402
import src  # noqa: E402


print("[Navigation] project root:", PROJECT_ROOT)
print("[Navigation] src package:", getattr(src, "__file__", None))
print("[Navigation] src search path:", list(getattr(src, "__path__", [])))


registry = TargetRegistry()

print("======================================")
print("Registered navigation targets")
print("======================================")

for target in registry.list_targets():
    print("target_id：", target.target_id)
    print("display_name：", target.display_name)
    print("enabled：", target.enabled)
    print("goal pose count：", len(target.goal_poses))
    print("--------------------------------------")

print("Available target_ids：")
for target_id in registry.list_target_ids():
    print("-", target_id)
