import importlib
import sys


PROJECT_ROOT = "/home/ps/ScanEverything/Test"
TARGET_ID = "line_maintenance_area"

while PROJECT_ROOT in sys.path:
    sys.path.remove(PROJECT_ROOT)

sys.path.insert(0, PROJECT_ROOT)
importlib.invalidate_caches()

for module_name in list(sys.modules):
    if module_name == "src" or module_name.startswith("src.navigation"):
        del sys.modules[module_name]

from src.navigation import (  # noqa: E402
    CommandValidator,
    NavigationCommand,
    NavigationError,
    TargetRegistry,
)
import src  # noqa: E402


print("[Navigation] project root:", PROJECT_ROOT)
print("[Navigation] src package:", getattr(src, "__file__", None))
print("[Navigation] src search path:", list(getattr(src, "__path__", [])))


registry = TargetRegistry()
validator = CommandValidator(registry)
command = NavigationCommand(
    action="navigate",
    target_id=TARGET_ID,
)

try:
    target = validator.validate(command)
except NavigationError as error:
    print("======================================")
    print("Navigation command rejected")
    print("======================================")
    print(error)
else:
    print("======================================")
    print("Navigation command accepted")
    print("======================================")
    print("target_id：", target.target_id)
    print("display_name：", target.display_name)
    print("goal pose count：", len(target.goal_poses))
