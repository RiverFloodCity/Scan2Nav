import importlib
import json
import math
import sys


PROJECT_ROOT = "/home/ps/ScanEverything/Test"

TARGET_ID = "line_maintenance_area"
POSE_ID = "default"
PRIORITY = 0

MAP_YAML_PATH = (
    "/home/ps/ScanEverything/Test/"
    "config/maps/final_scene_occupancy.yaml"
)

while PROJECT_ROOT in sys.path:
    sys.path.remove(PROJECT_ROOT)

sys.path.insert(0, PROJECT_ROOT)
importlib.invalidate_caches()

for module_name in list(sys.modules):
    if module_name == "src" or module_name.startswith("src."):
        del sys.modules[module_name]

from src.ideal_base_controller import ROBOT_PATH  # noqa: E402
from src.navigation import (  # noqa: E402
    InvalidTargetIdError,
    OccupancyGridMap,
    TargetRegistry,
    UnknownNavigationTargetError,
)
from src.navigation.pose_utils import quaternion_wxyz_to_yaw  # noqa: E402


def get_robot_world_pose():
    try:
        import omni.usd
        from pxr import Usd, UsdGeom
    except Exception as error:
        raise RuntimeError(
            "无法导入 Isaac Sim Stage API。"
            "请在 Isaac Sim Script Editor 中运行本脚本。"
        ) from error

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError(
            "当前没有可用 Stage。请先在 Isaac Sim 中打开正式场景。"
        )

    prim = stage.GetPrimAtPath(ROBOT_PATH)
    if not prim or not prim.IsValid():
        raise RuntimeError(
            f"找不到机器人 Prim：{ROBOT_PATH}"
        )

    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        raise RuntimeError(
            f"机器人 Prim 不支持 Xformable：{ROBOT_PATH}"
        )

    transform = xformable.ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    translation = transform.ExtractTranslation()
    quaternion = transform.ExtractRotationQuat()
    imaginary = quaternion.GetImaginary()
    yaw = quaternion_wxyz_to_yaw(
        quaternion.GetReal(),
        imaginary[0],
        imaginary[1],
        imaginary[2],
    )

    return (
        float(translation[0]),
        float(translation[1]),
        float(translation[2]),
        yaw,
    )


def print_available_target_ids(registry):
    print("Available target_ids:")
    for target_id in registry.list_target_ids():
        print("-", target_id)


registry = TargetRegistry()

try:
    target = registry.get_target(TARGET_ID)
except (InvalidTargetIdError, UnknownNavigationTargetError) as error:
    print("======================================")
    print("Navigation target is not valid or not found")
    print("======================================")
    print(error)
    print_available_target_ids(registry)
    raise

grid_map = OccupancyGridMap.from_yaml(MAP_YAML_PATH)
x, y, z, yaw = get_robot_world_pose()
yaw_degrees = math.degrees(yaw)

print("======================================")
print("Navigation goal pose capture")
print("======================================")
print("TARGET_ID：", TARGET_ID)
print("display_name：", target.display_name)
print("POSE_ID：", POSE_ID)
print("PRIORITY：", PRIORITY)
print("ROBOT_PATH：", ROBOT_PATH)
print("robot world position x, y, z：", (x, y, z))
print("yaw radians：", yaw)
print("yaw degrees：", yaw_degrees)
print("map yaml：", MAP_YAML_PATH)
print("map image：", grid_map.metadata.image_path)

try:
    row, col = grid_map.world_to_grid(x, y)
except ValueError as error:
    print("grid row, col：outside")
    print("occupancy status：outside")
    print("当前位置超出地图范围，不建议写入目标配置。")
    print("reason：", error)
else:
    occupancy_value = grid_map.occupancy_value(row, col)
    occupancy_status = grid_map.status_at(row, col)
    print("grid row, col：", (row, col))
    print("occupancy value：", occupancy_value)
    print("occupancy status：", occupancy_status)

    if occupancy_status == "free":
        goal_pose = {
            "pose_id": POSE_ID,
            "x": x,
            "y": y,
            "yaw": yaw,
            "priority": PRIORITY,
        }
        print("======================================")
        print("JSON snippet for config/navigation_targets.json")
        print("======================================")
        print(
            json.dumps(
                goal_pose,
                ensure_ascii=False,
                indent=2,
            )
        )
    elif occupancy_status == "occupied":
        print("当前位置位于障碍栅格，不建议写入目标配置。")
    else:
        print("当前位置位于未知区域，不建议写入目标配置。")
