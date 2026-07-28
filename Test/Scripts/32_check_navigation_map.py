import importlib
import sys


PROJECT_ROOT = "/home/ps/ScanEverything/Test"
MAP_YAML_PATH = (
    "/home/ps/ScanEverything/Test/config/maps/final_scene_occupancy.yaml"
)
CHECK_POINTS = [
    (0.0, 0.0),
]

while PROJECT_ROOT in sys.path:
    sys.path.remove(PROJECT_ROOT)

sys.path.insert(0, PROJECT_ROOT)
importlib.invalidate_caches()

for module_name in list(sys.modules):
    if module_name == "src" or module_name.startswith("src."):
        del sys.modules[module_name]

import src  # noqa: E402
from src.ideal_base_controller import ROBOT_PATH  # noqa: E402
from src.navigation import MapMetadata, OccupancyGridMap  # noqa: E402


print("[Navigation] project root:", PROJECT_ROOT)
print("[Navigation] src package:", getattr(src, "__file__", None))
print("[Navigation] src search path:", list(getattr(src, "__path__", [])))
print("[Navigation] OccupancyGridMap:", OccupancyGridMap)
print("[Navigation] MapMetadata:", MapMetadata)


def print_world_point(grid_map, label, x, y):
    print(label)
    print("  world：", (x, y))
    try:
        row, col = grid_map.world_to_grid(x, y)
    except ValueError as error:
        print("  grid：outside")
        print("  reason：", error)
        return

    print("  grid row, col：", (row, col))
    print("  occupancy value：", grid_map.occupancy_value(row, col))
    print("  status：", grid_map.status_at(row, col))


def get_robot_world_position():
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
            "当前没有可用 Stage。请先在 Isaac Sim 中打开场景。"
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
    return float(translation[0]), float(translation[1]), float(translation[2])


grid_map = OccupancyGridMap.from_yaml(MAP_YAML_PATH)
metadata = grid_map.metadata
min_x, min_y, max_x, max_y = grid_map.world_bounds()

print("======================================")
print("Navigation map check")
print("======================================")
print("YAML path：", MAP_YAML_PATH)
print("PNG path：", metadata.image_path)
print("width, height：", metadata.width, metadata.height)
print("resolution：", metadata.resolution)
print(
    "origin：",
    (metadata.origin_x, metadata.origin_y, metadata.origin_yaw),
)
print("world bounds min_x, min_y, max_x, max_y：", (min_x, min_y, max_x, max_y))
print("negate：", metadata.negate)
print("occupied_thresh：", metadata.occupied_thresh)
print("free_thresh：", metadata.free_thresh)
print("======================================")

print_world_point(grid_map, "Check point", 0.0, 0.0)

for index, (x, y) in enumerate(CHECK_POINTS):
    print_world_point(grid_map, f"CHECK_POINTS[{index}]", x, y)

print("======================================")
print("Robot")
print("======================================")
print("ROBOT_PATH：", ROBOT_PATH)

robot_x, robot_y, robot_z = get_robot_world_position()
print("robot world position x, y, z：", (robot_x, robot_y, robot_z))
print_world_point(grid_map, "Robot map cell", robot_x, robot_y)
