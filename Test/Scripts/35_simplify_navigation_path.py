import importlib
import math
import sys


PROJECT_ROOT = "/home/ps/ScanEverything/Test"
TARGET_ID = "line_maintenance_area"
POSE_ID = None

MAP_YAML_PATH = (
    "/home/ps/ScanEverything/Test/config/maps/final_scene_occupancy.yaml"
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
    AStarPlanner,
    CommandValidator,
    GridCell,
    NavigationCommand,
    NavigationError,
    OccupancyGridMap,
    PathSimplifier,
    TargetRegistry,
    load_planner_config,
)
from src.navigation.exceptions import GoalOutsideMapError, StartOutsideMapError  # noqa: E402
from src.navigation.path_debug import (  # noqa: E402
    DEBUG_SIMPLIFIED_LEGEND,
    save_simplified_path_debug_artifacts,
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
        raise RuntimeError("当前没有可用 Stage。请先打开正式场景。")

    prim = stage.GetPrimAtPath(ROBOT_PATH)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"找不到机器人 Prim：{ROBOT_PATH}")

    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        raise RuntimeError(f"机器人 Prim 不支持 Xformable：{ROBOT_PATH}")

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


def select_goal_pose(target):
    if POSE_ID is not None:
        for goal_pose in target.goal_poses:
            if goal_pose.pose_id == POSE_ID:
                return goal_pose
        raise ValueError(
            f"Target {target.target_id!r} has no pose_id {POSE_ID!r}."
        )

    return sorted(
        target.goal_poses,
        key=lambda pose: (pose.priority, pose.pose_id),
    )[0]


def world_to_cell_or_raise(grid_map, x, y, *, is_start):
    try:
        row, col = grid_map.world_to_grid(x, y)
    except ValueError as error:
        error_cls = StartOutsideMapError if is_start else GoalOutsideMapError
        label = "start" if is_start else "goal"
        raise error_cls(
            f"{label} world point is outside map: x={x}, y={y}. "
            "No snapping or clipping was applied."
        ) from error
    return GridCell(row=row, col=col)


try:
    registry = TargetRegistry()
    validator = CommandValidator(registry)
    target = validator.validate(
        NavigationCommand(
            action="navigate",
            target_id=TARGET_ID,
        )
    )
    goal_pose = select_goal_pose(target)

    occupancy_map = OccupancyGridMap.from_yaml(MAP_YAML_PATH)
    planner_config = load_planner_config()
    planner = AStarPlanner(
        occupancy_map=occupancy_map,
        config=planner_config,
    )

    robot_x, robot_y, robot_z, robot_yaw = get_robot_world_pose()
    start_cell = world_to_cell_or_raise(
        occupancy_map,
        robot_x,
        robot_y,
        is_start=True,
    )
    goal_cell = world_to_cell_or_raise(
        occupancy_map,
        goal_pose.x,
        goal_pose.y,
        is_start=False,
    )

    planned_path = planner.plan(
        start_cell=start_cell,
        goal_cell=goal_cell,
        target_id=target.target_id,
        pose_id=goal_pose.pose_id,
        exact_start_world=(robot_x, robot_y),
        exact_goal_world=(goal_pose.x, goal_pose.y),
        goal_yaw=goal_pose.yaw,
    )

    simplifier = PathSimplifier(
        traversable_mask=planner.planning_grid.traversable,
        prevent_corner_cutting=planner_config.prevent_corner_cutting,
    )
    simplified_path = simplifier.simplify(
        planned_path,
        occupancy_map=occupancy_map,
    )
    simplifier.validate_simplified_path(simplified_path.simplified_grid_path)

    artifacts = save_simplified_path_debug_artifacts(
        occupancy_map=occupancy_map,
        planning_grid=planner.planning_grid,
        simplified_path=simplified_path,
        start_world=(robot_x, robot_y, robot_yaw),
        goal_world=(goal_pose.x, goal_pose.y, goal_pose.yaw),
    )

    compression_ratio = (
        1.0
        - simplified_path.simplified_node_count
        / simplified_path.original_node_count
    )

    print("======================================")
    print("Navigation path simplification result")
    print("======================================")
    print("target_id：", target.target_id)
    print("pose_id：", goal_pose.pose_id)
    print("robot world pose x, y, z, yaw：", (robot_x, robot_y, robot_z, robot_yaw))
    print("robot yaw degrees：", math.degrees(robot_yaw))
    print("goal world pose x, y, yaw：", (goal_pose.x, goal_pose.y, goal_pose.yaw))
    print("goal yaw degrees：", math.degrees(goal_pose.yaw))
    print("start grid row, col：", (start_cell.row, start_cell.col))
    print("goal grid row, col：", (goal_cell.row, goal_cell.col))
    print("inflation radius m：", planner_config.inflation_radius_m)
    print("inflation radius cells：", planner.planning_grid.inflation_radius_cells)
    print("original node count：", simplified_path.original_node_count)
    print("simplified node count：", simplified_path.simplified_node_count)
    print("node compression ratio：", compression_ratio)
    print("original length m：", simplified_path.original_length_m)
    print("simplified length m：", simplified_path.simplified_length_m)
    print("visibility checks：", simplified_path.visibility_checks)
    print("simplified nodes：")
    for index, (cell, point) in enumerate(
        zip(
            simplified_path.simplified_grid_path,
            simplified_path.simplified_world_path,
        )
    ):
        print(
            "  ",
            index,
            "grid(row, col)=",
            (cell.row, cell.col),
            "world(x, y)=",
            (point.x, point.y),
        )
    print("debug png：", artifacts.png_path)
    print("debug png 4x：", artifacts.scaled_png_path)
    print("debug json：", artifacts.json_path)
    print(DEBUG_SIMPLIFIED_LEGEND)

except NavigationError as error:
    print("======================================")
    print("Navigation path simplification failed")
    print("======================================")
    print(error)
    raise
