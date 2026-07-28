"""Isaac Sim adapter and lifecycle for automatic Scan2Nav execution."""

from __future__ import annotations

import builtins
import math
from pathlib import Path
import time
import traceback
from typing import Callable

from .a_star_planner import AStarPlanner, load_planner_config
from .command_validator import CommandValidator
from .exceptions import (
    GoalOutsideMapError,
    NavigationError,
    NavigationExecutionError,
    StartOutsideMapError,
)
from .follower_models import FollowerConfig, NavigationState, NavigationStatus
from .models import GoalPose, NavigationCommand
from .occupancy_grid import OccupancyGridMap
from .path_follower import PathFollower, load_follower_config
from .path_simplifier import PathSimplifier
from .planning_models import GridCell, PlannerConfig
from .pose_utils import quaternion_wxyz_to_yaw
from .simplification_models import SimplifiedPath
from .target_registry import TargetRegistry


PROJECT_ROOT = Path("/home/ps/ScanEverything/Test")
DEFAULT_MAP_YAML_PATH = PROJECT_ROOT / "config" / "maps" / "final_scene_occupancy.yaml"
GLOBAL_BASE_CONTROLLER_NAME = "TIANYI_BASE_CTRL"
_ACTIVE_EXECUTOR = None


class DefaultIsaacPoseProvider:
    """Read robot world pose from the current Isaac Sim Stage."""

    def __init__(self, robot_path: str):
        self.robot_path = robot_path

    def get_pose(self) -> tuple[float, float, float, float]:
        try:
            import omni.usd
            from pxr import Usd, UsdGeom
        except Exception as error:
            raise RuntimeError(
                "无法导入 Isaac Sim Stage API。"
                "请在 Isaac Sim Script Editor 中运行导航执行器。"
            ) from error

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("当前没有可用 Stage。请先打开正式场景。")

        prim = stage.GetPrimAtPath(self.robot_path)
        if not prim or not prim.IsValid():
            raise RuntimeError(f"找不到机器人 Prim：{self.robot_path}")

        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            raise RuntimeError(f"机器人 Prim 不支持 Xformable：{self.robot_path}")

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


class DefaultSimulationCallbackRegistry:
    """Register one physics callback through Isaac Sim SimulationManager."""

    def register(self, callback: Callable):
        from isaacsim.core.simulation_manager import (
            SimulationEvent,
            SimulationManager,
        )

        return SimulationManager.register_callback(
            callback,
            event=SimulationEvent.PHYSICS_POST_STEP,
        )

    def deregister(self, callback_id) -> None:
        from isaacsim.core.simulation_manager import SimulationManager

        SimulationManager.deregister_callback(callback_id)


class NavigationExecutor:
    """Plan, simplify, and follow a navigation target through base set_cmd."""

    def __init__(
        self,
        *,
        robot_path: str,
        map_yaml_path: str | Path = DEFAULT_MAP_YAML_PATH,
        target_registry: TargetRegistry | None = None,
        command_validator: CommandValidator | None = None,
        occupancy_map: OccupancyGridMap | None = None,
        planner_config: PlannerConfig | None = None,
        follower_config: FollowerConfig | None = None,
        pose_provider=None,
        base_controller=None,
        callback_registry=None,
        time_provider: Callable[[], float] = time.monotonic,
        warning_sink: Callable[[str], None] = print,
    ):
        self.robot_path = robot_path
        self.map_yaml_path = Path(map_yaml_path)
        self.target_registry = target_registry
        self.command_validator = command_validator
        self.occupancy_map = occupancy_map
        self.planner_config = planner_config
        self.follower_config = follower_config
        self.pose_provider = pose_provider
        self.base_controller = base_controller
        self.callback_registry = callback_registry
        self.time_provider = time_provider
        self.warning_sink = warning_sink

        self.planner: AStarPlanner | None = None
        self.follower: PathFollower | None = None
        self.simplified_path: SimplifiedPath | None = None
        self.callback_id = None
        self.started_at: float | None = None
        self.running = False
        self._last_blocked_warning_at: float | None = None
        self._progress_anchor_x: float | None = None
        self._progress_anchor_y: float | None = None
        self._progress_anchor_time: float | None = None
        self._status = NavigationStatus(
            state=NavigationState.IDLE,
            message="Navigation executor is idle.",
        )

    def start(self, target_id: str, pose_id: str | None = None) -> SimplifiedPath:
        """Plan and start non-blocking physical callback execution."""
        global _ACTIVE_EXECUTOR

        if self.running:
            raise NavigationExecutionError("Navigation executor is already running.")

        old_executor = _ACTIVE_EXECUTOR
        if old_executor is not None and old_executor is not self:
            old_executor.cancel()

        self._set_status(
            state=NavigationState.PLANNING,
            target_id=target_id,
            pose_id=pose_id,
            message="Planning navigation path.",
        )

        try:
            base_controller = self._resolve_base_controller()
            self.base_controller = base_controller
            registry = self.target_registry or TargetRegistry()
            validator = self.command_validator or CommandValidator(registry)
            target = validator.validate(
                NavigationCommand(
                    action="navigate",
                    target_id=target_id,
                )
            )
            goal_pose = select_goal_pose(target.goal_poses, pose_id)

            occupancy_map = self.occupancy_map or OccupancyGridMap.from_yaml(
                self.map_yaml_path
            )
            planner_config = self.planner_config or load_planner_config()
            follower_config = self.follower_config or load_follower_config()
            planner = AStarPlanner(
                occupancy_map=occupancy_map,
                config=planner_config,
            )

            pose_provider = self.pose_provider or DefaultIsaacPoseProvider(
                self.robot_path
            )
            robot_x, robot_y, _, robot_yaw = pose_provider.get_pose()
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
            simplifier.validate_simplified_path(
                simplified_path.simplified_grid_path
            )

            follower = PathFollower(
                world_path=simplified_path.simplified_world_path,
                goal_yaw=simplified_path.goal_yaw,
                config=follower_config,
            )

            self.pose_provider = pose_provider
            self.occupancy_map = occupancy_map
            self.planner_config = planner_config
            self.follower_config = follower_config
            self.planner = planner
            self.follower = follower
            self.simplified_path = simplified_path
            self.callback_registry = (
                self.callback_registry or DefaultSimulationCallbackRegistry()
            )
            self.started_at = self.time_provider()
            self._reset_progress_anchor(robot_x, robot_y)
            self.callback_id = self.callback_registry.register(
                self._on_physics_step
            )
            self.running = True
            _ACTIVE_EXECUTOR = self

            initial_state = (
                NavigationState.ALIGN_GOAL_YAW
                if planned_path.length_m <= follower_config.goal_position_tolerance
                else NavigationState.ALIGN_TO_PATH
            )
            follower.state = initial_state
            self._set_status(
                state=initial_state,
                target_id=target.target_id,
                pose_id=goal_pose.pose_id,
                message="Navigation callback registered.",
                current_x=robot_x,
                current_y=robot_y,
                current_yaw=robot_yaw,
                goal_x=goal_pose.x,
                goal_y=goal_pose.y,
                goal_yaw=goal_pose.yaw,
                distance_to_goal=planned_path.length_m,
                yaw_error=0.0,
                path_deviation=0.0,
                current_segment_index=0,
                last_vx=0.0,
                last_wz=0.0,
            )
            return simplified_path

        except Exception as error:
            self._safe_stop_base(hard=True)
            self._cleanup_callback()
            self.running = False
            self._set_status(
                state=NavigationState.FAILED,
                target_id=target_id,
                pose_id=pose_id,
                message=f"Navigation failed to start: {error!r}",
            )
            raise

    def cancel(self) -> NavigationStatus:
        """Cancel active navigation and stop safely."""
        self._safe_stop_base(hard=False)
        self._cleanup_callback()
        self.running = False
        self._set_status(
            state=NavigationState.CANCELLED,
            message="Navigation was cancelled.",
            last_vx=0.0,
            last_wz=0.0,
        )
        self._clear_active_executor()
        return self.status()

    def shutdown(self) -> NavigationStatus:
        """Safe shutdown; equivalent to cancel for first-version executor."""
        return self.cancel()

    def status(self) -> NavigationStatus:
        """Return the latest status snapshot."""
        return self._status

    def _on_physics_step(self, dt, context) -> None:
        if not self.running or self.follower is None:
            return

        try:
            assert self.pose_provider is not None
            assert self.occupancy_map is not None
            robot_x, robot_y, _, robot_yaw = self.pose_provider.get_pose()
            try:
                row, col = self.occupancy_map.world_to_grid(robot_x, robot_y)
            except ValueError as error:
                raise NavigationExecutionError(
                    "Robot ran outside occupancy map during navigation: "
                    f"x={robot_x}, y={robot_y}."
                ) from error

            raw_status = self.occupancy_map.status_at(row, col)
            inflated_blocked = (
                self.planner is not None
                and self.planner.planning_grid.is_inflated_blocked(
                    GridCell(row=row, col=col)
                )
            )

            output = self.follower.step(
                x=robot_x,
                y=robot_y,
                yaw=robot_yaw,
            )

            blocked_failure = self._blocked_cell_failure_message(
                x=robot_x,
                y=robot_y,
                row=row,
                col=col,
                raw_status=raw_status,
                inflated_blocked=inflated_blocked,
            )
            if blocked_failure is not None:
                self._set_status_from_output(
                    output,
                    current_x=robot_x,
                    current_y=robot_y,
                    current_yaw=robot_yaw,
                )
                self._fail_runtime(NavigationExecutionError(blocked_failure))
                return

            self._warn_on_blocked_cell(
                x=robot_x,
                y=robot_y,
                row=row,
                col=col,
                raw_status=raw_status,
                inflated_blocked=inflated_blocked,
                path_deviation=output.path_deviation,
                state=output.state,
                vx=output.vx,
                wz=output.wz,
            )

            stall_failure = self._stall_failure_message(
                x=robot_x,
                y=robot_y,
                output=output,
            )
            if stall_failure is not None:
                self._set_status_from_output(
                    output,
                    current_x=robot_x,
                    current_y=robot_y,
                    current_yaw=robot_yaw,
                )
                self._fail_runtime(NavigationExecutionError(stall_failure))
                return

            self._set_base_cmd(output.vx, output.wz)
            self._set_status_from_output(
                output,
                current_x=robot_x,
                current_y=robot_y,
                current_yaw=robot_yaw,
            )

            if output.state == NavigationState.SUCCEEDED:
                self._safe_stop_base(hard=False)
                self._cleanup_callback()
                self.running = False
                self._clear_active_executor()
            elif output.state == NavigationState.FAILED:
                self._safe_stop_base(hard=True)
                self._cleanup_callback()
                self.running = False
                self._clear_active_executor()

        except Exception as error:
            traceback.print_exc()
            self._fail_runtime(error)
            return

    def _fail_runtime(self, error: Exception) -> None:
        self._safe_stop_base(hard=True)
        self._cleanup_callback()
        self.running = False
        self._set_status(
            state=NavigationState.FAILED,
            message=f"Navigation runtime failure: {error!r}",
            last_vx=0.0,
            last_wz=0.0,
        )
        self._clear_active_executor()

    def _blocked_cell_failure_message(
        self,
        *,
        x: float,
        y: float,
        row: int,
        col: int,
        raw_status: str,
        inflated_blocked: bool,
    ) -> str | None:
        assert self.follower_config is not None
        if raw_status == "occupied" and self.follower_config.stop_on_raw_occupied_cell:
            return (
                "Robot entered raw occupied map cell and "
                "stop_on_raw_occupied_cell=true: "
                f"x={x}, y={y}, row={row}, col={col}."
            )
        if (
            inflated_blocked
            and self.follower_config.stop_on_inflated_blocked_cell
        ):
            return (
                "Robot entered inflated planning blocked cell and "
                "stop_on_inflated_blocked_cell=true: "
                f"x={x}, y={y}, row={row}, col={col}, "
                f"raw_status={raw_status}."
            )
        return None

    def _warn_on_blocked_cell(
        self,
        *,
        x: float,
        y: float,
        row: int,
        col: int,
        raw_status: str,
        inflated_blocked: bool,
        path_deviation: float,
        state: NavigationState,
        vx: float,
        wz: float,
    ) -> None:
        assert self.follower_config is not None
        if not self.follower_config.warn_on_blocked_cell:
            return
        if raw_status == "free" and not inflated_blocked:
            return

        now = self.time_provider()
        if (
            self._last_blocked_warning_at is not None
            and now - self._last_blocked_warning_at < 1.0
        ):
            return

        self._last_blocked_warning_at = now
        self.warning_sink(
            "[NavigationExecutor] blocked-cell warning: "
            f"world=({x:.3f}, {y:.3f}), grid=({row}, {col}), "
            f"raw_status={raw_status}, inflated_blocked={inflated_blocked}, "
            f"path_deviation={path_deviation:.3f}, state={state.value}, "
            f"vx={vx:.3f}, wz={wz:.3f}."
        )

    def _stall_failure_message(self, *, x: float, y: float, output) -> str | None:
        assert self.follower_config is not None
        if output.state != NavigationState.TRACK_PATH or output.vx <= 0.0:
            self._reset_progress_anchor(x, y)
            return None

        now = self.time_provider()
        if self._progress_anchor_time is None:
            self._reset_progress_anchor(x, y)
            return None

        assert self._progress_anchor_x is not None
        assert self._progress_anchor_y is not None
        moved = math.hypot(
            x - self._progress_anchor_x,
            y - self._progress_anchor_y,
        )
        elapsed = now - self._progress_anchor_time

        if moved >= self.follower_config.minimum_progress_distance:
            self._reset_progress_anchor(x, y)
            return None

        if elapsed >= self.follower_config.progress_timeout_seconds:
            return (
                "Navigation made no progress while TRACK_PATH commanded "
                "positive vx; possible Mesh collision or path-following stall. "
                f"elapsed={elapsed:.3f} s, moved={moved:.3f} m, "
                "minimum_progress_distance="
                f"{self.follower_config.minimum_progress_distance:.3f} m."
            )

        return None

    def _reset_progress_anchor(self, x: float, y: float) -> None:
        self._progress_anchor_x = float(x)
        self._progress_anchor_y = float(y)
        self._progress_anchor_time = self.time_provider()

    def _resolve_base_controller(self):
        controller = self.base_controller
        if controller is None:
            controller = getattr(
                builtins,
                GLOBAL_BASE_CONTROLLER_NAME,
                None,
            )
        if controller is None:
            raise NavigationExecutionError(
                "builtins.TIANYI_BASE_CTRL does not exist. "
                "Run Scripts/start_base_controller.py first."
            )
        for method_name in ("set_cmd", "soft_stop", "hard_stop"):
            if not callable(getattr(controller, method_name, None)):
                raise NavigationExecutionError(
                    "Base controller is missing required method: "
                    f"{method_name}."
                )
        if getattr(controller, "running", True) is False:
            raise NavigationExecutionError("Base controller is not running.")
        return controller

    def _set_base_cmd(self, vx: float, wz: float) -> None:
        if self.base_controller is None:
            raise NavigationExecutionError("Base controller is not available.")
        self.base_controller.set_cmd(float(vx), float(wz))

    def _safe_stop_base(self, *, hard: bool) -> None:
        if self.base_controller is None:
            return
        try:
            self.base_controller.set_cmd(0.0, 0.0)
        except Exception:
            pass
        try:
            if hard:
                self.base_controller.hard_stop()
            else:
                self.base_controller.soft_stop()
        except Exception:
            pass

    def _cleanup_callback(self) -> None:
        if self.callback_id is None or self.callback_registry is None:
            self.callback_id = None
            return
        try:
            self.callback_registry.deregister(self.callback_id)
        except Exception as error:
            print("[NavigationExecutor] 注销导航物理回调失败：", repr(error))
        self.callback_id = None

    def _clear_active_executor(self) -> None:
        global _ACTIVE_EXECUTOR
        if _ACTIVE_EXECUTOR is self:
            _ACTIVE_EXECUTOR = None

    def _set_status_from_output(
        self,
        output,
        *,
        current_x: float,
        current_y: float,
        current_yaw: float,
    ) -> None:
        current_status = self._status
        self._set_status(
            state=output.state,
            target_id=current_status.target_id,
            pose_id=current_status.pose_id,
            message=output.message,
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            goal_x=current_status.goal_x,
            goal_y=current_status.goal_y,
            goal_yaw=current_status.goal_yaw,
            distance_to_goal=output.distance_to_goal,
            yaw_error=output.yaw_error,
            path_deviation=output.path_deviation,
            current_segment_index=output.current_segment_index,
            last_vx=output.vx,
            last_wz=output.wz,
        )

    def _set_status(
        self,
        *,
        state: NavigationState,
        target_id: str | None = None,
        pose_id: str | None = None,
        message: str = "",
        current_x: float | None = None,
        current_y: float | None = None,
        current_yaw: float | None = None,
        goal_x: float | None = None,
        goal_y: float | None = None,
        goal_yaw: float | None = None,
        distance_to_goal: float | None = None,
        yaw_error: float | None = None,
        path_deviation: float | None = None,
        current_segment_index: int = 0,
        last_vx: float = 0.0,
        last_wz: float = 0.0,
    ) -> None:
        elapsed = 0.0
        if self.started_at is not None:
            elapsed = max(0.0, self.time_provider() - self.started_at)
        previous = self._status
        self._status = NavigationStatus(
            state=state,
            target_id=target_id if target_id is not None else previous.target_id,
            pose_id=pose_id if pose_id is not None else previous.pose_id,
            message=message,
            current_x=current_x if current_x is not None else previous.current_x,
            current_y=current_y if current_y is not None else previous.current_y,
            current_yaw=(
                current_yaw if current_yaw is not None else previous.current_yaw
            ),
            goal_x=goal_x if goal_x is not None else previous.goal_x,
            goal_y=goal_y if goal_y is not None else previous.goal_y,
            goal_yaw=goal_yaw if goal_yaw is not None else previous.goal_yaw,
            distance_to_goal=(
                distance_to_goal
                if distance_to_goal is not None
                else previous.distance_to_goal
            ),
            yaw_error=yaw_error if yaw_error is not None else previous.yaw_error,
            path_deviation=(
                path_deviation
                if path_deviation is not None
                else previous.path_deviation
            ),
            current_segment_index=current_segment_index,
            last_vx=last_vx,
            last_wz=last_wz,
            elapsed_seconds=elapsed,
        )


def select_goal_pose(
    goal_poses: tuple[GoalPose, ...],
    pose_id: str | None,
) -> GoalPose:
    """Select a goal pose using the Script/34 and Script/35 rule."""
    if pose_id is not None:
        for goal_pose in goal_poses:
            if goal_pose.pose_id == pose_id:
                return goal_pose
        raise NavigationExecutionError(f"Target has no pose_id {pose_id!r}.")

    return sorted(goal_poses, key=lambda pose: (pose.priority, pose.pose_id))[0]


def world_to_cell_or_raise(
    grid_map: OccupancyGridMap,
    x: float,
    y: float,
    *,
    is_start: bool,
) -> GridCell:
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
