import math

import numpy as np

from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import (
    SimulationEvent,
    SimulationManager,
)


# ============================================================
# 机器人配置
# ============================================================

# 可以填写 Articulation Root，也可以填写其内部的 base 刚体路径。
# 启动时会自动解析到真正的 Articulation Root。
ROBOT_PATH = "/World/Robot/Geometry/base"

# 机器人视觉正前方相对根坐标系 +X 的偏转角：
# +X: 0，+Y: 90，-X: 180，-Y: -90
FORWARD_YAW_OFFSET_DEG = 0.0


# ============================================================
# 速度限制
# ============================================================

MAX_LINEAR_SPEED = 0.30       # m/s
MAX_ANGULAR_SPEED = 0.50      # rad/s

MAX_LINEAR_ACCEL = 0.40       # m/s^2
MAX_ANGULAR_ACCEL = 0.80      # rad/s^2


def to_numpy(value):
    """将 Isaac Sim Experimental API 返回的 Warp 数组转为 NumPy。"""
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.asarray(value)


class IdealDifferentialBaseController:
    """
    理想化差速底盘控制器。

    输入：
        vx：机器人机体前后速度，单位 m/s
        wz：绕世界 Z 轴的角速度，单位 rad/s

    说明：
        - 不支持横向速度 vy。
        - 该控制器直接设置 Articulation Root 的速度状态，
          不模拟轮关节、电机、轮胎牵引和滑移。
        - 当前项目中，底盘与地面的零摩擦物理材料应已保存在 USD 中。
    """

    def __init__(self, robot_path: str):
        self.requested_robot_path = robot_path
        self.resolved_robot_path = None

        self.robot = None
        self.callback_id = None
        self.running = False

        self.target_vx = 0.0
        self.target_wz = 0.0

        self.current_vx = 0.0
        self.current_wz = 0.0

        self.forward_offset = math.radians(FORWARD_YAW_OFFSET_DEG)

    def start(self):
        """解析 Articulation Root、创建接口并注册物理回调。"""
        if self.running:
            print("底盘控制器已经在运行，无需重复启动")
            return

        if not SimulationManager.is_simulating():
            raise RuntimeError(
                "物理仿真尚未运行。请先点击 Isaac Sim 的 Play，"
                "等待物理场景完成初始化后再启动控制器。"
            )

        # 明确解析真正的 Articulation Root，避免把后代刚体误当成根节点。
        resolved_paths = Articulation.fetch_articulation_root_api_prim_paths(
            [self.requested_robot_path]
        )
        resolved_path = resolved_paths[0] if resolved_paths else None

        if resolved_path is None:
            raise RuntimeError(
                "无法从以下路径解析 Articulation Root："
                f"{self.requested_robot_path}"
            )

        self.resolved_robot_path = resolved_path
        self.robot = Articulation(self.resolved_robot_path)

        if not self.robot.valid:
            raise RuntimeError(
                f"Articulation 路径无效：{self.resolved_robot_path}"
            )

        if not self.robot.is_physics_tensor_entity_initialized():
            raise RuntimeError(
                "Articulation 的物理张量接口尚未初始化。"
                "请确认已点击 Play，并等待一到两个物理步后重试。"
            )

        self.callback_id = SimulationManager.register_callback(
            self._on_physics_step,
            event=SimulationEvent.PHYSICS_POST_STEP,
        )
        self.running = True

        print("======================================")
        print("理想底盘控制器启动成功")
        print("请求路径：", self.requested_robot_path)
        print("解析后的 Articulation Root：", self.resolved_robot_path)
        print("Callback ID：", self.callback_id)
        print("最大线速度：", MAX_LINEAR_SPEED, "m/s")
        print("最大角速度：", MAX_ANGULAR_SPEED, "rad/s")
        print("======================================")
        print(
            "控制示例：import builtins; "
            "builtins.TIANYI_BASE_CTRL.set_cmd(0.1, 0.0)"
        )
        print(
            "停止示例：import builtins; "
            "builtins.TIANYI_BASE_CTRL.hard_stop()"
        )

    def set_cmd(self, vx: float, wz: float):
        """设置机体前向速度和绕 Z 轴角速度。"""
        if not self.running or self.robot is None:
            raise RuntimeError(
                "底盘控制器尚未启动，或已经关闭。"
            )

        vx = float(vx)
        wz = float(wz)

        if not math.isfinite(vx) or not math.isfinite(wz):
            raise ValueError("vx 和 wz 必须是有限数值")

        self.target_vx = float(
            np.clip(vx, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
        )
        self.target_wz = float(
            np.clip(wz, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
        )

        print(
            f"目标速度：vx={self.target_vx:.3f} m/s，"
            f"wz={self.target_wz:.3f} rad/s"
        )

    @staticmethod
    def approach(
        current: float,
        target: float,
        maximum_change: float,
    ) -> float:
        """按最大变化量逐步逼近目标值。"""
        difference = target - current

        if abs(difference) <= maximum_change:
            return target

        return current + math.copysign(maximum_change, difference)

    @staticmethod
    def quaternion_to_yaw(quaternion) -> float:
        """将 Isaac Sim Core 的 wxyz 四元数转换为世界坐标系 Yaw。"""
        w, x, y, z = [float(v) for v in quaternion]

        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    def _on_physics_step(self, dt, context):
        """每个物理步后更新一次根节点速度。"""
        if not self.running or self.robot is None:
            return

        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            return

        try:
            self.current_vx = self.approach(
                self.current_vx,
                self.target_vx,
                MAX_LINEAR_ACCEL * dt,
            )
            self.current_wz = self.approach(
                self.current_wz,
                self.target_wz,
                MAX_ANGULAR_ACCEL * dt,
            )

            _, orientations = self.robot.get_world_poses()
            quaternion = to_numpy(orientations)[0]

            root_yaw = self.quaternion_to_yaw(quaternion)
            forward_yaw = root_yaw + self.forward_offset

            vx_world = math.cos(forward_yaw) * self.current_vx
            vy_world = math.sin(forward_yaw) * self.current_vx

            linear_velocities, _ = self.robot.get_velocities()
            linear_np = to_numpy(linear_velocities)
            vz_world = float(linear_np[0, 2])

            self.robot.set_velocities(
                linear_velocities=np.array(
                    [[vx_world, vy_world, vz_world]],
                    dtype=np.float32,
                ),
                angular_velocities=np.array(
                    [[0.0, 0.0, self.current_wz]],
                    dtype=np.float32,
                ),
            )

        except Exception as error:
            print(
                "[IdealBaseController] 物理回调执行失败：",
                repr(error),
            )
            self.shutdown()

    def soft_stop(self):
        """按加速度限制平滑停止。"""
        self.set_cmd(vx=0.0, wz=0.0)

    def hard_stop(self):
        """立即清零水平速度和绕 Z 轴角速度。"""
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.current_vx = 0.0
        self.current_wz = 0.0

        if self.robot is None:
            return

        try:
            linear_velocities, _ = self.robot.get_velocities()
            vz_world = float(to_numpy(linear_velocities)[0, 2])

            self.robot.set_velocities(
                linear_velocities=np.array(
                    [[0.0, 0.0, vz_world]],
                    dtype=np.float32,
                ),
                angular_velocities=np.zeros(
                    (1, 3),
                    dtype=np.float32,
                ),
            )
            print("底盘已紧急停止")

        except Exception as error:
            print(
                "停止时物理接口不可用：",
                repr(error),
            )

    def status(self):
        """打印控制器当前状态。"""
        print("======================================")
        print("running：", self.running)
        print("requested path：", self.requested_robot_path)
        print("resolved root：", self.resolved_robot_path)
        print("target vx：", self.target_vx)
        print("target wz：", self.target_wz)
        print("current vx：", self.current_vx)
        print("current wz：", self.current_wz)
        print("callback id：", self.callback_id)
        print("======================================")

    def shutdown(self):
        """停止控制，并注销物理回调。"""
        if not self.running and self.callback_id is None:
            return

        try:
            self.hard_stop()
        except Exception:
            pass

        self.running = False

        if self.callback_id is not None:
            try:
                SimulationManager.deregister_callback(self.callback_id)
            except Exception as error:
                print("注销物理回调失败：", repr(error))

        self.callback_id = None
        print("底盘控制器已关闭")
