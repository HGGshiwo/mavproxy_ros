from enum import Enum
from logging import getLogger
import math
import threading
import time
from typing import Callable, Optional, Tuple

import numpy as np

import rospy
import tf
import tf.transformations


logger = getLogger(__name__)
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, TwistStamped


class CtrlFrame(Enum):
    ENU = "enu"
    BODY = "body"


class YawController:
    """
    yaw -> yaw_rate
    """

    def __init__(self, clear_timeout=0.5):
        self.last_call = None
        self.clear_timeout = clear_timeout  # x秒后清空之前记录的状态
        self.clear()

    def clear(self):
        self.yr_filtered = 0.0  # 低通滤波后的 yaw rate 输出
        self.prev_yaw_err: Optional[float] = None  # 帧间 yaw 误差，用于跨 ±π 边界展开

    def __call__(self, cur_yaw: float, target_direction: float):
        """
        Args:
            cur_yaw: ENU
            target_direction: ENU

        Returns:
            yaw_rate: FLU
        """
        if self.last_call is None:
            self.last_call = time.time()
        if time.time() - self.last_call < self.clear_timeout:
            self.clear()
        YAW_DEADBAND = 0.05  # yaw 死区 (rad ≈ 3°)，消除零点附近高频抖动
        KP_YAW = 0.6  # yaw 比例增益，偏低以减少振荡
        YAW_LPF_ALPHA = 0.4  # yaw 输出一阶低通滤波系数（越小越平滑）

        # 计算 yaw 误差（归一化到 [-π, π]，并做帧间展开防跨界跳变）
        yaw_err = math.atan2(
            math.sin(target_direction - cur_yaw),
            math.cos(target_direction - cur_yaw),
        )
        if self.prev_yaw_err is not None:
            delta = yaw_err - self.prev_yaw_err
            if delta > math.pi:
                yaw_err -= 2 * math.pi
            elif delta < -math.pi:
                yaw_err += 2 * math.pi
        self.prev_yaw_err = yaw_err

        # 死区 + 比例控制，死区内归零避免零点附近反复振荡
        if abs(yaw_err) < YAW_DEADBAND:
            yr_raw = 0.0
        else:
            # yr是FLU坐标系, 因为都是逆时针，所以和yaw的ENU坐标系一致
            yr_raw = max(-1.0, min(1.0, yaw_err * KP_YAW))

        # 一阶低通滤波（EMA），平滑 yaw rate 输出，减少符号反转引起的抖动
        self.yr_filtered = (
            YAW_LPF_ALPHA * yr_raw + (1.0 - YAW_LPF_ALPHA) * self.yr_filtered
        )
        yr = self.yr_filtered
        return yr


class VelController:

    """
    支持将不同类型的期望值转为vel + yaw_rate控制:
    - 位置 + yaw + yaw_rate （没有指定yaw则调整到速度方向）
    - 速度 + yaw + yaw_rate （没有指定yaw则调整到速度方向）
    - 位置+速度 + yaw + yaw_rate（速度作为前馈，位置作为反馈，类似于优先控制速度的同时控制位置）
    """

    def __init__(
        self,
        controller_cb: Callable,
        control_hz: float = 10,
        max_acc: float = 1,
        max_yaw_acc: float = 1,
        vel_timeout: float = 1,
    ):
        self.dt = 1.0 / control_hz
        self.max_acc = max_acc
        self.max_raw_rate = max_yaw_acc

        self.controller_cb = controller_cb
        self.vel = None
        self.pos = None
        self.yaw = None
        self.yaw_rate = None

        self.target_pos = None
        self.target_vel = None
        self.target_yaw = None
        self.target_yaw_rate = None
        self.vel_timeout = vel_timeout
        self.vel_timestamp = 0

        self.yaw_controller = YawController()
        self.target_lock = threading.Lock()
        self.control_runner = threading.Thread(target=self._control_loop, daemon=True)
        self.control_runner.start()

    def update_state(
        self,
        vel: Tuple[float, float, float] = None,
        pos: Tuple[float, float, float] = None,
        yaw=None,
        yaw_rate=None,
    ):
        if vel is not None:
            self.vel = vel
        if pos is not None:
            self.pos = pos
        if yaw is not None:
            self.yaw = yaw
        if yaw_rate is not None:
            self.yaw_rate = yaw_rate

    def vel_body2enu(self, vel: Tuple[float, float, float]):
        init_yaw = self.yaw
        tx_orig, ty_orig = vel[0], vel[1]
        tx = tx_orig * math.cos(init_yaw) - ty_orig * math.sin(init_yaw)
        ty = tx_orig * math.sin(init_yaw) + ty_orig * math.cos(init_yaw)
        return (tx, ty, vel[-1])

    def vel_enu2body(self, vel: Tuple[float, float, float]):
        init_yaw = self.yaw
        tx_orig, ty_orig = vel[0], vel[1]
        tx = tx_orig * math.cos(init_yaw) + ty_orig * math.sin(init_yaw)
        ty = -tx_orig * math.sin(init_yaw) + ty_orig * math.cos(init_yaw)
        return (tx, ty, vel[-1])

    def pos_body2enu(self, pos: Tuple[float, float, float]):
        init_pos = self.pos
        init_yaw = self.yaw
        tx_orig, ty_orig = pos[0], pos[1]
        tx = init_pos[0] + tx_orig * math.cos(init_yaw) - ty_orig * math.sin(init_yaw)
        ty = init_pos[1] + tx_orig * math.sin(init_yaw) + ty_orig * math.cos(init_yaw)
        return (tx, ty, pos[-1])

    def set_target(
        self,
        pos: Tuple[float, float, float] = None,
        vel: Tuple[float, float, float] = None,
        yaw: Optional[float] = None,
        yaw_rate: Optional[float] = None,
        frame: Optional[str] = "enu",
    ):
        """
        仅pos/vel, 则另一个控制量的速度上的作用变为0
        仅yaw/yaw_rate，pos和vel保持之前的目标值

        frame:
          "enu"  — ENU坐标系下: x为E, y为N, z为U
          "body" — 机体坐标系下: x为前, y为左, z为下
          yaw: ENU/NED
        """
        if self.vel is None or self.pos is None or self.yaw is None:
            logger.warning("No odom, ignore set_target")
            return
        logger.info(
            f"set_target: p={pos}, v={vel}, yaw={yaw}, yaw_rate={yaw_rate}, frame={frame}"
        )
        self.vel_timestamp = time.time()
        if frame == "body":
            if pos is not None:
                pos = self.pos_body2enu(pos)
            if vel is not None:
                vel = self.vel_body2enu(vel)
            if yaw is not None:
                yaw = yaw + self.yaw
            frame = "enu"

        with self.target_lock:
            # yaw+yaw_rate和pos+vel组内互相覆盖，组间不会覆盖
            group1 = yaw is not None or yaw_rate is not None
            group2 = pos is not None or vel is not None
            
            if group2:
                self.target_pos = pos
                self.target_vel = vel
            if group1:
                self.target_yaw = yaw
                self.target_yaw_rate = yaw_rate
        if pos is not None:
            goal_pub = rospy.Publisher(
                "/move_base_simple/vel_controller/goal", PoseStamped, queue_size=1
            )
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = pos[0]
            goal.pose.position.y = pos[1]
            if yaw is not None:
                orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
            goal.pose.orientation.x = orientation[0]
            goal.pose.orientation.y = orientation[1]
            goal.pose.orientation.z = orientation[2]
            goal.pose.orientation.w = orientation[3]
            goal_pub.publish(goal)

    def _get_yaw_rate(self, yaw: float = None, yaw_rate: float = None):
        """根据目标的yaw和yaw_rate计算最后的yaw_rate"""
        _yaw_rate = None
        if yaw is not None:  # yaw -> yaw_rate
            _yaw_rate = self.yaw_controller(self.yaw, yaw)
            _yaw_rate = self._limit_yaw_rate(_yaw_rate)

        if _yaw_rate is not None:
            if yaw_rate is not None:
                yaw_rate = _yaw_rate + yaw_rate
            else:
                yaw_rate = _yaw_rate
        return yaw_rate


    def _limit_yaw_rate(self, yaw_rate_target: float):
        max_d_yaw_rate = self.max_yaw_acc * self.dt
        last_yaw_rate = self.yaw_rate
        d_yaw_rate = yaw_rate_target - last_yaw_rate
        if abs(d_yaw_rate) > max_d_yaw_rate:
            d_yaw_rate = max_d_yaw_rate if d_yaw_rate > 0 else -max_d_yaw_rate
        yaw_rate_limited = last_yaw_rate + d_yaw_rate
        return yaw_rate_limited

    def _limit_vel(self, vx_target: float, vy_target: float, yaw_rate_target: float):
        """限制加速度和角加速度，返回限制后的速度

        :param vx_target: 目标x方向速度 (m/s)
        :param vy_target: 目标y方向速度 (m/s)
        :return: (vx_limited, vy_limited)
        """
        max_dv = self.max_acc * self.dt

        last_vel_x = self.vel[0]
        last_vel_y = self.vel[1]

        dvx = vx_target - last_vel_x
        dvy = vy_target - last_vel_y

        d_vel = math.sqrt(dvx**2 + dvy**2)
        if d_vel > max_dv:
            scale = max_dv / d_vel
            dvx *= scale
            dvy *= scale

        vx_limited = last_vel_x + dvx
        vy_limited = last_vel_y + dvy

        return vx_limited, vy_limited

    def _vel_from_pos(self, target_p, forward_only):
        """从位置控制计算的速度控制量(body)
        Args:
            forward_only: 只计算vx
        """
        POS_THRESHOLD = 0.15  # 到位阈值 (m)
        KP = 1.0  # 比例增益

        tx = float(target_p[0])
        ty = float(target_p[1]) if len(target_p) > 1 else 0.0
        cur_pos = self.pos
        ex = tx - cur_pos[0]
        ey = ty - cur_pos[1]

        dist = math.sqrt(ex**2 + ey**2)
        if dist <= POS_THRESHOLD:
            return 0, 0, dist
        # 距离小于2m, 使用二次函数调整
        if dist < 2:
            dist = dist * dist / 4
        if forward_only:
            return KP * dist, 0
        vx, vy, _ = self.vel_enu2body(KP * ex, KP * ey)
        return vx, vy, dist

    def _control_loop(self):
        """
        进行持续控制
        """
        MAX_VEL = 1.0
        while not rospy.is_shutdown():
            time.sleep(self.dt)
            with self.target_lock:
                # 检查是否超时
                timeout = time.time() - self.vel_timestamp > self.vel_timeout
                if self.target_vel is not None and timeout:
                    self.target_vel = [0, 0, 0]
                if self.target_yaw_rate is not None and timeout:
                    self.target_yaw_rate = 0

                if (
                    (
                        self.target_pos is None
                        and self.target_vel is None
                        and self.target_yaw is None
                        and self.target_yaw_rate is None
                    )
                    or self.yaw is None
                    or self.pos is None
                    or self.vel is None
                ):
                    continue  # 无控制量

                target_p = self.target_pos
                target_v = self.target_vel
                target_yaw = self.target_yaw
                target_yaw_rate = self.target_yaw_rate

            # 计算位置控制对速度的影响
            vx_body, vy_body, yaw_rate = 0, 0, 0
            cur_yaw = self.yaw

            if target_p is not None:
                forward_only = target_yaw is None and yaw_rate is None
                vx, vy, dist = self._vel_from_pos(target_p, forward_only)
                vx_body += vx
                vy_body += vy

            if target_v is not None:
                vx, vy, _ = self.vel_enu2body([vx, vy, 0])
                vx_body += vx
                vy_body += vy

            # yaw_rate不区分body or enu，直接视为body系即可
            if target_yaw is not None:
                vr = self.yaw_controller(self.yaw, target_yaw)
                yaw_rate += vr

            if target_yaw_rate is not None:
                yaw_rate += target_yaw_rate

            # 角加速度限制
            yaw_rate = self._limit_yaw_rate(yaw_rate)

            # 由角速度引入的速度限制
            err = np.clip(np.sqrt(np.abs(yaw_rate)) * 3, -np.pi, np.pi)
            scale = math.cos(err)
            # 最大速度限制
            v_norm = np.sqrt(vx_body * vx_body + vy_body * vy_body)
            v_norm_new = np.clip(v_norm * scale, 1e-8, MAX_VEL)
            vx_body = vx_body / v_norm * v_norm_new
            vy_body = vy_body / v_norm * v_norm_new

            # 加速度限制
            vx_body, vy_body = self._limit_vel(vx_body, vy_body)
            logger.error(
                f"dis_err={dist:.2f} dir={target_yaw:.2f} cur_yaw={cur_yaw:.2f} err={self.yaw_controller.prev_yaw_err:.2f} yr={yaw_rate:.2f} vx={vx_body:.2f}"
            )
            self.controller_cb(vx_body, vy_body, yaw_rate)
            # with self.v_max_lock:
            #     max_vel = self.max_forward_vel if self.max_forward_vel else MAX_VEL
            # left_x, left_y, right_x = self._vel_to_axis(
            #     vx_body, vy_body, yr, max_vel, max_yaw_rate=1.0
            # )
            # self.move_axis_no_dead_zone(left_x, left_y, right_x, 0)
