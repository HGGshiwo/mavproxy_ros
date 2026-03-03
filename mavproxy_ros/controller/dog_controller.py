#!/usr/bin/python3
# -*- coding: utf-8 -*-
import math
import struct
import threading
import time
from logging import getLogger
from typing import Optional

import numpy as np

import rospy
from event_callback import CallbackManager, http
from event_callback.components.http.core import HTTPConfig
from event_callback.components.http.message_handler import MessageType
from event_callback.components.socket import (
    SocketClientConfig,
    SocketServerConfig,
    socketc,
    sockets,
)
from event_callback.utils import setup_logger
from nav_msgs.msg import Odometry

from mavproxy_ros.utils import post_json
import tf
import tf.transformations

from .base_controller import BaseController
from .dog_utils import *

logger = getLogger(__file__)
setup_logger()


def router(data: bytes):
    """路由函数：解析指令码并返回对应的CommandType"""
    try:
        command_id = struct.unpack("<I", data[0:4])[0]
        return CommandType(command_id)

    except Exception as e:
        print(f"指令解析失败！data: 0x{data.hex()}, error: {e}")
        return None


# host = "192.168.3.20"  # 实际机器人IP（无线接入）
# port = 43893
# server_port = 43893
# server_host = "0.0.0.0"

# host = "192.168.1.103"  # 有线接入IP
# host = "192.168.2.103"  # 通讯接口IP
# server_host = "192.168.3.157"

# 测试使用
server_host = "0.0.0.0"
server_port = 9111
host = "localhost"
port = 9112


class DogController(CallbackManager, BaseController):
    def __init__(self):
        # 组件配置
        config = [
            SocketServerConfig(
                host=server_host,
                port=server_port,
                socket_type="udp",
                register=False,
                decode=False,
                router=router,
            ),
            SocketClientConfig(
                host=host,
                port=port,
                register=False,
                socket_type="udp",
                decode=False,
                router=router,
            ),
            HTTPConfig(port=8001),
        ]
        self.max_backward_vel = None
        self.max_forward_vel = None
        self.v_max_lock = threading.Lock()
        self.basic_state = None
        self.is_run = None
        self.paltform_height = None
        self.max_accel = 1.0
        self.max_yaw_accel = 0.2
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_yaw_rate = 0.0
        self._last_cmd_time = None
        self._accel_lock = threading.Lock()
        super().__init__(config)
        # 订阅 ENU odom，用于位置控制
        self._odom: Optional[Odometry] = None
        self._odom_lock = threading.Lock()
        rospy.Subscriber(
            "/mavros/local_position/odom", Odometry, self._odom_callback, queue_size=1
        )
        # 位置控制后台线程管理
        self._pos_ctrl_thread: Optional[threading.Thread] = None
        self._pos_ctrl_stop = threading.Event()
        self._pos_ctrl_lock = threading.Lock()
        heartbeat_t = threading.Thread(
            target=self.heartbeat_thread, daemon=True, name="heartbeat-thread"
        )
        heartbeat_t.start()
        logger.info("心跳线程已启动（2Hz）")

    def is_pland_enable(self):
        """该模式下是否允许精准降落"""
        return False

    def _post_init(self):
        # 对于狗，禁止自动上锁
        post_json("/set_param", data=dict(param=dict(DISARM_DELAY=dict(value=0))))

    def do_takeoff(self, alt: float):
        """起立"""
        data = pack_q25_udp_cmd(CommandType.TOGGLE_STAND_DOWN)
        socketc.send_to_server(self, data)

    def do_land(self):
        data = pack_q25_udp_cmd(CommandType.TOGGLE_STAND_DOWN)
        socketc.send_to_server(self, data)

    def _odom_callback(self, msg: Odometry):
        with self._odom_lock:
            self._odom = msg

    def _get_current_pos_enu(self) -> Optional[list]:
        """获取当前 ENU 坐标系下的位置 [x, y, z]"""
        with self._odom_lock:
            if self._odom is None:
                return None

            pos = self._odom.pose.pose.position
            return [pos.x, pos.y, pos.z]

    def _get_current_yaw_enu(self) -> float:
        """获取当前 ENU 坐标系下的 yaw 角（rad）"""
        with self._odom_lock:
            if self._odom is None:
                return 0.0
            q = self._odom.pose.pose.orientation
            # 四元数转 yaw
            _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            return yaw

    @staticmethod
    def _vel_to_axis(
        vx_body: float,
        vy_body: float,
        yaw_rate: float,
        max_vel: float = 1.0,
        max_yaw_rate: float = 1.0,
    ) -> tuple:
        """将机体系速度（m/s）映射到 [-1000, 1000] 的轴指令

        :param max_vel: 线速度满量程（m/s），用于缩放 vx/vy
        :param max_yaw_rate: 偏航角速度满量程（rad/s），独立缩放 yaw_rate
        """
        vel_scale = 1000.0 / max(max_vel, 1e-6)
        yaw_scale = 1000.0 / max(max_yaw_rate, 1e-6)
        left_x = int(max(-1000, min(1000, vx_body * vel_scale)))
        left_y = int(max(-1000, min(1000, vy_body * vel_scale)))
        right_x = int(max(-1000, min(1000, yaw_rate * yaw_scale)))
        return left_x, left_y, right_x

    def do_send_cmd(
        self,
        *,
        p=None,
        v=None,
        a=None,
        yaw=None,
        yaw_rate=None,
        frame: Optional[str] = "enu",
    ):
        """
        发送运动指令。

        - 速度控制（v 不为 None）：发送一次轴指令即返回。
        - 位置控制（p 不为 None）：在后台线程中持续发送，直到到达目标位置。
        - 速度位置控制: v, p都不为None：等效于速度控制

        frame:
          "enu"  — ENU坐标系下: x为E, y为N, z为U
          "body" — 机体坐标系下: x为前, y为左, z为下
        """
        resolved_frame: str = frame if frame is not None else "enu"
        if v is not None:
            self._stop_position_control()
            self._send_velocity_cmd(v, yaw_rate, resolved_frame)
        elif p is not None:
            self._start_position_control(p, yaw, resolved_frame)
        else:
            self._stop_position_control()
            with self._accel_lock:
                self._last_vx = 0.0
                self._last_vy = 0.0
                self._last_yaw_rate = 0.0
                self._last_cmd_time = None
            self.move_axis_no_dead_zone(0, 0, 0, 0)

    # ---- 速度控制 ----
    def _send_velocity_cmd(self, v: list, yaw_rate: Optional[float], frame: str):
        """将速度指令转换为轴指令并发送一次"""
        vx, vy = float(v[0]), float(v[1]) if len(v) > 1 else 0.0
        yr = float(yaw_rate) if yaw_rate is not None else 0.0
        if frame == "enu":
            cur_yaw = self._get_current_yaw_enu()
            vx_body, vy_body = self._enu_vel_to_body(vx, vy, cur_yaw)
        elif frame == "body":
            vx_body, vy_body = vx, vy
        else:
            raise ValueError(f"不支持的坐标系: {frame}")

        vx_body, vy_body, yr = self._limit_acceleration(vx_body, vy_body, yr)

        with self.v_max_lock:
            max_vel = self.max_forward_vel if self.max_forward_vel else 1.0
        left_x, left_y, right_x = self._vel_to_axis(vx_body, vy_body, yr, max_vel)
        self.move_axis_no_dead_zone(left_x, left_y, right_x, 0)

    # ---- 位置控制 ----
    def _start_position_control(
        self, target_p: list, target_yaw: Optional[float], frame: str
    ):
        """停止旧的位置控制线程，启动新的位置控制线程"""
        self._stop_position_control()
        with self._pos_ctrl_lock:
            self._pos_ctrl_stop.clear()
            self._pos_ctrl_thread = threading.Thread(
                target=self._position_control_loop,
                args=(target_p, target_yaw, frame),
                daemon=True,
                name="pos-ctrl-thread",
            )
            self._pos_ctrl_thread.start()

    def _stop_position_control(self):
        """停止当前位置控制线程（如存在）"""
        with self._pos_ctrl_lock:
            if self._pos_ctrl_thread and self._pos_ctrl_thread.is_alive():
                self._pos_ctrl_stop.set()
                self._pos_ctrl_thread.join(timeout=2.0)
                self._pos_ctrl_thread = None
        with self._accel_lock:
            self._last_vx = 0.0
            self._last_vy = 0.0
            self._last_yaw_rate = 0.0
            self._last_cmd_time = None

    def _position_control_loop(
        self, target_p: list, target_yaw: Optional[float], frame: str
    ):
        """
        位置控制循环（后台线程）：用简单比例控制不断发送速度指令，
        直到到达目标位置（XY 误差 < 阈值）或被外部停止。

        如果未指定yaw，则自动将机头对准目标方向并同时前进。
        """
        POS_THRESHOLD = 0.15  # 到位阈值 (m)
        KP = 1.0  # 比例增益
        MAX_VEL = 1  # 最大控制速度 (m/s)
        RATE = 10  # 控制频率 (Hz)
        dt = 1.0 / RATE
        tx = float(target_p[0])
        ty = float(target_p[1]) if len(target_p) > 1 else 0.0
        # 目标 yaw（仅在 ENU 模式下支持）
        target_yaw_val = float(target_yaw) if target_yaw is not None else None
        # body 坐标系：在进入循环前一次性转换为 ENU 绝对目标点
        if frame == "body":
            # 等待第一帧 odom
            init_pos = None
            while init_pos is None and not self._pos_ctrl_stop.is_set():
                init_pos = self._get_current_pos_enu()
                if init_pos is None:
                    logger.warning("尚未收到 odom，等待（body->ENU转换）...")
                    time.sleep(dt)
            if self._pos_ctrl_stop.is_set() or init_pos is None:
                logger.info("位置控制线程退出（等待odom期间被停止）")
                return
            assert init_pos is not None  # 让类型检查器确认非 None
            init_yaw = self._get_current_yaw_enu()
            tx_orig, ty_orig = tx, ty
            tx = (
                init_pos[0]
                + tx_orig * math.cos(init_yaw)
                - ty_orig * math.sin(init_yaw)
            )
            ty = (
                init_pos[1]
                + tx_orig * math.sin(init_yaw)
                + ty_orig * math.cos(init_yaw)
            )
            frame = "enu"  # 后续统一按 ENU 处理

        FREEZE_DIST = 0.8  # 距目标 < 此值时锁定 target_direction (m)，防位置噪声驱动方向跳变

        logger.info(f"位置控制启动: target=({tx:.2f}, {ty:.2f}), frame={frame}")
        frozen_direction: Optional[float] = None  # 接近目标时锁定的方向
        compute_yaw_rate = YawController()
        while not self._pos_ctrl_stop.is_set():
            cur_pos = self._get_current_pos_enu()
            if cur_pos is None:
                logger.warning("尚未收到 odom，等待...")
                time.sleep(dt)
                continue

            if frame == "enu":
                ex = tx - cur_pos[0]
                ey = ty - cur_pos[1]
            else:
                raise ValueError(f"不支持的坐标系: {frame}")

            dist = math.sqrt(ex**2 + ey**2)
            if dist < POS_THRESHOLD:
                logger.info(f"已到达目标位置，误差={dist:.3f}m")
                # 发送零速停止
                self.move_axis_no_dead_zone(0, 0, 0, 0)
                break

            # 获取当前yaw
            cur_yaw = self._get_current_yaw_enu()

            # 确定目标朝向
            if target_yaw_val is not None:
                target_direction = target_yaw_val
            else:
                # 远离目标时持续更新朝向；接近目标时锁定方向，避免位置噪声驱动方向抖动
                if dist > FREEZE_DIST or frozen_direction is None:
                    frozen_direction = math.atan2(ey, ex)  # enu xy -> enu yaw
                target_direction = frozen_direction

            yr = compute_yaw_rate(cur_yaw, target_direction)

            scale = math.cos(np.clip(np.sqrt(np.abs(compute_yaw_rate.prev_yaw_err)) * 3, -np.pi, np.pi))
            vx_body = min(KP * dist, MAX_VEL) * max(0.0, scale)
            vy_body = 0.0

            vx_body, vy_body, yr = self._limit_acceleration(vx_body, vy_body, yr)

            logger.error(
                f"dir={target_direction:.2f} cur_yaw={cur_yaw:.2f} err={compute_yaw_rate.prev_yaw_err:.2f} yr={yr:.2f} vx={vx_body:.2f}"
            )

            with self.v_max_lock:
                max_vel = self.max_forward_vel if self.max_forward_vel else MAX_VEL
            left_x, left_y, right_x = self._vel_to_axis(
                vx_body, vy_body, yr, max_vel, max_yaw_rate=1.0
            )
            self.move_axis_no_dead_zone(left_x, left_y, right_x, 0)
            time.sleep(dt)
        logger.info("位置控制线程退出")

    @staticmethod
    def _enu_vel_to_body(vx_enu: float, vy_enu: float, yaw: float) -> tuple:
        """ENU 速度旋转到机体系（FLU：x=前, y=左）"""
        vx_body = vx_enu * math.cos(yaw) + vy_enu * math.sin(yaw)
        vy_body = -vx_enu * math.sin(yaw) + vy_enu * math.cos(yaw)
        return vx_body, vy_body

    def _limit_acceleration(
        self, vx_target: float, vy_target: float, yaw_rate_target: float
    ) -> tuple:
        """限制加速度和角加速度，返回限制后的速度

        :param vx_target: 目标x方向速度 (m/s)
        :param vy_target: 目标y方向速度 (m/s)
        :param yaw_rate_target: 目标角速度 (rad/s)
        :return: (vx_limited, vy_limited, yaw_rate_limited)
        """
        now = time.time()
        with self._accel_lock:
            if self._last_cmd_time is None:
                dt = 0.0
            else:
                dt = now - self._last_cmd_time
            self._last_cmd_time = now

            if dt <= 0:
                self._last_vx = vx_target
                self._last_vy = vy_target
                self._last_yaw_rate = yaw_rate_target
                return vx_target, vy_target, yaw_rate_target

            max_dv = self.max_accel * dt
            max_d_yaw_rate = self.max_yaw_accel * dt

            dvx = vx_target - self._last_vx
            dvy = vy_target - self._last_vy
            d_yaw_rate = yaw_rate_target - self._last_yaw_rate

            d_vel = math.sqrt(dvx**2 + dvy**2)
            if d_vel > max_dv:
                scale = max_dv / d_vel
                dvx *= scale
                dvy *= scale

            if abs(d_yaw_rate) > max_d_yaw_rate:
                d_yaw_rate = max_d_yaw_rate if d_yaw_rate > 0 else -max_d_yaw_rate

            vx_limited = self._last_vx + dvx
            vy_limited = self._last_vy + dvy
            yaw_rate_limited = self._last_yaw_rate + d_yaw_rate

            self._last_vx = vx_limited
            self._last_vy = vy_limited
            self._last_yaw_rate = yaw_rate_limited

            return vx_limited, vy_limited, yaw_rate_limited

    def check_alt(self, *args, **kwargs):
        """检查是否到达了指定的高度，这里简化为检测是否是站立状态"""
        return self.basic_state in [1, 2, 3, 4, 0x10]

    def check_hover(self, *args, **kwargs):
        return self.basic_state in [1, 2, 3, 4, 0x10]

    def move_raw_old(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        yaw: Optional[float] = None,
    ):
        """虚拟摇杆的三轴运动控制（X/Y轴速度 + Yaw角速度），机体左手坐标系FRU
        :param x: 摇杆偏移机体X轴控制(对应摇杆Y)
        :param y: 摇杆偏移机体Y轴控制(对应摇杆X)
        :param yaw: Yaw摇杆
        """
        if x is not None:
            data = pack_q25_udp_cmd(CommandType.MOVE_X_AXIS, parameter_size=x)
            socketc.send_to_server(self, data)
        if y is not None:
            data = pack_q25_udp_cmd(CommandType.MOVE_Y_AXIS, parameter_size=y)
            socketc.send_to_server(self, data)
        if yaw is not None:
            data = pack_q25_udp_cmd(CommandType.MOVE_YAW_AXIS, parameter_size=yaw)
            socketc.send_to_server(self, data)

    def move_axis_no_dead_zone(
        self, left_x: int = 0, left_y: int = 0, right_x: int = 0, right_y: int = 0
    ):
        """【新无死区】三轴运动控制（推荐使用），机体坐标系FLU，
        注意这个API的xy是相反的，和文档不一样

        :param left_x: X轴速度（-1000~1000，正向前）
        :param left_y: Y轴速度（-1000~1000，正向左）
        :param right_x: Yaw角速度（-1000~1000，正左转）
        :param right_y: 预留（固定为0）
        """
        # 校验参数范围
        left_x, left_y = left_y, left_x
        left_x = max(-1000, min(1000, left_x))
        left_y = max(-1000, min(1000, left_y))
        right_x = max(-1000, min(1000, right_x))
        right_y = 0  # 强制预留字段为0
        axis_data = AxisCommand(
            left_x=left_x, left_y=left_y, right_x=right_x, right_y=right_y
        )
        # 打包并发送指令（50Hz频率由调用方保证或新增线程控制）
        data = pack_q25_udp_cmd(
            command_type=CommandType.AXIS_COMMAND_NO_DEAD_ZONE,
            parameter_size=len(axis_data.to_bytes()),
            data=axis_data,
        )
        socketc.send_to_server(self, data)

    # ===================== 接收类指令（SOCKET监听）=====================
    @sockets.recv(CommandType.BATTERY_LEVEL_REPORT, frequency=1)
    def battery_level_report(self, data: bytes):
        """接收电池电量数据（0.5Hz）"""
        _, data_obj = unpack_q25_udp_cmd(data)
        if not isinstance(data_obj, BatteryLevel):
            return

        json_data = {"battery_level": data_obj.level}
        http.ws_send(self, json_data, MessageType.STATE)

    @sockets.recv(CommandType.MOTION_STATE_REPORT, frequency=1)
    def motion_state_report(self, data: bytes):
        """接收运动状态数据（200Hz）"""
        _, data_obj = unpack_q25_udp_cmd(data)
        if not isinstance(data_obj, MotionStateData):
            return

        with self.v_max_lock:
            self.max_forward_vel = data_obj.max_forward_vel
            self.max_backward_vel = data_obj.max_backward_vel
        # 修正gait_desc判断（文档：0x20=行走，0x23=跑步）
        gait_desc = "未知步态"
        if data_obj.gait_state == 0x20:
            gait_desc = "行走"
        elif data_obj.gait_state == 0x23:
            gait_desc = "跑步"
        _json_data = {
            "basic_state": data_obj.basic_state,
            "gait_state": data_obj.gait_state,
            "gait_desc": gait_desc,
            "max_forward_vel": round(data_obj.max_forward_vel, 2),
            "max_backward_vel": round(data_obj.max_backward_vel, 2),
            "position": [
                round(data_obj.pos_x, 3),
                round(data_obj.pos_y, 3),
                round(data_obj.pos_yaw, 3),
            ],
            "position_desc": f"{data_obj.pos_x:.1f}, {data_obj.pos_y:.1f}, {data_obj.pos_yaw:.1f}",
            "velocity": [
                round(data_obj.vel_x, 3),
                round(data_obj.vel_y, 3),
                round(data_obj.vel_yaw, 3),
            ],
            "run_distance": round(data_obj.robot_distance, 1),
            "charge_state": data_obj.auto_charge_state,
        }
        # 补充basic_state=0x10（L模式）映射
        state_map = {
            0: "趴下状态",
            1: "正在起立状态",
            2: "初始站立状态",
            3: "力控站立状态",
            4: "踏步状态",
            5: "正在趴下状态",
            6: "软急停/摔倒状态",
            0x10: "L模式",
        }
        basic_state_desc = state_map.get(
            data_obj.basic_state, f"未知状态({data_obj.basic_state})"
        )
        self.basic_state = data_obj.basic_state
        self.is_run = data_obj.gait_state == 0x23
        http.ws_send(
            self,
            dict(
                basic_state_desc=basic_state_desc,
                gait_desc=gait_desc,
                max_forward_vel=data_obj.max_forward_vel,
                max_backward_vel=data_obj.max_backward_vel,
            ),
            MessageType.STATE,
        )

    @sockets.recv(CommandType.RUN_STATUS_REPORT, frequency=1)
    def run_status_report(self, data: bytes):
        """接收运行状态数据（200Hz）"""
        _, data_obj = unpack_q25_udp_cmd(data)
        if not isinstance(data_obj, RcsData):
            return

        # json_data = {
        #     "robot_name": data_obj.robot_name,
        #     "current_milege": data_obj.current_milege,
        #     "total_milege": data_obj.total_milege,
        #     "current_run_time": data_obj.current_run_time,
        #     "total_run_time": data_obj.total_run_time,
        #     "motion_mode": "导航" if data_obj.is_nav_mode == 1 else "手动",
        #     "joystick": {
        #         "lx": round(data_obj.joystick_lx, 3),
        #         "ly": round(data_obj.joystick_ly, 3),
        #         "rx": round(data_obj.joystick_rx, 3),
        #         "ry": round(data_obj.joystick_ry, 3),
        #     },
        #     "errors": {
        #         "imu_error": data_obj.imu_error,
        #         "wifi_error": data_obj.wifi_error,
        #         "driver_heat_warn": data_obj.driver_heat_warn,
        #         "driver_error": data_obj.driver_error,
        #         "motor_heat_warn": data_obj.motor_heat_warn,
        #         "battery_low_warn": data_obj.battery_low_warn,
        #     },
        # }
        error = {
            "imu_error": data_obj.imu_error,
            "wifi_error": data_obj.wifi_error,
            "driver_heat_warn": data_obj.driver_heat_warn,
            "driver_error": data_obj.driver_error,
            "motor_heat_warn": data_obj.motor_heat_warn,
            "battery_low_warn": data_obj.battery_low_warn,
        }
        error = ", ".join([k for k, v in error.items() if v is True])
        http.ws_send(self, dict(error=error), MessageType.ERROR)

    # @sockets.recv(CommandType.SENSOR_DATA_REPORT, frequency=10)
    # def sensor_data_report(self, data: bytes):
    #     """接收运动控制传感器数据（200Hz）"""
    #     _, data_obj = unpack_q25_udp_cmd(data)
    #     if not isinstance(data_obj, ControllerSensorData):
    #         return
    #     imu = data_obj.imu_data
    #     json_data = {
    #         "imu": {
    #             "timestamp": imu.timestamp,
    #             "angle": [round(imu.roll, 2), round(imu.pitch, 2), round(imu.yaw, 2)],
    #             "angular_vel": [
    #                 round(imu.omega_x, 3), round(imu.omega_y, 3), round(imu.omega_z, 3)
    #             ],
    #             "acceleration": [
    #                 round(imu.acc_x, 3), round(imu.acc_y, 3), round(imu.acc_z, 3)
    #             ],
    #         },
    #         "joint_pos": self._format_joint_data(data_obj.joint_pos),
    #         "joint_vel": self._format_joint_data(data_obj.joint_vel),
    #         "joint_torque": self._format_joint_data(data_obj.joint_tau),
    #     }
    # http.ws_send(self, json_data, MessageType.STATE)
    # @sockets.recv(CommandType.CONTROLLER_SAFE_DATA_REPORT, frequency=1)
    # def controller_safe_report(self, data: bytes):
    #     """接收运动控制系统数据（1Hz）"""
    #     _, data_obj = unpack_q25_udp_cmd(data)
    #     if not isinstance(data_obj, ControllerSafeData):
    #         return
    #     json_data = {
    #         "motor_temperatures": [
    #             round(temp, 1) for temp in data_obj.motor_temperatures
    #         ],
    #         "driver_temperatures": data_obj.driver_temperatures,
    #         "cpu": {
    #             "temperature": round(data_obj.cpu_info.temperature, 1),
    #             "frequency": round(data_obj.cpu_info.frequency, 0),
    #         },
    #     }
    #     http.ws_send(self, json_data, MessageType.STATE)
    # @sockets.recv(CommandType.BATTERY_CHARGE_STATE_REPORT, frequency=10)
    # def battery_charge_report(self, data: bytes):
    #     """接收电池充电状态数据（0.5Hz）"""
    #     _, data_obj = unpack_q25_udp_cmd(data)
    #     if not isinstance(data_obj, BatteryChargeState):
    #         return
    #     json_data = {
    #         "level": data_obj.level,
    #         "is_charging": data_obj.is_charging,
    #         "charge_desc": "充电中" if data_obj.is_charging else "未充电",
    #     }
    #     http.ws_send(self, json_data, MessageType.STATE)
    @sockets.recv(CommandType.ERROR_CODE_REPORT, frequency=10)
    def error_code_report(self, data: bytes):
        """接收错误码数据"""
        _, data_obj = unpack_q25_udp_cmd(data)
        if not isinstance(data_obj, ErrorCode):
            return

        level_desc = {0: MessageType.INFO, 1: MessageType.WARN, 2: MessageType.ERROR}
        # json_data = {
        #     "error_level": data_obj.error_level,
        #     "level_desc": level_desc.get(data_obj.error_level, "未知"),
        #     "error_code": hex(data_obj.error_code),
        #     "error_msg": data_obj.error_msg,
        # }
        level = level_desc.get(data_obj.error_level, MessageType.INFO)
        error = {
            level.name.lower(): f"0x{hex(data_obj.error_code)}: {data_obj.error_msg}"
        }
        http.ws_send(self, error, level)

    # ===================== 辅助工具方法 =====================
    @staticmethod
    def _format_joint_data(joint_data: LegJointData) -> dict:
        """格式化关节数据（位置/速度/力矩）"""
        return {
            "fl": {  # 左前腿
                "hipx": round(joint_data.fl_hipx, 3),
                "hipy": round(joint_data.fl_hipy, 3),
                "knee": round(joint_data.fl_knee, 3),
            },
            "fr": {  # 右前腿
                "hipx": round(joint_data.fr_hipx, 3),
                "hipy": round(joint_data.fr_hipy, 3),
                "knee": round(joint_data.fr_knee, 3),
            },
            "hl": {  # 左后腿
                "hipx": round(joint_data.hl_hipx, 3),
                "hipy": round(joint_data.hl_hipy, 3),
                "knee": round(joint_data.hl_knee, 3),
            },
            "hr": {  # 右后腿
                "hipx": round(joint_data.hr_hipx, 3),
                "hipy": round(joint_data.hr_hipy, 3),
                "knee": round(joint_data.hr_knee, 3),
            },
        }

    # ===================== 心跳发送线程（2Hz）=====================
    def heartbeat_thread(self):
        """心跳指令发送线程（按文档要求2Hz频率）"""
        while True:
            try:
                # 打包手动模式心跳指令（指令值0，基本指令）
                data = pack_q25_udp_cmd(CommandType.MANUAL_HEARTBEAT, parameter_size=0)
                # 直接通过socketc发送
                socketc.send_to_server(self, data)
                time.sleep(0.5)  # 2Hz = 每0.5秒一次
            except Exception as e:
                print(f"心跳发送失败：{e}")
                time.sleep(0.5)
