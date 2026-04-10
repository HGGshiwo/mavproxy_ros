#!/usr/bin/python3
# -*- coding: utf-8 -*-
import struct
import threading
import time
from logging import getLogger
from typing import Callable, Optional

import numpy as np
import rospy
import tf
import tf.transformations
from event_callback.components.http.proxy import HTTP_ProxyComponent
from event_callback.components.socket.udp import UDPComponent
from event_callback.core import BaseManager
from event_callback.ros_utils import create_wsproxy
from event_callback.utils import setup_logger, throttle
from nav_msgs.msg import Odometry

from mavproxy_ros.controller.controller_utils import VelController
from mavproxy_ros.message_handler import MessageType

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
# host = "192.168.1.103"
# port = 43893
# server_port = 43893
# server_host = "0.0.0.0"


# host = "192.168.1.103"  # 有线接入IP
# host = "192.168.2.103"  # 通讯接口IP
# server_host = "192.168.3.157"
# 测试使用
# server_host = "0.0.0.0"
# server_port = 9111
# host = "localhost"
# port = 9112


class DogController(BaseManager, BaseController):
    def __init__(self, trigger_land: Callable):
        # 组件配置
        server_port = int(rospy.get_param("~udp_server_port"))
        server_host = rospy.get_param("~udp_server_host")
        port = int(rospy.get_param("~udp_port"))
        host = rospy.get_param("~udp_host")

        udp_comp = UDPComponent(router=router)
        udp_comp.start_server(server_host, server_port)
        self.udp_comp = udp_comp

        self.udp_port = port
        self.udp_host = host
        http_comp = HTTP_ProxyComponent()

        self.max_backward_vel = None
        self.max_forward_vel = None
        self.v_max_lock = threading.Lock()
        self.basic_state = None
        self.is_run = None
        self.paltform_height = None
        self.max_accel = 1.0
        self.max_yaw_accel = 1.0
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_yaw_rate = 0.0
        self._last_cmd_time = None
        self._accel_lock = threading.Lock()
        BaseManager.__init__(self, udp_comp, http_comp)
        BaseController.__init__(self, trigger_land)
        # 订阅 ENU odom，用于位置控制
        self._odom: Optional[Odometry] = None
        self._odom_lock = threading.Lock()
        rospy.Subscriber(
            "/mavros/local_position/odom", Odometry, self._odom_callback, queue_size=1
        )
        self.vel_controller = VelController(self.controller_cb)

        self.wsproxy = create_wsproxy()
        heartbeat_t = threading.Thread(
            target=self.heartbeat_thread, daemon=True, name="heartbeat-thread"
        )
        heartbeat_t.start()
        logger.info("心跳线程已启动（2Hz）")

    def controller_cb(self, vx_body, vy_body, yaw_rate):
        # logger.info(f"controller_cb {vx_body} {vy_body} {yaw_rate}")
        with self.v_max_lock:
            max_vel = self.max_forward_vel if self.max_forward_vel else 1
        left_x, left_y, right_x = self._vel_to_axis(
            vx_body, vy_body, yaw_rate, max_vel, max_yaw_rate=1.0
        )
        self.move_axis_no_dead_zone(left_x, left_y, right_x, 0)

    def is_pland_enable(self):
        """该模式下是否允许精准降落"""
        return False

    def send_to_server(self, data):
        self.udp_comp.send(data, self.udp_host, self.udp_port)

    def do_takeoff(self, alt: float):
        """起立"""
        data = pack_q25_udp_cmd(CommandType.TOGGLE_STAND_DOWN)
        self.send_to_server(data)

    def do_land(self):
        self.loiter()
        data = pack_q25_udp_cmd(CommandType.TOGGLE_STAND_DOWN)
        self.send_to_server(data)

    def loiter(self):
        for i in range(20):  # 发送2s的停止指令，防止还有速度，可能需要更加优雅的实现
            self.do_send_cmd(v=[0, 0, 0], yaw_rate=0, frame="body")
            time.sleep(0.1)

    def check_arrive(
        self, cur_pos: Tuple[float, float, float], goal: Tuple[float, float, float]
    ):
        dis = np.sqrt((cur_pos[0] - goal[0]) ** 2 + (cur_pos[1] - goal[1]) ** 2)
        return dis

    def _odom_callback(self, msg: Odometry):
        with self._odom_lock:
            self._odom = msg
        vel = msg.twist.twist.linear
        yaw_rate = msg.twist.twist.angular.z
        pos = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        self.vel_controller.update_state(
            vel=[vel.x, vel.y, vel.z],
            pos=[pos.x, pos.y, pos.z],
            yaw=yaw,
            yaw_rate=yaw_rate,
        )

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
        yaw_rate = -yaw_rate  # yaw_rate是反的？实际是顺时针为正？？？
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
        logger.info(
            f"do_send_cmd: p={p}, v={v}, a={a}, yaw={yaw} yr={yaw_rate} frame={frame}"
        )
        self.vel_controller.set_target(p, v, yaw, yaw_rate, frame)

    def is_alt_enable(self):
        return False

    def is_prearm_enable(self):
        """是否进行Prearm检查, 还是用arm代替"""
        return False

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
            self.send_to_server(data)
        if y is not None:
            data = pack_q25_udp_cmd(CommandType.MOVE_Y_AXIS, parameter_size=y)
            self.send_to_server(data)
        if yaw is not None:
            data = pack_q25_udp_cmd(CommandType.MOVE_YAW_AXIS, parameter_size=yaw)
            self.send_to_server(data)

    def set_joystick(
        self,
        left_x: float = 0,
        left_y: float = 0,
        right_x: float = 0,
        right_y: float = 0,
    ):
        """【新无死区】三轴运动控制（推荐使用），机体坐标系FLU，
        注意这个API的xy是相反的，和文档不一样

        :param left_x: X轴速度（-1~1，正向前）
        :param left_y: Y轴速度（-1~1，正向左）
        :param right_x: Yaw角速度（-1~1，正左转）
        :param right_y: 预留（固定为0）
        """
        return self.move_axis_no_dead_zone(
            int(left_x * 1e3), int(left_y * 1e3), int(right_x * 1e3), int(right_y * 1e3)
        )

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
        self.send_to_server(data)

    # ===================== 接收类指令（SOCKET监听）=====================
    @UDPComponent.on_message(CommandType.BATTERY_LEVEL_REPORT)
    @throttle(frequency=1)
    def battery_level_report(self, data: bytes):
        """接收电池电量数据（0.5Hz）"""
        _, data_obj = unpack_q25_udp_cmd(data)
        if not isinstance(data_obj, BatteryLevel):
            return

        json_data = {"battery_level": data_obj.level}
        self.wsproxy.state(json_data)

    @UDPComponent.on_message(CommandType.MOTION_STATE_REPORT)
    @throttle(frequency=1)
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
        if data_obj.basic_state == 0 and self.basic_state != 0:
            """防止在发布了起飞后，但是还没有响应的情况下出发"""
            self.trigger_land()
        self.basic_state = data_obj.basic_state
        self.is_run = data_obj.gait_state == 0x23
        # http_proxy.ws_send(
        #     self,
        #     dict(
        #         basic_state_desc=basic_state_desc,
        #         gait_desc=gait_desc,
        #         max_forward_vel=data_obj.max_forward_vel,
        #         max_backward_vel=data_obj.max_backward_vel,
        #     ),
        #     MessageType.STATE,
        # )
        self.wsproxy.state(dict(basic_state=basic_state_desc))

    @UDPComponent.on_message(CommandType.RUN_STATUS_REPORT)
    @throttle(frequency=1)
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
        error = [k for k, v in error.items() if v is True]
        if len(error) != 0:
            error = ", ".join(error)
            self.wsproxy.error(error)

    @UDPComponent.on_message(CommandType.ERROR_CODE_REPORT)
    @throttle(frequency=10)
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
        self.wsproxy.error(error)

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
                self.send_to_server(data)
                time.sleep(0.5)  # 2Hz = 每0.5秒一次
            except Exception as e:
                print(f"心跳发送失败：{e}")
                time.sleep(0.5)
