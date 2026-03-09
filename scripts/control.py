#!/usr/bin/python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import sys
from pathlib import Path

from mavproxy_ros.controller import BaseController

_SCRIPT_DIR = Path(os.path.dirname(os.path.abspath(__file__)))
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))
import copy
import json
import logging
import math
import threading
import time
from enum import Enum
from typing import Tuple

import numpy as np
import rospy
import tf
import tf.transformations
from event_callback import http_proxy, ros
from event_callback.core import CallbackManager
from event_callback.ros_utils import ROSProxy, rosparam_field, rospy_init_node
from event_callback.utils import setup_logger, throttle
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import HomePosition, RCIn, State, StatusText, SysStatus
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from nav_msgs.msg import Odometry
from pyproj import CRS, Transformer
from quadrotor_msgs.msg import PositionCommand
from rsos_msgs.msg import PointObj
from sensor_msgs.msg import NavSatFix, Range
from std_msgs.msg import Empty, Float64, String
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

from mavproxy_ros.control_model import *
from mavproxy_ros.ctrl_node import CtrlNode as _CtrlNode
from mavproxy_ros.ctrl_node import EventType, Runner
from mavproxy_ros.pid_controller import PIDController
from mavproxy_ros.utils import ERROR_RESPONSE, SUCCESS_RESPONSE, post_json

STOP_SPAN = 100  # 检测到目标后抑制重复检测的冷却时间(s)
TAKEOFF_THRESHOLD = 0.05  # 起飞/调高到达判定阈值(比例)，实际误差 = 目标高度 × 此值
LIFTING_TIMEOUT = 3  # 调整高度卡死超时(s)：周期内高度/偏航变化 < 0.1 则强制进入 WP

# 调整高度卡死判定阈值：高度或偏航变化量 < 此值视为卡死(m/rad)
LIFTING_STALL_THRESHOLD = 0.1
YAW_TOLERANCE = 0.1  # 航向对齐容差(rad)，用于 LiftingNode 和 PosVelYawNode
POSVEL_ARRIVE_DISTANCE = 0.5  # posvel 到达目标点的判定距离(m)
setup_logger(Path(__file__).parent.parent.joinpath("log").absolute())
logger = logging.getLogger(__name__)


class NodeType(Enum):
    INIT = "初始状态"
    GROUND = "地面状态"
    TAKING_OFF = "正在起飞"
    TAKING_OFF2 = "航点起飞"
    HOVER = "悬停状态"
    LIFTING = "调整高度"
    WP = "航点模式"
    FOLLOW = "跟随模式"
    LANDING = "正在降落"
    POSVEL_MOVE = "编队移动"
    POSVEL_YAW = "编队偏航"  # 到达目标位置后调整到目标yaw


class CEventType(EventType):
    SET_TAKEOFF = "set_takeoff"
    SET_WP = "set_wp"
    SET_LAND = "set_land"
    DETECT = "detect"
    DISARM = "DISARM"
    WP_FINISH = "wp_finish"
    STOP_FOLLOW = "stop_follow"
    SET_POSVEL = "set_posvel"
    POSVEL_YAW_DONE = "posvel_yaw_done"


class CtrlNode(_CtrlNode):
    land_enable = True
    detect_enable = False
    wp_enable = False
    ground_enable = True  # 是否进入ground状态
    context: Control

    def __init__(self, node_type: NodeType):
        super().__init__(node_type)
        if self.land_enable:
            self._register(CEventType.SET_LAND, self.land_cb)
        if self.detect_enable:
            self._register(CEventType.DETECT, self.detect_cb)
        if self.ground_enable:
            self._register(CEventType.DISARM, self.land_done_cb)

    def land_done_cb(self):
        self.step(NodeType.GROUND)

    def land_cb(self):
        logger.info("set land")
        # self.context.do_land()
        self.step(NodeType.LANDING)
        post_json("stop_record")

    def detect_cb(self, msg):
        context = self.context
        if time.time() - context.last_send < STOP_SPAN:
            return

        if self.type != NodeType.FOLLOW:
            context.node_before_detect = self.type
        context.do_detect(msg)
        self.step(NodeType.FOLLOW)

    def run_wp_event(self, event: dict):
        event_type = event["eventType"]
        data = {}
        if event_type == "video":
            if event["eventStatus"] == "on":
                url = "start_record"
                data = {"bag_name": event.get("eventParam", "")}
            elif event["eventStatus"] == "off":
                url = "stop_record"
        elif event_type == "hat":
            if event["eventStatus"] == "on":
                url = "start_detect"
                data = {"type": "nohardhat"}
            elif event["eventStatus"] == "off":
                url = "stop_detect"
        elif event_type == "smoke":
            if event["eventStatus"] == "on":
                url = "start_detect"
                data = {"type": "smoke"}
            elif event["eventStatus"] == "off":
                url = "stop_detect"
        elif event_type == "gimbal":
            url = "set_gimbal"
            angle = event.get("eventParam", "25")
            try:
                angle = float(angle)
            except Exception:
                self.ws_pub.publish(
                    json.dumps(
                        {"type": "error", "error": f"参数: {angle} 无法转为数字!"}
                    )
                )
                return

            data = {"mode": "body", "angle": angle}
        post_json(url, data)


class InitNode(CtrlNode):
    """初始状态：等待 odom 和高度数据就绪后自动跳转。

    退出条件：
    - rel_alt 和 odom 均不为 None 后，判断是否已在空中：
      - 是 → HOVER
      - 否 → GROUND
    """

    ground_enable = False
    land_enable = False

    def __init__(self):
        super().__init__(NodeType.INIT)

    @CtrlNode.on(CEventType.IDLE)
    def idle_cb(self):
        context = self.context
        if context.rel_alt is None or context.odom is None:
            return

        if context.check_hover():
            self.step(NodeType.HOVER)
        else:
            self.step(NodeType.GROUND)


class GroundNode(CtrlNode):
    """地面状态：等待起飞指令。

    退出条件：
    - 收到 SET_TAKEOFF → 执行起飞 → TAKING_OFF
    - 收到 SET_WP     → 以航点第一个点高度为起飞高度 → TAKING_OFF2
    - 检测到已在空中（check_hover）→ HOVER（掉电重启场景）
    """

    def __init__(self):
        super().__init__(NodeType.GROUND)

    @CtrlNode.on(CEventType.IDLE)
    def idle_cb(self):
        context = self.context
        if context.check_hover():
            self.step(NodeType.HOVER)


class TakeoffNode(CtrlNode):
    """起飞状态（手动起飞）：发送起飞指令并等待到达目标高度。

    退出条件：
    - check_alt(takeoff_alt, TAKEOFF_THRESHOLD) 成立 → 发布起飞完成事件 → HOVER
    """

    land_enable = True
    wp_enable = True

    def __init__(self):
        super().__init__(NodeType.TAKING_OFF)

    def enter(self):
        context = self.context
        context.do_takeoff(context.takeoff_alt)

    @CtrlNode.on(CEventType.IDLE)
    def idle_cb(self):
        context = self.context
        if context.check_alt(context.takeoff_alt, TAKEOFF_THRESHOLD):
            context.do_pub_takeoff()
            self.step(NodeType.HOVER)


class Takeoff2Node(CtrlNode):
    """起飞状态（航点起飞）：由地面收到 SET_WP 触发，起飞到航点第一个点的高度。

    退出条件：
    - check_alt(takeoff_alt, TAKEOFF_THRESHOLD) 成立 → 发布起飞完成事件 → LIFTING
    """

    wp_enable = True
    land_enable = True

    def __init__(self):
        super().__init__(NodeType.TAKING_OFF2)

    def enter(self):
        context = self.context
        context.do_takeoff(context.takeoff_alt)

    @CtrlNode.on(CEventType.IDLE)
    def idle_cb(self):
        context = self.context
        if not context.control.is_alt_enable() or context.check_alt(
            context.takeoff_alt, TAKEOFF_THRESHOLD
        ):
            context.do_pub_takeoff()
            self.step(NodeType.LIFTING)


class HoverNode(CtrlNode):
    """悬停状态：进入时发送零速指令，等待外部指令触发状态转换。

    退出条件（均由外部事件触发）：
    - SET_TAKEOFF → TAKING_OFF
    - SET_WP      → LIFTING
    - SET_LAND    → LANDING
    - DETECT      → FOLLOW
    """

    detect_enable = True
    land_enable = True
    wp_enable = True

    def __init__(self):
        super().__init__(NodeType.HOVER)

    def enter(self):
        context = self.context
        cmd_msg = Twist()
        cmd_msg.linear.x = 0
        cmd_msg.linear.y = 0
        cmd_msg.linear.z = 0
        context.cmd_vel_cb(cmd_msg)


class LiftingNode(CtrlNode):
    """调整高度和航向状态：爬升/下降到下一航点的目标高度，同时将机头转向目标方向。

    退出条件（满足任一即立即退出）：
    - 正常到达：check_alt(lift_alt, TAKEOFF_THRESHOLD) 且偏航误差 < YAW_TOLERANCE → WP
    - 卡死超时：每 LIFTING_TIMEOUT 秒检查一次，若高度和偏航变化量均 < LIFTING_STALL_THRESHOLD
               则判定为卡死，强制进入 WP
    """

    wp_enable = True
    land_enable = True
    detect_enable = True

    def __init__(self):
        super().__init__(NodeType.LIFTING)

    def enter(self):
        context = self.context
        context.lift_alt = context.waypoint[context.wp_idx][-1]
        diff_x, diff_y, diff_z = context.gps_target2enu_diff(
            context.waypoint[context.wp_idx]
        )
        # odom = context.get_cur_odom()
        # self.px = odom.pose.pose.position.x
        # self.py = odom.pose.pose.position.y
        self.start_time = time.time()
        self.last_alt = context.rel_alt
        self.last_yaw = context.yaw
        try:
            context.lift_yaw = context.enu_xy2yaw(diff_x, diff_y)  # 这里是enu的yaw
        except Exception as e:
            logger.exception(str(e))

    @CtrlNode.on(CEventType.IDLE)
    def idle_cb(self):
        context = self.context
        # context.do_send_cmd(
        #     pz=context.lift_alt, px=self.px, py=self.py, yaw=context.lift_yaw
        # )
        vz = np.clip(context.lift_alt - context.rel_alt, -1, 1)
        context.do_send_cmd(v=[0, 0, vz], yaw=context.lift_yaw)
        # context.do_send_cmd(yaw=context.lift_yaw)
        yaw_diff = context.check_yaw(context.lift_yaw)
        alt_diff = math.fabs(context.rel_alt - context.lift_alt)
        alt_diff = alt_diff if context.control.is_alt_enable() else 0

        context.ws_pub.publish(
            json.dumps(
                {
                    "type": "state",
                    "yaw_diff": f"{yaw_diff:.2f}",
                    "alt_diff": f"{alt_diff:.2f}",
                }
            )
        )
        # 达到目标高度和航向则立即退出
        if context.lift_yaw is None or yaw_diff < YAW_TOLERANCE:
            if not context.control.is_alt_enable() or context.check_alt(
                context.lift_alt, TAKEOFF_THRESHOLD
            ):
                context.do_send_cmd(v=[0, 0, 0])
                self.step(NodeType.WP)
                return

        if time.time() - self.start_time > LIFTING_TIMEOUT:  # 卡死超时，强制进入 WP
            if math.fabs(self.last_yaw - context.yaw) < LIFTING_STALL_THRESHOLD:
                if (
                    not context.control.is_alt_enable()
                    or math.fabs(self.last_alt - context.rel_alt)
                    < LIFTING_STALL_THRESHOLD
                ):
                    context.do_send_cmd(v=[0, 0, 0])
                    self.step(NodeType.WP)
            self.last_yaw = context.yaw
            self.last_alt = context.rel_alt
            self.start_time = time.time()


class WpNode(CtrlNode):
    """航点飞行状态：飞向当前航点，到达后切换到下一航点或结束任务。

    退出条件：
    - planner_enable=False 时：check_arrive(goal) 成立 → 触发 WP_FINISH
    - planner_enable=True 时：由规划器发布 /ego_planner/finish_event → 触发 WP_FINISH
    - WP_FINISH 后：
      - 还有剩余航点 → LIFTING（调高到下一航点高度）
      - 全部完成且 land=True → LANDING
      - 全部完成且 land=False → HOVER
    """

    detect_enable = True
    land_enable = True
    wp_enable = True

    def __init__(self):
        super().__init__(NodeType.WP)

    def enter(self):
        context = self.context
        goal = context.gps_target2goal(context.waypoint[context.wp_idx])
        px = goal.pose.position.x
        py = goal.pose.position.y
        pz = goal.pose.position.z
        self.goal = [px, py, pz]
        logger.info(f"wp target: {[px, py, pz]}")
        self.planner_enable = context.planner_enable  # 按照进入的时候是否开启判断
        if self.planner_enable:
            context.wp_pub.publish(goal)
        else:
            context.stop_pub.publish(Empty())
            context.do_send_cmd(p=[px, py, pz])
        if context.nodeEventList is not None:
            event_list = context.nodeEventList[context.wp_idx]
            logger.info(f"event list: {event_list}")
            if event_list is not None:
                for event in event_list:
                    self.run_wp_event(event)

    def exit(self):
        context = self.context
        context.stop_pub.publish(Empty())

    @CtrlNode.on(CEventType.IDLE)
    @throttle(2)
    def idle_cb(self):
        context = self.context
        arrive = context.check_arrive(self.goal)
        if not self.planner_enable and arrive:
            self.wp_finish_cb()

    @CtrlNode.on(CEventType.WP_FINISH)
    def wp_finish_cb(self):
        context = self.context
        if not context.land:  # 非降落/返航情况下，发布进度
            context.do_ws_pub(
                {
                    "type": "event",
                    "event": "progress",
                    "cur": context.wp_idx + 1,  # wp_idx是到达点的前一个点
                    "total": len(context.waypoint),
                }
            )
        context.do_ws_pub({"type": "state", "wp_idx": context.wp_idx + 1})
        logger.info(f"wp done: {context.waypoint[context.wp_idx]}")
        context.wp_idx += 1
        if context.wp_idx >= len(context.waypoint):
            if context.land:
                self.step(NodeType.LANDING)
            else:
                self.step(NodeType.HOVER)
        else:
            self.step(NodeType.LIFTING)


MAX_X_SPEED = 0.5  # 精准降落最大横向速度(m/s)
MAX_Y_SPEED = 0.5  # 精准降落最大横向速度(m/s)
MAX_Z_SPEED = 1  # 精准降落最大垂直速度(m/s)
PLAND_ALT_THRESHOLD = 0.2  # 精准降落触地判定高度(m)，低于此值执行 do_land
PLAND_CENTER_THRESHOLD = 0.1  # 精准降落允许下降的最大中心误差(m)
PLAND_TARGET_TIMEOUT = 0.05  # 精准降落目标数据超时阈值(s)，超时则忽略该帧


class LandNode(CtrlNode):
    """降落状态：执行降落动作，支持普通降落和精准降落两种模式。

    普通降落（pland_enable=False）：
    - enter 时直接调用 do_land，交由飞控自主降落。

    精准降落（pland_enable=True）：
    - 依赖 /mavproxy/landing_target 提供目标相对位置和偏航误差。
    - 中心误差 < PLAND_CENTER_THRESHOLD 时才允许下降（vz < 0）。
    - 测距仪高度 < PLAND_ALT_THRESHOLD 时视为已触地，调用 do_land → GROUND。
    - 目标数据超时（> PLAND_TARGET_TIMEOUT）则停止控制输出，等待新数据。
    - 遥控器有输入时优先响应遥控器速度，覆盖视觉控制。
    """

    wp_enable = True
    land_enable = True

    def __init__(self):
        super().__init__(NodeType.LANDING)
        self.pid_x_controller = PIDController(
            1, 0.005, 0.01, setpoint=0, output_min=-MAX_X_SPEED, output_max=MAX_X_SPEED
        )
        self.pid_y_controller = PIDController(
            1, 0.005, 0.01, setpoint=0, output_min=-MAX_Y_SPEED, output_max=MAX_Y_SPEED
        )
        self.pid_yaw_controller = PIDController(
            1, 0.005, 0.01, setpoint=0, output_min=-1, output_max=1
        )

    def enter(self):
        context = self.context
        post_json("set_gimbal", {"mode": "body", "angle": 90})
        if not context.pland_enable:
            context.control.do_land()
        post_json("stop_record")

    def _get_rc_speed(self, context):
        if context.rc_channel is None:
            return None

        stamp, rc_channels = context.rc_channel
        time_jump = math.fabs((rospy.Time.now() - stamp).to_sec())
        if time_jump < 0.05:  # 响应遥控器的输入
            if rc_channels[0] == 1500 and rc_channels[1] == 1500:  # 遥控器无输入
                return None

            vy = ((rc_channels[0] - 1500) / 500) * MAX_Y_SPEED
            vy = np.clip(vy, -MAX_Y_SPEED, MAX_Y_SPEED)
            vx = ((rc_channels[1] - 1500) / 500) * MAX_X_SPEED
            vx = np.clip(vx, -MAX_X_SPEED, MAX_X_SPEED)
            return vx, vy, 0, 0

    def _get_landing_speed(self, context: Control):
        """F-L-U坐标系下的速度"""
        landing_target = copy.deepcopy(context.landing_target)
        timestamp, x, y, yaw = landing_target
        logger.info(f"yaw: {yaw}")
        time_jump = math.fabs((rospy.Time.now() - timestamp).to_sec())
        if time_jump > PLAND_TARGET_TIMEOUT:  # 20hz
            logger.warning(
                f"target too old: {time_jump:.3f} > {PLAND_TARGET_TIMEOUT}s, ignore"
            )
            return None

        vz = 0
        z_err = np.sqrt(x * x + y * y)
        if z_err < PLAND_CENTER_THRESHOLD:  # 误差足够小，允许下降
            if context.rangefinder_alt is None:
                logger.warning("no rangefinder data found, ignore")
                return

            vz = np.clip(-context.rangefinder_alt, -1, 1)
        stamp = rospy.Time.now().to_sec()
        rel_alt = np.clip(context.rangefinder_alt, 0.1, 10)
        # 控制像素中心点靠近目标点
        vx = -self.pid_x_controller(y * rel_alt, stamp)
        vy = -self.pid_y_controller(x * rel_alt, stamp)
        yaw_rate = -self.pid_yaw_controller(yaw, stamp)
        return vx, vy, vz, yaw_rate

    def exit(self):
        context = self.context
        context.do_ws_pub({"type": "event", "event": "disarm"})

    @CtrlNode.on(CEventType.IDLE)
    def idle_cb(self):
        context = self.context
        if not context.pland_enable:
            return

        if context.rangefinder_alt < PLAND_ALT_THRESHOLD:
            context.control.do_land()
            logger.info(f"land done, alt: {context.rangefinder_alt}")
            self.step(NodeType.GROUND)
            return

        if context.landing_target is None:
            return

        v_xyz = self._get_rc_speed(context)
        if v_xyz is None:
            v_xyz = self._get_landing_speed(context)
        if v_xyz is None:
            return

        vx, vy, vz, yaw_rate = v_xyz
        logger.info(f"plan control: {[vx, vy, vz, yaw_rate]}")
        context.do_send_cmd(v=[vx, vy, vz], yaw_rate=yaw_rate, frame="body")


class FollowNode(CtrlNode):
    """目标跟随状态：接收感知模块的目标速度，持续跟随目标。

    退出条件：
    - 收到 STOP_FOLLOW → 关闭检测节点，回到进入跟随前的状态
      （若进入前为 WP，则退回 LIFTING；其他状态原路返回）
    """

    detect_enable = True

    def __init__(self):
        super().__init__(NodeType.FOLLOW)

    @CtrlNode.on(CEventType.STOP_FOLLOW)
    def stop_follow_cb(self):
        context = self.context
        context.last_send = time.time()
        rospy.set_param("/UAV0/perception/yolo_detection/enable_detection", False)
        rospy.set_param("/UAV0/perception/yolo_detection_smoke/enable_detection", False)
        rospy.set_param(
            "/UAV0/perception/object_location/object_location_node/enable_send", False
        )
        node_before_detect = (
            NodeType.LIFTING
            if context.node_before_detect == NodeType.WP
            else context.node_before_detect
        )
        self.step(node_before_detect)


class PosVelMoveNode(CtrlNode):
    """定点移动状态：持续调用do_send_cmd移动，到达目标点或超时则退回之前的状态。

    行为差异取决于 context.posvel_fix_yaw：
      - fix_yaw=True : 运动过程中始终锁定给定的yaw
                       到达目标点后进入 POSVEL_YAW 调整终点yaw（若有）。
      - fix_yaw=False: 运动过程中将机头始终朝向目标直线方向（实时更新yaw），
                       到达目标点后进入 POSVEL_YAW 调整终点yaw（若有）。

    到达判定：在 IDLE 中用 odom 当前位置与目标 ENU 坐标计算三维距离，与 WpNode 逻辑一致。
    超时判定：超过 context.posvel_timeout 秒没有收到 SET_POSVEL 调用，退回前一状态。
    """

    land_enable = True

    def __init__(self):
        super().__init__(NodeType.POSVEL_MOVE)

    def enter(self):
        self._last_call_time = time.time()
        # 缓存目标 ENU 坐标，供 IDLE 中到达判定使用
        self._goal_enu = None

    def _update_goal_enu(self):
        """根据当前 posvel_target_pos 计算并缓存目标的 ENU 绝对坐标"""
        context = self.context
        diff_x, diff_y, diff_z = context.gps_target2enu_diff(context.posvel_target_pos)
        odom = context.odom
        if odom is None:
            return

        self._goal_enu = [
            diff_x + odom.pose.pose.position.x,
            diff_y + odom.pose.pose.position.y,
            diff_z + odom.pose.pose.position.z,
        ]

    @CtrlNode.on(CEventType.SET_POSVEL)
    def set_posvel_cb(self, pos, vel):
        """收到新的posvel请求时更新目标并刷新超时计时"""
        context = self.context
        context.posvel_target_pos = pos
        context.posvel_target_vel = vel
        self._last_call_time = time.time()
        self._update_goal_enu()
        diff_x, diff_y, diff_z = context.gps_target2enu_diff(pos)
        distance = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        v = vel
        if distance < v * v:
            v = math.sqrt(distance)
        radian = math.atan2(diff_y, diff_x)
        vx = v * math.cos(radian)
        vy = v * math.sin(radian)
        vz = np.clip(diff_z, -1, 1)
        if context.posvel_fix_yaw:
            # fix_yaw=True：始终锁定进入时记录的初始机头朝向
            context.do_send_cmd(v=[vx, vy, vz], yaw=context.posvel_target_yaw)
        else:
            # fix_yaw=False：实时将机头朝向目标直线方向（ENU弧度）
            heading_yaw = context.enu_xy2yaw(diff_x, diff_y)
            context.do_send_cmd(v=[vx, vy, vz], yaw=heading_yaw)

    @CtrlNode.on(CEventType.IDLE)
    def idle_cb(self):
        """
        周期检查两个退出条件：
          1. 到达目标点（与 WpNode 相同的三维 odom 距离判断）→ 进入 POSVEL_YAW
          2. 超时（超过 context.posvel_timeout 秒无新指令）→ 退回前一状态
        """
        context = self.context
        # ── 超时检查 ──────────────────────────────────────────────────────────
        elapsed = time.time() - self._last_call_time
        if elapsed > context.posvel_timeout:
            logger.info(
                f"[PosVelMove] timeout ({elapsed:.2f}s), returning to prev state"
            )
            self.step(context.posvel_node_before)
            return

        # ── 到达判定（与 WpNode 一致：用 odom 三维距离） ─────────────────────
        if self._goal_enu is None:
            self._update_goal_enu()
        if self._goal_enu is None:
            return

        odom = context.odom
        if odom is None:
            return

        arrive = context.check_arrive(self._goal_enu, POSVEL_ARRIVE_DISTANCE)
        if arrive:
            self.step(NodeType.POSVEL_YAW)


class PosVelYawNode(CtrlNode):
    """终点偏航调整状态：到达目标位置后将机头调整到目标yaw，完成后退回之前状态。

    两种 fix_yaw 模式下均可能进入此状态：
      - fix_yaw=True : 到达终点后调整到 posvel_target_yaw（若有）。
      - fix_yaw=False: 到达终点后调整到 posvel_target_yaw（若有）。
    若 posvel_target_yaw 为 None，则直接退回前一状态，不做任何调整。
    """

    land_enable = True

    def __init__(self):
        super().__init__(NodeType.POSVEL_YAW)

    def enter(self):
        context = self.context
        # 如果没有指定目标yaw，直接退回前一状态
        if context.posvel_target_yaw is None:
            logger.info("[PosVelYaw] no target yaw, returning to prev state")
            self.step(context.posvel_node_before)

    @CtrlNode.on(CEventType.IDLE)
    def idle_cb(self):
        context = self.context
        target_yaw = context.posvel_target_yaw
        if target_yaw is None:
            self.step(context.posvel_node_before)
            return

        # 持续发送原地悬停+偏航调整指令
        context.do_send_cmd(v=[0, 0, 0], yaw=target_yaw)
        yaw_diff = context.check_yaw(target_yaw)
        logger.debug(f"[PosVelYaw] yaw_diff: {yaw_diff:.3f} rad")
        if yaw_diff < YAW_TOLERANCE:
            logger.info("[PosVelYaw] yaw aligned, returning to prev state")
            self.step(context.posvel_node_before)


class Control(CallbackManager, ROSProxy):
    def __init__(self, component_config=None, mixins=None):
        super().__init__(component_config, mixins)
        self.send_time = 0
        # 状态机使用的变量
        self.rel_alt = None
        self.sys_status = None
        self.state = ""
        self.last_send = -1
        self.waypoint = []
        self.nodeEventList = None
        self.wp_idx = 0
        self.takeoff_lat = 0
        self.takeoff_lon = 0
        self.takeoff_alt = 0
        self.lift_alt = 0
        self.lift_yaw = None  # ENU,向东为正,逆时针增加
        self.arm = False
        self.mode = "UNKNOWN"
        self.odom = None
        # 融合经纬度
        self.lat = 0
        self.lon = 0
        # gps经纬度
        self.gps_lat = 0
        self.gps_lon = 0
        self.gps_alt = 0
        self.land = False
        self.speed = 0
        self.odom_lock = threading.Lock()
        self._wp_raw = None
        self.yaw = None  # NED, 向北为正, 顺时针增加
        self.rangefinder_alt = None  # 测距仪高度
        self.rc_channel = None  # 遥控器输入
        # 精准降落
        self.landing_target = None
        # posvel fix_yaw 模式使用的变量
        self.posvel_target_pos = None  # 目标GPS坐标 [lon, lat, alt]
        self.posvel_target_vel = 0  # 目标速度
        self.posvel_target_yaw = (
            None  # 目标偏航角（ENU弧度），到达终点后调整到此yaw；None表示不调整
        )
        self.posvel_fix_yaw = (
            True  # 是否固定机头方向（True=锁定初始yaw，False=跟随运动方向）
        )
        self.posvel_timeout = 2.0  # 超时时长（秒），由接口传入
        self.posvel_node_before = NodeType.HOVER  # 进入posvel模式前的状态
        self.runner = Runner(
            node_list=[
                InitNode(),
                GroundNode(),
                TakeoffNode(),
                Takeoff2Node(),
                HoverNode(),
                LiftingNode(),
                WpNode(),
                FollowNode(),
                LandNode(),
                PosVelMoveNode(),
                PosVelYawNode(),
            ],
            context=self,
            step_cb=self.step_cb,
        )
        logger.info("wait for mavros service...")
        rospy.wait_for_service("/mavros/cmd/arming", timeout=5)
        rospy.wait_for_service("/mavros/set_mode", timeout=5)
        rospy.wait_for_service("/mavros/cmd/takeoff", timeout=5)
        logger.info("control done")
        self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.cmd_service = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        self.wp_pub = rospy.Publisher(
            "/move_base_simple/goal2", PoseStamped, queue_size=-1
        )
        self.ws_pub = rospy.Publisher("ws", String, queue_size=-1)
        self.stop_pub = rospy.Publisher("/egoplanner/stopplan", Empty, queue_size=-1)
        self.target_pub = rospy.Publisher(
            "/UAV0/perception/object_location/obj_lla", PointStamped, queue_size=1
        )
        self.planner_enable = rosparam_field("planner_enable", True)
        self.auto_planner_enable = (
            self.planner_enable
        )  # 是否允许在停止检测后自动打开避障
        self.pland_enable = rosparam_field(
            "pland_enable", default=True
        )  # 是否进行精准降落
        self.min_alt_threshold = rosparam_field("min_alt_threshold", 0.5)
        self.controller_name = rospy.get_param(
            "/mavproxy/control/controller_name", None
        )

        def trigger_land():
            """由外部触发直接进入地面状态"""
            self.runner.step(NodeType.GROUND)

        self.control = BaseController.create(
            self.controller_name, trigger_land=trigger_land
        )
        self.do_send_cmd = self.control.do_send_cmd
        self.pland_enable = self.control.is_pland_enable() and self.pland_enable

    @property
    def planner_desc(self):
        return "启用" if self.planner_enable else "关闭"

    @property
    def pland_desc(self):
        return "启用" if self.pland_enable else "关闭"

    def _post_init(self):
        verison = rospy.get_param("/mavros/version", "No version")
        self.do_ws_pub(
            {
                "type": "state",
                "planner": self.planner_desc,
                "version": verison,
                "state": self.runner.node.type.value,
                "pland": self.pland_desc,
            }
        )
        logger.info("publish done")

    def set_wp_cb(
        self, waypoint, nodeEventList=None, speed=None, land=False, rtl=False
    ):
        if len(waypoint) == 0 and not rtl:
            raise ValueError("No waypoint found!")

        if (len(waypoint) == 1 and not rtl) or (rtl and len(waypoint) == 0):
            # 如果只有一个航点(rtl为0个), 本来是不允许的, 现在额外插入一个
            waypoint.insert(0, [0, 0, 10])

        if self.runner.node.type == NodeType.GROUND:
            self.takeoff_alt = waypoint[0][-1]
            next_state = NodeType.TAKING_OFF2
        else:
            next_state = NodeType.LIFTING

        # set_wp start
        logger.info(f"set wp return: {rtl} wp: {waypoint}")
        self.land = land or rtl
        self.nodeEventList = nodeEventList
        if rtl:
            return_alt = waypoint[-1][-1]  # 最后一个点的高度作为返航高度
            waypoint.append([self.takeoff_lon, self.takeoff_lat, return_alt])
        self.waypoint = waypoint[1:]
        self.wp_idx = 0
        # set_wp done

        self.set_mode_service(0, "GUIDED")
        self.do_pub_wp(waypoint[0:1] + self.waypoint, land or rtl)
        self.runner.step(next_state)

    def do_ws_pub(self, data: dict):
        self.ws_pub.publish(json.dumps(data))

    def step_cb(self, prev, cur):
        self.ws_pub.publish(json.dumps({"type": "state", "state": cur.value}))
        self.control.state_change(prev, cur)

    def enu_xy2yaw(self, diff_x, diff_y):
        """注意, yaw正东为0度,逆时针为正!"""
        bearing_rad = None
        bearing_rad = math.atan2(diff_y, diff_x)  # 核心：参数顺序x(E), y(N)
        if bearing_rad < 0:  # 标准化到0-2π范围
            bearing_rad += 2 * math.pi
        return bearing_rad

    def do_pub_wp(self, waypoint, land):
        out = []
        for i, wp in enumerate(waypoint):
            if i == 0:
                command = "takeoff"
            elif i == len(waypoint) - 1 and land:
                command = "land"
            else:
                command = "wp"
            out.append(
                {"num": i, "lat": wp[1], "lon": wp[0], "alt": wp[2], "command": command}
            )
        self.do_ws_pub({"mission_data": out, "type": "state"})
        logger.info(f'pub wp {json.dumps({"mission_data": out, "type": "state"})}')

    def do_pub_takeoff(self):
        self.do_ws_pub({"type": "event", "event": "takeoff"})

    def do_detect(self, msg):
        if msg.score < 0:
            msg.velocity.x = 0
            msg.velocity.y = 0
            msg.velocity.z = 0
        else:
            goal = PointStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            lon, lat, alt = self.enu2gps([msg.pos.x, msg.pos.y, msg.pos.z])
            goal.point.x = lon
            goal.point.y = lat
            goal.point.z = alt
            self.target_pub.publish(goal)
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = msg.velocity.x
        cmd_vel_msg.linear.y = msg.velocity.y
        cmd_vel_msg.linear.z = msg.velocity.z
        self.ws_pub.publish(
            json.dumps(
                {
                    "type": "state",
                    "follow x": msg.velocity.x,
                    "follow y": msg.velocity.y,
                    "follow z": msg.velocity.z,
                }
            )
        )
        self.cmd_vel_cb(cmd_vel_msg)

    def check_hover(self):
        return self.control.check_hover(self.arm, self.rel_alt)

    def check_alt(self, target: float, threshold: float):
        return math.fabs(self.rel_alt - target) < max(
            target * threshold, self.min_alt_threshold
        )

    def check_yaw(self, yaw_enu):
        yaw_ned = math.pi / 2 - yaw_enu
        if yaw_ned < 0:
            yaw_ned += 2 * math.pi
        if yaw_ned > 2 * math.pi:
            yaw_ned -= 2 * math.pi
        return math.fabs(self.yaw - yaw_ned)

    def check_arrive(self, goal: Tuple[float, float, float], tolerance: float = 2):
        if self.odom is None:
            return False

        cur_pos = [
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y,
            self.odom.pose.pose.position.z,
        ]
        dis = self.control.check_arrive(cur_pos, goal)
        self.ws_pub.publish(json.dumps({"type": "state", "dis": f"{dis:.2f}"}))
        return dis < tolerance

    def do_takeoff(self, alt):
        self.set_mode_service(0, "GUIDED")
        self.takeoff_lat = self.lat
        self.takeoff_lon = self.lon
        self.takeoff_alt = alt
        if self.check_hover():
            odom = self.get_cur_odom()
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            self.do_send_cmd(p=[x, y, alt])
        else:
            # 解锁
            if not self.arm:
                out = self.prearm()["msg"]
                if out.get("arm", False) == False:
                    raise ValueError(out["reason"])

                self._do_arm()
                logger.info("Vehicle armed")
            # 持续发布目标点
            logger.info("Taking off...")
            self.control.do_takeoff(alt)
            logger.info("Takeoff command finished.")

    def quaternion_to_ned_yaw(self, quaternion):
        """
        将四元数(ENU系)转换为罗盘航向角 (Compass Heading / NED Yaw)
        0° = 北，90° = 东，顺时针为正
        """
        euler = euler_from_quaternion(quaternion)
        yaw_rad = euler[2]  # ENU yaw (东为0)

        # 转换为罗盘航向：90度(北) - ENU航向
        heading_rad = math.pi / 2 - yaw_rad

        # 规范化到 0 ~ 2pi 范围
        if heading_rad < 0:
            heading_rad += 2 * math.pi
        if heading_rad >= 2 * math.pi:
            heading_rad -= 2 * math.pi

        return heading_rad

    def enu2gps(self, enu):
        """
        已知ENU坐标系下(odom)机体位置，求目标点GPS坐标
        """
        lng = self.lon
        lat = self.lat
        # WGS84地理坐标系
        crs_wgs84 = CRS.from_epsg(4326)
        # home点的UTM投影
        crs_utm = CRS.from_proj4(
            f"+proj=utm +zone={(int((lng + 180) / 6) + 1)} +datum=WGS84 +units=m +no_defs"
        )
        # 创建转换器
        transformer = Transformer.from_crs(crs_wgs84, crs_utm)
        # home点的UTM坐标
        home_x, home_y = transformer.transform(lat, lng)
        odom = self.get_cur_odom()
        # enu坐标系下目标点相对飞机的偏移
        enu_diff = [
            enu[0] - odom.pose.pose.position.x,
            enu[1] - odom.pose.pose.position.y,
            enu[2] - odom.pose.pose.position.z,
        ]
        # 目标点的UTM坐标
        target_x = home_x + enu_diff[0]
        target_y = home_y + enu_diff[1]
        # 逆变换，UTM->WGS84
        transformer_inv = Transformer.from_crs(crs_utm, crs_wgs84)
        target_lat, target_lng = transformer_inv.transform(target_x, target_y)
        # 高度
        target_alt = self.rel_alt + enu_diff[2]
        return [target_lng, target_lat, target_alt]

    def gps_target2enu_diff(self, gps):
        """获取enu坐标系下相对机体位置的偏移"""
        lng = self.lon
        lat = self.lat
        # WGS84地理坐标系
        crs_wgs84 = CRS.from_epsg(4326)
        # 自动获取home点的UTM投影
        crs_utm = CRS.from_proj4(
            f"+proj=utm +zone={(int((lng + 180) / 6) + 1)} +datum=WGS84 +units=m +no_defs"
        )
        # 创建转换器
        transformer = Transformer.from_crs(crs_wgs84, crs_utm)
        # home点的UTM坐标
        home_x, home_y = transformer.transform(lat, lng)
        # print(f"home: {home_x}, {home_y}, {lat}, {lng}")
        # 目标点
        target_x, target_y = transformer.transform(gps[1], gps[0])
        return [target_x - home_x, target_y - home_y, gps[2] - self.rel_alt]

    def gps_target2goal(self, gps):
        """gps目标点转为move_base_simple/goal目标"""
        diff_x, diff_y, diff_z = self.gps_target2enu_diff(gps)
        # ENU坐标
        odom = self.get_cur_odom()
        enu_x = diff_x + odom.pose.pose.position.x  # 东向
        enu_y = diff_y + odom.pose.pose.position.y  # 北向
        enu_z = diff_z + odom.pose.pose.position.z  # 上向
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = enu_x
        goal.pose.position.y = enu_y
        goal.pose.position.z = enu_z
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        return goal

    def get_cur_odom(self, t_cmd=None) -> Odometry:
        with self.odom_lock:
            if self.odom is None:
                logger.error("wait for odom")
                return

            odom_msg = copy.deepcopy(self.odom)
        if t_cmd is not None:
            t_odom = odom_msg.header.stamp.to_sec()
            time_diff = abs(t_odom - t_cmd)
            if time_diff >= 0.2:
                logger.error(f"odom and PositionCommand time diff:{time_diff}")
                return

        return odom_msg

    def _do_arm(self, timeout=10):
        start_time = time.time()
        self.state = ""
        self.arm_service(True)
        rate = rospy.Rate(5)
        while True:
            if self.arm:
                break

            if time.time() - start_time > timeout:
                raise TimeoutError("arm timeout")

            if self.state.startswith("Arm: "):
                raise ValueError(self.state)

            rate.sleep()

    @ros.topic("/mavros/global_position/raw/gps_vel", TwistStamped, 10)
    def gps_vel_cb(self, msg: TwistStamped):
        # enu -> ned
        vx = msg.twist.linear.y
        vy = msg.twist.linear.x
        self.do_ws_pub(dict(x_vel=vx, y_vel=vy))

    @ros.topic("/mavros/home_position/home", HomePosition)
    def home_callback(self, msg: HomePosition):
        if self.takeoff_lat == 0 and self.takeoff_lon == 0 and self.takeoff_alt == 0:
            self.takeoff_lat = msg.geo.latitude
            self.takeoff_lon = msg.geo.longitude
            self.takeoff_alt = msg.geo.altitude
            logger.info(
                f"set_home: {self.takeoff_lon} {self.takeoff_lat} {self.takeoff_alt}"
            )

    @ros.topic("/mavros/global_position/global", NavSatFix)
    def gps_cb(self, data: NavSatFix):
        self.lat = data.latitude
        self.lon = data.longitude
        self.do_ws_pub(dict(lat=self.lat, lon=self.lon))

    @ros.topic("/mavros/global_position/raw/fix", NavSatFix)
    def gps_cb2(self, data: NavSatFix):
        self.gps_lat = data.latitude
        self.gps_lon = data.longitude
        self.gps_alt = data.altitude

    @ros.topic("/ego_planner/finish_event", Empty)
    def wp_done_cb(self, data=None):
        self.runner.trigger(CEventType.WP_FINISH)

    @ros.topic("/mavros/local_position/odom", Odometry)
    def odom_cb(self, msg: Odometry):
        with self.odom_lock:
            self.odom = msg
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        self.yaw = self.quaternion_to_ned_yaw(quaternion)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.do_ws_pub(dict(yaw=self.yaw, x_vel_body=vx, v_vel_body=vy))

    @ros.topic("/UAV0/perception/object_location/location_vel", PointObj)
    def target_cb(self, msg):
        self.runner.trigger(CEventType.DETECT, msg=msg)

    @ros.topic("/cmd_vel", Twist)
    def cmd_vel_cb(self, cmd_vel_msg: Twist):
        odom_msg = self.get_cur_odom()
        if odom_msg is None:
            return

        # 2. 获取 cmd_vel 中的速度（机体坐标系，无需转换）
        vx = cmd_vel_msg.linear.x  # 机体前向速度
        vy = cmd_vel_msg.linear.y  # 机体左向速度
        vz = cmd_vel_msg.linear.z  # 机体上向速度
        # delta_yaw = cmd_vel_msg.angular.z  # 期望的yaw增量（弧度）
        target_yaw = cmd_vel_msg.angular.z
        # 4. 发送 MAVLink 速度+yaw控制
        # 构造MAVLink消息
        self.do_send_cmd(v=[vx, vy, vz], yaw=target_yaw, frame="body")

    @ros.topic("/mavros/rc/in", RCIn)
    def rcin_cb(self, msg: RCIn):
        channels = list(msg.channels)
        while len(channels) < 18:
            channels.append(1500)
        self.rc_channel = (msg.header.stamp, channels)

    @ros.topic("/mavros/global_position/rel_alt", Float64)
    def rel_alt_cb(self, data):
        self.rel_alt = data.data

    @ros.topic("/mavros/sys_status", SysStatus)
    def systatus_cb(self, data):
        self.sys_status = data

    @ros.topic("/mavros/statustext/recv", StatusText)
    def state_cb(self, data):
        self.state = data.text

    @ros.topic("/mavros/ws", String)
    def detect_cb(self, data):
        try:
            self.ws_pub.publish(data)
        except json.JSONDecodeError:
            pass

    @ros.topic("/mavros/state", State)
    def mode_cb(self, data):
        if self.arm == True and data.armed == False:
            self.runner.trigger(CEventType.DISARM)
        self.arm = data.armed
        land_mode = ["RTL", "LAND"]
        # print(f"set land: {self.mode not in land_mode} {data.mode} {data.mode in land_mode}")
        if (self.mode not in land_mode) and (data.mode in land_mode):
            self.runner.trigger(CEventType.SET_LAND)
        self.mode = data.mode

    @ros.topic("/drone_0_ego_planner_node/optimal_list", Marker)
    def optimal_cb(self, msg):
        cur = time.time()
        if cur - self.send_time < 1:
            return

        self.send_time = cur
        if self.odom is None:
            return

        xyz_list = []
        for pt in msg.points:
            xyz_list.append([pt.x, pt.y, pt.z])
        wp_list = []
        for xyz in xyz_list:
            gps = self.enu2gps(xyz)
            wp_list.append(gps)
        self.ws_pub.publish(json.dumps({"type": "state", "waypoint": wp_list}))

    @ros.topic("/planning/pos_cmd", PositionCommand)
    def cmd_cb(self, msg):
        if self.runner.node.type != NodeType.WP:
            return

        if not self.planner_enable:
            return

        self.do_send_cmd(
            p=[msg.position.x, msg.position.y, msg.position.z],
            v=[msg.velocity.x, msg.velocity.y, msg.velocity.z],
            a=[msg.acceleration.x, msg.acceleration.y, msg.acceleration.z],
            yaw=msg.yaw,
        )

    @ros.topic("/mavproxy/landing_target", PoseStamped)
    def landing_target_cb(self, msg: PoseStamped):
        quat = msg.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w]
        )
        self.landing_target = (
            msg.header.stamp,
            msg.pose.position.x,
            msg.pose.position.y,
            yaw,
        )

    @ros.topic("/mavros/distance_sensor/rangefinder_pub", Range)
    def rangefinder_cb(self, msg: Range):
        self.rangefinder_alt = msg.range

    @http_proxy.get("/get_gpsv2")
    def get_gpsv2(self):
        return SUCCESS_RESPONSE(
            {
                "pos": [self.lon, self.lat, self.rel_alt],
                "mode": self.mode,
                "arm": self.arm,
                "dis": {"current": 0, "min": 0, "max": 0},
                "gps_n": 10,
                "baro": -1,
                "gps": [self.gps_lon, self.gps_lat, self.gps_alt],
            }
        )

    @http_proxy.post("/set_posvel")
    def set_pos_vel(self, data: SetPosVelModel):
        """
        编队位置-速度控制接口（/set_posvel）

        ## 参数说明：
        - data.pos     : 目标GPS坐标 [lon, lat, alt]
        - data.vel     : 期望飞行速度（m/s）
        - data.yaw     : 目标偏航角（ENU弧度，可为 None）；到达终点后调整到此yaw，None则不调整
        - data.fix_yaw : 是否固定机头方向
                         True  → 运动全程锁定目标点的机头朝向
                         False → 机头始终朝向运动方向（实时更新yaw）
        - data.timeout : 接口的超时时间

        ## 状态流程（两种模式均相同）：

        其他状态 ──→ POSVEL_MOVE（移动）──→ POSVEL_YAW（调整终点yaw，若有）──→ 前一状态

        fix_yaw 影响的仅是 POSVEL_MOVE 过程中的 yaw 控制方式：

        - fix_yaw=True : 每帧发 yaw=目标点的yaw
        - fix_yaw=False: 每帧发 yaw=当前→目标的方位角（机头跟随运动方向）
        """
        # 记录进入 posvel 模式之前的状态（排除 posvel 内部状态自身，避免覆盖）
        cur_node_type = self.runner.node.type
        posvel_states = (NodeType.POSVEL_MOVE, NodeType.POSVEL_YAW)
        if cur_node_type not in posvel_states:
            self.posvel_node_before = cur_node_type
        # 保存目标参数
        self.posvel_target_pos = data.pos
        self.posvel_target_vel = data.vel
        self.posvel_target_yaw = data.yaw  # yaw 可为 None，表示到达后不额外调整
        self.posvel_fix_yaw = data.fix_yaw
        self.posvel_timeout = data.timeout  # 超时时长由接口传入
        if cur_node_type == NodeType.POSVEL_MOVE:
            # 已在移动中，直接触发目标更新
            self.runner.trigger(CEventType.SET_POSVEL, pos=data.pos, vel=data.vel)
        else:
            # 从其他状态进入：记录当前机头朝向（fix_yaw=True 时全程使用），然后直接进入移动
            self.runner.step(NodeType.POSVEL_MOVE)
        return SUCCESS_RESPONSE()

    @http_proxy.post("/stop_follow")
    def stop_follow(self):
        self.stop_planner()  # 关闭避障
        self.runner.trigger(CEventType.STOP_FOLLOW)
        return SUCCESS_RESPONSE()

    @http_proxy.post("/set_waypoint")
    def set_waypoint(self, data: SetWaypointModel):
        if self.runner.node.type == NodeType.INIT:
            return ERROR_RESPONSE("初始化中!")
        self._wp_raw = copy.deepcopy(data.waypoint)
        self.set_wp_cb(
            waypoint=data.waypoint,
            nodeEventList=data.nodeEventList,
            speed=data.speed,
            land=data.land,
            rtl=data.rtl,
        )
        return SUCCESS_RESPONSE()

    @http_proxy.get("/get_waypoint")
    def get_waypoint(self):
        return SUCCESS_RESPONSE(self._wp_raw)

    @http_proxy.post("/set_mode")
    def route_set_mode(self, data: SetModeModel):
        self.set_mode_service(0, data.mode)
        return SUCCESS_RESPONSE()

    @http_proxy.post("/land")
    def route_land(self, data: SetWaypointModel):
        if data.waypoint is None or len(data.waypoint) == 0:
            self.runner.trigger(CEventType.SET_LAND)
        else:
            data.land = True
            data.rtl = False
            return self.set_waypoint(data)
        return SUCCESS_RESPONSE()

    @http_proxy.post("/return")
    def route_return(self, data: SetWaypointModel):
        data.rtl = True
        data.land = False
        return self.set_waypoint(data)

    @http_proxy.post("/takeoff")
    def takeoff(self, data: TakeoffModel):
        if self.runner.node.type == NodeType.INIT:
            return ERROR_RESPONSE("初始化中, 无法起飞!")
        self.takeoff_alt = data.alt
        self.runner.step(NodeType.TAKING_OFF)
        return SUCCESS_RESPONSE()

    @http_proxy.post("/loiter")
    def loiter(self):
        if self.runner.node.type not in [NodeType.INIT, NodeType.GROUND]:
            self.runner.step(NodeType.HOVER)
        self.control.loiter()
        return SUCCESS_RESPONSE()

    @http_proxy.get("/get_gps")
    def get_gps(self):
        rel_alt = 0 if self.rel_alt is None else self.rel_alt
        yaw = 0 if self.yaw is None else self.yaw
        return {"msg": [self.lon, self.lat, rel_alt, yaw], "status": "success"}

    @http_proxy.post("/stop_planner")
    def stop_planner(self):
        self.planner_enable = False
        self.ws_pub.publish(json.dumps({"type": "state", "planner": self.planner_desc}))
        return SUCCESS_RESPONSE()

    @http_proxy.post("/start_planner")
    def start_planner(self, auto=False):
        if auto and not self.auto_planner_enable:
            return SUCCESS_RESPONSE("planner_enable=False时不允许自动打开避障")

        self.planner_enable = True
        self.ws_pub.publish(json.dumps({"type": "state", "planner": self.planner_desc}))
        return SUCCESS_RESPONSE()

    @http_proxy.get("/get_planner")
    def get_planner(self):
        return SUCCESS_RESPONSE(msg=self.planner_enable)

    @http_proxy.get("/get_pland")
    def get_pland(self):
        return SUCCESS_RESPONSE(msg=self.pland_enable)

    @http_proxy.post("/stop_pland")
    def stop_pland(self):
        self.pland_enable = False
        self.ws_pub.publish(json.dumps({"type": "state", "pland": self.pland_desc}))
        return SUCCESS_RESPONSE()

    @http_proxy.post("/start_pland")
    def start_pland(self):
        self.pland_enable = True
        self.ws_pub.publish(json.dumps({"type": "state", "pland": self.pland_desc}))
        return SUCCESS_RESPONSE()

    @http_proxy.post("/arm")
    def do_arm(self):
        self._do_arm()
        return SUCCESS_RESPONSE()

    @http_proxy.get("/prearms")
    def prearm(self):
        if not self.control.is_prearm_enable():
            try:
                self.do_arm()
                return SUCCESS_RESPONSE(dict(arm=True))
            except Exception as e:
                return ERROR_RESPONSE(dict(arm=False, reason=str(e)))

        # mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
        if (
            rospy.get_param("/mavros/param/ARMING_CHECK", 0) == 0
        ):  # 禁用prearm 检查，则始终返回True
            return SUCCESS_RESPONSE({"arm": True})
        self.state = ""
        response = self.cmd_service(
            command=401,  # MAV_CMD_RUN_PREARM_CHECKS
            confirmation=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0,
        )
        bits = 0x10000000
        if (self.sys_status.sensors_health & bits) == bits:
            return SUCCESS_RESPONSE({"arm": True})

        for i in range(1000):
            time.sleep(0.01)
            if self.state.startswith("PreArm: "):
                return SUCCESS_RESPONSE({"arm": False, "reason": self.state})

        return SUCCESS_RESPONSE({"arm": False, "reason": "wait for reason timeout"})

    @http_proxy.post("/reboot_fcu")
    def reboot_fcu(self):
        rospy.wait_for_service("/mavros/cmd/command", timeout=4)
        cmd_service = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (246)
        # param1=1: reboot autopilot, param2=0: do nothing for companion computer
        response = cmd_service(
            command=246,  # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
            param1=1.0,  # 1 = reboot autopilot
            param2=0.0,  # 0 = do nothing to companion
            param3=0.0,
            param4=0.0,
            confirmation=0,
        )
        return SUCCESS_RESPONSE("OK" if response.success else "Failed")


if __name__ == "__main__":
    rospy_init_node("control")
    control_node = Control()
    rospy.spin()
