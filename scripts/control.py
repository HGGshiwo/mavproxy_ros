#!/usr/bin/python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import functools
import os
import sys
from pathlib import Path

from event_callback.components.http.proxy import HTTP_ProxyComponent
from event_callback.components.ros import ROSComponent

from mavproxy_ros.controller import BaseController
from mavproxy_ros.state_estimator import StateEstimator

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
from typing import List, Tuple

import numpy as np
import rospy
import tf
import tf.transformations
from event_callback.core import BaseComponent, BaseEvent, BaseManager
from event_callback.ros_utils import rospy_init_node
from event_callback.utils import setup_logger, throttle
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import HomePosition, RCIn, State, StatusText, SysStatus
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rsos_msgs.msg import PointObj
from sensor_msgs.msg import NavSatFix, Range
from std_msgs.msg import Empty, Float64, String
from std_srvs.srv import Trigger, TriggerRequest
from visualization_msgs.msg import Marker

from mavproxy_ros.control_model import *
from mavproxy_ros.utils import (
    ERROR_RESPONSE,
    SUCCESS_RESPONSE,
    post_json,
    wait_for_debugger,
)

STOP_SPAN = 100  # 检测到目标后抑制重复检测的冷却时间(s)
TAKEOFF_THRESHOLD = 0.05  # 起飞/调高到达判定阈值(比例)，实际误差 = 目标高度 × 此值
LIFTING_TIMEOUT = 3  # 调整高度卡死超时(s)：周期内高度/偏航变化 < 0.1 则强制进入 WP
# 调整高度卡死判定阈值：高度或偏航变化量 < 此值视为卡死(m/rad)
LIFTING_STALL_THRESHOLD = 0.1
YAW_TOLERANCE = 0.1  # 航向对齐容差(rad)，用于 LiftingNode 和 PosVelYawNode
POSVEL_ARRIVE_DISTANCE = 0.5  # posvel 到达目标点的判定距离(m)


setup_logger(Path(__file__).parent.parent.joinpath("log").absolute())
logger = logging.getLogger(__name__)


class BaseState(BaseComponent):
    enter = BaseEvent()
    exit = BaseEvent()
    idle = BaseEvent()


InitState = type("InitState", (BaseState,), {})
GroundState = type("GroundState", (BaseState,), {})
TakeoffState = type("TakeoffState", (BaseState,), {})
Takeoff2State = type("Takeoff2State", (BaseState,), {})
HoverState = type("HoverState", (BaseState,), {})
LandState = type("LandState", (BaseState,), {})
WaypointState = type("WaypointState", (BaseState,), {})
LiftState = type("LiftState", (BaseState,), {})
FollowState = type("FollowState", (BaseState,), {})
PosvelMoveState = type("PosvelMoveState", (BaseState,), {})
PosvelYawState = type("PosvelYawState", (BaseState,), {})


class NodeType(Enum):
    INIT = ("初始状态", InitState)
    GROUND = ("地面状态", GroundState)
    TAKING_OFF = ("正在起飞", TakeoffState)
    TAKING_OFF2 = ("航点起飞", Takeoff2State)
    HOVER = ("悬停状态", HoverState)
    LIFTING = ("调整高度", LiftState)
    WP = ("航点模式", WaypointState)
    FOLLOW = ("跟随模式", FollowState)
    LANDING = ("正在降落", LandState)
    POSVEL_MOVE = ("编队移动", PosvelMoveState)
    POSVEL_YAW = ("编队偏航", PosvelYawState)  # 到达目标位置后调整到目标yaw


def state_guard(exclude: bool, raise_error: bool, *state: List[BaseState]):
    def wrapper(func):
        state_set = set(state)

        @functools.wraps(func)
        def new_func(self, *args, **kwargs):
            valid = True
            if self.node_type in state_set:
                if exclude:
                    valid = False
            else:
                if not exclude:
                    valid = False
            if valid:
                return func(self, *args, **kwargs)
            if raise_error:
                state_name = ", ".join([str(s) for s in state])
                error_msg = "不允许在" if exclude else "必须在"
                raise RuntimeError(
                    f"{func.__name__} {error_msg}{state_name}执行, 当前状态: {self.node_type}!"
                )

        return new_func

    return wrapper


class Control(BaseManager):
    def __init__(self, ros_node: ROSComponent, http_node: HTTP_ProxyComponent):
        self.state_map = {item: item.value[1]() for item in NodeType}
        BaseManager.__init__(self, ros_node, http_node, *self.state_map.values())
        self.init_control()
        self.idle_hz = 10
        threading.Thread(target=self.idle, daemon=True).start()

    def init_control(self):
        self.node_type = NodeType.INIT
        self.planner_enable = rospy.get_param("~planner_enable", True)
        self.pland_enable = rospy.get_param(
            "~pland_enable", default=True
        )  # 是否进行精准降落
        self.min_alt_threshold = rospy.get_param("~min_alt_threshold", 0.5)

        self.state_estimator = StateEstimator()
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
        self.target_takeoff_alt = None

        # gps经纬度
        self.gps_lat = 0
        self.gps_lon = 0
        self.gps_alt = 0
        self.land = False
        self.speed = 0

        self._wp_raw = None

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
        self.pland_start_srv = rospy.ServiceProxy("/pland/start", Trigger)
        self.pland_stop_srv = rospy.ServiceProxy("/pland/stop", Trigger)
        self.target_pub = rospy.Publisher(
            "/UAV0/perception/object_location/obj_lla", PointStamped, queue_size=1
        )

        self.auto_planner_enable = (
            self.planner_enable
        )  # 是否允许在停止检测后自动打开避障

        self.controller_name = rospy.get_param(
            "/mavproxy/control/controller_name", None
        )

        def trigger_land():
            """由外部触发直接进入地面状态"""
            if self.node_type == NodeType.GROUND:
                return

            logger.error("trigger land")
            self.step(NodeType.GROUND)

        self.control = BaseController.create(
            self.controller_name, trigger_land=trigger_land
        )
        self.do_send_cmd = self.control.do_send_cmd
        self.pland_enable = self.control.is_pland_enable() and self.pland_enable

    @HTTP_ProxyComponent.on_ready()
    def on_ready(self):
        verison = rospy.get_param("/mavros/version", "No version")
        self.do_ws_pub(
            {
                "type": "state",
                "planner": self.planner_desc,
                "version": verison,
                "state": self.node_type.value[0],
                "pland": self.pland_desc,
            }
        )
        logger.info("publish done")

    def idle(self):
        rate = rospy.Rate(self.idle_hz)
        while not rospy.is_shutdown():
            out = self.state_map.get(self.node_type).trigger("idle", {})
            for o in out:
                o.result()
            rate.sleep()

    def step(self, node_type: NodeType):
        prev = self.node_type
        cur = node_type
        logger.info(f"STATE: {prev} -> {cur}")
        if hasattr(self, "control"):
            self.control.state_change(prev, cur)
        prev_node = self.state_map.get(prev)
        prev_node.block_trigger("exit", {})
        cur_node = self.state_map.get(cur)
        self.node_type = cur
        cur_node.block_trigger("enter", {})
        self.ws_pub.publish(json.dumps({"type": "state", "state": cur.value[0]}))

    @property
    def planner_desc(self):
        return "启用" if self.planner_enable else "关闭"

    @property
    def pland_desc(self):
        return "启用" if self.pland_enable else "关闭"

    def check_state(self):
        if not self.state_estimator.odom_ok:
            return NodeType.INIT
        if self.check_hover():
            return NodeType.HOVER
        else:
            return NodeType.GROUND

    def do_ws_pub(self, data: dict):
        self.ws_pub.publish(json.dumps(data))

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

    def check_hover(self):
        return self.control.check_hover(self.arm, self.rel_alt)

    def check_alt(self, target: float, threshold: float):
        return math.fabs(self.rel_alt - target) < max(
            target * threshold, self.min_alt_threshold
        )

    def check_yaw(self, yaw_enu):
        """
        计算目标偏航角与当前偏航角的最小夹角
        返回值范围: [-pi, pi]
        正数和负数明确表示了相对方向（比如正为左，负为右）
        """
        # 直接求差值
        diff = yaw_enu - self.state_estimator.enu_yaw

        # 利用 atan2(sin, cos) 魔法将其完美映射到 [-pi, pi] 的最短路径
        theta = math.atan2(math.sin(diff), math.cos(diff))

        return math.fabs(theta)

    def check_arrive(self, goal: Tuple[float, float, float], tolerance: float = 2):
        if not self.state_estimator.odom_ok:
            return False

        cur_pos = [
            self.state_estimator.enu_x,
            self.state_estimator.enu_y,
            self.state_estimator.enu_z,
        ]
        dis = self.control.check_arrive(cur_pos, goal)
        self.ws_pub.publish(json.dumps({"type": "state", "dis": f"{dis:.2f}"}))
        return dis < tolerance

    def gps_target2goal(self, gps):
        """gps目标点转为move_base_simple/goal目标"""
        diff_x, diff_y, diff_z = self.state_estimator.gps2enu_body(gps)
        # ENU坐标
        enu_x = diff_x + self.state_estimator.enu_x  # 东向
        enu_y = diff_y + self.state_estimator.enu_y  # 北向
        enu_z = diff_z + self.state_estimator.enu_z  # 上向
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

    def arm_vehicle(self, timeout=10):
        arm, reason = self.prearm_check()
        if not arm:
            raise RuntimeError(reason)
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

    @InitState.idle()
    def init_state_idle(self):
        if not self.state_estimator.odom_ok:
            return
        if self.check_hover():
            self.step(NodeType.HOVER)
        else:
            self.step(NodeType.GROUND)

    @GroundState.idle()
    def ground_state_idle(self):
        if self.check_hover():
            self.step(NodeType.HOVER)

    @Takeoff2State.enter()
    @TakeoffState.enter()
    def takeoff_enter(self):
        self.set_mode_service(0, "GUIDED")
        alt = self.target_takeoff_alt
        # 已经起飞则退化为调整高度
        if self.check_hover():
            x = self.state_estimator.enu_x
            y = self.state_estimator.enu_y
            self.do_send_cmd(p=[x, y, alt])
            return
        if not self.arm:
            try:
                self.arm_vehicle()
                logger.info("Vehicle armed")
            except Exception as e:
                logger.error(e)
                state = self.check_state()
                self.step(state)
                raise e

        # 持续发布目标点
        logger.info("Taking off...")
        self.control.do_takeoff(alt)
        logger.info("Takeoff command finished.")

        self.takeoff_lat = self.state_estimator.lat
        self.takeoff_lon = self.state_estimator.lon
        self.takeoff_alt = alt

    @Takeoff2State.idle()
    @TakeoffState.idle()
    def takeoff_idle(self):
        if self.control.is_alt_enable():
            if not self.check_alt(self.takeoff_alt, TAKEOFF_THRESHOLD):
                return
        else:
            if not self.check_hover():  # 只检查是否悬停
                return
        self.do_ws_pub({"type": "event", "event": "takeoff"})
        if self.node_type == NodeType.TAKING_OFF2:
            self.step(NodeType.LIFTING)
        else:
            self.step(NodeType.HOVER)

    @WaypointState.enter()
    def waypoint_state_enter(self):
        goal = self.gps_target2goal(self.waypoint[self.wp_idx])
        px = goal.pose.position.x
        py = goal.pose.position.y
        pz = goal.pose.position.z
        self.goal = [px, py, pz]
        logger.info(f"wp target: {[px, py, pz]}")
        if self.planner_enable:
            self.wp_pub.publish(goal)
        else:
            self.stop_pub.publish(Empty())
            self.do_send_cmd(p=[px, py, pz])
        if self.nodeEventList is not None:
            event_list = self.nodeEventList[self.wp_idx]
            logger.info(f"event list: {event_list}")
            if event_list is not None:
                for event in event_list:
                    self.run_wp_event(event)

    @WaypointState.idle()
    @throttle(2)
    def waypoint_state_idle(self):
        arrive = self.check_arrive(self.goal)
        if not self.planner_enable and arrive:
            self.waypoint_finish()

    @WaypointState.exit()
    def waypoint_state_exit(self):
        self.stop_pub.publish(Empty())

    @HoverState.idle()
    def hover_state_idle(self):
        self.do_send_cmd(v=[0, 0, 0], yaw_rate=0, frame="body")

    @LandState.enter()
    def land_state_enter(self):
        """
        降落状态：执行降落动作，支持普通降落和精准降落两种模式。
        """
        post_json("set_gimbal", {"mode": "body", "angle": 90})
        post_json("stop_record")

        if not self.pland_enable:
            self.control.do_land()
        else:
            rospy.wait_for_service("/pland/start", 10)
            self.pland_start_srv(TriggerRequest())

    @LandState.exit()
    def land_state_exit(self):
        if self.pland_enable:
            rospy.wait_for_service("/pland/stop", 10)
            self.pland_stop_srv(TriggerRequest())
        self.do_ws_pub({"type": "event", "event": "disarm"})

    @LiftState.enter()
    def lift_state_enter(self):
        self.lift_alt = self.waypoint[self.wp_idx][-1]
        diff_x, diff_y, diff_z = self.state_estimator.gps2enu_body(
            self.waypoint[self.wp_idx]
        )

        self.start_time = time.time()
        self.last_alt = self.rel_alt
        self.last_yaw = self.state_estimator.ned_yaw
        try:
            self.lift_yaw = self.enu_xy2yaw(diff_x, diff_y)  # 这里是enu的yaw
        except Exception as e:
            logger.exception(str(e))

    @LiftState.idle()
    def lift_state_idle(self):
        vz = np.clip(self.lift_alt - self.rel_alt, -1, 1)
        self.do_send_cmd(v=[0, 0, vz], yaw=self.lift_yaw)
        yaw_diff = self.check_yaw(self.lift_yaw)
        alt_diff = math.fabs(self.rel_alt - self.lift_alt)
        alt_diff = alt_diff if self.control.is_alt_enable() else 0
        self.ws_pub.publish(
            json.dumps(
                {
                    "type": "state",
                    "yaw_diff": f"{yaw_diff:.2f}",
                    "alt_diff": f"{alt_diff:.2f}",
                }
            )
        )
        # 达到目标高度和航向则立即退出
        if self.lift_yaw is None or yaw_diff < YAW_TOLERANCE:
            if not self.control.is_alt_enable() or self.check_alt(
                self.lift_alt, TAKEOFF_THRESHOLD
            ):
                self.do_send_cmd(v=[0, 0, 0])
                self.step(NodeType.WP)
                return

        if time.time() - self.start_time > LIFTING_TIMEOUT:  # 卡死超时，强制进入 WP
            if (
                math.fabs(self.last_yaw - self.state_estimator.ned_yaw)
                < LIFTING_STALL_THRESHOLD
            ):
                if (
                    not self.control.is_alt_enable()
                    or math.fabs(self.last_alt - self.rel_alt) < LIFTING_STALL_THRESHOLD
                ):
                    self.do_send_cmd(v=[0, 0, 0])
                    self.step(NodeType.WP)
            self.last_yaw = self.state_estimator.ned_yaw
            self.last_alt = self.rel_alt
            self.start_time = time.time()

    def set_posvel(self, pos, vel):
        """收到新的posvel请求时更新目标并刷新超时计时"""
        self.posvel_target_pos = pos
        self.posvel_target_vel = vel
        self._last_call_time = time.time()

    def send_posvel_cmd(self):
        diff_x, diff_y, diff_z = self.state_estimator.gps2enu_body(
            self.posvel_target_pos
        )
        distance = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        v = self.posvel_target_vel
        if distance < v * v:
            v = math.sqrt(distance)
        radian = math.atan2(diff_y, diff_x)
        vx = v * math.cos(radian)
        vy = v * math.sin(radian)
        vz = np.clip(diff_z, -1, 1)
        if self.posvel_fix_yaw:
            # fix_yaw=True：始终锁定进入时记录的初始机头朝向
            self.do_send_cmd(v=[vx, vy, vz], yaw=self.posvel_target_yaw)
        else:
            # fix_yaw=False：实时将机头朝向目标直线方向（ENU弧度）
            heading_yaw = self.enu_xy2yaw(diff_x, diff_y)
            self.do_send_cmd(v=[vx, vy, vz], yaw=heading_yaw)

    @PosvelMoveState.idle()
    def posvel_state_idle(self):
        """
        周期检查两个退出条件：
          1. 到达目标点（与 WpNode 相同的三维 odom 距离判断）→ 进入 POSVEL_YAW
          2. 超时（超过 self.posvel_timeout 秒无新指令）→ 退回前一状态
        """
        # ── 超时检查 ──────────────────────────────────────────────────────────
        elapsed = time.time() - self._last_call_time
        if elapsed > self.posvel_timeout:
            logger.info(
                f"[PosvelMove] timeout ({elapsed:.2f}s), returning to prev state"
            )
            self.step(self.posvel_node_before)
            return
        goal = self.state_estimator.gps2enu(self.posvel_target_pos)
        arrive = self.check_arrive(goal, POSVEL_ARRIVE_DISTANCE)
        if arrive:
            self.step(NodeType.POSVEL_YAW)
            return
        self.send_posvel_cmd()

    @PosvelYawState.enter()
    def pos_vel_yaw_state_enter(self):
        # 如果没有指定目标yaw，直接退回前一状态
        if self.posvel_target_yaw is None:
            logger.info("[PosvelYaw] no target yaw, returning to prev state")
            self.step(self.posvel_node_before)

    @PosvelYawState.idle()
    def pos_vel_yaw_state_idle(self):
        target_yaw = self.posvel_target_yaw
        if target_yaw is None:
            self.step(self.posvel_node_before)
            return

        # 持续发送原地悬停+偏航调整指令
        self.do_send_cmd(v=[0, 0, 0], yaw=target_yaw)
        yaw_diff = self.check_yaw(target_yaw)
        logger.info(f"[PosvelYaw] yaw_diff: {yaw_diff:.3f} rad")
        if yaw_diff < YAW_TOLERANCE:
            logger.info("[PosvelYaw] yaw aligned, returning to prev state")
            self.step(self.posvel_node_before)

    def land_done_cb(self):
        self.step(NodeType.GROUND)

    def land_cb(self):
        logger.info("set land")
        self.step(NodeType.LANDING)
        post_json("stop_record")

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

    def set_wp_cb(
        self, waypoint, nodeEventList=None, speed=None, land=False, rtl=False
    ):
        self._wp_raw = copy.deepcopy(waypoint)
        if len(waypoint) == 0 and not rtl:
            raise ValueError("No waypoint found!")

        if (len(waypoint) == 1 and not rtl) or (rtl and len(waypoint) == 0):
            # 如果只有一个航点(rtl为0个), 本来是不允许的, 现在额外插入一个
            waypoint.insert(0, [0, 0, 10])
        if self.node_type == NodeType.GROUND:
            self.target_takeoff_alt = waypoint[0][-1]
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
        self.step(next_state)

    def prearm_check(self):
        if not self.control.is_prearm_enable():
            try:
                self.arm_vehicle()
                return True, ""

            except Exception as e:
                return False, str(e)

        # mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
        if (
            rospy.get_param("/mavros/param/ARMING_CHECK", 0) == 0
        ):  # 禁用prearm 检查，则始终返回True
            return True, ""

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
            return True, ""

        for i in range(1000):
            time.sleep(0.01)
            if self.state.startswith("PreArm: "):
                return False, self.state

        return False, "wait for reason timeout"

    def waypoint_finish(self):
        if not self.land:  # 非降落/返航情况下，发布进度
            self.do_ws_pub(
                {
                    "type": "event",
                    "event": "progress",
                    "cur": self.wp_idx + 1,  # wp_idx是到达点的前一个点
                    "total": len(self.waypoint),
                }
            )
        self.do_ws_pub({"type": "state", "wp_idx": self.wp_idx + 1})
        logger.info(f"wp done: {self.waypoint[self.wp_idx]}")
        self.wp_idx += 1
        if self.wp_idx >= len(self.waypoint):
            if self.land:
                self.step(NodeType.LANDING)
            else:
                self.step(NodeType.HOVER)
        else:
            self.step(NodeType.LIFTING)

    @ROSComponent.on_topic("/mavros/global_position/raw/gps_vel", TwistStamped, 10)
    def gps_vel_cb(self, msg: TwistStamped):
        # enu -> ned
        vx = msg.twist.linear.y
        vy = msg.twist.linear.x
        self.do_ws_pub(dict(x_vel=vx, y_vel=vy))

    @ROSComponent.on_topic("/mavros/home_position/home", HomePosition)
    def home_callback(self, msg: HomePosition):
        if self.takeoff_lat == 0 and self.takeoff_lon == 0 and self.takeoff_alt == 0:
            self.takeoff_lat = msg.geo.latitude
            self.takeoff_lon = msg.geo.longitude
            self.takeoff_alt = msg.geo.altitude
            logger.info(
                f"set_home: {self.takeoff_lon} {self.takeoff_lat} {self.takeoff_alt}"
            )

    @ROSComponent.on_topic("/mavros/global_position/global", NavSatFix)
    def gps_cb(self, data: NavSatFix):
        self.state_estimator.set_state(lat=data.latitude, lon=data.longitude)
        self.do_ws_pub(dict(lat=data.latitude, lon=data.longitude))

    @ROSComponent.on_topic("/mavros/global_position/raw/fix", NavSatFix)
    def gps_cb2(self, data: NavSatFix):
        self.gps_lat = data.latitude
        self.gps_lon = data.longitude
        self.gps_alt = data.altitude

    @ROSComponent.on_topic("/ego_planner/finish_event", Empty)
    @state_guard(False, False, NodeType.WP)
    def wp_done_cb(self, data=None):
        self.waypoint_finish()

    @ROSComponent.on_topic("/mavros/local_position/odom", Odometry)
    def odom_cb(self, msg: Odometry):
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.state_estimator.set_state(
            enu_x=msg.pose.pose.position.x,
            enu_y=msg.pose.pose.position.y,
            enu_z=msg.pose.pose.position.z,
            quaternion=quaternion,
        )

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        self.do_ws_pub(
            dict(yaw=self.state_estimator.ned_yaw, x_vel_body=vx, v_vel_body=vy)
        )

    @ROSComponent.on_topic("/UAV0/perception/object_location/location_vel", PointObj)
    @state_guard(True, False, NodeType.INIT, NodeType.GROUND)
    def target_cb(self, msg):
        if time.time() - self.last_send < STOP_SPAN:
            return

        if self.node_type != NodeType.FOLLOW:
            self.node_before_detect = self.node_type
        if msg.score < 0:
            msg.velocity.x = 0
            msg.velocity.y = 0
            msg.velocity.z = 0
        else:
            goal = PointStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            lon, lat, alt = self.state_estimator.enu2gps(
                [msg.pos.x, msg.pos.y, msg.pos.z]
            )
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
        self.step(NodeType.FOLLOW)

    @ROSComponent.on_topic("/cmd_vel2", Twist)
    @state_guard(True, False, NodeType.INIT)
    def cmd_vel_cb(self, cmd_vel_msg: Twist):
        # 2. 获取 cmd_vel 中的速度（机体坐标系，无需转换）
        vx = cmd_vel_msg.linear.x  # 机体前向速度
        vy = cmd_vel_msg.linear.y  # 机体左向速度
        vz = cmd_vel_msg.linear.z  # 机体上向速度
        # delta_yaw = cmd_vel_msg.angular.z  # 期望的yaw增量（弧度）
        target_yaw = cmd_vel_msg.angular.z
        # 4. 发送 MAVLink 速度+yaw控制
        # 构造MAVLink消息
        self.do_send_cmd(v=[vx, vy, vz], yaw=target_yaw, frame="body")

    @ROSComponent.on_topic("/mavros/rc/in", RCIn)
    def rcin_cb(self, msg: RCIn):
        channels = list(msg.channels)
        while len(channels) < 18:
            channels.append(1500)
        self.rc_channel = (msg.header.stamp, channels)

    @ROSComponent.on_topic("/mavros/global_position/rel_alt", Float64)
    def rel_alt_cb(self, data):
        self.rel_alt = data.data

    @ROSComponent.on_topic("/mavros/sys_status", SysStatus)
    def systatus_cb(self, data):
        self.sys_status = data

    @ROSComponent.on_topic("/mavros/statustext/recv", StatusText)
    def state_cb(self, data):
        self.state = data.text

    @ROSComponent.on_topic("/mavros/ws", String)
    def on_mavros_ws(self, data):
        try:
            self.ws_pub.publish(data)
        except json.JSONDecodeError:
            pass

    @ROSComponent.on_topic("/mavros/state", State)
    def mode_cb(self, data):
        if self.arm == True and data.armed == False:
            self.step(NodeType.GROUND)
        self.arm = data.armed
        land_mode = ["RTL", "LAND"]
        # print(f"set land: {self.mode not in land_mode} {data.mode} {data.mode in land_mode}")
        if (self.mode not in land_mode) and (data.mode in land_mode):
            self.land_cb()
        self.mode = data.mode

    @ROSComponent.on_topic("/drone_0_ego_planner_node/optimal_list", Marker)
    @state_guard(True, False, NodeType.INIT)
    def optimal_cb(self, msg):
        cur = time.time()
        if cur - self.send_time < 1:
            return

        self.send_time = cur

        xyz_list = []
        for pt in msg.points:
            xyz_list.append([pt.x, pt.y, pt.z])
        wp_list = []
        for xyz in xyz_list:
            gps = self.state_estimator.enu2gps(xyz)
            wp_list.append(gps)
        self.ws_pub.publish(json.dumps({"type": "state", "waypoint": wp_list}))

    @ROSComponent.on_topic("/planning/pos_cmd", PositionCommand)
    @state_guard(False, False, NodeType.WP)
    def cmd_cb(self, msg):
        if not self.planner_enable:
            return

        self.do_send_cmd(
            p=[msg.position.x, msg.position.y, msg.position.z],
            v=[msg.velocity.x, msg.velocity.y, msg.velocity.z],
            a=[msg.acceleration.x, msg.acceleration.y, msg.acceleration.z],
            yaw=msg.yaw,
        )

    @ROSComponent.on_topic("/mavproxy/landing_target", PoseStamped)
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

    @ROSComponent.on_topic("/mavros/distance_sensor/rangefinder_pub", Range)
    def rangefinder_cb(self, msg: Range):
        self.rangefinder_alt = msg.range

    @HTTP_ProxyComponent.on_get("/get_gpsv2")
    def get_gpsv2(self):
        return SUCCESS_RESPONSE(
            {
                "pos": [
                    self.state_estimator.lon,
                    self.state_estimator.lat,
                    self.rel_alt,
                ],
                "mode": self.mode,
                "arm": self.arm,
                "dis": {"current": 0, "min": 0, "max": 0},
                "gps_n": 10,
                "baro": -1,
                "gps": [self.gps_lon, self.gps_lat, self.gps_alt],
            }
        )

    @HTTP_ProxyComponent.on_post("/set_posvel")
    @state_guard(True, True, NodeType.GROUND, NodeType.INIT)
    def on_post_set_posvel(self, data: SetPosVelModel):
        """
        编队位置-速度控制接口（/set_posvel）

        ## 参数说明：
        - data.pos     : 目标GPS坐标 [lon, lat, alt]
        - data.vel     : 期望飞行速度（m/s）
        - data.yaw     : 目标偏航角（NED弧度，可为 None）；到达终点后调整到此yaw，None则不调整
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
        cur_node_type = self.node_type
        posvel_states = (NodeType.POSVEL_MOVE, NodeType.POSVEL_YAW)
        if cur_node_type not in posvel_states:
            self.posvel_node_before = cur_node_type
        # 保存目标参数
        if len(data.pos) == 2:
            data.pos.append(0)
        self.posvel_target_pos = data.pos
        self.posvel_target_vel = data.vel
        if data.yaw is not None:
            data.yaw = np.pi / 2 - data.yaw
        self.posvel_target_yaw = data.yaw  # yaw 可为 None，表示到达后不额外调整
        self.posvel_fix_yaw = data.fix_yaw
        self.posvel_timeout = data.timeout  # 超时时长由接口传入

        # 更新目标
        self.set_posvel(pos=data.pos, vel=data.vel)
        if cur_node_type != NodeType.POSVEL_MOVE:
            self.step(NodeType.POSVEL_MOVE)
        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_post("/stop_follow")
    @state_guard(False, True, NodeType.FOLLOW)
    def stop_follow(self):
        """Follow状态下，退出follow"""
        self.stop_planner()  # 关闭避障

        self.last_send = time.time()
        rospy.set_param("/UAV0/perception/yolo_detection/enable_detection", False)
        rospy.set_param("/UAV0/perception/yolo_detection_smoke/enable_detection", False)
        rospy.set_param(
            "/UAV0/perception/object_location/object_location_node/enable_send", False
        )
        node_before_detect = (
            NodeType.LIFTING
            if self.node_before_detect == NodeType.WP
            else self.node_before_detect
        )
        self.step(node_before_detect)
        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_post("/set_waypoint")
    @state_guard(True, True, NodeType.INIT)
    def set_waypoint(self, data: SetWaypointModel):
        self.set_wp_cb(
            waypoint=data.waypoint,
            nodeEventList=data.nodeEventList,
            speed=data.speed,
            land=data.land,
            rtl=data.rtl,
        )
        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_get("/get_waypoint")
    def get_waypoint(self):
        return SUCCESS_RESPONSE(self._wp_raw)

    @HTTP_ProxyComponent.on_post("/set_mode")
    def route_set_mode(self, data: SetModeModel):
        self.set_mode_service(0, data.mode)
        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_post("/land")
    def on_post_land(self, data: SetWaypointModel):
        if data.waypoint is None or len(data.waypoint) == 0:
            self.land_cb()
        else:
            data.land = True
            data.rtl = False
            return self.set_waypoint(data)

        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_post("/return")
    def on_post_return(self, data: SetWaypointModel):
        data.rtl = True
        data.land = False
        return self.set_waypoint(data)

    @HTTP_ProxyComponent.on_post("/takeoff")
    @state_guard(True, True, NodeType.INIT)
    def takeoff(self, data: TakeoffModel):
        """
        在GROUND状态时，尝试起飞，在非GROUND状态(INIT外)，调整高度并悬停
        """
        self.target_takeoff_alt = data.alt
        self.step(NodeType.TAKING_OFF)
        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_post("/loiter")
    def loiter(self):
        if self.node_type not in [NodeType.INIT, NodeType.GROUND]:
            self.step(NodeType.HOVER)
        self.control.loiter()
        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_get("/get_gps")
    def get_gps(self):
        rel_alt = 0 if self.rel_alt is None else self.rel_alt
        return {
            "msg": [
                self.state_estimator.lon,
                self.state_estimator.lat,
                rel_alt,
                self.state_estimator.ned_yaw,
            ],
            "status": "success",
        }

    @HTTP_ProxyComponent.on_post("/stop_planner")
    def stop_planner(self):
        self.planner_enable = False
        self.ws_pub.publish(json.dumps({"type": "state", "planner": self.planner_desc}))
        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_post("/start_planner")
    def start_planner(self, auto=False):
        if auto and not self.auto_planner_enable:
            return SUCCESS_RESPONSE("planner_enable=False时不允许自动打开避障")

        self.planner_enable = True
        self.ws_pub.publish(json.dumps({"type": "state", "planner": self.planner_desc}))
        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_get("/get_planner")
    def get_planner(self):
        return SUCCESS_RESPONSE(msg=self.planner_enable)

    @HTTP_ProxyComponent.on_get("/get_pland")
    def get_pland(self):
        return SUCCESS_RESPONSE(msg=self.pland_enable)

    @HTTP_ProxyComponent.on_post("/stop_pland")
    def stop_pland(self):
        self.pland_enable = False
        self.ws_pub.publish(json.dumps({"type": "state", "pland": self.pland_desc}))
        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_post("/start_pland")
    def start_pland(self):
        self.pland_enable = True
        self.ws_pub.publish(json.dumps({"type": "state", "pland": self.pland_desc}))
        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_post("/disarm")
    def on_disarm(self):
        self.arm_service(False)
        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_post("/arm")
    def on_post_arm(self):
        self.arm_vehicle()
        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_get("/prearms")
    def on_get_prearms(self):
        arm, reason = self.prearm_check()
        return SUCCESS_RESPONSE(dict(arm=arm, reason=reason))

    @HTTP_ProxyComponent.on_post("/set_joystick")
    def set_joystick(self, data: JoystickModel):
        """
        left_x: X轴速度（-1~1，正向前）
        left_y: Y轴速度（-1~1，正向左）
        right_x: Yaw角速度（-1~1，正左转（逆时针））
        right_y: z轴速度(-1~1，正向上)
        """
        self.control.set_joystick(data.left_x, data.left_y, data.right_x, data.right_y)
        return SUCCESS_RESPONSE()

    @HTTP_ProxyComponent.on_post("/reboot_fcu")
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

    @ROSComponent.on_topic("restart", String)
    def on_restart(self, data):
        self.init_control()
        return SUCCESS_RESPONSE()


if __name__ == "__main__":
    rospy_init_node("control")
    # wait_for_debugger()
    ros_node = ROSComponent()
    http_node = HTTP_ProxyComponent()
    control_node = Control(ros_node, http_node)
    rospy.spin()
