#!/usr/bin/python3
# -*- coding: utf-8 -*-
from __future__ import annotations

from base.ctrl_node import Runner
import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
from mavros_msgs.msg import PositionTarget, SysStatus, StatusText, HomePosition
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Vector3, Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String
from rsos_msgs.msg import PointObj
import json
import threading
from pyproj import CRS, Transformer
import copy
import numpy as np
from base.utils import ERROR_RESPONSE, SUCCESS_RESPONSE, post_json
from base.node import Node
import time
import math
from mavros_msgs.msg import State
from visualization_msgs.msg import Marker
from enum import Enum
from base.ctrl_node import CtrlNode as _CtrlNode, EventType, Event
from tf.transformations import euler_from_quaternion


STOP_SPAN = 100
TAKEOFF_THRESHOLD = 0.05  # 高度判断阈值(比例)
HOVER_THRESHOLD = 1  # 判断为悬停状态(绝对高度)


class NodeType(Enum):
    INIT = "初始化"
    GROUND = "地面"
    TAKING_OFF = "正在起飞"
    TAKING_OFF2 = "航点中起飞"
    HOVER = "悬停"
    LIFTING = "调整高度"
    WP = "航点模式"
    FOLLOW = "跟随模式"
    LANDING = "正在降落"


class CEventType(EventType):
    ODOM_OK = "odom_ok"
    SET_TAKEOFF = "set_takeoff"
    SET_WP = "set_wp"
    SET_LAND = "set_land"
    DETECT = "detect"
    TAKEOFF_DONE = "takeoff_done"
    LIFT_DONE = "lift_done"
    DISARM = "DISARM"
    WP_FINISH = "wp_finish"
    STOP_FOLLOW = "stop_follow"


class CtrlNode(_CtrlNode):
    land_enable = True
    detect_enable = False
    wp_enable = False
    takeoff_enable = False
    ground_enable = True  # 是否进入ground状态

    context: Control

    def __init__(self, node_type: NodeType):
        super().__init__(node_type)
        if self.land_enable:
            self._register(CEventType.SET_LAND, self.land_cb)
        if self.detect_enable:
            self._register(CEventType.DETECT, self.detect_cb)
        if self.wp_enable:
            self._register(CEventType.SET_WP, self.set_wp_cb)
        if self.takeoff_enable:
            self._register(CEventType.SET_TAKEOFF, self.takeoff_cb)

        if self.ground_enable:
            self._register(CEventType.DISARM, self.land_done_cb)

    def land_done_cb(self):
        self.step(NodeType.GROUND)

    def land_cb(self):
        print("set land")
        self.context.do_land()
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

    def set_wp(self, waypoint: list, nodeEventList: list | None, land: bool, rtl: bool):
        print(f"set wp return: {rtl} wp: {waypoint}")
        context = self.context
        context.land = land or rtl
        context.nodeEventList = nodeEventList
        if rtl:
            return_alt = waypoint[-1][-1]  # 最后一个点的高度作为返航高度
            waypoint.append([context.takeoff_lon, context.takeoff_lat, return_alt])
        context.waypoint = waypoint[1:]
        context.wp_idx = 0

    def set_wp_cb(
        self, waypoint, nodeEventList=None, speed=None, land=False, rtl=False
    ):
        if len(waypoint) == 0 and not rtl:
            raise ValueError("No waypoint found!")
        if (len(waypoint) == 1 and not rtl) or (rtl and len(waypoint) == 0):
            # 如果只有一个航点(rtl为0个), 本来是不允许的, 现在额外插入一个
            waypoint.insert(0, [0, 0, 10])
        self.set_wp(waypoint, nodeEventList, land, rtl)
        context = self.context
        context.set_mode_service(0, "GUIDED")
        context.do_pub_wp(waypoint[0:1] + context.waypoint, land or rtl)
        self.step(NodeType.LIFTING)

    def takeoff_cb(self, alt: float):
        context = self.context
        context.takeoff_alt = alt
        self.step(NodeType.TAKING_OFF)

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
                self.ws_pub.publish(json.dumps({"type": "error", "error": f"参数: {angle} 无法转为数字!"}))
                return
            data = {"mode": "body", "angle": angle}

        post_json(url, data)
        

class InitNode(CtrlNode):
    ground_enable = False
    land_enable = False

    def __init__(self):
        super().__init__(NodeType.INIT)

    @CtrlNode.on(CEventType.ODOM_OK)
    def odom_ok_cb(self):
        context = self.context
        if context.check_hover():
            self.step(NodeType.HOVER)
        else:
            self.step(NodeType.GROUND)


class GroundNode(CtrlNode):
    takeoff_enable = True

    def __init__(self):
        super().__init__(NodeType.GROUND)

    @CtrlNode.on(CEventType.SET_WP)
    def set_wp_cb(
        self, waypoint, nodeEventList=None, speed=None, land=False, rtl=False
    ):
        context = self.context
        context.takeoff_alt = waypoint[0][-1]
        self.set_wp(waypoint, nodeEventList, land, rtl)
        context.do_pub_wp(waypoint[0:1] + context.waypoint, land or rtl)
        self.step(NodeType.TAKING_OFF2)

    @CtrlNode.on(CEventType.IDLE)
    def idle_cb(self):
        context = self.context
        if context.check_hover():
            self.step(NodeType.HOVER)


class TakeoffNode(CtrlNode):
    land_enable = True
    wp_enable = True
    takeoff_enable = True

    def __init__(self):
        super().__init__(NodeType.TAKING_OFF)

    def enter(self):
        context = self.context
        context.do_takeoff(context.takeoff_alt)

    @CtrlNode.on(CEventType.TAKEOFF_DONE)
    def takeoff_done_cb(self):
        context = self.context
        context.do_pub_takeoff()
        self.step(NodeType.HOVER)


class Takeoff2Node(CtrlNode):
    wp_enable = True
    land_enable = True
    takeoff_enable = True

    def __init__(self):
        super().__init__(NodeType.TAKING_OFF2)

    def enter(self):
        context = self.context
        context.do_takeoff(context.takeoff_alt)

    @CtrlNode.on(CEventType.TAKEOFF_DONE)
    def takeoff_done_cb(self):
        context = self.context
        context.do_pub_takeoff()
        self.step(NodeType.LIFTING)


class HoverNode(CtrlNode):
    detect_enable = True
    land_enable = True
    wp_enable = True
    takeoff_enable = True

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
        except:
            import traceback

            traceback.print_exc()

    @CtrlNode.on(CEventType.IDLE)
    def idle_cb(self):
        context = self.context
        # context.do_send_cmd(
        #     pz=context.lift_alt, px=self.px, py=self.py, yaw=context.lift_yaw
        # )
        
        vz = np.clip(context.lift_alt - context.rel_alt, -1, 1)
        context.do_send_cmd(
            vz=vz, vx=0, vy=0, yaw=context.lift_yaw
        )
        
        # context.do_send_cmd(yaw=context.lift_yaw)
        yaw_diff = context.check_yaw(context.lift_yaw)
        alt_diff = math.fabs(context.rel_alt - context.lift_alt)
        context.ws_pub.publish(json.dumps({"type": "state", "lift_diff": f"yaw: {yaw_diff:.2f} alt: {alt_diff:.2f}"}))
        if time.time() - self.start_time > 3: # 3s内移动太小，则退出
            if math.fabs(self.last_alt - context.rel_alt) < 0.1 and math.fabs(self.last_yaw - context.yaw) < 0.1:
                context.do_send_cmd(vx=0, vy=0, vz=0)
                self.step(NodeType.WP)
                        
            
            self.last_yaw = context.yaw
            self.last_alt = context.rel_alt
            self.start_time = time.time()
            
        
    @CtrlNode.on(CEventType.LIFT_DONE)
    def lift_done_cb(self):
        context = self.context
        for i in range(10):
            context.do_send_cmd(vx=0, vy=0, vz=0)
            time.sleep(0.1)  # 额外等待1秒完成高度调整
        self.step(NodeType.WP)


class WpNode(CtrlNode):
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
        print(px, py, pz)
        self.planner_enable = context.planner_enable  # 按照进入的时候是否开启判断
        if self.planner_enable:
            context.wp_pub.publish(goal)
        else:
            context.stop_pub.publish(Empty())
            context.do_send_cmd(px=px, py=py, pz=pz)

        if context.nodeEventList is not None:
            event_list = context.nodeEventList[context.wp_idx]
            print(f"event list: {event_list}")
            if event_list is not None:
                for event in event_list:
                    self.run_wp_event(event)

    def exit(self):
        context = self.context
        context.stop_pub.publish(Empty())

    @CtrlNode.on(CEventType.IDLE)
    def idle_cb(self):
        context = self.context
        cur_pos = [
            context.odom.pose.pose.position.x,
            context.odom.pose.pose.position.y,
            context.odom.pose.pose.position.z,
        ]
        dis = np.sqrt(
            (cur_pos[0] - self.goal[0]) ** 2
            + (cur_pos[1] - self.goal[1]) ** 2
            + (cur_pos[2] - self.goal[2]) ** 2
        )
        context.ws_pub.publish(json.dumps({"type": "state", "dis": dis}))

        if not self.planner_enable:
            if dis < 2:
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
        context.do_ws_pub(
            {
                "type": "state",
                "wp_idx": context.wp_idx + 1,
            }
        )
        print(f"wp done: {context.waypoint[context.wp_idx]}")
        context.wp_idx += 1
        if context.wp_idx >= len(context.waypoint):
            if context.land:
                self.step(NodeType.LANDING)
            else:
                self.step(NodeType.HOVER)
        else:
            self.step(NodeType.LIFTING)


class LandNode(CtrlNode):
    takeoff_enable = True
    wp_enable = True
    land_enable = True

    def __init__(self):
        super().__init__(NodeType.LANDING)

    def enter(self):
        context = self.context
        post_json("set_gimbal", {"mode": "body", "angle": 90})
        context.do_land()
        post_json("stop_record")


class FollowNode(CtrlNode):
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


class Control(Node):
    def __init__(self):
        super().__init__()
        rospy.loginfo("wait for mavros service...")
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/cmd/takeoff")
        rospy.loginfo("control done")
        self.takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.cmd_service = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        self.wp_pub = rospy.Publisher(
            "/move_base_simple/goal2", PoseStamped, queue_size=-1
        )
        self.ws_pub = rospy.Publisher("/mavproxy/ws", String, queue_size=-1)
        self.stop_pub = rospy.Publisher("/egoplanner/stopplan", Empty, queue_size=-1)
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=1
        )
        self.target_pub = rospy.Publisher(
            "/UAV0/perception/object_location/obj_lla", PointStamped, queue_size=1
        )

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
        self.lat = 0
        self.lon = 0
        self.land = False
        self.speed = 0
        self.odom_lock = threading.Lock()
        self._wp_raw = None
        self.yaw = None  # NED, 向北为正, 顺时针增加
        self.planner_enable = self._get_param("planner_enable", True)
        self.auto_planner_enable = self.planner_enable  # 是否允许在停止检测后自动打开避障

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
            ],
            context=self,
            step_cb=self.step_cb,
        )

    def register(self):
        super().register()
        verison = rospy.get_param("/mavros/version", "No version")
        self.do_ws_pub(
            {"type": "state", "planner": self.planner_enable, "version": verison}
        )

    def do_ws_pub(self, data):
        self.ws_pub.publish(json.dumps(data))

    def step_cb(self, prev, cur):
        self.ws_pub.publish(json.dumps({"type": "state", "state": cur.value}))

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
        print(f'pub wp {json.dumps({"mission_data": out, "type": "state"})}')

    def do_send_cmd(
        self,
        *,
        px=None,
        py=None,
        pz=None,
        vx=None,
        vy=None,
        vz=None,
        ax=None,
        ay=None,
        az=None,
        yaw=None,
        yaw_rate=None,
    ):
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "local_ned"

        # 坐标系选择
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        type_mask = 0
        ctrl_data = []
        for data, ignore in zip(
            [px, py, pz, vx, vy, vz, ax, ay, az, yaw, yaw_rate],
            [
                "PX",
                "PY",
                "PZ",
                "VX",
                "VY",
                "VZ",
                "AFX",
                "AFY",
                "AFZ",
                "YAW",
                "YAW_RATE",
            ],
        ):
            # 类型掩码：使用位置+速度
            if data is None:
                type_mask = type_mask | getattr(PositionTarget, f"IGNORE_{ignore}")
                data = 0
            ctrl_data.append(data)

        # 设置值
        target.position = Vector3(*ctrl_data[0:3])
        target.velocity = Vector3(*ctrl_data[3:6])
        target.acceleration_or_force = Vector3(*ctrl_data[6:9])
        target.yaw = ctrl_data[9]
        target.yaw_rate = ctrl_data[10]
        target.type_mask = type_mask
        # 发布
        self.setpoint_pub.publish(target)

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

    def do_land(self):
        self.set_mode_service(0, "LAND")

    def do_takeoff(self, alt):
        self.set_mode_service(0, "GUIDED")

        self.takeoff_lat = self.lat
        self.takeoff_lon = self.lon
        self.takeoff_alt = alt

        if self.rel_alt > HOVER_THRESHOLD and self.arm == True:
            odom = self.get_cur_odom()
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            self.do_send_cmd(px=x, py=y, pz=alt)
        else:
            # 解锁
            out = self.prearm()["msg"]
            if out.get("arm", False) == False:
                raise ValueError(out["reason"])
            self._do_arm()
            rospy.loginfo("Vehicle armed")

            # 持续发布目标点
            rospy.loginfo("Taking off...")
            response = self.takeoff_srv(
                min_pitch=0,
                yaw=0,
                latitude=0,
                longitude=0,
                altitude=alt,  # Target altitude in meters
            )
            rospy.loginfo("Takeoff command finished.")

    def quaternion_to_enu_yaw(self, quaternion):
        """
        将四元数转换为ENU坐标系的Yaw角

        参数:
            quaternion: [x, y, z, w]

        返回:
            yaw_enu_deg: ENU坐标系下的航向角（0-360°，0°=北）
        """
        # 转换为欧拉角（顺序：roll, pitch, yaw）
        # 注意：这是机体坐标系相对于ENU坐标系的旋转
        euler = euler_from_quaternion(quaternion)

        # MAVROS默认发布的是机体系到ENU的旋转
        # 机体系：X前，Y左，Z上
        # ENU系：X东，Y北，Z天

        # 从四元数得到的yaw是机体相对于ENU的航向
        # 但需要调整以获得正确的ENU航向
        yaw_rad = euler[2]  # 机头方向相对于东轴的夹角

        # 转换为ENU航向：从北开始，顺时针为正
        # 公式：enu_yaw = 90° - yaw_rad（度数）
        enu_yaw_rad = math.pi / 2 - yaw_rad

        # 规范化到0-2π范围
        if enu_yaw_rad < 0:
            enu_yaw_rad += 2 * math.pi
        if enu_yaw_rad > 2 * math.pi:
            enu_yaw_rad -= 2 * math.pi

        return enu_yaw_rad

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
                print("error: wait for odom")
                return
            odom_msg = copy.deepcopy(self.odom)
        if t_cmd is not None:
            t_odom = odom_msg.header.stamp.to_sec()
            time_diff = abs(t_odom - t_cmd)
            if time_diff >= 0.2:
                print(f"error: odom and PositionCommand time diff:{time_diff}")
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

    @Node.ros("/mavros/home_position/home", HomePosition)
    def home_callback(self, msg):
        if self.takeoff_lat == 0 and self.takeoff_lon == 0 and self.takeoff_alt == 0:
            self.takeoff_lat = msg.geo.latitude
            self.takeoff_lon = msg.geo.longitude
            self.takeoff_alt = msg.geo.altitude
            print(f"set_home: {self.takeoff_lon} {self.takeoff_lat} {self.takeoff_alt}")

    @Node.ros("/mavros/global_position/global", NavSatFix)
    def gps_cb(self, data: NavSatFix):
        self.lat = data.latitude
        self.lon = data.longitude

    @Node.ros("/mavros/global_position/raw/fix", NavSatFix)
    def gps_cb2(self, data: NavSatFix):
        self.gps_lat = data.latitude
        self.gps_lon = data.longitude
        self.gps_alt = data.altitude

    @Node.ros("/ego_planner/finish_event", Empty)
    def wp_done_cb(self, data=None):
        self.runner.trigger(CEventType.WP_FINISH)

    @Node.ros("/mavros/local_position/odom", Odometry)
    def odom_cb(self, msg):
        if self.rel_alt is not None:
            self.runner.trigger(CEventType.ODOM_OK)
        with self.odom_lock:
            self.odom = msg
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        self.yaw = self.quaternion_to_enu_yaw(quaternion)

    @Node.ros("/UAV0/perception/object_location/location_vel", PointObj)
    def target_cb(self, msg):
        self.runner.trigger(CEventType.DETECT, msg=msg)

    @Node.ros("/cmd_vel", Twist)
    def cmd_vel_cb(self, cmd_vel_msg):
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
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "local_ned"

        # 坐标系选择
        target.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED

        # 类型掩码：使用位置+速度
        target.type_mask = (
            PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY  # 忽略加速度x
            | PositionTarget.IGNORE_AFZ  # 忽略加速度y
            | PositionTarget.IGNORE_PX  # 忽略加速度z
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_YAW_RATE  # 忽略偏航速率
        )

        # 设置值
        target.velocity = Vector3(vx, vy, vz)  # 速度（机体坐标系，z取反，NED下为正）
        target.yaw = target_yaw
        # 发布
        self.setpoint_pub.publish(target)

    def check_alt(self, target, threshold):
        min_alt_threshold = self._get_param("min_alt_threshold", 0.5)
        return math.fabs(self.rel_alt - target) < max(
            target * threshold, min_alt_threshold
        )

    def check_yaw(self, yaw_enu):
        yaw_ned = math.pi / 2 - yaw_enu
        if yaw_ned < 0:
            yaw_ned += 2 * math.pi
        if yaw_ned > 2 * math.pi:
            yaw_ned -= 2 * math.pi
        return math.fabs(self.yaw - yaw_ned)

    def check_hover(self):
        """判断是否处于悬停状态"""
        return self.arm == True and self.rel_alt >= HOVER_THRESHOLD

    @Node.ros("/mavros/global_position/rel_alt", Float64)
    def rel_alt_cb(self, data):
        self.rel_alt = data.data
        if self.odom is not None:
            self.runner.trigger(CEventType.ODOM_OK)
        if self.check_alt(self.takeoff_alt, TAKEOFF_THRESHOLD):
            self.runner.trigger(CEventType.TAKEOFF_DONE)
        if self.check_alt(self.lift_alt, TAKEOFF_THRESHOLD):
            if self.lift_yaw is not None and self.check_yaw(self.lift_yaw) < 0.1:
                self.runner.trigger(CEventType.LIFT_DONE)

    @Node.ros("/mavros/sys_status", SysStatus)
    def systatus_cb(self, data):
        self.sys_status = data

    @Node.ros("/mavros/statustext/recv", StatusText)
    def state_cb(self, data):
        self.state = data.text

    @Node.ros("/mavros/ws", String)
    def detect_cb(self, data):
        try:
            self.ws_pub.publish(data)
        except json.JSONDecodeError:
            pass

    @Node.ros("/mavros/state", State)
    def mode_cb(self, data):
        if self.arm == True and data.armed == False:
            self.runner.trigger(CEventType.DISARM)

        self.arm = data.armed
        land_mode = ["RTL", "LAND"]
        # print(f"set land: {self.mode not in land_mode} {data.mode} {data.mode in land_mode}")
        if (self.mode not in land_mode) and (data.mode in land_mode):
            self.runner.trigger(CEventType.SET_LAND)
        self.mode = data.mode

    @Node.ros("/drone_0_ego_planner_node/optimal_list", Marker)
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

    @Node.ros("/planning/pos_cmd", PositionCommand)
    def cmd_cb(self, msg):
        if self.runner.node.type != NodeType.WP:
            return
        if not self.planner_enable:
            return
        self.do_send_cmd(
            px=msg.position.x,
            py=msg.position.y,
            pz=msg.position.z,
            vx=msg.velocity.x,
            vy=msg.velocity.y,
            vz=msg.velocity.z,
            ax=msg.acceleration.x,
            ay=msg.acceleration.y,
            az=msg.acceleration.z,
            yaw=msg.yaw,
        )

    @Node.route("/get_gpsv2", "GET")
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

    @Node.route("/set_posvel", "POST")
    def set_pos_vel(self, pos, vel):
        v = vel
        diff_x, diff_y, diff_z = self.gps_target2enu_diff(pos)
        distance = math.sqrt((diff_x * diff_x) + (diff_y * diff_y))
        if distance < 0.5:
            distance = 0
        if distance < v * v:
            v = math.sqrt(distance)

        radian = math.atan2(diff_y, diff_x)
        vx = v * math.cos(radian)
        vy = v * math.sin(radian)
        vz = np.clip(-diff_z, -1, 1)
        self.do_send_cmd(vx=vx, vy=vy, vz=vz)
        return SUCCESS_RESPONSE()

    @Node.route("/stop_follow", "POST")
    def stop_follow(self):
        self.stop_planner()  # 关闭避障
        self.runner.trigger(CEventType.STOP_FOLLOW)
        return SUCCESS_RESPONSE()

    @Node.route("/set_waypoint", "POST")
    def set_waypoint(
        self, waypoint, nodeEventList=None, speed=None, land=False, rtl=False
    ):
        self._wp_raw = copy.deepcopy(waypoint)
        self.runner.trigger(
            CEventType.SET_WP,
            waypoint=waypoint,
            nodeEventList=nodeEventList,
            speed=speed,
            land=land,
            rtl=rtl,
        )
        return SUCCESS_RESPONSE()

    @Node.route("/get_waypoint", "GET")
    def get_waypoint(self):
        return SUCCESS_RESPONSE(self._wp_raw)

    @Node.route("/set_mode", "POST")
    def route_set_mode(self, mode):
        self.set_mode_service(0, mode)
        return SUCCESS_RESPONSE()

    @Node.route("/land", "POST")
    def route_land(self, waypoint=None, speed=None):
        if waypoint is None or len(waypoint) == 0:
            self.runner.trigger(CEventType.SET_LAND)
        else:
            self.runner.trigger(
                CEventType.SET_WP, waypoint=waypoint, speed=speed, land=True
            )
        return SUCCESS_RESPONSE()

    @Node.route("/return", "POST")
    def route_return(self, waypoint=None, speed=None):
        if waypoint is None:
            waypoint = []
        self.runner.trigger(CEventType.SET_WP, waypoint=waypoint, speed=speed, rtl=True)
        return SUCCESS_RESPONSE()

    @Node.route("/takeoff", "POST")
    def takeoff(self, alt):
        self.runner.trigger(CEventType.SET_TAKEOFF, alt=alt)
        return SUCCESS_RESPONSE()

    @Node.route("/get_gps", "GET")
    def get_gps(self):
        rel_alt = 0 if self.rel_alt is None else self.rel_alt
        yaw = 0 if self.yaw is None else self.yaw
        return {"msg": [self.lon, self.lat, rel_alt, yaw], "status": "succecss"}

    @Node.route("/stop_planner", "POST")
    def stop_planner(self):
        self.planner_enable = False
        self.ws_pub.publish(json.dumps({"type": "state", "planner": "disable"}))
        return SUCCESS_RESPONSE()

    @Node.route("/start_planner", "POST")
    def start_planner(self, auto=False):
        if auto and not self.auto_planner_enable:
            return SUCCESS_RESPONSE("planner_enable=False时不允许自动打开避障")
        self.planner_enable = True
        self.ws_pub.publish(json.dumps({"type": "state", "planner": "enable"}))
        return SUCCESS_RESPONSE()

    @Node.route("/get_planner", "GET")
    def get_planner(self):
        return SUCCESS_RESPONSE(msg=self.planner_enable)

    @Node.route("/arm", "POST")
    def do_arm(self):
        self._do_arm()
        return SUCCESS_RESPONSE()

    @Node.route("/prearms", "GET")
    def prearm(self):
        # mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
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

    @Node.route("/reboot_fcu", "POST")
    def reboot_fcu(self):
        rospy.wait_for_service("/mavros/cmd/command")
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
    control_node = Control()
    control_node.run()
