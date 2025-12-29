#!/usr/bin/python3
# -*- coding: utf-8 -*-
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
from base.utils import ERROR_RESPONSE, SUCCESS_RESPONSE
from base.node import Node
import time
import math
from mavros_msgs.msg import State
from visualization_msgs.msg import Marker
from enum import Enum
from base.ctrl_node import CtrlNode as _CtrlNode, EventType, Event

STOP_SPAN = 100
TAKEOFF_THRESHOLD = 0.1
HOVER_THRESHOLD = 0.1

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
    LAND_DONE = "land_done"
    WP_FINISH = "wp_finish"
    STOP_FOLLOW = "stop_follow"
    
class CtrlNode(_CtrlNode):
    land_enable = False
    detect_enable = False
    wp_enable = False
    takeoff_enable = False
    
    def __init__(self, node_type):
        super().__init__(node_type)
        if self.land_enable:
            self._register(CEventType.SET_LAND ,self.land_cb)
        if self.detect_enable:
            self._register(CEventType.DETECT, self.detect_cb)
        if self.wp_enable:
            self._register(CEventType.SET_WP, self.set_wp_cb)
        if self.takeoff_enable:
            self._register(CEventType.SET_TAKEOFF, self.takeoff_cb)
            
    def land_cb(self):
        self.context.do_land()    
        self.step(NodeType.LANDING)
        
    def detect_cb(self, msg):
        context = self.context
        if time.time() - context.last_send < STOP_SPAN:
            return
        if self.type != NodeType.FOLLOW: 
            context.node_before_detect = self.type
        context.do_detect(msg)
        self.step(NodeType.FOLLOW)
    
    def set_wp(self, waypoint, land, rtl):
        print(f"set wp return: {rtl} wp: {waypoint}")
        context = self.context
        context.land = land or rtl
        if rtl:
            return_alt = waypoint[-1][-1] # 最后一个点的高度作为返航高度
            waypoint.append([context.takeoff_lon, context.takeoff_lat, return_alt])
        context.waypoint = waypoint[1:]
        context.wp_idx = 0
    
    def set_wp_cb(self, waypoint, speed=None, land=False, rtl=False):
        print(f"set wp return123: {rtl} wp: {waypoint}")
        if len(waypoint) == 0 and not rtl:
            raise ValueError("No waypoint found!")
        if (len(waypoint) == 1 and not rtl) or (rtl and len(waypoint) == 0):
            # 如果只有一个航点(rtl为0个), 本来是不允许的, 现在额外插入一个
            waypoint.insert(0, [0, 0, 10]) 
        self.set_wp(waypoint, land, rtl) 
        context = self.context
        context.do_pub_wp(waypoint[0:1] + context.waypoint, land or rtl) 
        self.step(NodeType.LIFTING)     
    
    def takeoff_cb(self, alt):
        context = self.context
        context.takeoff_alt = alt
        self.step(NodeType.TAKING_OFF)
        
class InitNode(CtrlNode):
    def __init__(self):
        super().__init__(NodeType.INIT)
    
    @CtrlNode.on(CEventType.ODOM_OK)
    def odom_ok_cb(self):
        context = self.context
        if context.rel_alt > HOVER_THRESHOLD:
            self.step(NodeType.HOVER)
        else:
            self.step(NodeType.GROUND)

class GroundNode(CtrlNode):
    takeoff_enable = True
    def __init__(self):
        super().__init__(NodeType.GROUND)
        
    @CtrlNode.on(CEventType.SET_WP)
    def set_wp_cb(self, waypoint, speed=None, land=False, rtl=False):
        context = self.context
        context.takeoff_alt = waypoint[0][-1]
        self.set_wp(waypoint, land, rtl)
        context.do_pub_wp(waypoint[0:1] + context.waypoint, land or rtl) 
        self.step(NodeType.TAKING_OFF2)
    
class TakeoffNode(CtrlNode):
    land_enable = True
    wp_enable = True
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
    
    @CtrlNode.on(CEventType.IDLE)
    def idle_cb(self):
        context = self.context
        odom = context.get_cur_odom()
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        context.do_send_cmd(pz=context.lift_alt, px=px, py=py)
    
    @CtrlNode.on(CEventType.LIFT_DONE)
    def lift_done_cb(self):
        self.step(NodeType.WP)

class WpNode(CtrlNode):
    detect_enable = True
    land_enable = True
    wp_enable = True
    def __init__(self):
        super().__init__(NodeType.WP)

    def enter(self):
        context = self.context
        context.wp_pub.publish(context.gps_target2goal(context.waypoint[context.wp_idx]))
    
    def exit(self):
        context = self.context
        context.stop_pub.publish(Empty())
    
    @CtrlNode.on(CEventType.WP_FINISH)
    def wp_finish_cb(self):
        context = self.context
        if not context.land: # 非降落/返航情况下，发布进度
            context.do_ws_pub({
                "type": "event",
                "event": "progress",
                "cur": context.wp_idx + 1, # wp_idx是到达点的前一个点
                "total": len(context.waypoint),
            })
        context.do_ws_pub({
            "type": "state",
            "wp_idx": context.wp_idx + 1,
        })
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
    def __init__(self):
        super().__init__(NodeType.LANDING)
    
    def enter(self):
        context = self.context
        context.do_land()

    @CtrlNode.on(CEventType.LAND_DONE)
    def land_done_cb(self):
        self.step(NodeType.GROUND)
        
class FollowNode(CtrlNode):
    detect_enable = True
    def __init__(self):
        super().__init__(NodeType.FOLLOW)
    
    @CtrlNode.on(CEventType.STOP_FOLLOW)
    def stop_follow_cb(self):
        context = self.context
        context.last_send = time.time()
        rospy.set_param("/UAV0/perception/yolo_detection/enable_detection", False)
        rospy.set_param("/UAV0/perception/object_location/object_location_node/enable_send", False)
        node_before_detect = NodeType.LIFTING if context.node_before_detect == NodeType.WP else context.node_before_detect
        self.step(node_before_detect)
        
class Control(Node):
    def __init__(self):
        super().__init__()
        rospy.loginfo("wait for mavros service...")
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/takeoff')
        rospy.loginfo("control done")
        self.takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.cmd_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        self.wp_pub = rospy.Publisher("/move_base_simple/goal2", PoseStamped, queue_size=-1)
        self.ws_pub = rospy.Publisher("/mavproxy/ws", String, queue_size=-1)
        self.stop_pub = rospy.Publisher("/egoplanner/stopplan", Empty, queue_size=-1)
        self.setpoint_pub = rospy.Publisher( '/mavros/setpoint_raw/local',PositionTarget, queue_size=1)
        self.target_pub = rospy.Publisher('/UAV0/perception/object_location/obj_lla', PointStamped, queue_size=1)
        
        self.send_time = 0
        
        # 状态机使用的变量
        self.rel_alt = None
        self.sys_status = None
        self.state = None
        self.last_send = -1
        self.waypoint = []
        self.wp_idx = 0
        self.takeoff_lat = 0
        self.takeoff_lon = 0
        self.takeoff_alt = 0
        self.lift_alt = 0
        
        self.arm = False
        self.mode = "UNKNOWN"
        self.odom = None
        self.lat = 0
        self.lon = 0
        self.land = False
        self.speed = 0
        self.odom_lock = threading.Lock()
    
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
                LandNode()
            ],
            context=self,
            step_cb=self.step_cb
        )
        
    def do_ws_pub(self, data):
        self.ws_pub.publish(json.dumps(data))
    
    def step_cb(self, prev, cur):
        self.ws_pub.publish(json.dumps({"type": "state", "state": cur.value}))
    
    def do_pub_wp(self, waypoint, land):
        out = []
        for i, wp in enumerate(waypoint):
            if i == 0:
                command = "takeoff"
            elif i == len(waypoint) - 1 and land:
                command = "land"
            else:
                command = "wp"
            out.append({"num": i, "lat": wp[1], "lon": wp[0], "alt": wp[2], "command": command})
        
        self.do_ws_pub({"mission_data": out, "type": "state"})
        print(f'pub wp {json.dumps({"mission_data": out, "type": "state"})}')
        
    def do_send_cmd(self, *, px=None, py=None, pz=None, vx=None, vy=None, vz=None, ax=None, ay=None, az=None, yaw=None, yaw_rate=None):
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "local_ned"
        
        # 坐标系选择
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        type_mask = 0
        ctrl_data = []
        for data, ignore in zip([px, py, pz, vx, vy, vz, ax, ay, az, yaw, yaw_rate],["PX", "PY", "PZ", "VX", "VY", "VZ", "AFX", "AFY", "AFZ", "YAW", "YAW_RATE"]):
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
        # 发布
        self.setpoint_pub.publish(target)
    
    def do_pub_takeoff(self):
        self.do_ws_pub({
            "type": "event",
            "event": "takeoff"
        })
    
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
        self.ws_pub.publish(json.dumps({
            "type": "state",
            "follow x": msg.velocity.x,
            "follow y": msg.velocity.y, 
            "follow z": msg.velocity.z
        }))
        self.cmd_vel_cb(cmd_vel_msg)
    
    def do_land(self):
        self.set_mode_service(0, 'LAND')
    
    def do_takeoff(self, alt):
        self.set_mode_service(0, 'GUIDED')
        
        self.takeoff_lat = self.lat
        self.takeoff_lon = self.lon
        self.takeoff_alt = alt
        
        if self.rel_alt > HOVER_THRESHOLD and self.arm == True:
            self.do_send_cmd(pz=alt)
        else:    
            # 解锁
            out = self.prearm()["msg"]
            if out.get("arm", False) == False:
                raise ValueError(out["reason"])
            res = self.arm_service(True)
            if res.result == 1:
                raise ValueError("can not arm")
            rospy.loginfo("Vehicle armed")
            
            # 持续发布目标点
            rospy.loginfo("Taking off...")
            response = self.takeoff_srv(
                min_pitch=0,
                yaw=0,
                latitude=0,
                longitude=0,
                altitude=alt  # Target altitude in meters
            )
            rospy.loginfo("Takeoff command finished.")
        
    
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
            enu[2] - odom.pose.pose.position.z
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
        """ 获取enu坐标系下相对机体位置的偏移
        """
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
        """gps目标点转为move_base_simple/goal目标
        """
        
        diff_x, diff_y, diff_z = self.gps_target2enu_diff(gps)
        # ENU坐标
        odom = self.get_cur_odom()
        enu_x = diff_x + odom.pose.pose.position.x # 东向
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
        
    def get_cur_odom(self, t_cmd=None):
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
            PositionTarget.IGNORE_AFX |      # 忽略加速度x
            PositionTarget.IGNORE_AFY |      # 忽略加速度y
            PositionTarget.IGNORE_AFZ |      # 忽略加速度z
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_YAW_RATE   # 忽略偏航速率
        )
        
        # 设置值
        target.velocity = Vector3(vx, vy, vz) # 速度（机体坐标系，z取反，NED下为正）
        target.yaw = target_yaw
        # 发布
        self.setpoint_pub.publish(target)
    
    def check_alt(self, target, threshold):
        return math.fabs(self.rel_alt - target) < math.max(target, 0.5) * threshold
    
    @Node.ros("/mavros/global_position/rel_alt", Float64)
    def rel_alt_cb(self, data):
        self.rel_alt = data.data
        if self.odom is not None:
            self.runner.trigger(CEventType.ODOM_OK)
        if self.check_alt(self.takeoff_alt, TAKEOFF_THRESHOLD):
           self.runner.trigger(CEventType.TAKEOFF_DONE)  
        if self.check_alt(0, TAKEOFF_THRESHOLD):
            self.runner.trigger(CEventType.LAND_DONE)
        if self.check_alt(self.lift_alt, TAKEOFF_THRESHOLD):
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
        self.arm = data.armed
        if data.mode in ["RTL", "LAND"]:
            self.runner.trigger(CEventType.SET_LAND)
        if self.arm == False:
            self.runner.trigger(CEventType.LAND_DONE)
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
        self.ws_pub.publish(json.dumps({
            "type": "state",
            "waypoint": wp_list
        }))
        
    @Node.ros("/planning/pos_cmd", PositionCommand)
    def cmd_cb(self, msg):
        if self.runner.node.type != NodeType.WP:
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
            yaw=msg.yaw
        )
           
    @Node.route("/get_gpsv2", "GET")
    def get_gpsv2(self):
        return SUCCESS_RESPONSE({
            "pos": [self.lon, self.lat, self.rel_alt],
            "mode": self.mode,
            "arm": self.arm,
            "dis": {"current": 0, "min": 0, "max": 0},
            "gps_n": 10,
            "baro": -1,
            "gps": [self.gps_lon, self.gps_lat, self.gps_alt],
        })
       
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
        vz = 0.2 * np.clip(diff_z, -0.5, 0.5)
        self.do_send_cmd(vx=vx, vy=vy, vz=vz)
        return SUCCESS_RESPONSE()
    
    @Node.route("/stop_follow", "POST")
    def stop_follow(self):
        self.runner.trigger(CEventType.STOP_FOLLOW)
        return SUCCESS_RESPONSE()
    
    @Node.route("/set_waypoint", "POST")
    def set_waypoint(self, waypoint, speed=None, land=False, rtl=False):
        rospy.loginfo(f"receive wp: {json.dumps(waypoint)}")
        self.runner.trigger(CEventType.SET_WP, waypoint=waypoint, speed=speed, land=land, rtl=rtl)
        return SUCCESS_RESPONSE()
    
    @Node.route("/set_mode", "POST")
    def route_set_mode(self, mode):
        self.set_mode_service(0, mode)
        return SUCCESS_RESPONSE()

    @Node.route("/land", "POST")
    def route_land(self, waypoint=None, speed=None):
        if waypoint is None or len(waypoint) == 0:
            self.runner.trigger(CEventType.SET_LAND)
        else:
            self.runner.trigger(CEventType.SET_WP, waypoint=waypoint, speed=speed, land=True)
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
        return {
            "msg": [self.lon, self.lat, rel_alt],
            "status": "succecss"
        }
    
    @Node.route("/arm", "POST")
    def do_arm(self):
        res = self.arm_service(True)
        return SUCCESS_RESPONSE()
    
    @Node.route("/prearms", "GET")
    def prearm(self):
        # mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
        self.state = None
        response = self.cmd_service(
            command=401,  # MAV_CMD_RUN_PREARM_CHECKS
            confirmation=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        bits = 0x10000000
        if (self.sys_status.sensors_health & bits) == bits:
            return SUCCESS_RESPONSE({"arm": True})
        for i in range(1000):
            time.sleep(0.01)
            if self.state is not None and self.state.startswith("PreArm: "):
                return SUCCESS_RESPONSE({"arm": False, "reason": self.state})
        return SUCCESS_RESPONSE({"arm": False, "reason": "wait for reason timeout"})
    
if __name__ == '__main__':
    control_node = Control()
    control_node.run()
    
    
