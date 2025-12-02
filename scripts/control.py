#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
from mavros_msgs.msg import PositionTarget, SysStatus, StatusText
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Vector3, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String

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

DETECT_SPAN = 1
STOP_SPAN = 10  
TAKEOFF_THRESHOLD = 1
    
class Control(Node):
    def __init__(self):
        super().__init__()
        rospy.loginfo("wait for mavros service...")
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/takeoff')
        rospy.loginfo("done")
        self.takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.cmd_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        
        self.setpoint_pub = rospy.Publisher( '/mavros/setpoint_raw/local',PositionTarget, queue_size=1)
        
        self.rel_alt = 0
        self.sys_status = None
        self.state = None
        self.last_send = -1
        self.following = False # 跟随发布的命令
        self.arm = False
        self.odom = None
        self.lat = None
        self.lon = None
        self.land = None
        self.speed = None
        self.waypoint = []
        self.takeoff_lat = None
        self.takeoff_lon = None
        self.takeoff_alt = None
        self.odom_lock = threading.Lock()
        
        self.wp_pub = rospy.Publisher("/move_base_simple/goal2", PoseStamped, queue_size=1)
        self.ws_pub = rospy.Publisher("/mavproxy/ws", String, queue_size=1)
        
        self.wp_idx = 0
        self.send_wp = False
        self.taking_off = False
    
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

    def set_waypoint(self, waypoint, speed, land=False, rtl=False):
        self.waypoint = copy.deepcopy(waypoint)
        self.land = land or rtl
        self.speed = speed
        if rtl:
            if self.arm == False:
                self.set_home(waypoint[0][-1])
            if self.takeoff_lon is None or self.takeoff_lat is None:
                raise ValueError("No takeoff position find")
            self.waypoint.append([self.takeoff_lon, self.takeoff_lat, self.takeoff_alt])
        
        self.wp_idx = 1 # 忽略第一个点
        if self.arm == True:    
            self.wp_done_cb()
        else:
            self.send_wp = True
            alt = waypoint[0][-1]
            self.route_takeoff(alt)
        
        out = []
        for i, wp in enumerate(self.waypoint):
            command = "wp"
            if i == 0:
                command = "takeoff"
            if i == len(self.waypoint) - 1:
                if self.land:
                    command = "land"
            out.append({"num": i, "lat": wp[1], "lon": wp[0], "alt": wp[2], "command": command})
        self.ws_pub.publish(json.dumps({"mission_data": out, "type": "state"}))
        
    def set_home(self, alt):
        self.takeoff_lat = self.lat
        self.takeoff_lon = self.lon
        self.takeoff_alt = alt
        self.taking_off = True
    
    def publish_cur_wp(self):
        if self.wp_idx >= len(self.waypoint):
            return
        self.wp_pub.publish(self.gps_target2goal(self.waypoint[self.wp_idx]))
    
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
    
    @Node.ros("/mavros/global_position/raw/fix", NavSatFix)
    def gps_cb(self, data: NavSatFix):
        self.lat = data.latitude
        self.lon = data.longitude
        
    @Node.ros("/ego_planner/finish_event", Empty)
    def wp_done_cb(self, data=None):
        if self.waypoint is None or len(self.waypoint) == 0:
            return
        self.ws_pub.publish(json.dumps({
            "event": "progress",
            "cur": self.wp_idx - 1, # wp_idx是到达点的前一个点
            "total": len(self.waypoint) - 1,
        }))
        if self.wp_idx >= len(self.waypoint):
            if self.land:
                self.set_mode_service(0, "LAND")
            return
        self.publish_cur_wp()
        self.wp_idx += 1
        
    @Node.ros("/mavros/local_position/odom", Odometry)  
    def odom_cb(self, msg):
        with self.odom_lock:
            self.odom = msg
        
    @Node.ros("/cmd_vel2", Twist)
    def cmd_vel_cb2(self, cmd_vel_msg):
        if not self.following:
            return
        return self.cmd_vel_cb(cmd_vel_msg)
    
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
        
    @Node.ros("/mavros/global_position/rel_alt", Float64)
    def rel_alt_cb(self, data):
        self.rel_alt = data.data
        if self.taking_off and \
            math.fabs(self.rel_alt - self.takeoff_alt) < TAKEOFF_THRESHOLD:
            self.ws_pub.publish(json.dumps({
                "type": "event",
                "event": "takeoff"
            }))
            self.taking_off = False
            if self.send_wp:
                self.wp_done_cb()
                self.send_wp = False
            
    @Node.ros("/mavros/sys_status", SysStatus)
    def systatus_cb(self, data):
        self.sys_status = data
    
    @Node.ros("/mavros/statustext/recv", StatusText)
    def state_cb(self, data):
        self.state = data.text
      
    @Node.ros("/mavros/ws", String)
    def detect_cb(self, data):
        try:
            data = json.loads(data.data)
            if data.get("event", None) != "detect":
                return
            cur_time = time.time()
            if cur_time - self.last_send < DETECT_SPAN:
                return
            self.following = True
            self.last_send = cur_time
            self.ws_pub.publish(json.dumps(data))
        except json.JSONDecodeError:
            pass
    
    @Node.ros("/mavros/state", State)
    def mode_cb(self, data):
        self.arm = data.armed
        
    @Node.ros("/planning/pos_cmd", PositionCommand)
    def cmd_cb(self, msg):
        if self.following:
            return
        p_des_odom = np.array([msg.position.x, msg.position.y, msg.position.z])
        yaw_des = msg.yaw #- np.pi / 2

        # 构造MAVLink消息
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "local_ned"
        
        # 坐标系选择
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # 类型掩码：使用位置+速度
        target.type_mask = (
            PositionTarget.IGNORE_AFX |      # 忽略加速度x
            PositionTarget.IGNORE_AFY |      # 忽略加速度y
            PositionTarget.IGNORE_AFZ |      # 忽略加速度z
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_YAW_RATE   # 忽略偏航速率
        )
        
        # 设置值
        # target.position = Vector3(ned_p[0], ned_p[1], ned_p[2])
        # target.velocity = Vector3(ned_v[0], ned_v[1], ned_v[2])
        # target.acceleration_or_force = Vector3(ned_a[0], ned_a[1], ned_a[2])
        target.position = Vector3(p_des_odom[0], p_des_odom[1], p_des_odom[2])
        target.yaw = yaw_des
        target.yaw_rate = 0
        
        # 发布
        self.setpoint_pub.publish(target)
    
    @Node.route("/pos_vel", "POST")
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
        # 构造MAVLink消息
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "local_ned"
        
        # 坐标系选择
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # 类型掩码：使用位置+速度
        target.type_mask = (
            PositionTarget.IGNORE_AFX |      # 忽略加速度x
            PositionTarget.IGNORE_AFY |      # 忽略加速度y
            PositionTarget.IGNORE_AFZ |      # 忽略加速度z
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_YAW_RATE   # 忽略偏航速率
        )
        
        # 设置值
        # target.position = Vector3(ned_p[0], ned_p[1], ned_p[2])
        # target.velocity = Vector3(ned_v[0], ned_v[1], ned_v[2])
        # target.acceleration_or_force = Vector3(ned_a[0], ned_a[1], ned_a[2])
        target.velocity = Vector3(vx, vy, vz)
        # 发布
        self.setpoint_pub.publish(target)
        return SUCCESS_RESPONSE()
    
    @Node.route("/stop_follow", "POST")
    def stop_follow(self):
        self.last_send = time.time() + STOP_SPAN
        self.following = False
        self.publish_cur_wp()
        return SUCCESS_RESPONSE()
    
    @Node.route("/set_waypoint", "POST")
    def route_set_waypoint(self, waypoint, speed=None, land=False, rtl=False):
        rospy.loginfo(f"receive wp: {json.dumps(waypoint)}")
        self.set_waypoint(waypoint, speed, land=land, rtl=rtl)
        return SUCCESS_RESPONSE()
    
    @Node.route("/set_mode", "POST")
    def route_set_mode(self, mode):
        self.set_mode_service(0, mode)
        return SUCCESS_RESPONSE()

    @Node.route("/land", "POST")
    def route_land(self, waypoint=None, speed=None):
        if waypoint is None:
            self.set_mode_service(0, 'LAND')
        else:
            self.set_waypoint(waypoint, speed, land=True)
        return SUCCESS_RESPONSE()
        
    @Node.route("/return", "POST")
    def route_return(self, waypoint=None, speed=None):
        if waypoint is None:
            waypoint = []
        self.set_waypoint(waypoint, speed, rtl=True)
        return SUCCESS_RESPONSE()
    
    @Node.route("/takeoff", "POST")
    def route_takeoff(self, alt):
        self.set_mode_service(0, 'GUIDED')
        
        # 解锁
        out = self.prearm()["msg"]
        if out.get("arm", False) == False:
            return ERROR_RESPONSE(out["reason"])
        res = self.arm_service(True)
        if res.result == 1:
            return ERROR_RESPONSE("can not arm")
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
        self.set_home(alt)
        return SUCCESS_RESPONSE()
    
    @Node.route("/get_gps", "GET")
    def get_gps(self):
        return {
            "msg": [self.lon, self.lat, self.rel_alt],
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
    
    
