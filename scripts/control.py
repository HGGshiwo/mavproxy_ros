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

class ROSWpControl:
    def __init__(self):
        self.odom = None
        self.lat = None
        self.lon = None
        self.land = None
        self.speed = None
        self.waypoint = None
        self.takeoff_lat = None
        self.takeoff_lon = None
        self.takeoff_alt = None
        self.odom_lock = threading.Lock()
        
        self.wp_pub = rospy.Publisher("/move_base_simple/goal2", PoseStamped, queue_size=1)
        self.setpoint_pub = rospy.Publisher( '/mavros/setpoint_raw/local',PositionTarget, queue_size=1)
        self.ws_pub = rospy.Publisher("/mavros/ws", String, queue_size=1)
        self.rel_alt = 0
        
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.gps_cb)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.rel_alt_cb)
        rospy.Subscriber("/ego_planner/finish_event", Empty, self.wp_done_cb)
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.cmd_cb)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        
    
    def gps_cb(self, data: NavSatFix):
        self.lat = data.latitude
        self.lon = data.longitude
        
    def gps2local(self, lat, lng, gps):
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
        print(f"home: {home_x}, {home_y}, {lat}, {lng}")
        # 目标点
        target_x, target_y = transformer.transform(gps[1], gps[0])
        print(f"target: {target_x}, {target_y}, {gps[1]}, {gps[0]}")
        
        # ENU坐标
        odom = self.get_cur_odom()
        enu_x = target_x - home_x + odom.pose.pose.position.x # 东向
        enu_y = target_y - home_y + odom.pose.pose.position.y  # 北向
        enu_z = gps[2] - self.rel_alt + odom.pose.pose.position.z  # 上向

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

    def set_waypoint(self, waypoint, speed, land):
        self.waypoint = copy.deepcopy(waypoint)
        self.land = land
        self.speed = speed
        if land:
            if self.takeoff_lon is None or self.takeoff_lat is None:
                print("No takeoff position find")
                return
            self.waypoint.append([self.takeoff_lon, self.takeoff_lat, self.takeoff_alt])
        self.wp_idx = 1 # 忽略第一个点
        self.wp_done_cb()
        
    def set_home(self, alt):
        self.takeoff_lat = self.lat
        self.takeoff_lon = self.lon
        self.takeoff_alt = alt
    
    def wp_done_cb(self, data=None):
        if self.waypoint is None:
            return
        self.ws_pub.publish(json.dumps({
            "event": "progress",
            "cur": self.wp_idx + 1,
            "total": len(self.waypoint),
        }))
        if self.wp_idx >= len(self.waypoint):
            if self.land:
                self.set_mode_service(0, "LAND")
            return
        self.wp_pub.publish(self.gps2local(self.lat, self.lon, self.waypoint[self.wp_idx]))
        self.wp_idx += 1

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
        
    def cmd_cb(self, msg):
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

    def odom_cb(self, msg):
        with self.odom_lock:
            self.odom = msg
    
    def rel_alt_cb(self, msg):
        self.rel_alt = msg.data
    
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
        
        self.ws_pub = rospy.Publihser("/mavproxy_ros/ws", String)
        
        self.wp_ctrl = ROSWpControl()
        self.rel_alt = 0
        self.sys_status = None
        self.state = None
        self.last_send = -1

    @Node.ros("/mavros/global_position/rel_alt", Float64)
    def rel_alt_cb(self, data):
        self.rel_alt = data.data
    
    @Node.ros("/mavros/sys_status", SysStatus)
    def systatus_cb(self, data):
        self.sys_status = data
    
    @Node.ros("/mavros/statustext/recv", StatusText)
    def state_cb(self, data):
        self.state = data.text
        
    @Node.ros("/mavros/ws", String)
    def detect_cb(self, data):
        try:
            data = json.loads(data)
            if data["event"] != "detect":
                return
            cur_time = time.time()
            if cur_time - self.last_send < 1:
                return
            self.last_send = cur_time
            self.ws_pub(json.dumps(data))
        except json.JSONDecodeError:
            pass
    
    @Node.ros("/cmd_vel2", Twist)
    def cmd_vel_cb2(self, data):
        if time.time() - self.last_send < 1:
            return
        self.wp_ctrl.cmd_vel_cb(data)
    
    @Node.route("/stop_follow", "POST")
    def stop_follow(self, data=None):
        self.last_send = time.time() + 10
        return SUCCESS_RESPONSE()
    
    @Node.route("/set_waypoint", "POST")
    def set_waypoint(self, data):
        waypoints = data["waypoint"]
        speed = data.get("speed", None)
        land = data.get("land", False)
        self.wp_ctrl.set_waypoint(waypoints, speed, land)
        return SUCCESS_RESPONSE()
    
    @Node.route("/set_mode", "POST")
    def set_mode(self, data):
        self.set_mode_service(0, data["mode"])
        return SUCCESS_RESPONSE()

    @Node.route("/land", "POST")
    def land(self, data=None):
        self.set_mode_service(0, 'LAND')
        return SUCCESS_RESPONSE()
        
    @Node.route("/return", "POST")
    def _return(self, data):
        waypoints = data["waypoints"]
        speed = data.get("speed", None)
        land = data.get("land", True)
        self.wp_ctrl.set_waypoint(waypoints, speed, land)
        return SUCCESS_RESPONSE()
    
    
    @Node.route("/takeoff", "POST")
    def takeoff(self, data):
        self.set_mode_service(0, 'GUIDED')
        
        # 解锁
        out = self.prearm()["msg"]
        if out["arm"] == False:
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
            altitude=data["alt"]  # Target altitude in meters
        )
        rospy.loginfo("Takeoff command finished.")
        self.wp_ctrl.set_home(data["alt"])
        return SUCCESS_RESPONSE()
    
    @Node.route("/get_gps", "GET")
    def get_gps(self, data=None):
        return {
            "msg": [self.wp_ctrl.lon, self.wp_ctrl.lat, self.rel_alt],
            "status": "succecss"
        }
    
    @Node.route("/arm", "POST")
    def do_arm(self, data=None):
        res = self.arm_service(True)
        return SUCCESS_RESPONSE()
    
    @Node.route("/prearms", "GET")
    def prearm(self, data=None):
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
    
    
