import math
from logging import getLogger
from typing import List, Optional, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandTOL, SetMode

from .base_controller import BaseController

HOVER_THRESHOLD = 1  # 判断为悬停状态(绝对高度)
logger = getLogger(__name__)


class DroneController(BaseController):
    def __init__(self):
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=1
        )
        self.set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)

    def check_hover(self, arm: bool, rel_alt: float):
        """判断是否处于悬停状态"""
        return arm == True and rel_alt >= HOVER_THRESHOLD

    def check_alt(
        self, rel_alt: float, min_alt_threshold: float, target: float, threshold: float
    ):
        return math.fabs(rel_alt - target) < max(target * threshold, min_alt_threshold)

    def do_takeoff(self, alt: float):
        response = self.takeoff_srv(
            min_pitch=0,
            yaw=0,
            latitude=0,
            longitude=0,
            altitude=alt,  # Target altitude in meters
        )

    def do_land(self):
        self.set_mode_service(0, "LAND")

    def loiter(self):
        self.set_mode_service(0, "LOITER")

    def check_arrive(
        self,
        cur_pos: Tuple[float, float, float],
        goal: Tuple[float, float, float],
    ):
        dis = np.sqrt(
            (cur_pos[0] - goal[0]) ** 2
            + (cur_pos[1] - goal[1]) ** 2
            + (cur_pos[2] - goal[2]) ** 2
        )
        return dis

    def do_send_cmd(
        self,
        *,
        p: Optional[List[float]] = None,
        v: Optional[List[float]] = None,
        a: Optional[List[float]] = None,
        yaw: Optional[float] = None,
        yaw_rate: Optional[float] = None,
        frame: Optional[str] = "enu",
    ):
        """
        机体坐标系下: x为前, y为左, z为下
        ENU坐标系下: x为E, y为N, z为U
        """
        if frame == "body":
            frame = PositionTarget.FRAME_BODY_OFFSET_NED
        elif frame == "enu":
            frame = PositionTarget.FRAME_LOCAL_NED
        else:
            raise ValueError(f"Not support frame name {frame}!")

        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "local_ned"
        # 坐标系选择
        target.coordinate_frame = frame
        type_mask = 0
        ctrl_data = []
        if p is None:
            for key in ["PX", "PY", "PZ"]:
                type_mask = type_mask | getattr(PositionTarget, f"IGNORE_{key}")
                p = [0, 0, 0]
        if v is None:
            for key in ["VX", "VY", "VZ"]:
                type_mask = type_mask | getattr(PositionTarget, f"IGNORE_{key}")
                v = [0, 0, 0]
        if a is None:
            for key in ["AFX", "AFY", "AFZ"]:
                type_mask = type_mask | getattr(PositionTarget, f"IGNORE_{key}")
                a = [0, 0, 0]
        if yaw is None:
            type_mask = type_mask | PositionTarget.IGNORE_YAW
            yaw = 0
        if yaw_rate is None:
            type_mask = type_mask | PositionTarget.IGNORE_YAW_RATE
            yaw_rate = 0
        # 设置值
        target.position = Vector3(*p)
        target.velocity = Vector3(*v)
        target.acceleration_or_force = Vector3(*a)
        target.yaw = yaw
        target.yaw_rate = yaw_rate
        target.type_mask = type_mask
        # 发布
        self.setpoint_pub.publish(target)
