#!/usr/bin/env python
# coding=utf-8
import math
import random
import time
import unittest

import numpy as np
import rospy
import rostest
from rsos_msgs.msg import PointObj
from std_msgs.msg import String

from mavproxy_ros.test.test_helper import TestHelper, sitl_env
from mavproxy_ros.test.utils import get_gps, gps_distance

XY_THRESHOLD = 2.5  # 目标的距离阈值
Z_THRESHOLD = 1.0  # 高度的距离阈值


def rmse(a):
    return np.sqrt(np.mean(np.array(a) ** 2))


def normalize_angle_2pi(angle):
    # 将任意角度标准化到 [0, 360) 之间 (对应 0 到 2pi)
    return (angle % (2 * np.pi) + 2 * np.pi) % (2 * np.pi)


def normalize_angle_pi(angle):
    # 先转到 0-360，如果大于 180，则减去 360
    a = normalize_angle_2pi(angle)
    if a >= np.pi:
        a -= 2 * np.pi
    return a


def delta_angle(current, target):
    diff = target - current
    # 用上面的 normalize_angle_180 函数把差值限制在 -180 到 180 之间
    return normalize_angle_pi(diff)


class TestDog(unittest.TestCase):
    """
    测试dog独有的接口
    """

    def setUp(self):
        # 初始化节点（对于rostest，必须用匿名节点）
        rospy.init_node("auto_test_director", anonymous=True)
        self.helper = TestHelper()

    def tearDown(self):
        pass

    def test_set_motion_state(self):
        pub = rospy.Publisher("/mavproxy/restart", String)

        IRIS_X = random.randint(-3, 3)
        IRIS_Y = random.randint(-3, 3)
        self.helper.set_robot_state(x=IRIS_X, y=IRIS_Y, z=0.2)

        with self.helper.sitl_env():
            pub.publish("restart")
            self.helper.init()
            self.helper.http_post("/stop_pland")
            self.helper.http_post("/stop_planner")
            self.helper.takeoff()

            for state in ["crawl", "walk", "run_low", "run_high"]:
                rospy.logerr(f"尝试切换速度状态到: {state}")
                res = self.helper.http_post("/set_motion_state", dict(state=state))
                assert res.get("status", None) == "success", res
                res = self.helper.http_get("/get_motion_state")
                assert res.get("msg", None) == state, res


if __name__ == "__main__":
    # 将 unittest 挂载到 rostest 框架上
    rostest.rosrun("mavproxy_ros", "test_dog", TestDog)
