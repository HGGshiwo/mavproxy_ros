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

MODEL_NAME = "iris_demo"
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


class TestPosvel(unittest.TestCase):
    def setUp(self):
        # 初始化节点（对于rostest，必须用匿名节点）
        rospy.init_node("auto_test_director", anonymous=True)
        self.helper = TestHelper()

    def tearDown(self):
        pass

    def test_case1(self):
        """
        1. 测试fix_yaw = False，到达目标点
        2. 测试fix_yaw = True，移动过程中yaw必须和target接近
        3. 测试超时功能（发送posvel，等待2s后处于悬停）
        """
        pub = rospy.Publisher("/mavproxy/restart", String)

        IRIS_X = random.randint(-3, 3)
        IRIS_Y = random.randint(-3, 3)
        self.helper.set_state(MODEL_NAME, x=IRIS_X, y=IRIS_Y, z=0.2)
        with sitl_env():
            pub.publish("restart")
            self.helper.init()
            self.helper.http_post("/stop_pland")
            self.helper.http_post("/stop_planner")
            self.helper.takeoff()

            home_lat = self.helper.state["lat"]
            home_lon = self.helper.state["lon"]

            def send_pos_vel(fix_yaw):
                start_lat = self.helper.state["lat"]
                start_lon = self.helper.state["lon"]

                bearing = random.random() * 2 * np.pi
                dist = random.randint(8, 16)
                yaw = random.random() * 2 * np.pi

                target_lat, target_lon = get_gps(start_lat, start_lon, bearing, dist)

                yaw_list = []
                target_yaw = bearing if not fix_yaw else yaw
                for i in range(10000):
                    time.sleep(0.1)

                    res = self.helper.http_post(
                        "/set_posvel",
                        dict(
                            pos=[target_lon, target_lat, 10],
                            vel=2,
                            timeout=2,
                            fix_yaw=fix_yaw,
                            yaw=yaw,
                        ),
                    )
                    assert res.get("status") == "success", res
                    if fix_yaw:
                        yaw_list.append(self.helper.state["yaw"])

                    dist = gps_distance(
                        self.helper.state["lon"],
                        self.helper.state["lat"],
                        target_lon,
                        target_lat,
                    )
                    if dist < 1:
                        break
                else:
                    raise TimeoutError(f"set posvel to target timeout, cur: {dist}")
                if fix_yaw:
                    # 这里简化下只验证最后10个，因为之前可能在调整
                    delta_yaw = [delta_angle(y, target_yaw) for y in yaw_list[-10:]]
                    yaw_rmse = rmse(delta_yaw)
                    assert (
                        yaw_rmse < 0.5
                    ), f"123 {bearing} {yaw_rmse}, {target_yaw}, {yaw_list}, {delta_yaw}"

                self.helper.wait_for_state("state", "悬停状态", 1000)

                if not fix_yaw:
                    diff = math.fabs(self.helper.state["yaw"] - yaw)
                    assert diff < 0.5, f"{self.helper.state['yaw']}, {yaw}, {diff}"

            send_pos_vel(False)
            send_pos_vel(True)

            # 测试超时
            lat, lon = get_gps(
                self.helper.state["lat"], self.helper.state["lon"], 0, 1000
            )
            res = self.helper.http_post("/set_posvel", dict(pos=[lon, lat], timeout=2))
            assert res.get("status", None) == "success", res
            assert self.helper.state["state"] == "编队移动", self.helper.state
            time.sleep(2.2)
            assert self.helper.state["state"] == "悬停状态", self.helper.state

            self.helper.wait_for_state("state", "悬停状态", 120)
            self.helper.http_post("/return")
            self.helper.wait_for_state("state", "地面状态", 120)
            dist_to_home = gps_distance(
                home_lon, home_lat, self.helper.state["lon"], self.helper.state["lat"]
            )
            assert dist_to_home < 2, dist_to_home


if __name__ == "__main__":
    # 将 unittest 挂载到 rostest 框架上
    rostest.rosrun("mavproxy_ros", "test_posvel", TestPosvel)
