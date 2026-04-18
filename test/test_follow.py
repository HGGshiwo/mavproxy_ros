#!/usr/bin/env python
# coding=utf-8
import math
import random
import time
import unittest

import rospy
import rostest
from rsos_msgs.msg import PointObj
from std_msgs.msg import String

from mavproxy_ros.test.test_helper import TestHelper, sitl_env
from mavproxy_ros.test.utils import gps_distance

MODEL_NAME = "iris_demo"
XY_THRESHOLD = 2.5  # 目标的距离阈值
Z_THRESHOLD = 1.0  # 高度的距离阈值

test_waypoint = [
    [120.14099425554586, 30.111834585498475, 10],
    [120.14089461794015, 30.111973425562567, 3],
    [120.1409098335351, 30.112108444153343, 40],
    [120.14101732435114, 30.112202702305442, 5],
    [120.14111941608508, 30.11227445732493, 10],
    [120.1409432096789, 30.112290166996786, 6],
    [120.14080283354473, 30.11221289237057, 10],
    [120.1408582968425, 30.112159394516993, 7],
]


class TestFollow(unittest.TestCase):
    def setUp(self):
        # 初始化节点（对于rostest，必须用匿名节点）
        rospy.init_node("auto_test_director", anonymous=True)
        self.pub = rospy.Publisher(
            "/UAV0/perception/object_location/location_vel", PointObj
        )
        self.helper = TestHelper()

    def tearDown(self):
        pass

    def test_case1(self):
        """
        1. 飞机从地面开始飞航线，
        2. 期间执行跟随任务，
        3. 停止跟随后继续执行航线，在一段时间中不再响应跟随
        4. 保护时间过后继续响应
        5. 飞一段航点，然后返航
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
            self.helper.takeoff(waypoint=test_waypoint)

            home_lat = self.helper.state["lat"]
            home_lon = self.helper.state["lon"]

            def test_follow():
                """发送跟随40秒，然后停止跟随"""
                lat = self.helper.state["lat"]
                lon = self.helper.state["lon"]
                init_z = self.helper.state["rel_alt"]

                msg = PointObj()
                msg.velocity.z = 1
                for i in range(400):
                    self.pub.publish(msg)
                    time.sleep(0.1)
                assert self.helper.state["state"] == "跟随模式", self.helper.state[
                    "state"
                ]
                lat_cur = self.helper.state["lat"]
                lon_cur = self.helper.state["lon"]
                xy_dist = gps_distance(lon, lat, lon_cur, lat_cur)
                z_dist = self.helper.state["rel_alt"] - init_z
                # assert xy_dist < XY_THRESHOLD, f"dist: {xy_dist}"
                assert z_dist > 15, f"z: {z_dist}"  # 40秒内至少上升了15m
                res = self.helper.http_post("/stop_follow")
                assert res.get("status", None) == "success", res
                assert self.helper.state["state"] != "跟随模式", self.helper.state

            stop_time = None
            for i, wp in enumerate(test_waypoint[1:]):
                for _ in range(10):
                    res = self.helper.ws_event_queue.get(timeout=500)
                    if res.get("event", None) == "progress":
                        break
                else:
                    raise RuntimeError("没有出现进度信息!")

                assert res.get("total", None) == len(test_waypoint) - 1, res
                assert res.get("cur", None) == i + 1, f"{res}, {i}"

                if i == 1:
                    test_follow()
                    stop_time = time.time()
                elif i == 2:
                    # 在100秒内不进入跟随
                    msg = PointObj()
                    msg.velocity.z = 1
                    for i in range(10):
                        self.pub.publish(msg)
                        time.sleep(0.1)
                    assert self.helper.state["state"] != "跟随模式", self.helper.state
                elif i == 3:
                    wait_time = max(0, 110 - time.time() + stop_time)
                    rospy.loginfo(f"等待{wait_time}秒，解除跟随保护")
                    time.sleep(wait_time)
                    # 110秒需要继续进入跟随
                    test_follow()

            self.helper.wait_for_state("state", "悬停状态", 120)
            self.helper.http_post("/return")
            self.helper.wait_for_state("state", "地面状态", 120)
            dist_to_home = gps_distance(
                home_lon, home_lat, self.helper.state["lon"], self.helper.state["lat"]
            )
            assert dist_to_home < 2, dist_to_home


if __name__ == "__main__":
    # 将 unittest 挂载到 rostest 框架上
    rostest.rosrun("mavproxy_ros", "test_follow", TestFollow)
