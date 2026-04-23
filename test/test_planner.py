#!/usr/bin/env python
# coding=utf-8
import math
import random
import time
import unittest

import rospy
import rostest

from mavproxy_ros.test.test_helper import Robot, http_post, sitl_env
from mavproxy_ros.test.utils import gps_distance

MODEL_NAME = "iris_demo"
XY_THRESHOLD = 3  # 目标的距离阈值
Z_THRESHOLD = 1.2  # 高度的距离阈值

test_waypoint = [
    [120.14099425554586, 30.111834585498475, 10],
    [120.14089461794015, 30.111973425562567, 3],
    [120.1409098335351, 30.112108444153343, 15],
    [120.14101732435114, 30.112202702305442, 5],
    [120.14111941608508, 30.11227445732493, 10],
    [120.1409432096789, 30.112290166996786, 6],
    [120.14080283354473, 30.11221289237057, 10],
    [120.1408582968425, 30.112159394516993, 7],
]


class TestPlanner(unittest.TestCase):
    def setUp(self):
        # 初始化节点（对于rostest，必须用匿名节点）
        rospy.init_node("auto_test_director", anonymous=True)
        self.robot = Robot()

    def tearDown(self):
        pass

    def test_case1(self):
        """
        1. 飞机起飞，悬停状态，飞一段航点，
        2. 打开避障，再飞一段航点，返航
        """

        IRIS_X = random.randint(-3, 3)
        IRIS_Y = random.randint(-3, 3)
        time.sleep(1)
        self.robot.set_state(x=IRIS_X, y=IRIS_Y, z=1.0)
        with sitl_env():
            self.robot.init()
            self.robot.takeoff()
            http_post("/stop_pland")
            http_post("/stop_planner")

            http_post("/set_waypoint", dict(waypoint=test_waypoint), check=True)
            home_lat = self.robot.state["lat"]
            home_lon = self.robot.state["lon"]
            for i, wp in enumerate(test_waypoint[1:]):
                if i == 3:
                    res = http_post("/start_planner")
                    assert res.get("status", None) == "success", res
                for _ in range(10):
                    res = self.robot.ws_event_queue.get(timeout=500)
                    if res.get("event", None) == "progress":
                        break
                else:
                    raise RuntimeError("没有出现进度信息!")

                assert res.get("total", None) == len(test_waypoint) - 1, res
                assert res.get("cur", None) == i + 1, f"{res}, {i}"
                lat = self.robot.state["lat"]
                lon = self.robot.state["lon"]
                dist = gps_distance(lon, lat, wp[0], wp[1])
                dist_z = math.fabs(self.robot.state["rel_alt"] - wp[2])
                assert dist < XY_THRESHOLD, f"dist: {dist}, wp:[{wp[0]}, {wp[1]}]"
                assert dist_z < Z_THRESHOLD, f"z: {dist_z}"

            self.robot.wait_for_state("state", "悬停状态", 120)
            http_post("/return", check=True)
            self.robot.wait_for_state("state", "地面状态", 120)
            dist_to_home = gps_distance(
                home_lon, home_lat, self.robot.state["lon"], self.robot.state["lat"]
            )
            assert dist_to_home < 2, dist_to_home


if __name__ == "__main__":
    # 将 unittest 挂载到 rostest 框架上
    rostest.rosrun("mavproxy_ros", "test_planner", TestPlanner)
