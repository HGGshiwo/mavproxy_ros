#!/usr/bin/env python
# coding=utf-8
import math
import random
import unittest

import rospy
import rostest
from std_msgs.msg import String

from mavproxy_ros.test.test_helper import TestHelper, sitl_env
from mavproxy_ros.test.utils import gps_distance

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


class TestWaypoint(unittest.TestCase):
    def setUp(self):
        # 初始化节点（对于rostest，必须用匿名节点）
        rospy.init_node("auto_test_director", anonymous=True)
        self.helper = TestHelper()

    def tearDown(self):
        pass

    def test_case1(self):
        """
        1. 飞机起飞，悬停状态，飞一段航点，
        2. 打开避障，再飞一段航点，返航
        """
        pub = rospy.Publisher("/mavproxy/restart", String)

        IRIS_X = random.randint(-3, 3)
        IRIS_Y = random.randint(-3, 3)

        self.helper.set_robot_state(x=IRIS_X, y=IRIS_Y, z=0.2)
        with self.helper.sitl_env():
            pub.publish("restart")
            self.helper.init()
            self.helper.takeoff()
            self.helper.http_post("/stop_pland")
            self.helper.http_post("/stop_planner")

            res = self.helper.http_post("/set_waypoint", dict(waypoint=test_waypoint))
            assert res["status"] == "success", res
            home_lat = self.helper.state["lat"]
            home_lon = self.helper.state["lon"]
            for i, wp in enumerate(test_waypoint[1:]):
                for _ in range(10):
                    try:
                        res = self.helper.ws_event_queue.get(timeout=50)
                    except:
                        continue
                    if res.get("event", None) == "progress":
                        break
                else:
                    raise RuntimeError("没有出现进度信息!")

                assert res.get("total", None) == len(test_waypoint) - 1, res
                rospy.logerr(f"i: {i}")
                assert res.get("cur", None) == i + 1, f"{res}, {i}"
                lat = self.helper.state["lat"]
                lon = self.helper.state["lon"]
                dist = gps_distance(lon, lat, wp[0], wp[1])
                assert dist < XY_THRESHOLD, f"dist: {dist}, wp:[{wp[0]}, {wp[1]}]"
                if self.helper.robot_type == "drone":
                    dist_z = math.fabs(self.helper.state["rel_alt"] - wp[2])
                    assert dist_z < Z_THRESHOLD, f"z: {dist_z}"

            self.helper.wait_for_state("state", "悬停状态", 120)
            self.helper.http_post("/return")
            self.helper.wait_for_state("state", "地面状态", 120)
            dist_to_home = gps_distance(
                home_lon, home_lat, self.helper.state["lon"], self.helper.state["lat"]
            )
            assert dist_to_home < 2.5, dist_to_home


if __name__ == "__main__":
    # 将 unittest 挂载到 rostest 框架上
    rostest.rosrun("mavproxy_ros", "test_waypoint", TestWaypoint)
