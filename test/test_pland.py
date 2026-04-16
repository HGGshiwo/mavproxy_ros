#!/usr/bin/env python
# coding=utf-8
import os
import random
import subprocess
import time
import unittest

import numpy as np
import rospy
import rostest
from std_msgs.msg import String

from mavproxy_ros.test.test_helper import TestHelper, sitl_env

MODEL_NAME = "iris_demo"
TARGET_THRESHOLD = 1


class TestPland(unittest.TestCase):
    def setUp(self):
        # 初始化节点（对于rostest，必须用匿名节点）
        rospy.init_node("auto_test_director", anonymous=True)
        self.helper = TestHelper()

    def tearDown(self):
        pass

    def test_pland(self):
        time_record = []
        dist_record = []
        succ_record = []
        pub = rospy.Publisher("/mavproxy/restart", String)
        TOTAL_ITER = 10
        for i in range(TOTAL_ITER):
            rospy.loginfo(f"test iter: {i+1}/{TOTAL_ITER}")
            BOARD_X = random.randint(-3, 3)
            BOARD_Y = random.randint(-3, 3)
            IRIS_X = random.randint(-3, 3)
            IRIS_Y = random.randint(-3, 3)
            self.helper.set_state(MODEL_NAME, x=IRIS_X, y=IRIS_Y, z=0.2)
            with sitl_env():
                pub.publish("restart")
                self.helper.init_and_takeoff()
                self.helper.set_state(
                    "apriltag", x=BOARD_X, y=BOARD_Y, pitch=np.pi / 2, yaw=np.pi / 2
                )

                self.helper.http_post("/stop_pland")
                start = time.time()
                self.helper.http_post("/land")
                self.helper.wait_for_state("state", "地面状态", 120)
                end = time.time()
                xyz, v_xyz, rpy = self.helper.get_state(MODEL_NAME)
                x, y, z = xyz
                x_diff = x - BOARD_X
                y_diff = y - BOARD_Y
                dist = np.sqrt(x_diff * x_diff + y_diff * y_diff)
                success = 1 if dist < TARGET_THRESHOLD else 0
                succ_record.append(success)

                if success:
                    time_record.append(end - start)
                    dist_record.append(dist)

        time_mean = np.mean(time_record)
        time_std = np.std(time_record)
        dist_mean = np.mean(dist_record)
        dist_std = np.std(dist_record)
        self.helper.print_report(
            dict(
                时间平均=time_mean,
                时间方差=time_std,
                距离平均=dist_mean,
                距离方差=dist_std,
            )
        )


if __name__ == "__main__":
    # 将 unittest 挂载到 rostest 框架上
    rostest.rosrun("mavproxy_ros", "test_pland", TestPland)
