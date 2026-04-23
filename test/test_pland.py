#!/usr/bin/env python
# coding=utf-8
import random
import threading
import time
import unittest
from pathlib import Path
from typing import Callable, Dict, List

import numpy as np
import rospy
import rostest

from mavproxy_ros.test.test_helper import Apriltag, Robot, http_post, sitl_env

MODEL_NAME = "iris_demo"
TARGET_THRESHOLD = 1

from easyboard import SummaryWriter


def get_param(params_to_set: Dict[str, float]):
    """设置精准降落所需要的参数"""
    p = {name: dict(value=value) for name, value in params_to_set.items()}
    return dict(param=p)


IRIS_X = 0
IRIS_Y = 0


class TestPland(unittest.TestCase):
    def setUp(self):
        # 初始化节点（对于rostest，必须用匿名节点）
        rospy.init_node("auto_test_director", anonymous=True)
        self.robot = Robot()
        self.board = Apriltag("apriltag")
        self.board.spawn()

    def get_log_dir(self):
        return Path(__file__).parent.parent.joinpath(
            "test_log", time.strftime("%Y-%m-%d %H:%M:%S")
        )

    def tearDown(self):
        pass

    def start_record(
        self, record_cb: Callable, writer: SummaryWriter, stop_event: threading.Event
    ):
        def _worker():
            step = 0
            while not stop_event.is_set():
                time.sleep(0.5)
                value = record_cb()
                writer.add_scalar("dist_to_board", value, step)

                roll = self.robot.state["roll"]
                pitch = self.robot.state["pitch"]

                writer.add_scalar("roll", roll, step)
                writer.add_scalar("pitch", pitch, step)
                step += 1

        record_thread = threading.Thread(target=_worker, daemon=True)
        record_thread.start()
        return record_thread

    def _test_apm_pland(self, tags: List[str], before_test=None, step_test=None):
        TOTAL_ITER = 10
        for i in range(TOTAL_ITER):

            rospy.loginfo(f"test iter: {i+1}/{TOTAL_ITER}")
            with SummaryWriter(self.get_log_dir(), tags=[*tags, f"round{i}"]) as writer:
                stop_event = threading.Event()
                self.robot.set_state(x=IRIS_X, y=IRIS_Y, z=1.0)

                def get_dist():
                    xyz, v_xyz, rpy = self.robot.get_state()
                    bxyz, v_xyz, rpy = self.board.get_state()
                    x, y, z = xyz
                    board_x, board_y, _ = bxyz
                    x_diff = x - board_x
                    y_diff = y - board_y
                    dist = np.sqrt(x_diff * x_diff + y_diff * y_diff)
                    return dist

                with sitl_env():
                    self.robot.init()
                    self.board.set_state(
                        100.0,
                        100.0,
                        1.0,
                        # pitch=np.pi / 2,
                        # yaw=np.pi / 2,
                    )
                    http_post("/start_pland", dict(pland_type="apm"), check=True)
                    http_post("/set_param", get_param(dict(PLND_ENABLED=1)), check=True)

                    # 这里手动设置一下PLND_ENABLED参数，reboot之后prearm会超时，原因未知
                    # rospy.logerr("reboot_fcu!")
                    # http_post("/reboot_fcu", check=True)
                    http_post(
                        "/set_param",
                        get_param(
                            dict(
                                PLND_TYPE=1,
                                PLND_OPTIONS=1,
                                SIM_SONAR_SCALE=10,
                                RNGFND1_TYPE=1,
                                RNGFND1_SCALING=10,
                                RNGFND1_PIN=0,
                                RNGFND1_MAX=50,
                                RNGFND1_MIN=0,
                            )
                        ),
                        check=True,
                    )

                    self.robot.takeoff(alt=10)
                    self.robot.wait_for_state("state", "悬停状态")
                    if before_test is None:
                        before_test = lambda x: None
                    self.board.before_cb = before_test

                    if step_test is None:
                        step_test = lambda: 0, 0, 0
                    self.board.step_cb = step_test

                    with self.board.moving(writer):
                        start = time.time()
                        http_post("/land")

                        record_thread = self.start_record(get_dist, writer, stop_event)

                        self.robot.wait_for_state("state", "地面状态", 120)
                        stop_event.set()
                        if record_thread.is_alive():
                            record_thread.join()
                        end = time.time()
                        dist = get_dist()
                        success = 1 if dist < TARGET_THRESHOLD else 0
                        writer.add_summary("success", success)
                        writer.add_summary("time", end - start)
                        writer.add_summary("dist", dist)
                        stop_event.clear()

    @unittest.skip("1")
    def test_apm_pland_static(self):
        """静止的Tag"""

        def before_test(writer: SummaryWriter):
            # 3-9的一个同心圆中选择
            R = 3 + random.random() * 9
            theta = random.random() * 2 * np.pi
            writer.add_config(dict(theta=theta, R=R))
            board_x = IRIS_X + R * np.cos(theta)
            board_y = IRIS_Y + R * np.sin(theta)
            self.board.set_state(
                x=board_x,
                y=board_y,
                z=0.015,
                # pitch=np.pi / 2,
                # yaw=np.pi / 2,
            )

        self._test_apm_pland(["apm", "pland", "static"], before_test)

    def test_apm_pland_circle(self):
        """1m/s匀速圆周运动的Tag"""
        v = 1.0
        R = 3 + random.random() * 9

        def before_test(writer: SummaryWriter):
            theta = 0
            board_x = IRIS_X + R * np.cos(theta)
            board_y = IRIS_Y + R * np.sin(theta)

            writer.add_config(dict(v=v, R=R))

            self.board.set_state(
                x=board_x,
                y=board_y,
                z=0.015,
                yaw=theta,
                # pitch=np.pi / 2,
                # yaw=np.pi / 2,
            )

        def step_test():
            return 0, v, v / R

        self._test_apm_pland(
            ["apm", "pland", "circle"], before_test, step_test=step_test
        )


if __name__ == "__main__":
    # 将 unittest 挂载到 rostest 框架上
    rostest.rosrun("mavproxy_ros", "test_pland", TestPland)
