#!/usr/bin/python3
# -*- coding: utf-8 -*-

from base.utils import FPSHelper
from base.node import Node
import numpy as np
from pupil_apriltags import Detector
import cv2
from mavros_msgs.msg import LandingTarget
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge


class Pland(Node):
    def __init__(self):
        super().__init__()
        self.tag_type = "tagCustom48h12"
        self.tag_id = 0
        self.camera_fov_x = 114
        self.camera_fov_y = 114
        self.detector = self.create_detector()
        self.landing_target_pub = rospy.Publisher(
            "/mavros/landing_target/raw", LandingTarget, queue_size=10
        )
        self.detect_res_pub = rospy.Publisher(
            "/pland_camera/result", Image, queue_size=10
        )
        self.bridge = CvBridge()
        self.fps_helper = FPSHelper(fps=1)

    def create_detector(self):
        return Detector(
            families=self.tag_type,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

    def detect_artag(self, detector, frame, return_frame=True):
        if detector is None:
            return None if not return_frame else (frame, None)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray)
        if len(tags) == 0:
            return None if not return_frame else (frame, None)
        tag = [t for t in tags if t.tag_id == self.tag_id][0]
        points = tag.corners.astype(int)  # 只用第一个二维码
        if not return_frame:
            return points

        for idx in range(len(tag.corners)):
            cv2.line(
                frame,
                tuple(tag.corners[idx - 1, :].astype(int)),
                tuple(tag.corners[idx, :].astype(int)),
                (0, 255, 0),
                5,
            )

        cv2.putText(
            frame,
            str(tag.tag_id),
            org=(
                tag.corners[0, 0].astype(int) + 10,
                tag.corners[0, 1].astype(int) + 10,
            ),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.8,
            color=(0, 0, 255),
            thickness=3,
        )
        return frame, points

    def artag2xy(self, detector, frame, fov_x=None, fov_y=None):
        # 创建二维码检测器
        frame, points = self.detect_artag(detector, frame, return_frame=True)
        res = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.detect_res_pub.publish(res)

        if points is None:
            return None

        qr_center = points.mean(axis=0)
        height, width = frame.shape[:2]
        img_center = np.array([width / 2, height / 2])
        delta = (qr_center - img_center) / img_center
        fov_xy = np.array([fov_x, fov_y])
        fov_xy = np.radians(fov_xy)
        angle = delta * (fov_xy / 2)
        return [float(angle[0]), float(angle[1])]  # 返回角度, 单位弧度

    @Node.ros("/pland_camera/image_raw", Image)
    def pland_cb(self, frame):
        frame = self.bridge.imgmsg_to_cv2(frame, desired_encoding="bgr8")
        xy = self.artag2xy(self.detector, frame, self.camera_fov_x, self.camera_fov_y)
        if xy is None:
            return
        self.set_landing_target(xy[0], xy[1])

    def set_landing_target(self, angle_x, angle_y):
        """
        设置着陆目标
        :param angle_x: 目标角度 X (弧度)
        :param angle_y: 目标角度 Y (弧度)
        """
        landing_target = LandingTarget()

        # 时间戳（微秒）
        landing_target.header.stamp = rospy.Time.now()
        landing_target.header.frame_id = "map"

        # 目标ID
        landing_target.target_num = 0
        landing_target.frame = 12  # MAV_FRAME_BODY_FRD
        landing_target.type = LandingTarget.VISION_FIDUCIAL

        # 目标角度（弧度）
        landing_target.angle = [angle_x, angle_y]

        # 距离（米）
        landing_target.distance = 0.0

        # 发布消息
        self.landing_target_pub.publish(landing_target)
        # rospy.loginfo(f"发送着陆目标: angle_x={angle_x}, angle_y={angle_y}")
        if self.fps_helper.step(block=False):
            rospy.loginfo(
                f"FPS:{self.fps_helper.fps} 发送着陆目标: angle_x={angle_x}, angle_y={angle_y}"
            )


if __name__ == "__main__":
    pland = Pland()
    pland.run()
