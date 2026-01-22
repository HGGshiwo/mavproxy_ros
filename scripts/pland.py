#!/usr/bin/python3
# -*- coding: utf-8 -*-

import threading
from base.utils import FPSHelper
from base.node import Node
import numpy as np
from pupil_apriltags import Detector
import cv2
from mavros_msgs.msg import LandingTarget
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import rospy
from cv_bridge import CvBridge
import math


class Pland(Node):
    def __init__(self):
        self.init_done = False
        super().__init__()
        self.detector = self.create_detector()
        self.landing_target_pub = rospy.Publisher(
            "/mavros/landing_target/raw", LandingTarget, queue_size=10
        )
        self.detect_res_pub = rospy.Publisher(
            "/pland_camera/result", Image, queue_size=10
        )
        self.bridge = CvBridge()
        self.fps_helper = FPSHelper(fps=1)
        self.camera_fov_xy = None
        self.info_lock = threading.Lock()
        self.detect_lock = threading.Lock()
        self.init_done = True
        print("pland init done")
        
    def create_detector(self):
        tag_type = self._get_param("tag_type", "tagCustom48h12")
        print("load detector")
        det = Detector(
            families=tag_type,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )
        print("load done")
        return det

    def detect_artag(self, detector, frame, return_frame=True):
        tag_id = self._get_param("tag_id", 0)
        if detector is None:
            return None if not return_frame else (frame, None)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        with self.detect_lock:
            tags = detector.detect(gray)
        
        # return frame, None
        if len(tags) == 0:
            return None if not return_frame else (frame, None)
        tag = [t for t in tags if t.tag_id == tag_id][0]
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

    # topic 在launch中修改
    def _pland_cb(self, frame: np.ndarray):
        if not self.init_done:
            return
        with self.info_lock:
            if self.camera_fov_xy is None:
                rospy.logwarn("No camera info received!, skip")
                # camera_fov_x = self._get_param("camera_fov_x", 114)
                # camera_fov_y = self._get_param("camera_fov_y", 114)
                return
            else:
                camera_fov_x = self.camera_fov_xy[0]
                camera_fov_y = self.camera_fov_xy[1]
        xy = self.artag2xy(self.detector, frame, camera_fov_x, camera_fov_y)
        if xy is None:
            return
        self.set_landing_target(xy[0], xy[1])

    @Node.ros("/pland_camera/image_raw", Image)
    def pland_cb(self, frame: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(frame, desired_encoding="bgr8")
            return self._pland_cb(frame)
        except Exception:
            import traceback
            traceback.print_exc()

    @Node.ros("/pland_camera/compressed", CompressedImage)
    def pland_cb2(self, frame: CompressedImage):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(frame, desired_encoding="bgr8")
            return self._pland_cb(frame)
        except Exception:
            import traceback
            traceback.print_exc()

    @Node.ros("/UAV0/sensor/video11_camera/cam_info", CameraInfo)
    def camera_info_cb(self, camera_info_msg: CameraInfo):
        """
        从camera_info消息计算水平和垂直FOV（单位：度）

        参数:
            camera_info_msg: sensor_msgs/CameraInfo消息

        返回:
            fov_x, fov_y (单位：度)
        """
        try:
            # 获取图像尺寸
            width = camera_info_msg.width
            height = camera_info_msg.height

            # 获取内参矩阵K（3x3行优先）
            # K = [fx  0  cx]
            #     [0  fy  cy]
            #     [0   0   1]
            fx = camera_info_msg.K[0]  # 水平焦距（像素）
            fy = camera_info_msg.K[4]  # 垂直焦距（像素）

            # 计算FOV（弧度）
            fov_x_rad = 2 * math.atan(width / (2 * fx))
            fov_y_rad = 2 * math.atan(height / (2 * fy))

            # 转换为度
            fov_x_deg = math.degrees(fov_x_rad)
            fov_y_deg = math.degrees(fov_y_rad)
            with self.info_lock:
                self.camera_fov_xy = [fov_x_deg, fov_y_deg]
        except Exception:
            import traceback
            traceback.print_exc()

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
                f"FPS:{self.fps_helper.fps} 发送着陆目标: angle_x={angle_x}, angle_y={angle_y} fov: {self.camera_fov_xy}"
            )


if __name__ == "__main__":
    pland = Pland()
    pland.run()
