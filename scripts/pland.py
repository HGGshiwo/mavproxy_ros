#!/usr/bin/python3
# -*- coding: utf-8 -*-

import threading
from typing import Any, Dict, List, Optional, Union
from base.utils import FPSHelper
from base.node import Node
import numpy as np
from pupil_apriltags import Detection, Detector
import cv2

from event_callback.utils import rosparam_field
from event_callback import ros
from event_callback.utils import ROSProxy, rostopic_field
from event_callback.core import CallbackManager, CallbackMixin
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped
import rospy
from cv_bridge import CvBridge
import math
import numpy.typing as npt
from sensor_msgs.msg import Range
import tf.transformations as tft
from geometry_msgs.msg import Quaternion


class Pland(CallbackManager, ROSProxy):
    def __init__(
        self,
        component_config: Optional[Dict[str, Any]] = None,
        mixins: List[CallbackMixin] = None,
    ):
        super().__init__(component_config, mixins)
        self.bridge = CvBridge()
        self.fps_helper = FPSHelper(fps=1)
        self.camera_fov_xy = None
        self.info_lock = threading.Lock()
        self.detect_lock = threading.Lock()

        self.tag_id = rosparam_field("tag_id", 0)
        self.tag_type = rosparam_field("tag_type", "tagCustom48h12")

        self.detector = self._create_detector()
        self.odom: Odometry = rostopic_field(
            "/mavros/local_position/odom", Odometry, timeout=0.1
        )
        self.camera_info = rostopic_field(
            "/UAV0/sensor/video11_camera/cam_info",
            CameraInfo,
            timeout=None,
            format=self._get_matrix,
        )

        # fmt: off
        self.landing_target_pub = rospy.Publisher("/mavproxy/landing_target", PoseStamped, queue_size=10)
        self.detect_res_pub = rospy.Publisher("/pland_camera/result", Image, queue_size=10)
        # fmt: on
        print("pland init done")

    def _create_detector(self):
        print("load detector")
        det = Detector(
            families=self.tag_type,
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )
        print("load done")
        return det

    def _get_control(self, frame: npt.NDArray, tag: Detection):
        """
        计算tag中心在图像坐标系的x, y轴方向偏移, x, y 的坐标是以图像中心为原点, x正, y负为坐标轴,
        yaw是相对于x,y轴正方向的旋转角度(弧度)(其实是相机的roll)
        """
        height, width = frame.shape[:2]
        img_center = np.array([width / 2, height / 2])
        qr_center = tag.center.astype(int)[np.newaxis, :]  # (1, 2)

        tag_corner = np.array([(-1, 1, 1), (1, 1, 1), (1, -1, 1), (-1, -1, 1)])
        tag_corner = np.matmul(tag_corner, tag.homography.T)  # p' = H * p
        z = tag_corner[:, 2:3]
        z = np.where(np.abs(z) < 1e-6, 1e-6, z)
        tag_corner = tag_corner[:, :2] / z
        points = np.concatenate([qr_center, tag_corner], axis=0)

        calib_points = self._get_calibrate_points(points)
        if calib_points is None:
            return None
        calib_qr_center = calib_points[0:1]
        calib_corner = calib_points[1:]

        # 左右,上下四条边的中点
        left = (calib_corner[0] + calib_corner[3]) / 2
        right = (calib_corner[1] + calib_corner[2]) / 2
        top = (calib_corner[0] + calib_corner[1]) / 2
        bottom = (calib_corner[2] + calib_corner[3]) / 2

        # 1. 计算向量：从 Bottom 指向 Top
        dx = top[0] - bottom[0]
        dy = top[1] - bottom[1]  # 注意：图像中向上意味着 dy 为负

        # 2. 计算相对于 X 轴正方向（向右）的角度
        raw_theta = np.arctan2(dy, dx)

        # 3. 转换为相对于 Y 轴负方向（向上）的角度
        # 加上 pi/2 (90度) 进行旋转补偿
        yaw = raw_theta + np.pi / 2

        # 可选：将角度规范化到 -pi 到 +pi 之间 (防止出现 > pi 的情况)
        if yaw > np.pi:
            yaw -= 2 * np.pi
        elif yaw < -np.pi:
            yaw += 2 * np.pi
        delta = (calib_qr_center - img_center) / (2 * img_center)
        return delta[0, 0], delta[0, 1], yaw

    def _get_matrix(self, msg: CameraInfo):
        """计算相机矩阵的逆"""
        K = np.array(msg.K).reshape((3, 3))
        K_inv = np.linalg.inv(K)
        return (K, K_inv)

    def _draw_result(self, frame: npt.NDArray, tag: Detection):
        frame = frame.copy()
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
        return frame

    def _detect(self, frame: npt.NDArray):
        """根据图片获取检测结果"""
        if self.detector is None:
            rospy.logerr_once("No detector!")
            return None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        with self.detect_lock:
            tags: List[Detection] = self.detector.detect(gray)
        if len(tags) == 0:
            return None
        tags = [t for t in tags if t.tag_id == self.tag_id]
        if len(tags) == 0:
            return None
        return tags[0]

    def _get_calibrate_points(
        self, pixel_points: npt.NDArray
    ) -> Union[npt.NDArray, None]:
        """
        计算像素点在相机无roll, pitch的情况下的像素坐标 (虚拟下视相机)
        """
        if self.odom is None:
            rospy.logerr("No odom")
            return None

        if self.camera_info is None:
            rospy.logerr("No camera_info")
            return None

        K, K_inv = self.camera_info

        # 1. 获取当前机体的四元数 (Body -> World/Odom)
        q_curr = self.odom.pose.pose.orientation
        q_curr_list = [q_curr.x, q_curr.y, q_curr.z, q_curr.w]

        # 2. 提取 Yaw，并构建一个只包含 Yaw 的“虚拟水平”四元数 (Virtual -> World/Odom)
        # 这样 Virtual Frame 的 Z 轴是垂直向下的(或者向上的，取决于Odom定义)，XY平面是水平的
        r, p, y = tft.euler_from_quaternion(q_curr_list)
        q_virt_list = tft.quaternion_from_euler(0, 0, y)

        # 3. 计算从 当前机体 到 虚拟水平机体 的相对旋转
        # 逻辑: V_world = R_curr * V_body  => V_body = R_curr_inv * V_world
        #       V_world = R_virt * V_virt  => V_virt = R_virt_inv * V_world
        # 代入: V_virt = R_virt_inv * (R_curr * V_body)
        # 相对旋转矩阵 R_relative = R_virt^T * R_curr

        # 使用 tf 库计算相对四元数: q_diff = q_virt_inv * q_curr
        q_virt_inv = tft.quaternion_inverse(q_virt_list)
        q_diff = tft.quaternion_multiply(q_virt_inv, q_curr_list)

        # 将相对四元数转换为旋转矩阵
        R_body_to_virt = tft.quaternion_matrix(q_diff)[:3, :3]

        # 4. 像素 -> 相机坐标系向量 (Camera Frame)
        N, _ = pixel_points.shape
        pixel_points_h = np.concatenate([pixel_points, np.ones((N, 1))], axis=1)
        v_cam = np.matmul(pixel_points_h, K_inv.T)  # Shape: (N, 3)

        # 5. 关键步骤：处理相机到机体的坐标系变换
        # 假设相机是标准下视安装：
        # 相机坐标系: X向右, Y向下, Z向前(指向地面)
        # 机体坐标系(ROS标准): X向前, Y向左, Z向上
        # 或者 机体坐标系(PX4/NED): X向前, Y向右, Z向下
        #
        # 这里我们需要定义 R_cam_to_body。
        # 最简单的方法是试错，或者根据实际安装。
        # 下面是一个常见的下视相机变换 (仅作参考，如果校正方向反了需要修改这里):
        # v_body.x (前) = -v_cam.y (图像上部是前)
        # v_body.y (左) = -v_cam.x (图像右部是右，即负左)
        # v_body.z (上) = -v_cam.z (图像深度向下，即负上)

        # 定义从相机坐标系到机体坐标系的旋转矩阵
        # 这是一个示例矩阵，对应：图像Top是机头方向，图像Right是机身右侧
        R_cam_to_body = np.array(
            [
                [0, -1, 0],  # Body X = -Cam Y
                [-1, 0, 0],  # Body Y = -Cam X
                [0, 0, -1],  # Body Z = -Cam Z
            ]
        )

        # 将向量转到机体坐标系
        v_body = np.matmul(v_cam, R_cam_to_body.T)

        # 6. 应用去倾斜旋转 (Body -> Virtual Level Body)
        # 注意: v_virt = R * v_body. 对于行向量: v_virt = v_body @ R.T
        v_virt_body = np.matmul(v_body, R_body_to_virt.T)

        # 7. 转回相机坐标系 (为了投影回像素)
        # v_virt_cam = R_body_to_cam * v_virt_body
        # R_body_to_cam = R_cam_to_body.T
        v_virt_cam = np.matmul(v_virt_body, R_cam_to_body)  # R.T.T = R

        # 8. 投影回像素平面
        # 归一化深度 (Z=1)
        z_coords = v_virt_cam[:, 2:3]
        z_coords[np.abs(z_coords) < 1e-6] = 1e-6
        v_virt_cam_norm = v_virt_cam / z_coords

        # 应用内参
        out = np.matmul(v_virt_cam_norm, K.T)

        return out[:, :2]

    def _set_landing_target(self, delta_x: float, delta_y: float, yaw: float):
        """
        设置着陆目标

        :param delta_x: 相机坐标系 X
        :param delta_x: 相机坐标系 Y
        :param yaw: 相机坐标系yaw
        """
        landing_target = PoseStamped()

        # 时间戳（微秒）
        landing_target.header.stamp = rospy.Time.now()
        landing_target.header.frame_id = "map"

        # quaternion_from_euler(roll, pitch, yaw) → 返回 (x, y, z, w) 格式的四元数
        quat = tft.quaternion_from_euler(0, 0, yaw)

        landing_target.pose.position.x = delta_x
        landing_target.pose.position.y = delta_y
        landing_target.pose.position.z = 0
        landing_target.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        # 发布消息
        self.landing_target_pub.publish(landing_target)
        if self.fps_helper.step(block=False):
            rospy.loginfo(f"FPS:{self.fps_helper.fps} 发送着陆目标: x={delta_x}, y={delta_y}")

    def _pland_cb(self, frame: npt.NDArray):
        detect_result = self._detect(frame)
        if detect_result is not None:
            control = self._get_control(frame, detect_result)
            if control is not None:
                x, y, yaw = control
                self._set_landing_target(x, y, yaw)
                frame = self._draw_result(frame, detect_result)
        res = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.detect_res_pub.publish(res)

    # topic 在launch中修改
    @ros.topic("/pland_camera/image_raw", Image)
    def pland_cb(self, frame: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(frame, desired_encoding="bgr8")
            return self._pland_cb(frame)
        except Exception:
            import traceback

            traceback.print_exc()

    @ros.topic("/pland_camera/compressed", CompressedImage)
    def pland_cb2(self, frame: CompressedImage):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(frame, desired_encoding="bgr8")
            return self._pland_cb(frame)
        except Exception:
            import traceback

            traceback.print_exc()

    # @ros.topic("/UAV0/sensor/video11_camera/cam_info", CameraInfo)
    # def camera_info_cb(self, camera_info_msg: CameraInfo):
    #     """
    #     从camera_info消息计算水平和垂直FOV（单位：度）

    #     参数:
    #         camera_info_msg: sensor_msgs/CameraInfo消息

    #     返回:
    #         fov_x, fov_y (单位：度)
    #     """
    #     try:
    #         # 获取图像尺寸
    #         width = camera_info_msg.width
    #         height = camera_info_msg.height

    #         # 获取内参矩阵K（3x3行优先）
    #         # K = [fx  0  cx]
    #         #     [0  fy  cy]
    #         #     [0   0   1]
    #         fx = camera_info_msg.K[0]  # 水平焦距（像素）
    #         fy = camera_info_msg.K[4]  # 垂直焦距（像素）

    #         # 计算FOV（弧度）
    #         fov_x_rad = 2 * math.atan(width / (2 * fx))
    #         fov_y_rad = 2 * math.atan(height / (2 * fy))

    #         # 转换为度
    #         fov_x_deg = math.degrees(fov_x_rad)
    #         fov_y_deg = math.degrees(fov_y_rad)
    #         with self.info_lock:
    #             self.camera_fov_xy = [fov_x_deg, fov_y_deg]
    #     except Exception:
    #         import traceback

    #         traceback.print_exc()


if __name__ == "__main__":
    pland = Pland()
    rospy.spin()
