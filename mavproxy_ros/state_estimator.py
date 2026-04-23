import math
import threading
import time
from typing import Any

from pyproj import CRS, Transformer
from tf.transformations import euler_from_quaternion


class TimeoutValue:
    def __init__(self, timeout=1.0):
        self.value = None
        self.time = 0
        self.timeout = timeout

    def __get__(self, instance, owner):
        if time.time() - self.time > self.timeout:
            return None
        return self.value

    def __set__(self, instance, value):
        self.value = value
        self.time = time.time()


class StateEstimator:
    def __init__(self, timeout=1.0):
        self.enu_x = TimeoutValue(timeout)
        self.enu_y = TimeoutValue(timeout)
        self.enu_z = TimeoutValue(timeout)
        self.enu_yaw = TimeoutValue(timeout)
        self.ned_yaw = TimeoutValue(timeout)
        self.pitch = TimeoutValue(timeout)
        self.roll = TimeoutValue(timeout)
        self.lon = TimeoutValue(timeout)
        self.lat = TimeoutValue(timeout)
        self.lock = threading.RLock()
        self.odom_ok = False
        self.timeout = timeout

    def set_state(
        self,
        *,
        enu_x: float = None,
        enu_y: float = None,
        enu_z: float = None,
        quaternion: Any = None,
        lat: float = None,
        lon: float = None,
    ):
        with self.lock:
            if enu_x is not None:
                self.enu_x = enu_x
            if enu_y is not None:
                self.enu_y = enu_y
            if enu_z is not None:
                self.enu_z = enu_z
            if lat is not None:
                self.lat = lat
            if lon is not None:
                self.lon = lon

            if quaternion is not None:
                self.roll, self.pitch, self.enu_yaw = euler_from_quaternion(quaternion)
                self.ned_yaw = self.enu_yaw2ned_yaw(self.enu_yaw)

            for value in [self.enu_x, self.enu_y, self.enu_yaw, self.lat, self.lon]:
                if value is None:
                    break
            else:
                self.odom_ok = True

    def enu_yaw2ned_yaw(self, enu_yaw):
        """
        将四元数(ENU系)转换为罗盘航向角 (Compass Heading / NED Yaw)
        0° = 北，90° = 东，顺时针为正
        """
        # 转换为罗盘航向：90度(北) - ENU航向
        heading_rad = math.pi / 2 - enu_yaw
        # 规范化到 0 ~ 2pi 范围
        if heading_rad < 0:
            heading_rad += 2 * math.pi
        if heading_rad >= 2 * math.pi:
            heading_rad -= 2 * math.pi
        return heading_rad

    def gps2enu(self, gps):
        """获取enu坐标系下gps对应的坐标"""
        with self.lock:
            dx, dy, dz = self.gps2enu_body(gps)
            return [dx + self.enu_x, dy + self.enu_y, dz + self.enu_z]

    def gps2enu_body(self, gps):
        """获取机体为原点的enu坐标系下gps对应坐标"""
        with self.lock:
            lng = self.lon
            lat = self.lat
            # WGS84地理坐标系
            crs_wgs84 = CRS.from_epsg(4326)
            # 自动获取home点的UTM投影
            crs_utm = CRS.from_proj4(
                f"+proj=utm +zone={(int((lng + 180) / 6) + 1)} +datum=WGS84 +units=m +no_defs"
            )
            # 创建转换器
            transformer = Transformer.from_crs(crs_wgs84, crs_utm)
            # home点的UTM坐标
            home_x, home_y = transformer.transform(lat, lng)
            # print(f"home: {home_x}, {home_y}, {lat}, {lng}")
            # 目标点
            target_x, target_y = transformer.transform(gps[1], gps[0])
            return [target_x - home_x, target_y - home_y, gps[2] - self.enu_z]

    def enu_body2gps(self, enu_body):
        with self.lock:
            enu = [
                enu_body[0] + self.enu_x,
                enu_body[1] + self.enu_y,
                enu_body[2] + self.enu_z,
            ]
            return self.enu2gps(enu)

    def enu2gps(self, enu):
        """
        已知ENU坐标系下(odom)机体位置，求目标点GPS坐标
        """
        with self.lock:
            lng = self.lon
            lat = self.lat
            # WGS84地理坐标系
            crs_wgs84 = CRS.from_epsg(4326)
            # home点的UTM投影
            crs_utm = CRS.from_proj4(
                f"+proj=utm +zone={(int((lng + 180) / 6) + 1)} +datum=WGS84 +units=m +no_defs"
            )
            # 创建转换器
            transformer = Transformer.from_crs(crs_wgs84, crs_utm)
            # home点的UTM坐标
            home_x, home_y = transformer.transform(lat, lng)
            # enu坐标系下目标点相对飞机的偏移
            enu_diff = [
                enu[0] - self.enu_x,
                enu[1] - self.enu_y,
                enu[2] - self.enu_z,
            ]
            # 目标点的UTM坐标
            target_x = home_x + enu_diff[0]
            target_y = home_y + enu_diff[1]
            # 逆变换，UTM->WGS84
            transformer_inv = Transformer.from_crs(crs_utm, crs_wgs84)
            target_lat, target_lng = transformer_inv.transform(target_x, target_y)
            # 高度
            target_alt = self.enu_z + enu_diff[2]
            return [target_lng, target_lat, target_alt]
