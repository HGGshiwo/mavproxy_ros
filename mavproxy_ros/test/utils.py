import math

import numpy as np


def distance(a, b):
    assert len(a) == len(b)
    ret = np.linalg.norm(np.array(a) - np.array(b))
    return ret.item()


def get_gps(lat, lon, bearing, distance_m=10):
    """
    0度=正北，90度=正东，180度=正南，270度=正西
    """
    R = 6371000.0  # 地球平均半径（米）

    # 1. 将角度转换为弧度
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    brng = math.radians(bearing)

    # 2. 计算目标纬度
    lat2 = math.asin(
        math.sin(lat1) * math.cos(distance_m / R)
        + math.cos(lat1) * math.sin(distance_m / R) * math.cos(brng)
    )

    # 3. 计算目标经度
    lon2 = lon1 + math.atan2(
        math.sin(brng) * math.sin(distance_m / R) * math.cos(lat1),
        math.cos(distance_m / R) - math.sin(lat1) * math.sin(lat2),
    )

    # 4. 将结果从弧度转换回角度
    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    return lat2, lon2


def gps_distance(lon1, lat1, lon2, lat2):
    R = 6371000  # 地球平均半径，单位为米

    # 将角度转换为弧度
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    # Haversine 公式
    a = (
        math.sin(delta_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c  # 返回米
