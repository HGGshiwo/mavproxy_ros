import math

import numpy as np


def distance(a, b):
    assert len(a) == len(b)
    ret = np.linalg.norm(np.array(a) - np.array(b))
    return ret.item()


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
