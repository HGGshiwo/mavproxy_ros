from dataclasses import field
from typing import Any, Dict, List, Optional, Tuple

from pydantic import BaseModel


class SetModeModel(BaseModel):
    mode: str


class TakeoffModel(BaseModel):
    alt: float


class SetWaypointModel(BaseModel):
    waypoint: List[Any] = field(default_factory=list)
    nodeEventList: List[Any] = None
    speed: float = None
    land: bool = False
    rtl: bool = False


class SetPosVelModel(BaseModel):
    pos: Tuple[float, float, float]  # 目标点, 到达速度=0
    vel: float  # 最大速度限制
    yaw: Optional[float] = None  # 到达后的期望偏航
    # True: 运动中始终固定为yaw指定的方向/False: 调整为目标点后再运动
    fix_yaw: Optional[bool] = True
    timeout: Optional[float] = 2  # 接口超时时间


# Other
class StartRecordModel(BaseModel):
    bag_name: str


class SetRosParamModel(BaseModel):
    name: str
    value: Any


class SetGimbalModel(BaseModel):
    mode: str
    angle: float


class SetExposureModel(BaseModel):
    shutter: float
    sensitivity: float


class StartDetectModel(BaseModel):
    type: str = "smoke"


# Param
class SetParamModel(BaseModel):
    param: Dict[str, Dict[str, float]]
