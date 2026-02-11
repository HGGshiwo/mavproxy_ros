from typing import Any, List, Dict
from pydantic import BaseModel


class SetModeModel(BaseModel):
    mode: str


class TakeoffModel(BaseModel):
    alt: float


class SetWaypointModel(BaseModel):
    waypoint: List[Any]
    nodeEventList: List[Any] = None
    speed: float = None
    land: bool = False
    rtl: bool = False


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
