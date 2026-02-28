from abc import ABC, abstractmethod
from typing import List, Optional

_controller_map = {}


class BaseController:

    def __init_subclass__(cls):
        name = cls.__name__.lower().replace("controller", "")
        _controller_map[name] = cls
        return super().__init_subclass__()

    def is_pland_enable(self):
        """该模式下是否允许精准降落"""
        return True

    @staticmethod
    def create(name: str, *args, **kwargs) -> "BaseController":
        if name not in _controller_map:
            raise ValueError(
                f"Unsupport controller name: {name}, must in {','.join(_controller_map.keys())}!"
            )

        return _controller_map[name](*args, **kwargs)

    def check_alt(
        self, rel_alt: float, min_alt_threshold: float, target: float, threshold: float
    ) -> bool:
        raise NotImplementedError()

    def check_hover(self, arm: bool, rel_alt: float) -> bool:
        """判断是否处于悬停状态"""
        raise NotImplementedError()

    def do_takeoff(self, alt: float):
        raise NotImplementedError()

    def do_land(self):
        raise NotImplementedError()

    def do_send_cmd(
        self,
        *,
        p: Optional[List[float]] = None,
        v: Optional[List[float]] = None,
        a: Optional[List[float]] = None,
        yaw: Optional[float] = None,
        yaw_rate: Optional[float] = None,
        frame: Optional[str] = "enu",
    ):
        raise NotImplementedError()
