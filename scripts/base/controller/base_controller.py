from abc import ABC, abstractmethod
from typing import List, Optional


class BaseController(ABC):
    @abstractmethod()
    def check_alt(
        self, rel_alt: float, min_alt_threshold: float, target: float, threshold: float
    ) -> bool:
        raise NotImplementedError()

    @abstractmethod()
    def check_hover(self, arm: bool, rel_alt: float) -> bool:
        """判断是否处于悬停状态"""
        raise NotImplementedError()

    @abstractmethod()
    def do_takeoff(self, alt: float):
        raise NotImplementedError()

    @abstractmethod()
    def do_land(self):
        raise NotImplementedError()

    @abstractmethod()
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
