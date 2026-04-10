import copy
import datetime
from enum import Enum
from typing import Any, Dict


class MessageHandler:
    def on_connect(self):
        return {}

    def on_send(self, data: Dict[str, Any]):
        return data


def is_json_equal(obj1, obj2):
    """
    递归比较两个 JSON 风格对象（dict/list/基本类型）是否完全相等

    参数:
        obj1: 第一个待比较的对象
        obj2: 第二个待比较的对象

    返回:
        bool: 相等返回 True，否则返回 False
    """
    # 第一步：检查类型是否相同
    if type(obj1) != type(obj2):
        return False

    # 第二步：处理基本数据类型（int/float/str/bool/None）
    if isinstance(obj1, (int, float, str, bool)) or obj1 is None:
        # 特殊处理浮点数精度问题（可选，根据实际需求调整）
        if isinstance(obj1, float):
            return abs(obj1 - obj2) < 1e-9
        return obj1 == obj2

    # 第三步：处理列表（有序，需逐元素比较）
    if isinstance(obj1, list):
        # 长度不同直接不相等
        if len(obj1) != len(obj2):
            return False
        # 逐个元素递归比较
        for item1, item2 in zip(obj1, obj2):
            if not is_json_equal(item1, item2):
                return False
        return True

    # 第四步：处理字典（无序，先比较键，再比较值）
    if isinstance(obj1, dict):
        # 键的集合不同直接不相等
        if obj1.keys() != obj2.keys():
            return False
        # 逐个键对应的值递归比较
        for key in obj1:
            if not is_json_equal(obj1[key], obj2[key]):
                return False
        return True

    # 第五步：处理 JSON 不支持的类型（如 set/tuple/对象等）
    return False


class StateMessageHandler(MessageHandler):
    def __init__(self):
        super().__init__()
        self.state = {}

    def _patch(self, obj_new: Any):
        new_data = {}
        for k, v in obj_new.items():
            if is_json_equal(self.state.get(k, None), v):
                continue
            new_data[k] = v
        return new_data

    def on_send(self, data: Dict[str, Any]):
        if hasattr(data, "type"):
            del data["type"]
        patch_data = self._patch(data)
        if len(patch_data) == 0:
            return None
        self.state.update(**copy.deepcopy(patch_data))
        patch_data["type"] = "state"
        return patch_data

    def on_connect(self):
        return {"type": "state", **copy.deepcopy(self.state)}


class EventMessageHandler(MessageHandler):
    name = "event"

    def __init__(self):
        super().__init__()
        self.state = []

    def on_send(self, data: Dict[str, Any]):
        """为消息添加时间戳"""
        timestamp = int(1000 * datetime.datetime.now().timestamp())
        data["timestamp"] = timestamp
        data["type"] = self.name
        self.state.append(copy.deepcopy(data))
        return data

    def on_connect(self):
        return {self.name: copy.deepcopy(self.state), "type": "state"}

    @staticmethod
    def create(name: str):
        _name = name

        class NamedEventMessageHandler(EventMessageHandler):
            name = _name

        return NamedEventMessageHandler


class MessageType(Enum):
    STATE = StateMessageHandler
    ERROR = EventMessageHandler.create("error")
    WARN = EventMessageHandler.create("warn")
    INFO = EventMessageHandler.create("info")
    DEBUG = EventMessageHandler.create("debug")
    EVENT = EventMessageHandler.create("event")
