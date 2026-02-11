#!/usr/bin/python3
# -*- coding: utf-8 -*-
import re
from base.node import SUCCESS_RESPONSE, Node
from event_callback.core import CallbackManager
from mavros_msgs.srv import ParamPull, ParamSet
from mavros_msgs.msg import ParamValue, OverrideRCIn
from base.utils import ERROR_RESPONSE
import xml.etree.ElementTree as ET
from event_callback import http_proxy, ros
from control_model import *
from typing import Any
import rospy
import rospkg
import os


def parse_param(param):
    if param.integer != 0:
        return param.integer
    if param.real != 0:
        return param.real
    return 0


class Param(CallbackManager):
    def __init__(self):
        super().__init__()
        rospy.wait_for_service("/mavros/param/pull")
        rospy.wait_for_service("/mavros/param/get")
        rospy.wait_for_service("/mavros/param/set")

        # pdef_path = str(Path(__file__).parent.parent.joinpath("config", "apm.pdef.xml").resolve())
        # rospy.loginfo(f"load pdef from: {Path(__file__).parent.resolve()}, {Path(__file__).parent.parent.resolve()}")
        r = rospkg.RosPack()
        path = r.get_path("mavproxy_ros")
        pdef_path = os.path.join(path, "config", "apm.pdef.xml")
        self.pdef = self.load_pdef(pdef_path)
        self.param_set_service = rospy.ServiceProxy("/mavros/param/set", ParamSet)
        self._pull_param()

    def _pull_param(self):
        param_pull = rospy.ServiceProxy("/mavros/param/pull", ParamPull)
        self.param_num = 0
        resp = param_pull(force_pull=True)
        if not resp.success:
            return ERROR_RESPONSE("参数拉取失败")
        rospy.loginfo(resp.param_received)

    def load_pdef(self, path):
        # 假设xml文件名为 parameters.xml
        tree = ET.parse(path)
        root = tree.getroot()
        params = {}
        for param in root.findall(".//param"):
            name = param.get("name").split(":")[-1]  # 参数名
            help_text = [param.get("documentation")]  # 注释
            default = None
            # 查找field标签中的Default
            for field in param.findall("field"):
                help_text.append(f"{field.attrib.get('name')}: {field.text}")
            params[name] = {"default": default, "help": help_text}
        return params

    def _set_param(self, param_name: str, value):
        """
        设置单个飞控参数
        :param param_name: 参数名称
        :param value: 参数值
        :return: 是否成功
        """
        try:
            # 创建参数对象
            param = ParamValue()
            if isinstance(value, int):
                param.integer = int(value)
            else:
                param.real = float(value)

            # 调用服务
            response = self.param_set_service(param_id=param_name, value=param)

            if response.success:
                rospy.loginfo(f"参数设置成功: {param_name} = {value}")
                return True
            else:
                rospy.logerr(f"参数设置失败: {param_name}")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return False

    @ros.topic("/mavros/param/param_value", ParamValue)
    def param_callback(msg):
        """监听参数变化并写入 ROS param"""
        param_name = msg.param_id
        param_value = msg.value.integer if msg.value.integer != 0 else msg.value.real
        # 将参数写入 ROS param，路径为 /mavros/param/参数名
        rospy.set_param(f"/mavros/param/{param_name}", param_value)
        rospy.loginfo(f"同步飞控参数 {param_name} = {param_value} 到 ROS param")

    @http_proxy.post("/set_param")
    def set_param(self, data: SetParamModel):
        out = {}
        for name, value in data.param.items():
            res = self._set_param(name, value["value"])
            out[name] = "参数设置成功!" if res else "参数设置失败"
        return SUCCESS_RESPONSE("\n".join([f"{k}: {v}" for k, v in out.items()]))

    @http_proxy.get("/params")
    def get_params(self):
        params = rospy.get_param(("/mavros/param"))
        data = []
        for key, value in params.items():
            pdef_data = self.pdef.get(key, {"help": [""], "default": None})
            result = re.sub(r"\d", "X", key.split("_")[0])
            data.append(
                dict(
                    key=key,
                    name=key,
                    help=pdef_data["help"],
                    value=value,
                    group=[result, key],
                )
            )
        return SUCCESS_RESPONSE(dict(param=dict(_value=data)))


if __name__ == "__main__":
    node = Param()
    rospy.spin()
