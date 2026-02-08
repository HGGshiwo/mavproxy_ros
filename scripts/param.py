#!/usr/bin/python3
# -*- coding: utf-8 -*-
from base.node import SUCCESS_RESPONSE, Node
from event_callback.core import CallbackManager
from mavros_msgs.srv import ParamPull, ParamGet
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

    @http_proxy.post("/set_param")
    def set_param(self, data: SetParamModel):
        rospy.set_param(data.name, data.value)
        return SUCCESS_RESPONSE("OK")

    @http_proxy.get("/params")
    def get_params(self):
        param_pull = rospy.ServiceProxy("/mavros/param/pull", ParamPull)
        self.param_num = 0
        resp = param_pull(force_pull=True)
        if not resp.success:
            return ERROR_RESPONSE("参数拉取失败")
        rospy.loginfo(resp.param_received)
        params = rospy.get_param(("/mavros/param"))
        data = {}
        for key, value in params.items():
            data[key] = {
                **self.pdef.get(key, {"help": [""], "default": None}),
                "value": value,
            }
        return SUCCESS_RESPONSE(data)


if __name__ == "__main__":
    node = Param()
    rospy.spin()
