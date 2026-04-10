#!/usr/bin/python3
# -*- coding: utf-8 -*-
import logging
import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import rospkg
import rospy
from event_callback.components.http import HTTP_ProxyComponent
from event_callback.components.ros import ROSComponent
from event_callback.core import BaseManager
from event_callback.ros_utils import rospy_init_node
from event_callback.utils import setup_logger
from mavros_msgs.msg import Param, ParamValue
from mavros_msgs.srv import ParamPull, ParamSet

from mavproxy_ros.control_model import *
from mavproxy_ros.node import ERROR_RESPONSE, SUCCESS_RESPONSE

logger = logging.getLogger(__name__)
setup_logger(Path(__file__).parent.parent.joinpath("log").absolute())


def parse_param(param):
    if param.integer != 0:
        return param.integer

    if param.real != 0:
        return param.real

    return 0


class Param(BaseManager):

    def __init__(self):
        super().__init__(ROSComponent(), HTTP_ProxyComponent())
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

    def _post_init(self):
        param = rospy.get_param("param", {}).get("param", {})
        for n, v in param.items():
            self._set_param(n, v)

    def _pull_param(self):
        param_pull = rospy.ServiceProxy("/mavros/param/pull", ParamPull)
        self.param_num = 0
        resp = param_pull(force_pull=True)
        if not resp.success:
            return ERROR_RESPONSE("参数拉取失败")

        logger.info(f"param received: {resp.param_received}")

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
                logger.info(f"参数设置成功: {param_name} = {value}")
                return True

            else:
                logger.error(f"参数设置失败: {param_name}")
                return False

        except rospy.ServiceException as e:
            logger.error(f"服务调用失败: {e}")
            return False

    @ROSComponent.on_topic("/mavros/param/param_value", Param)
    def param_callback(self, msg: Param):
        """监听参数变化并写入 ROS param"""
        param_name = msg.param_id
        param_value = msg.value.integer if msg.value.integer != 0 else msg.value.real
        # 将参数写入 ROS param，路径为 /mavros/param/参数名
        rospy.set_param(f"/mavros/param/{param_name}", param_value)
        # logger.info(f"同步飞控参数 {param_name} = {param_value} 到 ROS param")

    @HTTP_ProxyComponent.on_post("/set_param")
    def set_param(self, data: SetParamModel):
        out = {}
        for name, value in data.param.items():
            res = self._set_param(name, value["value"])
            out[name] = "参数设置成功!" if res else "参数设置失败"
        return SUCCESS_RESPONSE("\n".join([f"{k}: {v}" for k, v in out.items()]))

    @HTTP_ProxyComponent.on_get("/params")
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
    rospy_init_node("other")
    node = Param()
    rospy.spin()
