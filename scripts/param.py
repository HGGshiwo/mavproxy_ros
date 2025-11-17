#!/usr/bin/python3
# -*- coding: utf-8 -*-
from base.node import SUCCESS_RESPONSE, Node
from mavros_msgs.srv import ParamPull, ParamGet
from mavros_msgs.msg import Param
from base.utils import ERROR_RESPONSE
import xml.etree.ElementTree as ET
from pathlib import Path
import json
import rospy
import time

def parse_param(param):
    if param.integer != 0:
        return param.integer
    if param.real != 0:
        return param.real
    return 0

class ParamNode(Node):
    def __init__(self):
        super().__init__()
        rospy.wait_for_service('/mavros/param/pull')
        rospy.wait_for_service('/mavros/param/get')
        rospy.wait_for_service('/mavros/param/set')
        self.get_srv = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        
        pdef_path = str(Path(__file__).parent.parent.joinpath("config", "apm.pdef.xml"))
        self.pdef = self.load_pdef(pdef_path)
        
    def load_pdef(self, path):  
        # 假设xml文件名为 parameters.xml
        tree = ET.parse(path)
        root = tree.getroot()
        params = {}
        for param in root.findall('.//param'):
            name = param.get('name').split(":")[-1]  # 参数名
            help_text = [param.get('documentation')]  # 注释
            default = None
            # 查找field标签中的Default
            for field in param.findall('field'):
                help_text.append(f"{field.attrib.get('name')}: {field.text}")
            params[name] = {
                'default': default,
                'help': help_text
            }
        return params
    
    @Node.route("/set_param", "POST")
    def set_param(self, data):
        super().set_param(data["name"], data["value"])
        return SUCCESS_RESPONSE("OK")
    
    @Node.route("/params", "GET")
    def get_params(self, _):
        
        param_pull = rospy.ServiceProxy('/mavros/param/pull', ParamPull)
        self.param_num = 0
        resp = param_pull(force_pull=True)
        if not resp.success:
            return ERROR_RESPONSE("参数拉取失败")
        rospy.loginfo(resp.param_received)
        params = rospy.get_param(('/mavros/param'))
        data = {}
        for key, value in params.items():
            data[key] = {
                **self.pdef.get(key, {"help": [""], "default": None}), 
                "value": value
            }    
        return SUCCESS_RESPONSE(data)
    
if __name__ == '__main__':
    node = ParamNode()
    node.run()