#!/usr/bin/python3
# -*- coding: utf-8 -*-
from base.node import SUCCESS_RESPONSE, Node
from mavros_msgs.srv import ParamPull, ParamGet
from base.utils import ERROR_RESPONSE
import xml.etree.ElementTree as ET
from pathlib import Path
import rospy
from rsos_msgs.srv import StartBagRecord
from std_msgs.srv import Trigger

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
    
    @Node.route("/start_record", "POST")
    def start_record(self, data):
        service_name = "/data_recorder/start_recording"
        rospy.wait_for_service(service_name)
        srv = rospy.ServiceProxy(service_name, StartBagRecord)
        res = srv(prefix=data["bag_name"])
        if not res.success:
            return ERROR_RESPONSE(res.message)
        return SUCCESS_RESPONSE()
    
    @Node.route("/stop_record", "POST")
    def stop_record(self, data=None):
        service_name = "/data_recorder/stop_recording"
        rospy.wait_for_service(service_name)
        srv = rospy.ServiceProxy(service_name, Trigger)
        res = srv()
        return SUCCESS_RESPONSE()
    
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