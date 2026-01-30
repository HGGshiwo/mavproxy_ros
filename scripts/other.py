#!/usr/bin/python3
# -*- coding: utf-8 -*-
from base.node import SUCCESS_RESPONSE, Node
from base.utils import post_json
import rospy
from std_msgs.msg import String
from base.utils import ERROR_RESPONSE
from rsos_msgs.srv import StartBagRecord
from std_srvs.srv import Trigger
import json
from rsos_msgs.srv import SetGimbalAngle, SetGimbalAngleResponse
from rsos_msgs.srv import SetCameraExposure
from mavros_msgs.msg import SysStatus

class Other(Node):
    def __init__(self):
        super().__init__()
        self.recording = False
        self.ws_pub = rospy.Publisher("ws", String, queue_size=-1)
        self.detect_param = {
            "nohardhat": "/UAV0/perception/yolo_detection/enable_detection",
            "smoke": "/UAV0/perception/yolo_detection_smoke/enable_detection"
        }
        self.follow_param_name = "/UAV0/perception/object_location/object_location_node/enable_send"
        
    @Node.ros("/mavros/sys_status", SysStatus)
    def sys_status_cb(self, msg: SysStatus):
        data = {}
        if msg.battery_remaining != -1:
            data["battery_remaining"] = msg.battery_remaining  # 百分比
        if msg.current_battery != -1:
            data["current_battery"] = msg.current_battery * 0.01 # cA * 0.01 = A 
        if msg.voltage_battery != 65536:
            data["voltage_battery"] = msg.voltage_battery / 1000
        self.ws_pub.publish(json.dumps({"type": "state", **data}))
        
           
    @Node.route("/start_record", "POST")
    def start_record(self, bag_name):
        service_name = "/data_recorder/start_recording"
        rospy.wait_for_service(service_name, timeout=5)
        srv = rospy.ServiceProxy(service_name, StartBagRecord)
        res = srv(prefix=bag_name)
        if not res.success:
            return ERROR_RESPONSE(res.message)
        self.ws_pub.publish(json.dumps({"type": "state", "record": True}))
        self.recording = True
        return SUCCESS_RESPONSE(res.message)
    
    @Node.route("/stop_record", "POST")
    def stop_record(self):
        service_name = "/data_recorder/stop_recording"
        rospy.wait_for_service(service_name, timeout=5)
        srv = rospy.ServiceProxy(service_name, Trigger)
        res = srv()
        self.ws_pub.publish(json.dumps({"type": "state", "record": False}))
        self.recording = False
        return SUCCESS_RESPONSE()

    @Node.route("/get_record", "GET")
    def get_record(self):
        return SUCCESS_RESPONSE(msg=self.recording)
    
    @Node.route("/set_ros_param", "POST")
    def set_ros_param(self, name, value):
        rospy.set_param(name, value)
        return SUCCESS_RESPONSE("OK")
    
    @Node.route("/get_ros_param{name:path}", "GET")
    def get_ros_param(self, name):
        try:
            return SUCCESS_RESPONSE(rospy.get_param(name))
        except KeyError as e:
            return ERROR_RESPONSE(str(e))
    
    @Node.route("/get_gimbal", "GET")
    def get_gimbal(self):
        try:
            mode = rospy.get_param("/UAV0/sensor/serial_gimbal/angle_mode")
            angle = rospy.get_param("/UAV0/sensor/serial_gimbal/gimbal_angle")
            return SUCCESS_RESPONSE({"mode": mode, "angle": angle})
        except Exception as e:
            return ERROR_RESPONSE(str(e))
    
    @Node.route("/set_gimbal", "POST")
    def set_gimbal(self, mode, angle):
        service_name = "/UAV0/sensor/serial_gimbal/set_gimbal_angle"
        rospy.wait_for_service(service_name, timeout=5)
        srv = rospy.ServiceProxy(service_name, SetGimbalAngle)
        res = srv(mode=mode, angle=angle)
        if not res.success:
            return ERROR_RESPONSE(res.message)
        return SUCCESS_RESPONSE(res.message)
    
    @Node.route("/get_exposure", "GET")
    def get_exposure(self):
        data = {
            "shutter": {
                "value": rospy.get_param("/UAV0/sensor/video11_camera/shutter", 50),
                "max": rospy.get_param("/UAV0/sensor/video11_camera/shutter_max", 100),
                "min": rospy.get_param("/UAV0/sensor/video11_camera/shutter_min", 0),
                "step": rospy.get_param("/UAV0/sensor/video11_camera/shutter_step", 1)
            },
            "sensitivity": {
                "value": rospy.get_param("/UAV0/sensor/video11_camera/ISO", 50),
                "max": rospy.get_param("/UAV0/sensor/video11_camera/ISO_max", 100),
                "min": rospy.get_param("/UAV0/sensor/video11_camera/ISO_min", 0),
                "step": rospy.get_param("/UAV0/sensor/video11_camera/ISO_step", 1)
            }
        }
        return SUCCESS_RESPONSE(data)
    
    @Node.route("/set_exposure", "POST")
    def set_exposure(self, shutter: float, sensitivity: float):
        service_name = " /UAV0/sensor/video11_camera/set_exposure"
        rospy.wait_for_service(service_name, timeout=5)
        srv = rospy.ServiceProxy(service_name, SetCameraExposure)
        res = srv(shutter=shutter, sensitivity=sensitivity)
        if not res.success:
            return ERROR_RESPONSE(res.message)
        return SUCCESS_RESPONSE(res.message)
    
    @Node.route("/start_detect", "POST")
    def start_detect(self, type="smoke"):
        for _, name in self.detect_param.items():
            rospy.set_param(name, False)
        if type not in self.detect_param:
            return ERROR_RESPONSE(f"{type} not in {', '.join(self.detect_param.keys())}")
        rospy.set_param(self.detect_param[type], True)
        self.ws_pub.publish(json.dumps({"type": "state", "detect": type}))
        rospy.set_param(self.follow_param_name, True)
        return SUCCESS_RESPONSE()
    
    @Node.route("/stop_detect", "POST")
    def stop_detect(self):
        for _, name in self.detect_param.items():
            rospy.set_param(name, False)
        rospy.set_param(self.follow_param_name, False)
        self.ws_pub.publish(json.dumps({"type": "state", "detect": "Not Start"}))
        post_json("start_planner", {"auto": True})
        return SUCCESS_RESPONSE()
    
    @Node.route("/get_detect", "GET")
    def get_detect(self):
        out = ""
        if rospy.get_param(self.follow_param_name, False):
            for _out, param_name in self.detect_param.items():
                if rospy.get_param(param_name, False):
                    out = _out
                    break     
        return SUCCESS_RESPONSE(out)
    
if __name__ == '__main__':
    node = Other()
    node.run()