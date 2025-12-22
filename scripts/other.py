#!/usr/bin/python3
# -*- coding: utf-8 -*-
from base.node import SUCCESS_RESPONSE, Node
import rospy
from std_msgs.msg import String
from base.utils import ERROR_RESPONSE
from rsos_msgs.srv import StartBagRecord
from std_srvs.srv import Trigger
import json
from rsos_msgs.srv import SetGimbalAngle, SetGimbalAngleResponse

class Other(Node):
    def __init__(self):
        super().__init__()
        self.recording = False
        self.ws_pub = rospy.Publisher("ws", String, queue_size=-1)
        self.detect_param_name = "/UAV0/perception/yolo_detection/enable_detection"
        self.follow_param_name = "/UAV0/perception/object_location/object_location_node/enable_send"
        
    @Node.route("/start_record", "POST")
    def start_record(self, bag_name):
        service_name = "/data_recorder/start_recording"
        rospy.wait_for_service(service_name)
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
        rospy.wait_for_service(service_name)
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
        rospy.wait_for_service(service_name)
        srv = rospy.ServiceProxy(service_name, SetGimbalAngle)
        res = srv(mode=mode, angle=angle)
        if not res.success:
            return ERROR_RESPONSE(res.message)
        return SUCCESS_RESPONSE(res.message)
    
    @Node.route("/start_detect", "POST")
    def start_detect(self):
        rospy.set_param(self.detect_param_name, True)
        rospy.set_param(self.follow_param_name, True)
        return SUCCESS_RESPONSE()
    
    @Node.route("/stop_detect", "POST")
    def stop_detect(self):
        rospy.set_param(self.detect_param_name, False)
        rospy.set_param(self.follow_param_name, False)
        return SUCCESS_RESPONSE()
    
    @Node.route("/get_detect", "GET")
    def get_detect(self):
        param1 = rospy.get_param(self.detect_param_name)
        param2 = rospy.get_param(self.follow_param_name)
        return SUCCESS_RESPONSE(param1 and param2)
    
if __name__ == '__main__':
    node = Other()
    node.run()