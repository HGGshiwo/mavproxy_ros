#!/usr/bin/python3
# -*- coding: utf-8 -*-
from base.node import SUCCESS_RESPONSE, Node
import rospy
from std_msgs.msg import String
from base.utils import ERROR_RESPONSE
from rsos_msgs.srv import StartBagRecord
from std_srvs.srv import Trigger
import json

class Other(Node):
    def __init__(self):
        super().__init__()
        self.ws_pub = rospy.Publisher("ws", String, queue_size=1)
        
    @Node.route("/start_record", "POST")
    def start_record(self, bag_name):
        service_name = "/data_recorder/start_recording"
        rospy.wait_for_service(service_name)
        srv = rospy.ServiceProxy(service_name, StartBagRecord)
        res = srv(prefix=bag_name)
        if not res.success:
            return ERROR_RESPONSE(res.message)
        self.ws_pub.publish(json.dumps({"type": "state", "record": True}))
        return SUCCESS_RESPONSE(res.message)
    
    @Node.route("/stop_record", "POST")
    def stop_record(self):
        service_name = "/data_recorder/stop_recording"
        rospy.wait_for_service(service_name)
        srv = rospy.ServiceProxy(service_name, Trigger)
        res = srv()
        self.ws_pub.publish(json.dumps({"type": "state", "record": False}))
        return SUCCESS_RESPONSE()

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
    
if __name__ == '__main__':
    node = Other()
    node.run()