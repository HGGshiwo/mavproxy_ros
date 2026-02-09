#!/usr/bin/python3
# -*- coding: utf-8 -*-
from base.node import SUCCESS_RESPONSE, Node
from base.utils import post_json
from event_callback import http_proxy, ros
from event_callback.core import CallbackManager
import rospy
from std_msgs.msg import String
from base.utils import ERROR_RESPONSE
from rsos_msgs.srv import StartBagRecord
from std_srvs.srv import Trigger
import json
from rsos_msgs.srv import SetGimbalAngle, SetGimbalAngleResponse

try:
    from rsos_msgs.srv import SetCameraExposure
except Exception:
    pass
from mavros_msgs.msg import SysStatus
from control_model import *


class Other(CallbackManager):
    def __init__(self):
        super().__init__()
        self.recording = False
        self.ws_pub = rospy.Publisher("ws", String, queue_size=-1)
        # fmt: off
        self.detect_param = {
            "nohardhat": "/UAV0/perception/yolo_detection/enable_detection",
            "smoke": "/UAV0/perception/yolo_detection_smoke/enable_detection",
        }
        self.follow_param_name = "/UAV0/perception/object_location/object_location_node/enable_send"
        # fmt: on

    @ros.topic("/mavros/sys_status", SysStatus)
    def sys_status_cb(self, msg: SysStatus):
        data = {}
        if msg.battery_remaining != -1:
            data["battery_remaining"] = msg.battery_remaining  # 百分比
        if msg.current_battery != -1:
            data["current_battery"] = msg.current_battery * 0.01  # cA * 0.01 = A
        if msg.voltage_battery != 65536:
            data["voltage_battery"] = msg.voltage_battery / 1000
        self.ws_pub.publish(json.dumps({"type": "state", **data}))

    @http_proxy.post("/start_record")
    def start_record(self, data: StartRecordModel):
        service_name = "/data_recorder/start_recording"
        rospy.wait_for_service(service_name, timeout=5)
        srv = rospy.ServiceProxy(service_name, StartBagRecord)
        res = srv(prefix=data.bag_name)
        if not res.success:
            return ERROR_RESPONSE(res.message)
        self.ws_pub.publish(json.dumps({"type": "state", "record": True}))
        self.recording = True
        return SUCCESS_RESPONSE(res.message)

    @http_proxy.post("/stop_record")
    def stop_record(self):
        service_name = "/data_recorder/stop_recording"
        rospy.wait_for_service(service_name, timeout=5)
        srv = rospy.ServiceProxy(service_name, Trigger)
        res = srv()
        self.ws_pub.publish(json.dumps({"type": "state", "record": False}))
        self.recording = False
        return SUCCESS_RESPONSE()

    @http_proxy.get("/get_record")
    def get_record(self):
        return SUCCESS_RESPONSE(msg=self.recording)

    @http_proxy.post("/set_ros_param")
    def set_ros_param(self, data: SetRosParamModel):
        rospy.set_param(data.name, data.value)
        return SUCCESS_RESPONSE("OK")

    @http_proxy.get("/get_ros_param{name:path}")
    def get_ros_param(self, name: str):
        try:
            return SUCCESS_RESPONSE(rospy.get_param(name))
        except KeyError as e:
            return ERROR_RESPONSE(str(e))

    @http_proxy.get("/get_gimbal")
    def get_gimbal(self):
        try:
            mode = rospy.get_param("/UAV0/sensor/serial_gimbal/angle_mode", 0)
            angle = rospy.get_param("/UAV0/sensor/serial_gimbal/gimbal_angle", 0)
            return SUCCESS_RESPONSE({"mode": mode, "angle": angle})
        except Exception as e:
            return ERROR_RESPONSE(str(e))

    @http_proxy.post("/set_gimbal")
    def set_gimbal(self, data: SetGimbalModel):
        service_name = "/UAV0/sensor/serial_gimbal/set_gimbal_angle"
        rospy.wait_for_service(service_name, timeout=3)
        srv = rospy.ServiceProxy(service_name, SetGimbalAngle)
        res = srv(mode=data.mode, angle=data.angle)
        if not res.success:
            return ERROR_RESPONSE(res.message)
        return SUCCESS_RESPONSE(res.message)

    @http_proxy.get("/get_exposure")
    def get_exposure(self):
        data = {
            "shutter": {
                "value": rospy.get_param("/UAV0/sensor/video11_camera/shutter", 50),
                "max": rospy.get_param("/UAV0/sensor/video11_camera/shutter_max", 100),
                "min": rospy.get_param("/UAV0/sensor/video11_camera/shutter_min", 0),
                "step": rospy.get_param("/UAV0/sensor/video11_camera/shutter_step", 1),
            },
            "sensitivity": {
                "value": rospy.get_param("/UAV0/sensor/video11_camera/ISO", 50),
                "max": rospy.get_param("/UAV0/sensor/video11_camera/ISO_max", 100),
                "min": rospy.get_param("/UAV0/sensor/video11_camera/ISO_min", 0),
                "step": rospy.get_param("/UAV0/sensor/video11_camera/ISO_step", 2),
            },
        }
        return SUCCESS_RESPONSE(data)

    @http_proxy.post("/set_exposure")
    def set_exposure(self, data: SetExposureModel):
        service_name = "/UAV0/sensor/video11_camera/set_exposure"
        rospy.wait_for_service(service_name, timeout=3)
        srv = rospy.ServiceProxy(service_name, SetCameraExposure)
        res = srv(shutter=int(data.shutter), sensitivity=int(data.sensitivity))
        if not res.success:
            return ERROR_RESPONSE(res.message)
        return SUCCESS_RESPONSE(res.message)

    @http_proxy.post("/start_detect")
    def start_detect(self, data: StartDetectModel):
        for _, name in self.detect_param.items():
            rospy.set_param(name, False)
        if data.type not in self.detect_param:
            return ERROR_RESPONSE(
                f"{data.type} not in {', '.join(self.detect_param.keys())}"
            )
        rospy.set_param(self.detect_param[data.type], True)
        self.ws_pub.publish(json.dumps({"type": "state", "detect": data.type}))
        rospy.set_param(self.follow_param_name, True)
        return SUCCESS_RESPONSE()

    @http_proxy.post("/stop_detect")
    def stop_detect(self):
        for _, name in self.detect_param.items():
            rospy.set_param(name, False)
        rospy.set_param(self.follow_param_name, False)
        self.ws_pub.publish(json.dumps({"type": "state", "detect": "Not Start"}))
        post_json("start_planner", {"auto": True})
        return SUCCESS_RESPONSE()

    @http_proxy.get("/get_detect")
    def get_detect(self):
        out = ""
        if rospy.get_param(self.follow_param_name, False):
            for _out, param_name in self.detect_param.items():
                if rospy.get_param(param_name, False):
                    out = _out
                    break
        return SUCCESS_RESPONSE(out)


if __name__ == "__main__":
    node = Other()
    rospy.spin()
