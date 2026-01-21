#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from mavros_msgs.srv import CommandBool, CommandBoolResponse
from mavros_msgs.srv import SetMode, SetModeResponse
from mavros_msgs.srv import CommandTOL, CommandTOLResponse
from mavros_msgs.srv import StreamRate, StreamRateResponse
from mavros_msgs.srv import ParamSet, ParamSetResponse


def arming_callback(req):
    rospy.loginfo("Dummy arming service called")
    return CommandBoolResponse(success=True)


def set_mode_callback(req):
    rospy.loginfo("Dummy set_mode service called: %s" % req.custom_mode)
    return SetModeResponse(mode_sent=True)


def takeoff_callback(req):
    rospy.loginfo("Dummy takeoff service called")
    return CommandTOLResponse(success=True)


def streamrate_callback(req):
    rospy.loginfo("Dummy stream rate called")
    return StreamRateResponse()


def paramset_callback(req):
    rospy.loginfo("Dummy param set called")
    return ParamSetResponse()


if __name__ == "__main__":
    rospy.init_node("dummy_mavros_services")

    srv1 = rospy.Service("/mavros/cmd/arming", CommandBool, arming_callback)
    srv2 = rospy.Service("/mavros/set_mode", SetMode, set_mode_callback)
    srv3 = rospy.Service("/mavros/cmd/takeoff", CommandTOL, takeoff_callback)
    srv4 = rospy.Service("/mavros/param/set", ParamSet, paramset_callback)
    srv5 = rospy.Service("/mavros/set_stream_rate", StreamRate, streamrate_callback)
    rospy.loginfo("Dummy MAVROS services are ready.")
    rospy.spin()
