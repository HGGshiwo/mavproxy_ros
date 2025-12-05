#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from rsos_msgs.msg import PointObj  # 确保已编译并source你的msg

def goal2_callback(msg):
    # 创建PointObj消息
    obj_msg = PointObj()
    obj_msg.pos.x = msg.pose.position.x
    obj_msg.pos.y = msg.pose.position.y
    obj_msg.pos.z = msg.pose.position.z
    obj_msg.velocity.x = 0.0
    obj_msg.velocity.y = 0.0
    obj_msg.velocity.z = 0.0
    # 发布
    pub.publish(obj_msg)
    rospy.loginfo("Published PointObj with pos: (%.2f, %.2f, %.2f)" % (
        obj_msg.pos.x, obj_msg.pos.y, obj_msg.pos.z))

def obj_lla_callback(msg):
    x = msg.point.x
    y = msg.point.y
    z = msg.point.z
    rospy.loginfo("Received obj_lla: x=%.6f, y=%.6f, z=%.6f" % (x, y, z))

if __name__ == '__main__':
    rospy.init_node('goal2_to_location_vel_node')
    pub = rospy.Publisher('/UAV0/perception/object_location/location_vel', PointObj, queue_size=10)
    rospy.Subscriber('/move_base_simple/goal2', PoseStamped, goal2_callback)
    rospy.Subscriber('/UAV0/perception/object_location/obj_lla', PointStamped, obj_lla_callback)
    rospy.loginfo("Node started, waiting for messages...")
    rospy.spin()