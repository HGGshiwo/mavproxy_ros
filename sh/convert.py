#!/usr/bin/env python3
import rospy
import tf
import geometry_msgs.msg
import nav_msgs.msg
from math import sin, cos
from quadrotor_msgs.msg import PositionCommand

def msg_to_odom(msg):
    odom = nav_msgs.msg.Odometry()
    
    # 设置header
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "map"
    odom.child_frame_id = "base_link"
    
    # 设置位置
    odom.pose.pose.position.x = msg.position.x
    odom.pose.pose.position.y = msg.position.y
    odom.pose.pose.position.z = msg.position.z
    
    # 设置方向（从yaw到四元数）
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, msg.yaw)
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]
    
    # 设置速度
    odom.twist.twist.linear.x = msg.velocity.x
    odom.twist.twist.linear.y = msg.velocity.y
    odom.twist.twist.linear.z = msg.velocity.z
    
    return odom

def callback(msg):
    odom = msg_to_odom(msg)
    pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('simple_odom_publisher')
    pub = rospy.Publisher('odom', nav_msgs.msg.Odometry, queue_size=10)
    rospy.Subscriber('/planning/pos_cmd', PositionCommand, callback)  # 替换YourMessageType
    rospy.spin()