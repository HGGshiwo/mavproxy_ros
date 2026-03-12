#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def cmd_vel_callback(msg):
    vx = msg.linear.x
    vy = msg.linear.y
    yaw_rate = msg.angular.z
    rospy.loginfo("vx=%.4f, vy=%.4f, yaw_rate=%.4f" % (vx, vy, yaw_rate))


def odom_callback(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [
        orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
    ]
    roll, pitch, yaw = euler_from_quaternion(orientation_list)
    rospy.loginfo("ENU yaw=%.4f rad" % (yaw))


if __name__ == "__main__":
    rospy.init_node("cmd_vel_debug_node")
    rospy.Subscriber("/go2/cmd_vel", Twist, cmd_vel_callback)
    rospy.loginfo("Subscribing to /go2/cmd_vel ...")
    rospy.Subscriber("/mavros/local_position/odom", Odometry, odom_callback)
    rospy.loginfo("Subscribing to /mavros/local_position/odom ...")
    rospy.spin()
