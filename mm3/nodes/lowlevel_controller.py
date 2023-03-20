#!/usr/bin/env python

import rospy
import tf.transformations
from geometry_msgs.msg import Twist


def callback(msg):
    pub_mobile = rospy.Publisher('/mmrobot/mobile_controller/cmd_vel', Twist, queue_size=10)
    twist = Twist()

    # rospy.loginfo("Received a /cmd_vel message!")
   # rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
   # rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    twist.linear.x = msg.linear.x * 1.01675169 + 0.001558362339107
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = msg.angular.z * 1.01675169 + 0.001558362339107
    pub_mobile.publish(twist)


def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)


    rospy.spin()

if __name__ == '__main__':
    listener()
