#!/usr/bin/env python
import argparse

import cv2
import numpy as np
import queue
import heapq

import matplotlib.pyplot as plt
import time

import rospy
from geometry_msgs.msg import Twist, Pose, Point
import sys, select, os

if os.name == 'nt':
    import msvcrt, time
else:
    import tty, termios
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates



msg = """
Control Your Robot!
---------------------------
Moving around:
        w    
   a         d
        x    
w/x: Move forward/backward
a/d: Change the angle of the steering wheels
s: change the steering angle to default
q/e : increase/decrease max speeds by 10%
space key : force stop
anything else : stop smoothly
CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('teleop')


    pub_mobile = rospy.Publisher('/mmrobot/mobile_controller/cmd_vel', Twist, queue_size=10)
    # sub_odom = rospy.Subscriber('/mmrobot/mobile_controller/odom', Odometry, callback1)
    # sub_odom = rospy.Subscriber('/mmrobot/mobile_controller/odom', Odometry, callback)
    #  rospy.init_node('oodometry', anonymous=True) #make node

    # Transformation from Gazebo world to image
    r = rospy.Rate(1)
    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if (key == '\x03'):
                break

            if key == 'w':
                control_speed += 0.1
            elif key == 'a':
                control_turn += 0.1
            elif key == 'd':
                control_turn -= 0.1
            elif key == 's':
                control_turn = 0
                control_speed = 0
            elif key == 'x':
                control_speed -= 0.1



            print("vel:", control_speed, "ang vel:", control_turn)

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub_mobile.publish(twist)

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub_mobile.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

