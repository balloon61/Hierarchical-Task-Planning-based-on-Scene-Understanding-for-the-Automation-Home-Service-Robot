#!/usr/bin/env python

import numpy as np
from math import pi

import rospy
from std_msgs.msg import String
from htn_planner.msg import action_msg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float64
import sympy as sp
from sympy.physics.mechanics import dynamicsymbols
import time
from pyquaternion import Quaternion


def __yaw_to_quat(yaw):
    """
      Computing corresponding quaternion q to angle yaw [rad]
      :param yaw
      :return: q
      """
    q = Quaternion(axis=[0, 0, 1], angle=yaw)
    return q.elements



class ROBOT:
    def __init__(self) -> None:
        # publisher
        self.pub_linear_slide = rospy.Publisher('/mmrobot/linear_joint/command', Float64, queue_size=10)

        self.pub_left_arm_j1 = rospy.Publisher('/mmrobot/l_arm_joint1/command', Float64, queue_size=10)
        self.pub_left_arm_j2 = rospy.Publisher('/mmrobot/l_arm_joint2/command', Float64, queue_size=10)
        self.pub_left_arm_j3 = rospy.Publisher('/mmrobot/l_arm_joint3/command', Float64, queue_size=10)
        self.pub_left_gripper = rospy.Publisher('/mmrobot/l_gripper_joint/command', Float64, queue_size=10)

        self.pub_right_arm_j1 = rospy.Publisher('/mmrobot/r_arm_joint1/command', Float64, queue_size=10)
        self.pub_right_arm_j2 = rospy.Publisher('/mmrobot/r_arm_joint2/command', Float64, queue_size=10)
        self.pub_right_arm_j3 = rospy.Publisher('/mmrobot/r_arm_joint3/command', Float64, queue_size=10)
        self.pub_right_gripper = rospy.Publisher('/mmrobot/r_gripper_joint1/command', Float64, queue_size=10)

        self.arm_reach_goal = rospy.Publisher('/arm_goal', String, queue_size=10)

        time.sleep(1)

        self.slide_height = -0.3

        self.left_q = np.array([self.slide_height, 0, pi/8, -pi/4])
        self.right_q = np.array([self.slide_height, 0, pi/8, -pi/4])
        self.l_offset = np.array([0, 0.11, 0.14, 1.21])
        self.r_offset = np.array([0, -0.11, -1.79, 1.19])


        self.init_robot()

        # self.l_offset1 = 0.11
        # self.l_offset2 = 0.14
        # self.l_offset3 = 1.21

        # self.r_offset1 = -0.11
        # self.r_offset2 = -1.79
        # self.r_offset3 = 1.19
    

    def init_robot(self):

        rospy.loginfo("initial robot")
        self.pub_linear_slide.publish(self.slide_height)
        self.pub_left_arm_j1.publish(self.left_q[1] + self.l_offset[1])
        self.pub_left_arm_j2.publish(self.left_q[2] + self.l_offset[2])
        self.pub_left_arm_j3.publish(self.left_q[3] + self.l_offset[3])

        self.pub_right_arm_j1.publish(self.right_q[1] + self.r_offset[1])
        self.pub_right_arm_j2.publish(self.right_q[2] + self.r_offset[2])
        self.pub_right_arm_j3.publish(self.right_q[3] + self.r_offset[3])
        # rospy.loginfo("initial robot")


    def update_arm_pose(self, slide_height:float, left_joint, right_joint):
        self.slide_height = slide_height
        self.left_q[1:-1] = left_joint
        self.right_q[1:-1] = right_joint
        self.init_robot()

    def __yaw_to_quat(self, yaw: float):
        """
        Computing corresponding quaternion q to angle yaw [rad]
        :param yaw
        :return: q
        """
        q = Quaternion(axis=[0, 0, 1], angle=yaw)
        return q.elements
    

    def movebase_client(self, x:float, y:float, yaw:float): # x:float, y:float, yaw:float
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        q = self.__yaw_to_quat(yaw)
    
        goal.target_pose.pose.orientation.w = q[0]
        goal.target_pose.pose.orientation.x = q[1]
        goal.target_pose.pose.orientation.y = q[2]
        goal.target_pose.pose.orientation.z = q[3]


        # client.send_goal(goal)
        # wait = client.wait_for_result()
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     return client.get_result()
        

    def forward_left(self, q):
        theta, a, d, alpha = dynamicsymbols("theta a d alpha")

        T = sp.Matrix([[sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
                    [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.sin(alpha) * sp.cos(theta), a * sp.sin(theta)],
                    [0, sp.sin(alpha), sp.cos(alpha), d], [0, 0, 0, 1]])

        T1 = T.subs({theta: 0, a: 0.0635, d: 0.13716 + q[0], alpha: -pi / 2})
        T2 = T.subs({theta: q[1], a: 0, d: 0.09144, alpha: pi / 2})
        T3 = T.subs({theta: q[2], a: 0.27559, d: 0, alpha: 0})
        T4 = T.subs({theta: q[3], a: 0.2667, d: 0, alpha: 0})
        return [T1, T2, T3, T4], T1 * T2 * T3 * T4


    def forward_right(self, q):
        theta, a, d, alpha = dynamicsymbols("theta a d alpha")

        T = sp.Matrix([[sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
                    [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.sin(alpha) * sp.cos(theta), a * sp.sin(theta)],
                    [0, sp.sin(alpha), sp.cos(alpha), d], [0, 0, 0, 1]])

        T1 = T.subs({theta: 0, a: 0.0635, d: 0.13716 + q[0], alpha: pi / 2})
        T2 = T.subs({theta: q[1], a: 0, d: 0.09144, alpha: pi / 2})
        T3 = T.subs({theta: q[2], a: 0.27559, d: 0, alpha: 0})
        T4 = T.subs({theta: q[3], a: 0.2667, d: 0, alpha: 0})
        return [T1, T2, T3, T4], T1 * T2 * T3 * T4


    def Jacobian(self, T_list):
        T01 = T_list[0]
        T02 = T01 * T_list[1]
        T03 = T02 * T_list[2]
        T04 = T03 * T_list[3]

        z0 = np.array([0, 0, 1])
        o0 = np.array([0, 0, 0])

        z1 = np.array(T01[0:3, 2]).astype(np.float64).squeeze(-1)
        o1 = np.array(T01[0:3, 3]).astype(np.float64).squeeze(-1)

        z2 = np.array(T02[0:3, 2]).astype(np.float64).squeeze(-1)
        o2 = np.array(T02[0:3, 3]).astype(np.float64).squeeze(-1)

        z3 = np.array(T03[0:3, 2]).astype(np.float64).squeeze(-1)
        o3 = np.array(T03[0:3, 3]).astype(np.float64).squeeze(-1)

        o4 = np.array(T04[0:3, 3]).astype(np.float64).squeeze(-1)

        J1 = np.zeros(6)
        J1[0:3] = z0
        J2 = np.concatenate([np.cross(z1, o4 - o1), z1], 0)
        J3 = np.concatenate([np.cross(z2, o4 - o2), z2], 0)
        J4 = np.concatenate([np.cross(z3, o4 - o3), z3], 0)

        J = np.array([J1, J2, J3, J4]).T
        return J
    
    def inverse(self, left_q, right_q, direction, offset):
        left_dedt = np.zeros(6).astype(np.float64)
        right_dedt = np.zeros(6).astype(np.float64)

        left_dedt[0] += 0.01 * direction[0]
        right_dedt[0] += 0.01 * direction[0]
        left_dedt[1] += 0.001 * direction[1]
        right_dedt[1] -= 0.001 * direction[1]
        left_dedt[2] += 0.01 * direction[2]
        right_dedt[2] += 0.01 * direction[2]

        left_dedt[-1] += 0.7 * direction[1]
        right_dedt[-1] -= 0.7 * direction[1]

        T_list, T = self.forward_left(left_q)
        init_pos = np.array(T[0:3, 3])
        cur_pos = init_pos

        start_time = time.time()

        while ((init_pos - cur_pos) ** 2).sum() ** 0.5 <= offset:
            T_list, T = self.forward_left(left_q)
            cur_pos = np.array(T[0:3, 3])

            l_J = self.Jacobian(T_list)

            l_dqdt = np.linalg.lstsq(l_J, left_dedt)[0]

            T_list, T = self.forward_right(right_q)
            r_J = self.Jacobian(T_list)
            r_dqdt = np.linalg.lstsq(r_J, right_dedt)[0]

            left_q += l_dqdt * 0.1
            right_q += r_dqdt * 0.1
            slide_height = left_q[0]

            self.pub_linear_slide.publish(slide_height)

            self.pub_left_arm_j1.publish(left_q[1] + self.l_offset[1])
            self.pub_left_arm_j2.publish(left_q[2] + self.l_offset[2])
            self.pub_left_arm_j3.publish(left_q[3] + self.l_offset[3])

            self.pub_right_arm_j1.publish(right_q[1] + self.r_offset[1])
            self.pub_right_arm_j2.publish(right_q[2] + self.r_offset[2])
            self.pub_right_arm_j3.publish(right_q[3] + self.r_offset[3])


    def Arm_Control(self):
        self.inverse(self.left_q, self.right_q, np.array([0,0,1]), 0.2)  
        self.inverse(self.left_q, self.right_q, np.array([0,-1,0]), 0.15)  
        self.inverse(self.left_q, self.right_q, np.array([0,0,1]), 0.2)  


        # goal_str = "Reach Goal %s" % rospy.get_time()
        # self.arm_reach_goal.publish(goal_str)
















# def action_callback(data):
#     which_action = 0
#     Is_finish = False
#     rospy.loginfo("Perform action: %s destination %s item %s", data.action, data.destination, data.item)
#     if data.destination == 'cabinet':
#        act = 1

#     if act == 1:
#         movebase_client()

#     if Is_finish is True:
#         which_action = which_action + 1
#         Is_finish = False
#         finish_pub.publish(which_action)

# if __name__=="__main__":

#     rospy.init_node('Executor_Node', anonymous = True)
#     rospy.Subscriber('action', action_msg, action_callback)
#     finish_pub = rospy.Publisher('finish', Int16, queue_size=10)


#     rospy.spin()
