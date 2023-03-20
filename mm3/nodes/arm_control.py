#!/usr/bin/env python
from __future__ import print_function

import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

import sys, select, termios, tty

import sympy as sp
from sympy.physics.mechanics import dynamicsymbols, Point, ReferenceFrame
from math import pi
import numpy as np
import time
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionResult
#from control_msgs.msg import FollowJointTrajectoryActionGoal
################################## moveit ##################################
# from six.moves import input

import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
from std_msgs.msg import String

#############################################################################

PI = 3.141592
l_offset1 = 0.11
l_offset2 = 0.14
l_offset3 = 1.21

r_offset1 = -0.11
r_offset2 = -1.79
r_offset3 = 1.19

LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.1

def all_close(goal, actual, tolerance):

    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroup:
    def __init__(self, group_name):
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self, value):

        move_group = self.move_group


        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0.2
        joint_goal[1] = 0.5
        joint_goal[2] = 0.5
        joint_goal[3] = value
#        joint_goal[6] = 0
#        joint_goal[7] = tau / 6  # 1/6 of a turn
#        joint_goal[8] = 0

        move_group.go(joint_goal, wait=True)

        move_group.stop()


        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, x, y, z, w):

        move_group = self.move_group


        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = w
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        move_group.set_pose_target(pose_goal)

        plan = move_group.go(wait=True)
        move_group.stop()

        move_group.clear_pose_targets()


        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):

        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):

        move_group = self.move_group

        move_group.execute(plan, wait=True)

    def get_info(self):
        move_group = self.move_group
        pose_goal = move_group.geometry_msgs.msg.Pose()
        print(pose_goal)

def joint_callback(data):
    # define publisher
    pub_linear_slide = rospy.Publisher('/mmrobot/linear_joint/command', Float64, queue_size=10)

    pub_left_arm_j1 = rospy.Publisher('/mmrobot/l_arm_joint1/command', Float64, queue_size=10)
    pub_left_arm_j2 = rospy.Publisher('/mmrobot/l_arm_joint2/command', Float64, queue_size=10)
    pub_left_arm_j3 = rospy.Publisher('/mmrobot/l_arm_joint3/command', Float64, queue_size=10)
    pub_left_gripper = rospy.Publisher('/mmrobot/l_gripper_joint/command', Float64, queue_size=10)

    pub_right_arm_j1 = rospy.Publisher('/mmrobot/r_arm_joint1/command', Float64, queue_size=10)
    pub_right_arm_j2 = rospy.Publisher('/mmrobot/r_arm_joint2/command', Float64, queue_size=10)
    pub_right_arm_j3 = rospy.Publisher('/mmrobot/r_arm_joint3/command', Float64, queue_size=10)
    pub_right_gripper = rospy.Publisher('/mmrobot/r_gripper_joint1/command', Float64, queue_size=10)

   # print(data.position)
    # Publish joint
    pub_linear_slide.publish(data.position[2])

    pub_left_arm_j1.publish(data.position[3])
    pub_left_arm_j2.publish(data.position[4])
    pub_left_arm_j3.publish(data.position[5])

    pub_right_arm_j1.publish(data.position[7])
    pub_right_arm_j2.publish(data.position[8])
    pub_right_arm_j3.publish(data.position[9])


right_arm = MoveGroup('mm3_right')
left_arm = MoveGroup('mm3_left')
mobile_plane = MoveGroup('mobile_plane')


def gui_callback(input):

    if input.data == "open door":
        left_arm.go_to_joint_state(3.14)
       # right_arm.go_to_joint_state(0)
    elif input.data == "test right arm":
        right_arm.go_to_pose_goal(0.3, 0.03, 0.5, 1)
    elif input.data == "test left arm":
        left_arm.go_to_pose_goal(0.3, 0.03, 0.5, 1)
    elif input.data == "test mobile plane":
        mobile_plane.go_to_pose_goal(1, 1, 1, 1)
    elif input.data == "get info":
        print("left arm joint", left_arm.get_info())
        print("right arm joint", right_arm.get_info())
        print("mobile plane joint", mobile_plane.get_info())
        # print("left arm pose", left_arm.get_current_pose("left_gripper_joint1"))
        # print("right arm pose", right_arm.get_current_pose("right_gripper_joint1"))
        # print("mobile plane pose", mobile_plane.get_current_pose("right_gripper_joint1"))

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('arm_control')

    # publisher

    rospy.Subscriber('/joint_states', JointState, joint_callback)
    rospy.Subscriber('/gui_command', String, gui_callback)
    rospy.spin()
