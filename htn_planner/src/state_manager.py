#!/usr/bin/env python3

import gtpyhop
import rospy
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal
import actionlib
from std_msgs.msg import String
from htn_planner.msg import action_msg

import test_harness as th  # code for use in paging and debugging

# We must declare the current domain before importing methods and actions.
# To make the code more portable, we don't hard-code the domain name, but
# instead use the name of the package.
the_domain = gtpyhop.Domain('Home_Service_Robot')
from method import *
from action import *
# Python 2/3 compatibility imports

import rospy
from executor import *

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt


class State_Manager:
    def __init__(self) -> None:
        # move_base_msgs/MoveBaseActionResult
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.move_base_goal_reach_callback)
        rospy.Subscriber("gui_command", String, self.input_callback)
        # Construct Robot object
        self.Robot = ROBOT()


        self.current_state = gtpyhop.State('Home_Service_initial_state')
        self.action_list = list() # this list is used for stored the action genersted from gtpyhop
        # Test Function State
        self.current_state.pos = {'robot': 'loc0', 'cola': 'table', 'dish': 'table', 'door': 'door', 'trash_can': 'trash_can'}
        self.current_state.status = {'frozen_veg_bag': 'close', 'jar': 'close', 'door': 'close', 
                        'fridge': 'close', 'trash_can': 'close', 'dish':'empty'}
        self.current_state.holding = {'hand': 'empty'}

        self.current_state.types = {
                    'robot': ['robot'],
                    'location': ['loc0', 'trash_can', 'fridge', 'cabinet', 'door', 'bed', 'table', 'hand', 'drain'],
                    'item': ['door', 'fridge', 'trash', 'dish', 'beer', 'cola'], 
                    'switchable_item': ['door', 'fridge', 'trash_can'],
                    'pourable_item': ['jar', 'dish'],
                    'portable_item': ['trash', 'cola', 'beer', 'dish']}  

        # self.task_list = ['open door', 'close door', 'open fridge', 'close fridge', 'open trash_can',
        #        'close trash_can', 'pour drink', 'clean table']
        self.task_list = ['open', 'close', 'pour', 'clean', 'pickup', 'put']

        self.position_dict = {'loc0': [0, 0, 0.0001], 'bookcase':[4.2979, -4.797, -1.5724], 'table':[1.16772, -1.0591, -1.3397], 'cabinet': [-4.3616, 2.4645, 1.8145], 'drain':[7.5, -3.5, 0.001],
                              'trash_can':[1.6931, -0.9217, 0.7041], 'door':[5.7857, -5.0974, -1.64], 'fridge':[7.9145, -1.3464, -0.165],'chair':[5.5523, 0.1573, 0.1352]}

        self.reach_goal = False
        self.current_action = None
        # Scene Understangind initial state
        # Test Function State
        # self.current_state.pos = {'robot': 'loc0', 'door': 'door', 'trash_can':'trash_can'}
        # self.current_state.status = {'door': 'close'}
        # self.current_state.holding = {'hand': 'empty'}

        # self.current_state.types = {
        #             'robot': ['robot'],
        #             'location': ['loc0', 'trash_can', 'fridge', 'cabinet', 'door', 'bed', 'table', 'hand', 'drain'],
        #             'item': ['door', 'fridge', 'trash', 'dish', 'beer', 'cola'], 
        #             'switchable_item': ['door', 'fridge', 'trash_can'],
        #             'pourable_item': ['jar', 'dish'],
        #             'portable_item': ['trash', 'cola', 'beer', 'dish']}  

        # # self.task_list = ['open door', 'close door', 'open fridge', 'close fridge', 'open trash_can',
        # #        'close trash_can', 'pour drink', 'clean table']
        # self.task_list = ['open', 'close', 'pour', 'clean', 'pickup', 'put']

        # self.position_dict = {'loc0': [0, 0, 0.0001], 'bookcase':[4.2979, -4.797, -1.5724], 'table':[1.16772, -1.0591, -1.3397], 'cabinet': [-4.3616, 2.4645, 1.8145], 'drain':[7.5, -3.5, 0.001],
        #                       'trash_can':[1.6931, -0.9217, 0.7041], 'door':[5.7857, -5.0974, -1.64], 'fridge':[7.9145, -1.3464, -0.165],'chair':[5.5523, 0.1573, 0.1352]}


    def move_base_goal_reach_callback(self, data):
        act, des = self.current_action
        rospy.loginfo("Reach Goal, Perform Next Action %s %s", act, des)
        self.reach_goal = True

    # call back function, get the gui input
    def input_callback(self, task)->None:
        rospy.loginfo("Task: %s", task.data)
        act, item = task.data.split()
        # If the input is in the task list, then execute the input command
        if act in self.task_list:
            goal = self.Generate_Goal(act, item)
            # goal = command_input(cmd)
            # Solve every goal using GTPyhop, create a action list
            for g in goal:
                if gtpyhop.get_type(g) != 'list':
                    plan = gtpyhop.find_plan(self.current_state, [g])
                else:
                    plan = gtpyhop.find_plan(self.current_state, g)
                # print('plan:', plan)
                for sub_action in plan:
                    self.action_list.append(sub_action)

            # plan = gtpyhop.find_plan(current_state, goal)
            print("plan", self.action_list)
            if(len(self.action_list) > 0):
                self.current_action = self.action_list.pop(0)
                action_type, action_destination = self.current_action
            while len(self.action_list) > 0:
                if action_type == 'move':
                    x, y, yaw = self.position_dict[action_destination]
                    rospy.loginfo("Robot Destination %s position %f %f %f", action_destination, x, y, yaw)
                    self.Robot.movebase_client(x, y, yaw)
                # action list:'open', 'close', 'pickup', 'put', 'pour'
                elif action_type in self.arm_action_list:
                    self.Robot.Arm_Control()
                    # pose = find_target(action_destination)
                    # moveit_client(pose)
                if self.reach_goal:
                    self.current_action = self.action_list.pop(0)
                    action_type, action_destination = self.current_action
                    self.reach_goal = False


        # try:
        #     print(self.action_list)
        #     self.action_publisher(self.action_list[0][0], self.action_list[0][1], 'none')
        # except rospy.ROSInterruptException:
        #     pass
    
    def Generate_Goal(self, cmd:String, item:String)->list():
        goal = list()
        rospy.loginfo("Generating Goal List")
        if(cmd == 'clean'): # example input: clean table
            goal.append(('Home_service', 'trash_can', 'open'))
            for key in self.current_state.pos:
                # for example, key is cola or dish, item is the table
                # check is there anything on the table
                # Maybe using bidict can be more efficiency
                if self.current_state.pos[key] == item:
                    # goal = [('Home_service', 'drain_cover', 'drain'), ('Home_service', 'robot', 'drain')] 
                    if(key == 'cola' or key == 'beer'):
                        goal.append(('Home_service', key, 'trash_can'))
                    elif(key == "dish"):
                        goal.append(('Home_service', key, 'drain'))

        elif(cmd == 'open'):
            goal.append(('Home_service', item, cmd))
        elif(cmd == 'close'):
            goal.append(('Home_service', item, cmd))
        elif(cmd == 'pour'):
            goal.append(('Home_service', item, 'sink'), ('Home_service', 'robot', 'sink')) 
        # elif(cmd == 'pickup'):
        #     return
        # elif(cmd == 'put'):
        #     return
        rospy.loginfo("Goal Generated")

        return goal
        # return goal

    def action_publisher(self, plan_act:String, plan_des:String, plan_item:String)->None:
        pub = rospy.Publisher('action', action_msg, queue_size=1)
        rate = rospy.Rate(1)
        msg = action_msg()

        while not rospy.is_shutdown():
            msg.action = plan_act
            msg.destination = plan_des
            msg.item = plan_item
            print(msg)
            pub.publish(msg)
            rate.sleep()


    # not tested
    def state_update(self, current_state, act):
        # Use for state update when perform the action
        return get_next_state(current_state, act)


    # not tested
    def satisfy(self, current_state, goal):
        # Check whether current state satisfies the goal
        # If  the door is already open, we do not have to plan this task

        if goal[1] in current_state.pos:
            if goal[2] == current_state.pos[goal[1]]:
                return True
        elif goal[1] in current_state.status:
            if goal[2] == current_state.status[goal[1]]:
                return True
        elif goal[1] in current_state.holding:
            if goal[2] == current_state.holding[goal[1]]:
                return True
        return False
    

    def Scene_Understanding_to_State(self):
        return None



if __name__=="__main__":

    rospy.init_node('State_Manager_Node', anonymous = True)
    gtpyhop.current_domain = the_domain

    gtpyhop.print_domain()

    S_M = State_Manager()

    # rospy.Subscriber('action', action_msg, action_callback)
    # finish_pub = rospy.Publisher('finish', Int16, queue_size=10)


    rospy.spin()

"""
current_state.pos = {'robot': 'loc0', 'drain_cover': 'drain', 'frozen_veggies_bag': 'fridge', 'jar': 'cabinet',
                     'door': 'door', 'faucet': 'sink', 'fridge': 'fridge', 'garbage_can': 'garbage_can',
                     'oven': 'oven', 'pyrex': 'cabinet', 'ziploc': 'cabinet', 'wine': 'table', 'garbage': 'table',
                     'dish': 'cabinet', 'jar_content': 'jar'}
current_state.status = {'frozen_veg_bag': 'close', 'jar': 'close', 'door': 'close', 'faucet': 'close',
                        'fridge': 'close', 'oven': 'open', 'pyrex': 'open', 'ziploc': 'open', 'garbage_can': 'open'}
current_state.holding = {'hand': 'empty'}

current_state.types = {
    'robot': ['robot'],
    'location': ['loc0', 'drain', 'fridge', 'cabinet', 'door', 'sink', 'garbage_can', 'oven', 'table', 'jar', 'floor',
                 'dish'],
    'item': ['drain_cover', 'frozen_veggies_bag', 'jar', 'door', 'faucet', 'fridge', 'garbage_cover', 'oven', 'pyrex',
             'ziploc', 'wine', 'garbage', 'dish', 'jar_content'],  # This is use for pos
    'item_has_switch': ['frozen_veg_bag', 'jar', 'door', 'faucet', 'fridge', 'oven', 'pyrex', 'ziploc', 'garbage_can'],
    'pour_able_item': ['wine', 'jar'],
    'portable_item': ['drain_cover', 'frozen_veggies_bag', 'jar', 'garbage_cover', 'pyrex', 'ziploc', 'wine', 'garbage',
                      'dish']}  # this is use for status (open, close

task_list = ['cap drain', 'uncap drain', 'pour jar', 'open door', 'close door', 'open faucet', 'close faucet', 'open fridge', 'close fridge', 'open garbage_can',
               'close garbage_can', 'open oven', 'close oven', 'open pyrex', 'close pyrex', 'open ziploc', 'close ziploc', 'pour drink', 'throw garbage']

position_dict = {'loc0': [0, 0, 0.01], 'drain': [-3.302980, 1.166715, 1.166715], 'fridge':[-2.676306, 1.800245, 1.593131], 'cabinet': [1.577489, -0.350017, -1.233426],'door': [3.321430, -1.693682, -0.298744],
                 'sink': [-3.302980, 1.166715, 1.166715], 'garbage_can': [-3.464717, -3.710056, 2.360309], 'oven': [-3.438507, 2.365099, 1.592062], 'table': [1.5, 1.5, 0.1], 'jar': [1.577489, -0.350017, -1.233426], 'floor': [0, 0, 0.01],
                 'dish': [-3.302980, 1.166715, 1.166715], 'stair': [0, -2, -1.57]}
arm_action_list = ['open', 'close', 'pickup', 'put', 'pour']



def command_input(cmd:string)->list():
    #    goal_state = gtpyhop.Multigoal('goal')

    if cmd == 'cap drain':
        goal = [('Home_service', 'drain_cover', 'drain'), ('Home_service', 'robot', 'drain')]  # tested
        # plan [('move', 'drain'), ('pickup', 'drain_cover', 'drain'), ('put', 'drain_cover', 'drain')]
    elif cmd == 'uncap drain':
        goal = [('Home_service', 'drain_cover', 'sink'), ('Home_service', 'robot', 'sink')]  # tested
        # plan [('move', 'drain'), ('pickup', 'drain_cover', 'drain'), ('move', 'sink'), ('put', 'drain_cover', 'sink')]
    elif cmd == 'pour jar':
        goal1 = [('Home_service', 'dish', 'table'), ('Home_service', 'robot', 'table')]
        goal2 = [('Home_service', 'jar', 'open')]
        goal3 = [('Home_service', 'jar', 'dish'), ('Home_service', 'robot', 'dish')]
        # goal4 = [('Home_service', 'jar_content', 'dish'), ('Home_service', 'robot', 'dish')]
        # goal4 = [('Home_service', 'jar', 'close')]
        goal = [goal1, goal2, goal3]
        # plan: [('move', 'cabinet'), ('pickup', 'dish', 'cabinet'), ('move', 'table'), ('put', 'dish', 'table'),
        #        ('move', 'cabinet'), ('open', 'jar'), ('move', 'cabinet'), ('pickup', 'jar', 'cabinet'), ('move', 'dish'), ('pour', 'jar', 'dish')]

    elif cmd == 'open door':
        goal = [('Home_service', 'door', 'open')]  # tested
        # plan [('move', 'door'), ('open', 'door')]
        # Bug: if the initial state is door open, the plan will become [('move', 'door'), ['close', 'door'], ['open', ''door]]
    elif cmd == 'close door':
        goal = [('Home_service', 'door', 'close')]  # done
        # plan[('move', 'door'), ('close', 'door')] But have to check the initial state
    elif cmd == 'open faucet':
        goal = [('Home_service', 'faucet', 'open')]  # tested
        # plan [('move', 'sink'), ('open', 'faucet')]
    elif cmd == 'close faucet':
        goal = [('Home_service', 'faucet', 'close')]  # tested
        # plan [('move', 'sink'), ('close', 'faucet')]
    elif cmd == 'open fridge':
        goal = [('Home_service', 'fridge', 'open')]  # tested
        # plan [('move', 'fridge'), ('open', 'fridge')]
    elif cmd == 'close fridge':
        goal = [('Home_service', 'fridge', 'close')]  # done
    elif cmd == 'open garbage_can':
        goal = [('Home_service', 'garbage_can', 'open')]  # tested
        # plan [('move', 'garbage_can'), ('open', 'garbage_can')]
    elif cmd == 'close garbage_can':
        goal = [('Home_service', 'garbage_can', 'close')]  # tested
        # plan [('move', 'garbage_can'), ('close', 'garbage_can')]
    elif cmd == 'open oven':
        goal = [('Home_service', 'oven', 'open')]  # tested
        # plan [('move', 'oven'), ('open', 'oven')]
    elif cmd == 'close oven':
        goal = [('Home_service', 'oven', 'close')]  # tested
        # plan [('move', 'oven'), ('close', 'oven')]
    elif cmd == 'open pyrex':
        goal = [('Home_service', 'pyrex', 'open')]  # tested
        # plan [('move', 'cabinet'), ('open', 'pyrex')]
    elif cmd == 'close pyrex':
        goal = [('Home_service', 'pyrex', 'close')]  # tested
        # plan [('move', 'cabinet'), ('close', 'pyrex')]
    elif cmd == 'open ziploc':
        goal = [('Home_service', 'ziploc', 'open')]  # tested
        # plan [('move', 'cabinet'), ('open', 'ziploc')]
    elif cmd == 'close ziploc':
        goal = [('Home_service', 'ziploc', 'close')]  # tested
        # plan [('move', 'cabinet'), ('close', 'ziploc')]
    elif cmd == 'pour drink':
        goal = [('Home_service', 'wine', 'sink'), ('Home_service', 'robot', 'sink')]  # tested
        # plan [('move', 'table'), ('pickup', 'wine', 'table'), ('move', 'sink'), ('pour', 'wine', 'sink')]
    elif cmd == 'throw garbage':
        goal = [('Home_service', 'garbage_can', 'open'), ('Home_service', 'garbage', 'garbage_can'),    # tested
                ('Home_service', 'robot', 'garbage_can')]
        # plan [('move', 'garbage_can'), ('open', 'garbage_can'), ('move', 'table'), ('pickup', 'garbage', 'table'), ('move', 'garbage_can'), ('put', 'garbage', 'garbage_can')]
    return goal


"""