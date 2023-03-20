#!/usr/bin/env python3
# Uncomment this to use it in debugging:
# from IPython import embed
# from IPython.terminal.debugger import set_trace

import gtpyhop
from std_msgs.msg import String
from htn_planner.msg import action_msg


import test_harness as th  # code for use in paging and debugging

# We must declare the current domain before importing methods and actions.
# To make the code more portable, we don't hard-code the domain name, but
# instead use the name of the package.
the_domain = gtpyhop.Domain('Home_Service_Robot')
import rospy
from method import *
from action import *

# from run_lazy_lookahead import *

def action_publisher(plan_act, plan_des, plan_item):
    pub = rospy.Publisher('action', action_msg, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("in here")
        msg = action_msg()
        msg.action = plan_act
        msg.destination = plan_des
        msg.item = plan_item
        pub.publish(msg)
        rate.sleep()


current_state = gtpyhop.State('Home_Service_initial_state')
current_state.pos = {'robot': 'loc0', 'drain_cover': 'sink', 'frozen_veggies_bag': 'fridge', 'jar': 'cabinet',
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
    'switchable_item': ['frozen_veg_bag', 'jar', 'door', 'faucet', 'fridge', 'oven', 'pyrex', 'ziploc', 'garbage_can'],
    'pourable_item': ['wine', 'jar'],
    'portable_item': ['drain_cover', 'frozen_veggies_bag', 'jar', 'garbage_cover', 'pyrex', 'ziploc', 'wine', 'garbage',
                      'dish']}  # this is use for status (open, close


def state_update(current_state, action):

    return


def state_check(current_state, goal):
    return False


def acyclic_check(action_list):
    # This function is design to remove the redundant action

    return False


def command_input(cmd):
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


if __name__ == "__main__":
    rospy.init_node('state_manager')

    current_state.display()
    cmd = 'throw garbage'

    action_list = []
    goal = command_input(cmd)
    for g in goal:
        t = gtpyhop.get_type(g)
        if g != 'list':
            l = [g]
            
            plan = gtpyhop.find_plan(current_state, l)
        else:         
            plan = gtpyhop.find_plan(current_state, g)
        #print('plan:', plan)
        for sub_action in plan:
            action_list.append(sub_action)
   # plan = gtpyhop.find_plan(current_state, goal)
    # print("plan", action_list)
    try:
        print(action_list)
        action_publisher('123','222', 'none')
    except rospy.ROSInterruptException:
        pass
    # rospy.spin()