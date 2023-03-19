
"""
Modified from Dana Nau <nau@umd.edu> action.py, July 14, 2021
"""

import gtpyhop

"""
Each gtpyhop action is a Python function. The 1st argument is the current
state, and the others are the action's usual arguments. This is analogous to
how methods are defined for Python classes (where the first argument is
always the name of the class instance). For example, the function
pickup(s,b) implements the action ('pickup', b).
The blocks-world actions use three state variables:
- pos[b] = block b's position, which may be 'table', 'hand', or another block.
- clear[b] = False if a block is on b or the hand is holding b, else True.
- holding['hand'] = name of the block being held, or False if 'hand' is empty.
"""

def get_next_state(state, act):
    if act[0] == 'move':
        return move(state, act[1])
    elif act[0] == 'pickup':
        return pickup(state, act[1], act[2])
    elif act[0] == 'put':
        return put(state, act[1], act[2])
    elif act[0] == 'open':
        return open(state, act[1])
    elif act[0] == 'close':
        return close(state, act[1])
    elif act[0] == 'pour':
        return pour(state, act[1], act[2])




def move(s, loc):

    s.pos['robot'] = loc
    return s

def pickup(s, item, loc):
    if s.pos[item] == loc and s.holding['hand'] == 'empty':
        s.pos[item] = 'hand'
        s.holding['hand'] = item
        return s

def put(s, item, loc):
    if s.pos[item] == 'hand':
        s.pos[item] = loc
        s.holding = 'empty'
        return s

def open(s, item):
    if s.status[item] == 'close':
        print("status", s.status[item])
        print(item)
        s.status[item] = 'open'
        print("status", s.status[item])

        return s
def close(s, item):
    if s.status[item] == 'open':
        s.status[item] = 'close'
        return s
def pour(s, item, loc):
    if s.pos[item] == 'hand':
        s.pos[item] = loc
        return s

gtpyhop.declare_actions(move, pickup, put, open, close, pour)


