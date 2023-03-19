"""
Method definitions for blocks_htn.
-- Dana Nau <nau@umd.edu>, July 14, 2021
"""

import gtpyhop


def is_a(state, variable, type):
    """
    In most classical planners, one would declare data-types for the parameters
    of each action, and the data-type checks would be done by the planner.
    GTPyhop doesn't have a way to do that, so the 'is_a' function gives us a
    way to do it in the preconditions of each action, command, and method.

    'is_a' doesn't implement subtypes (e.g., if rigid.type[x] = y and
    rigid.type[x] = z, it doesn't infer that rigid.type[x] = z. It wouldn't be
    hard to implement this, but it isn't needed in the simple-travel domain.
    """
    print(variable, type)
    return variable in state.types[type]


# pick up and put method
def m_put_item_to_loc(state, item, loc):
   # print(loc, state.pos[item])
    if state.holding['hand'] == 'empty':
        if is_a(state, item, 'item'):
            if state.pos['robot'] == state.pos[item]:
                return [('pickup', item, state.pos[item]), ('move', loc), ('put', item, loc)]
            else:
                # print("in else")
                return [('move', state.pos[item]), ('pickup', item, state.pos[item]), ('move', loc), ('put', item, loc)]

    return False


# gtpyhop.declare_task_methods('pickup', m_pickup)


# def m_put(state, item, loc):
#     print(item, is_a(state, item, 'portable_item'))
#     if is_a(state, loc, 'location'):
#         if state.holding['hand'] != 'empty':
#             if state.pos['robot'] == loc:
#                 return [('put', state.holding['hand'], loc)]
#             else:
#                 return [('move', loc), ('put', state.holding['hand'], loc)]

#     return False


# gtpyhop.declare_task_methods('put', m_put)


# open method
def m_open(state, item, loc):
    # print("in open function")
    if is_a(state, item, 'switchable_item'):
        if state.status[item] == 'close':
            print("in close")
            if state.pos[item] == state.pos['robot']:
                return [('open', item)]
            else:
                return [('move', state.pos[item]), ('open', item)]

    return False


# gtpyhop.declare_task_methods('open', m_open)


# open method
def m_close(state, item, loc):
    if is_a(state, item, 'switchable_item'):
        if state.status[item] == 'open':
            if state.pos[item] == state.pos['robot']:
                return [('close', item)]
            else:
                return [('move', state.pos[item]), ('close', item)]

    return False


# gtpyhop.declare_task_methods('close', m_close)


# pour method
def m_pour(state, item, loc):
   # print(state.holding['hand'], "1111111")
    if state.holding['hand'] != 'empty':
        if is_a(state, state.holding['hand'], 'pourable_item'):
            if state.pos['robot'] == loc:
                return [('pour', state.holding['hand'], loc)]
            else:
                return [('move', loc), ('pour', state.holding['hand'], loc)]

    return False


gtpyhop.declare_task_methods('Home_service', m_close, m_open, m_pour, m_put_item_to_loc)
