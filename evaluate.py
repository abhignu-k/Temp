import problem 
from node import Node
from priority_queue import PriorityQueue
import time

class SearchTimeOutError(Exception):
    pass

def compute_g(algorithm, node, goal_state):
    """
        Evaluates the g() value.

        Parameters
        ===========
            algorithm: str
                The algorithm type based on which the g-value will be computed.
            node: Node
                The node whose g-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The g-value for the node.
    """

    if algorithm == "bfs":
        return node.get_depth()

    if algorithm == "astar":

        return NotImplementedError()

    elif algorithm == "gbfs":

        return NotImplementedError()

    elif algorithm == "ucs":

        return NotImplementedError()

    elif algorithm == "custom-astar":

        return NotImplementedError()

    # Should never reach here.
    assert False
    return float("inf")


def compute_h(algorithm, node, goal_state):
    """
        Evaluates the h() value.

        Parameters
        ===========
            algorithm: str
                The algorithm type based on which the h-value will be computed.
            node: Node
                The node whose h-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The h-value for the node.
    """

    if algorithm == "bfs":
        return 0

    if algorithm == "astar":

        return NotImplementedError()

    elif algorithm == "gbfs":

        return NotImplementedError()

    elif algorithm == "ucs":
        
        return NotImplementedError()

    elif algorithm == "custom-astar":

        return NotImplementedError()

    # Should never reach here.
    assert False
    return float("inf")


def get_manhattan_distance(from_state, to_state):
    return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)


def get_custom_heuristic(from_state, to_state):
    
    return NotImplementedError()

def graph_search(algorithm, time_limit):
    """
        Performs a search using the specified algorithm.
        
        Parameters
        ===========
            algorithm: str
                The algorithm to be used for searching.
            time_limit: int
                The time limit in seconds to run this method for.
                
        Returns
        ========
            tuple(list, int)
                A tuple of the action_list and the total number of nodes
                expanded.
    """
    
    # The helper allows us to access the problem functions.
    helper = problem.Helper()
    
    # Get the initial and the goal states.
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()[0]
    print(init_state)
    print(helper.get_goal_state())

    #Initialize the init node of the search tree and compute its f_score
    init_node = Node(init_state, None, 0, None, 0)
    f_score = compute_g(algorithm, init_node, goal_state) \
        + compute_h(algorithm, init_node, goal_state)

       
    # Initialize the fringe as a priority queue.
    visited={}
    priority_queue = PriorityQueue()
    priority_queue.push(f_score, init_node)
    visited[init_node.get_state()]=1
    

    # action_list should contain the sequence of actions to execute to reach from init_state to goal_state
    action_list = []

    # total_nodes_expanded maintains the total number of nodes expanded during the search
    total_nodes_expanded = 0
    time_limit = time.time() + time_limit
    while not priority_queue.is_empty() and (time.time() < time_limit):
        current_node = priority_queue.pop()
        total_nodes_expanded += 1

        if (goal_state.x,goal_state.y)==(current_node.get_state().x,current_node.get_state().y):
            while current_node.get_action() is not None:
                action_list.append(current_node.get_action())
                current_node = current_node.get_parent()
            action_list.reverse()
            return action_list, total_nodes_expanded
        
        succ_dict=helper.get_successor(current_node.get_state())
        for action in succ_dict:
            successor_node = Node(succ_dict[action][0], current_node, current_node.get_depth() + 1, action, succ_dict[action][1])
            f_score = compute_g(algorithm, successor_node, goal_state) + compute_h(algorithm, successor_node, goal_state)

            if successor_node.get_state() in visited:
                continue
            visited[successor_node.get_state()]=1
            priority_queue.push(f_score, successor_node)

    
    '''
    YOUR CODE HERE

    Remove "raise NotImplementedError() and add your code.

    Your code for graph search should populate action_list and set total_nodes_expanded
    The automated script will verify their values

    In addition to this you must also write code for:
    1. compute_g
    2. compute_h
    '''

    if time.time() >= time_limit:
    
        raise SearchTimeOutError("Search timed out after %u secs." % (time_limit))

    return action_list, total_nodes_expanded