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
    
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()[0]
    
    # Initialize the init node of the search tree
    init_node = Node(init_state, None, 0, None, 0)
    f_score = compute_g(algorithm, init_node, goal_state) + compute_h(algorithm, init_node, goal_state)
    
    # Initialize the priority queue (or fringe)
    priority_queue = PriorityQueue()
    priority_queue.push(f_score, init_node)
    
    # Variables to store the result
    action_list = []
    total_nodes_expanded = 0
    end_time = time.time() + time_limit
    
    while not priority_queue.is_empty():
        if time.time() >= end_time:
            raise SearchTimeOutError("Search timed out after %u secs." % time_limit)
        
        current_node = priority_queue.pop()
        total_nodes_expanded += 1
        
        # Check if we reached the goal
        if current_node.get_state() == goal_state:
            # Backtrack to construct the path
            while current_node.get_parent() is not None:
                action_list.append(current_node.get_action())
                current_node = current_node.get_parent()
            action_list.reverse()
            return action_list, total_nodes_expanded
        
        # Expand the node
        for action, next_state in helper.get_successor_states(current_node.get_state()):
            child_node = Node(next_state, current_node, current_node.get_depth() + 1, action, current_node.get_total_action_cost() + 1)
            f_score = compute_g(algorithm, child_node, goal_state) + compute_h(algorithm, child_node, goal_state)
            priority_queue.push(f_score, child_node)
    
    return action_list, total_nodes_expanded


def compute_g(algorithm, node, goal_state):
    if algorithm == "bfs":
        return node.get_depth()  # In BFS, g(n) is just the depth
    elif algorithm == "ucs":
        return node.get_total_action_cost()  # UCS uses the cost of actions
    elif algorithm == "astar":
        return node.get_total_action_cost()  # A* also uses the cost of actions for g
    elif algorithm == "gbfs":
        return 0  # GBFS only cares about h, not g
    elif algorithm == "custom-astar":
        # Custom cost function, implement based on specific requirement
        return node.get_total_action_cost()
    else:
        raise ValueError(f"Unknown algorithm {algorithm}")


def compute_h(algorithm, node, goal_state):
    current_state = node.get_state()
    if algorithm == "bfs" or algorithm == "ucs":
        return 0  # BFS and UCS do not use heuristics
    elif algorithm == "gbfs" or algorithm == "astar":
        return get_manhattan_distance(current_state, goal_state)  # Heuristic using Manhattan distance
    elif algorithm == "custom-astar":
        return get_custom_heuristic(current_state, goal_state)  # Custom heuristic
    else:
        raise ValueError(f"Unknown algorithm {algorithm}")


def get_manhattan_distance(from_state, to_state):
    return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)
