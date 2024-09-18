def compute_g(algorithm, node, goal_state):
    if algorithm == "bfs":
        return node.path_cost  # For BFS, path cost might typically be the depth or number of steps from the root.
    elif algorithm == "ucs":
        return node.path_cost  # For UCS, path cost accumulates the step costs.
    elif algorithm == "astar":
        return node.path_cost  # A* also uses path cost, but it will add heuristic later.
    elif algorithm == "gbfs":
        return 0  # Greedy BFS doesn't account for the path cost in its priority queue.
    elif algorithm == "custom-astar":
        return node.path_cost  # Custom A* can modify how path cost is considered.
    else:
        raise NotImplementedError(f"G-value computation for {algorithm} is not implemented.")


def compute_h(algorithm, node, goal_state):
    if algorithm in ["astar", "gbfs", "custom-astar"]:
        # Implement heuristic based on problem specifics, e.g., Euclidean distance for spatial problems.
        return abs(node.state.x - goal_state.x) + abs(node.state.y - goal_state.y)
    else:
        return 0  # BFS and UCS do not use heuristics.


def graph_search(algorithm, time_limit):
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()

    init_node = Node(init_state, None, 0, None, 0)
    f_score = compute_g(algorithm, init_node, goal_state) + compute_h(algorithm, init_node, goal_state)

    priority_queue = PriorityQueue()
    priority_queue.push(f_score, init_node)

    action_list = []
    total_nodes_expanded = 0
    start_time = time.time()

    while not priority_queue.is_empty() and (time.time() - start_time < time_limit):
        current_node = priority_queue.pop()
        total_nodes_expanded += 1

        if helper.goal_test(current_node.state, goal_state):
            while current_node.parent is not None:
                action_list.append(current_node.action)
                current_node = current_node.parent
            action_list.reverse()
            return action_list, total_nodes_expanded

        for action, successor_state, cost in helper.successors(current_node.state):
            successor_node = Node(successor_state, current_node, current_node.path_cost + cost, action)
            f_score = compute_g(algorithm, successor_node, goal_state) + compute_h(algorithm, successor_node, goal_state)
            priority_queue.push(f_score, successor_node)

    if time.time() - start_time >= time_limit:
        raise SearchTimeOutError("Search timed out after {} seconds.".format(time_limit))

    return action_list, total_nodes_expanded
