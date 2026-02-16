import collections

def get_shortest_path(start, goal, obstacles, width, height):
    """
    Standard BFS to find the shortest path on a grid.
    Returns a list of coordinates [(x1, y1), (x2, y2), ...]
    """
    queue = collections.deque([(start, [start])])
    visited = {start}
    
    while queue:
        (x, y), path = queue.popleft()
        if (x, y) == goal:
            return path
        
        # 4-neighbors (UP, DOWN, LEFT, RIGHT)
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and \
               (nx, ny) not in obstacles and (nx, ny) not in visited:
                visited.add((nx, ny))
                queue.append(((nx, ny), path + [(nx, ny)]))
    return None

def manhattan_dist(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def move_recursive(u_idx, t, path, agent_positions, agents_at_dest, cell_to_agent):
    """
    Recursive Function: MOVE(u, t, pi, x, Ar)
    """
    next_v = path[t + 1]
    
    # Check if next_v is occupied by an agent w who is already at destination
    w_idx = cell_to_agent.get(next_v)
    
    if w_idx is not None and w_idx in agents_at_dest:
        # Recursive call: move the occupying agent first
        leader, next_t = move_recursive(w_idx, t + 1, path, agent_positions, agents_at_dest, cell_to_agent)
        
        # Update positions
        old_pos = agent_positions[u_idx]
        del cell_to_agent[old_pos]
        agent_positions[u_idx] = next_v
        cell_to_agent[next_v] = u_idx
        
        return leader, next_t
    else:
        # Move agent u to next_v
        old_pos = agent_positions[u_idx]
        if old_pos in cell_to_agent:
            del cell_to_agent[old_pos]
        
        agent_positions[u_idx] = next_v
        cell_to_agent[next_v] = u_idx
        
        return u_idx, t + 1

def extend_to_destination_set(agents, initial_config, destinations, obstacles, width, height):
    """
    Algorithm 1: Extension to Destination Set D
    
    :param agents: List of agent IDs
    :param initial_config: Dict {agent_id: (x, y)}
    :param destinations: Set of (x, y) coordinates
    :param obstacles: Set of (x, y) coordinates (from .map file)
    """
    x_pos = initial_config.copy()
    
    # A_r: agents already at a destination vertex
    # A_s: agents not yet at destination
    agents_at_dest = {i for i in agents if x_pos[i] in destinations}
    agents_to_move = [i for i in agents if i not in agents_at_dest]
    
    # Helper for quick lookup: cell -> agent_id
    cell_to_agent = {pos: i for i, pos in x_pos.items()}
    
    # Store all calculated paths
    all_trajectories = collections.defaultdict(list)

    while agents_to_move:
        # Phase 1: Path Selection
        # V_free = D \ {x(i) for i in A_r}
        occupied_destinations = {x_pos[i] for i in agents_at_dest}
        v_free = list(destinations - occupied_destinations)
        
        # Find (j, d) = argmin dist(x(a), v)
        best_dist = float('inf')
        selected_agent = None
        selected_dest = None
        
        for a_idx in agents_to_move:
            for d_v in v_free:
                d = manhattan_dist(x_pos[a_idx], d_v)
                if d < best_dist:
                    best_dist = d
                    selected_agent = a_idx
                    selected_dest = d_v
        
        # Shortest Path calculation
        path = get_shortest_path(x_pos[selected_agent], selected_dest, obstacles, width, height)
        
        if not path:
            print(f"Warning: No path found for agent {selected_agent}")
            agents_to_move.remove(selected_agent)
            continue

        # Phase 2: Recursive Execution
        curr_agent = selected_agent
        t = 0
        path_length = len(path) - 1
        
        while t < path_length:
            # We record the state before each move if needed for visualization
            # For now, we execute the recursive move
            curr_agent, t = move_recursive(curr_agent, t, path, x_pos, agents_at_dest, cell_to_agent)
        
        # Update sets
        agents_at_dest.add(selected_agent)
        agents_to_move.remove(selected_agent)
        
    return x_pos