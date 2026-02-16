import collections

def get_path(start, goal, obstacles, width, height):
    """Standard BFS pathfinding."""
    queue = collections.deque([(start, [start])])
    visited = {start}
    while queue:
        (x, y), path = queue.popleft()
        if (x, y) == goal:
            return path
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in obstacles and (nx, ny) not in visited:
                visited.add((nx, ny))
                queue.append(((nx, ny), path + [(nx, ny)]))
    return None

def get_cycle_path(x1, x0, excluded_edge, obstacles, width, height):
    """
    Finds a path from x1 back to x0 without using the edge (x0, x1).
    Used to form the cycle C = {(x0, x1)} + P'
    """
    queue = collections.deque([(x1, [x1])])
    visited = {x1}
    
    while queue:
        curr, path = queue.popleft()
        if curr == x0:
            return path
        
        x, y = curr
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = x + dx, y + dy
            # Check boundaries and obstacles
            if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in obstacles:
                # Crucial: verify we are not using the forbidden direct edge back
                if (curr == x1 and (nx, ny) == x0):
                    continue
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append(((nx, ny), path + [(nx, ny)]))
    return None

def rotate_agents_on_cycle(cycle, agent_positions, cell_to_agent):
    """
    Performs a synchronous rotation of agents along the directed cycle.
    Each agent at cycle[i] moves to cycle[i+1] (with cycle[-1] moving to cycle[0]).
    """
    # Identify which agents are involved
    # cycle is [x0, x1, x2, ..., x_n] where x_n connects to x0
    moving_agents = []
    for vertex in cycle:
        moving_agents.append(cell_to_agent.get(vertex))

    # Save original positions of agents in the cycle
    original_positions = {a_idx: agent_positions[a_idx] for a_idx in moving_agents if a_idx is not None}

    # Shift positions: agent at cycle[i] moves to cycle[i+1]
    for i in range(len(cycle)):
        a_idx = moving_agents[i]
        if a_idx is not None:
            next_vertex = cycle[(i + 1) % len(cycle)]
            agent_positions[a_idx] = next_vertex
    
    # Rebuild the cell_to_agent map for the consistency
    for pos in original_positions.values():
        if pos in cell_to_agent:
            del cell_to_agent[pos]
            
    for a_idx, pos in agent_positions.items():
        cell_to_agent[pos] = a_idx

def pivot_visit_algorithm(agents_to_task, initial_config, pivot_o, obstacles, width, height):
    """
    Algorithm: Each tasked agent reaches pivot 'o' via cycle rotations.
    """
    current_config = initial_config.copy()
    cell_to_agent = {pos: i for i, pos in current_config.items()}
    
    # Trajectory log if needed: all_configs = [initial_config.copy()]

    for agent_a in agents_to_task:
        x0 = current_config[agent_a]
        if x0 == pivot_o:
            continue
            
        # Path P from x0 to o
        path_to_pivot = get_path(x0, pivot_o, obstacles, width, height)
        
        if not path_to_pivot:
            print(f"Warning: Pivot unreachable for agent {agent_a}")
            continue

        # Move agent 'a' along the path P step by step
        for i in range(len(path_to_pivot) - 1):
            u = path_to_pivot[i]
            v = path_to_pivot[i+1] # This is x1 in the algorithm
            
            # Find the path P' to complete the cycle
            p_prime = get_cycle_path(v, u, (u, v), obstacles, width, height)
            
            if not p_prime:
                # This would only happen if the graph is not 2-edge-connected
                print(f"Graph is not 2-edge-connected at {u}-{v}")
                break
                
            # The full cycle is P' followed by the edge (u, v)
            # Simplest representation for rotation: p_prime starts at v, ends at u
            rotate_agents_on_cycle(p_prime, current_config, cell_to_agent)
            
            # Agent 'a' is now at v.
            
    return current_config