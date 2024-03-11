from collections import deque

def bfs(grid):
    start = None
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 2:
                start = (i, j)
                break

    if start is None:
        return -1  

    queue = deque([(start[0], start[1], 0)]) 
    visited = set([start])

    directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]
    parent = {}  # Key: (x, y) cell coordinates, Value: (parent_x, parent_y) 

    while queue:
        x, y, dist = queue.popleft()

        if grid[x][y] == 3:  # Found the goal
            return dist, parent

        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            if (0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and 
                grid[new_x][new_y] != 1 and (new_x, new_y) not in visited):

                queue.append((new_x, new_y, dist + 1))
                visited.add((new_x, new_y))
                parent[(new_x, new_y)] = (x, y)  # Record the parent

    return -1, None  # No path found

def reconstruct_path(parent, start, goal):
    path = [goal]
    current_cell = goal
    while current_cell != start:
        current_cell = parent[current_cell]
        path.insert(0, current_cell) 
    return path

def get_directions(path):
    agent_orientation = (0, 1)  # Assuming initial orientation facing down
    directions = []

    # Special case for the first step (handling initial obstacle)
    if len(path) > 1 and path[0][0] != path[1][0] and path[0][1] != path[1][1]:
        # Determine the direction of the first movement based on starting and next cell
        dx, dy = path[1][0] - path[0][0], path[1][1] - path[0][1]
        if dx > 0:  # Need to turn right first (assuming initial down orientation)
            directions.append("TURN RIGHT 45")
            agent_orientation = update_orientation(1, 0, agent_orientation)  # Update orientation
        else:  # Need to turn left first
            directions.append("TURN LEFT 45")
            agent_orientation = update_orientation(-1, 0, agent_orientation)  # Update orientation

    # Loop through the remaining path cells
    for i in range(1, len(path)):
        prev_cell = path[i - 1]
        current_cell = path[i]
        dx = current_cell[0] - prev_cell[0]
        dy = current_cell[1] - prev_cell[1]

        # Determine relative movement based on agent orientation
        relative_dx, relative_dy = rotate_direction(dx, dy, agent_orientation)

        # Update agent orientation based on relative movement 
        agent_orientation = update_orientation(relative_dx, relative_dy, agent_orientation)

        # Translate relative movement to agent instructions
        if relative_dx > 0:
            directions.append("FORWARD")
        elif relative_dx < 0:
            directions.append("REVERSE")
        elif relative_dy > 0:
            directions.append("TURN RIGHT 45")
        elif relative_dy < 0:
            directions.append("TURN LEFT 45")

    return directions

def rotate_direction(dx, dy, orientation):
    #  This logic depends on how your grid defines directions
    #  Here's an example assuming positive y is down:
    if orientation == (0, 1):  # Facing down (positive y)
        return dx, dy
    elif orientation == (1, 0):  # Facing right (positive x)
        return dy, -dx
    elif orientation == (0, -1):  # Facing up (negative y)
        return -dx, -dy
    else:  # Facing left (negative x)
        return -dy, dx

def update_orientation(dx, dy, orientation):
    print(orientation)
    if dx > 0:  # Moved right relative to agent
        if orientation == (0, 1):  # Was facing down
            return (1, 0)  # Now facing right
        elif orientation == (1, 0):  # Was facing right
            return (0, -1)  # Now facing up
        elif orientation == (0, -1):  # Was facing up
            return (-1, 0)  # Now facing left
        else:  # Was facing left
            return (0, 1)  # Now facing down 

    elif dx < 0:  # Moved left relative to agent
        # Similar logic as above, but rotating in the opposite direction
        if orientation == (0, 1):  # Was facing down
            return (-1, 0)  # Now facing left
        elif orientation == (1, 0):  # Was facing right
            return (0, 1)  # Now facing down
        elif orientation == (0, -1):  # Was facing up
            return (1, 0)  # Now facing right
        else:  # Was facing left
            return (0, -1)  # Now facing up

    elif dy > 0:  # Moved 'forward' relative to agent (down in our case)
        return orientation  # No change in orientation  

    elif dy < 0:  # Moved 'backward' relative to agent (up in our case)
        # Rotate 180 degrees
        return (-orientation[0], -orientation[1])  

    else:  # Didn't move 
        return orientation 

grid = [
    [1, 2, 1, 0],
    [0, 1, 1, 0],
    [0, 1, 1, 0],
    [1, 0, 0, 1],
    [0, 1, 1, 0],
    [0, 1, 3, 1]
] # 0 = space, 1 = obstacle, 2 = start, 3 = goal

result, path_info = bfs(grid)
if result != -1:
    optimal_path = reconstruct_path(path_info, (0, 1), (5, 2))  # start and goal
    print(f"Optimal Path Found! Distance: {result}")
    print(optimal_path)
    directions = get_directions(optimal_path)
    print("Directions:")
    print(directions) 
else:
    print("No path found")
    
