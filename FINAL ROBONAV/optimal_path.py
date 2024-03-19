# Define grid dimensions and fill it with values
grid = [
    [3, 1, 2, 1],  # Furthest row (goal)
    [1, 1, 2, 1],
    [2, 2, 1, 1],
    [1, 1, 1, 1],
    [1, 2, 1, 1],
    [1, 1, 1, 1]   # Nearest row (start)
]

# Define possible movements (including diagonals)
movements = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]

def heuristic(a, b):
    """
    Calculate the heuristic cost between two positions.
    
    Args:
        a (tuple): Coordinates of position a (row, column).
        b (tuple): Coordinates of position b (row, column).
    
    Returns:
        int: Heuristic cost between positions a and b.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    """
    Find the optimal path from start to goal using A* algorithm.
    
    Args:
        grid (list): 2D list representing the grid with values.
        start (tuple): Coordinates of the start position (row, column).
        goal (tuple): Coordinates of the goal position (row, column).
    
    Returns:
        list: List of coordinates representing the optimal path from start to goal.
    """
    frontier = []   # Frontier queue to store positions to be explored
    frontier.append(start)   # Add start position to the frontier
    came_from = {}   # Dictionary to store parent positions for each position
    cost_so_far = {}   # Dictionary to store the cost of reaching each position from the start
    came_from[start] = None   # Start position has no parent
    cost_so_far[start] = 0   # Cost of reaching start position is 0

    while frontier:
        current = frontier.pop(0)   # Get the next position from the frontier

        if current == goal:
            break   # Stop if the goal position is reached

        # Explore all possible movements from the current position
        for dx, dy in movements:
            next_pos = (current[0] + dx, current[1] + dy)   # Calculate next position
            # Check if the next position is within the grid boundaries
            if 0 <= next_pos[0] < len(grid) and 0 <= next_pos[1] < len(grid[0]):
                new_cost = cost_so_far[current] + grid[next_pos[0]][next_pos[1]]   # Calculate new cost
                # Check if the next position is accessible and has lower cost
                if grid[next_pos[0]][next_pos[1]] != 2 and (next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]):
                    cost_so_far[next_pos] = new_cost   # Update cost of reaching next position
                    priority = new_cost + heuristic(goal, next_pos)   # Calculate priority
                    frontier.append(next_pos)   # Add next position to the frontier
                    came_from[next_pos] = current   # Update parent position for next position

    # Reconstruct the optimal path from start to goal
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

# Define start and goal positions
start = (0, 3)  # Start position (row, column)
goal = (0, 0)   # Goal position (row, column)

# Find optimal path
optimal_path = astar(grid, start, goal)
print("Optimal path:", optimal_path)
