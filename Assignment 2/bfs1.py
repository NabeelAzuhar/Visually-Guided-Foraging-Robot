# Untitled - By: thegr - Thu Mar 7 2024

from collections import deque

def bfs(grid):
    start = None
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 2:
                start = (i, j)
                break

    if start is None:
        return -1  # No start point found

    queue = deque([(start[0], start[1], 0, None)])  # (x, y, distance, direction)
    visited = set([start])

    directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]
    direction_map = {
        (0, 1): "DOWN",
        (1, 0): "RIGHT",
        (0, -1): "UP",
        (-1, 0): "LEFT",
        (1, 1): "DOWN_RIGHT",
        (-1, 1): "UP_RIGHT",
        (1, -1): "DOWN_LEFT",
        (-1, -1): "UP_LEFT"
    }

    while queue:
            x, y, dist, prev_direction = queue.popleft()

            if grid[x][y] == 3 and dist != 0: # Found the goal!
            # Calculate final direction if needed
                current_direction = direction_map.get((x - prev_direction[0], y - prev_direction[1])) 
                if current_direction:
                    print(f"Step {dist}: Move {current_direction}")
                return dist

            print(f"Exploring cell: ({x}, {y})")
            grid[x][y] = 4  # Mark visited cells with a '4' (or another number)
            
            print(f"Queue: {queue}") # Add this line

            for dx, dy in directions:
                new_x, new_y = x + dx, y + dy
                if (0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and 
                    grid[new_x][new_y] != 1 and (new_x, new_y) not in visited):
                    
                    new_direction = (dx, dy) # Store the actual direction of movement
                    if prev_direction and not is_opposite(prev_direction, new_direction):
                        turn_direction = get_turn_direction(prev_direction, new_direction)
                        print(f"Step {dist}: Turn {turn_direction}")
                    else:
                        print(f"Step {dist}: Move FORWARD")  # Default is forward

                    queue.append((new_x, new_y, dist + 1, new_direction))
                    visited.add((new_x, new_y))

    return -1  # No path found


def get_turn_direction(prev_direction, new_direction):
    if is_opposite(prev_direction, new_direction): 
        return "Error: Cannot turn 180 degrees" # Handle 180-degree turns

    # Calculate relative angles
    angle_diff = ((new_direction[0] - prev_direction[0]), (new_direction[1] - prev_direction[1])) 

    if angle_diff in [(0, 1), (-1, 1), (-1, 0)]:  # Left 45-degree variants
        return "LEFT 45"
    elif angle_diff in [(0, -1), (1, -1), (1, 0)]: # Right 45-degree variants
        return "RIGHT 45"
    else: 
        return "Error: Invalid Turn" 

def is_opposite(direction1, direction2):
    return direction1[0] == -direction2[0] and direction1[1] == -direction2[1]


# Example usage
grid = [
  [2, 0, 1, 0],
  [0, 1, 1, 0],
  [1, 0, 1, 0],
  [0, 1, 0, 1],
  [0, 1, 1, 0],
  [0, 1, 3, 1]
]

result = bfs(grid)
print(grid)     # Print the grid
if result != -1:
    print(f"Shortest path distance: {result}")
else:
    print("No path found")
