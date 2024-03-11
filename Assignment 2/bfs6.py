def bfs(grid):
    start = None
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 3:
                goal = (i, j)
            if grid[i][j] == 2:
                start = (i, j)

    if start is None:
        return -1, None  # No path found

    queue = [(start[0], start[1], 0)]
    visited = set([start])

    directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]
    parent = {}  

    while queue:
        x, y, dist = queue.pop(0)
        if grid[x][y] == 3:  # Found the goal
            path = [goal]
            current_cell = goal
            while current_cell != start:
                current_cell = parent[current_cell]
                path.insert(0, current_cell)  
            return dist, path 

        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            if (0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and 
                grid[new_x][new_y] != 1 and (new_x, new_y) not in visited):

                queue.append((new_x, new_y, dist + 1)) 
                visited.add((new_x, new_y))
                parent[(new_x, new_y)] = (x, y) 

    return -1, None  # No path found


def get_directions(path):
    directions = []
    for i in range(1, len(path)):
        prev_cell = path[i - 1]
        current_cell = path[i]
        dx = current_cell[0] - prev_cell[0]
        dy = current_cell[1] - prev_cell[1]

        if dx == 1 and dy == 0:
            directions.append("DOWN")
        elif dx == -1 and dy == 0:
            directions.append("UP")
        elif dx == 0 and dy == 1:
            directions.append("RIGHT")
        elif dx == 0 and dy == -1:
            directions.append("LEFT")
        elif dx == 1 and dy == 1:
            directions.append("DOWN RIGHT")
        elif dx == -1 and dy == 1:
            directions.append("UP RIGHT")
        elif dx == 1 and dy == -1:
            directions.append("DOWN LEFT")
        elif dx == -1 and dy == -1:
            directions.append("UP LEFT")

    return directions

### Robo commands


def down():
    return

def up():
    return

def right():
    return

def left():
    return

def down_right():
    return

def down_left():
    return

def up_right():
    return

def up_left():
    return
###

grid = [
    [1, 2, 1, 0], # x is downward
    [0, 1, 1, 0], # y is across
    [1, 0, 1, 0],
    [0, 1, 1, 0],
    [0, 1, 1, 0],
    [1, 0, 3, 1]
] # 0 = space, 1 = obstacle, 2 = start, 3 = goal

result = bfs(grid)
if result != -1:
    print(f"Optimal Path Found! Distance: {result}")
    print(result[1])
else:
    print("No path found")
    
