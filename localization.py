class Localization(object):
    def __init__(self, w_center, pan_pos):
        self.w_center = w_center
        self.pan_pos = pan_pos
        self.pos = [2, 0]
        self.my_dir = 0
        self.cur_grid = [0, 0, 0]
        self.next_grid = [0, 0, 0]
        self.last_grid = [0, 0, 0]
        self.destination = [0, 5]
        self.map = [[0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0 ,0, 0]]
        # 0 1 2 3
        # 1 - - -  y
        # 2 - - -  ^
        # 3 - - -  |
        # 4 - - -  |
        # 5 - - -  ----> x

        # 0 = space, 1 = obstacle, 2 = start, 3 = goal

    def bfs(self): # Returns: shortest distance, path taken
        start = None
        grid = self.map
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


    def get_next_grid(self, level):
        return map(level)

    def update_local_map(self, lane_mark, obstacle, pos):
        lane_block_size = 0
        obstacle_block_size = 0
        lane_cy = 0
        obs_cy = 0
        if lane_mark != None:
            center = self.w_centre + 20
            angle_err = lane_mark.cx() - center
            if angle_err < 80 or pos != 1:
                lane_block_size = lane_mark.pixels()
                lane_cy = lane_mark.cy()
#            print('\n' * 2)
#            print('Code:       ', lane_mark.code())
#            print('X-pos:      ',lane_mark.cx())
#            print('Pan angle:  ', self.servo.pan_pos)
#            print('Angle err:  ', angle_err)
#            print('Block size: ', lane_mark.pixels())
        if obstacle != None:
            center = self.w_centre + 20
            angle_err = obstacle.cx() - center
            if abs(angle_err) < 80 or pos != 1:
                obstacle_block_size = obstacle.pixels()
                obs_cy = obstacle.cy()
#            print('\n' * 2)
#            print('Code:       ', obstacle.code())
#            print('X-pos:      ',obstacle.cx())
#            print('Pan angle:  ', self.servo.pan_pos)
#            print('Angle err:  ', angle_err)
#            print('Block size: ', obstacle.pixels())
        # print('obs_size: ', obstacle_block_size)
        # print('lane_size: ', lane_block_size)
        if pos == 1:
            if obs_cy > lane_cy:
                self.next_grid[pos] = 1
            else:
                self.next_grid[pos] = 0

        else:
            if obstacle_block_size > lane_block_size:
                self.next_grid[pos] = 1
            else:
                self.next_grid[pos] = 0


    def update_my_pos(self, sun_blob):
        # According to current observation of sun, lane_mark and obstacles
        # stored in current_grid, next_grid, update my current postion and direction
        pass


    def get_my_next_step(self, level):
        # According to the global map and current position and direction,
        # return current next movemement toward the destination;
        # basically 5 directions
        #     2  3  4
        #      \ | /
        #     1--|--5
        pass
