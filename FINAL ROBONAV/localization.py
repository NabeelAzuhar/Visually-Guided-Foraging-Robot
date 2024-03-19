import detector as dt

class Localiser(object):
    def __init__(self, thresholds) -> None:
        self.global_map_created = False
        self.global_map = [[0] * 4 for _ in range(6)]  # 6x4 array representing the grid
        self.local_map = []  # contains all instances of what the robot sees in front of it [diagonal left, front, diagonal right]
        
        self.fov_row = 4
        self.robot_col = -1
        self.direction_tracker = []  # contains the direction the robot moved to get to the local map
        self.direction_label = {
            "left": 2,
            "diag_left": 1,
            "forwards": 0,
            "diag_right": -1,
            "right": -2,
        }

    
    def update_map(self, found_ids: list) -> None:
        """
        Update the local map or global map (if already created) with the found_ids.
        Args:
            found_ids (list): List of found ids [front, diagonal left, diagonal right].
        """
        if self.global_map_created:
            self.update_global_map(found_ids)
        else:
            self.update_local_map(found_ids)

    
    def update_global_map(self, found_ids: list) -> None:
        """
        Update the global map with the found_ids.
        Args:
            found_ids (list): List of found ids [front, diagonal left, diagonal right].
        """
        map = [found_ids[1], found_ids[0], found_ids[2]]  # [diagonal left, front, diagonal right]
        self.global_map[self.fov_row][self.robot_col - 1 : self.robot_col + 2] = map  # Add the new map to the global map

    
    def update_local_map(self, found_ids: list) -> None:
        """
        Update the local map with the found_ids.
        Args:
            found_ids (list): List of found ids [front, diagonal left, diagonal right].
        """
        new_map = [found_ids[1], found_ids[0], found_ids[2]]  # [diagonal left, front, diagonal right]
        self.local_map.append(new_map)  # Add the new map to the local map
        
        # If edge (id = -1) is in the new map then create a global map
        if -1 in new_map:
            self.create_global_map(new_map.index(-1))
            self.global_map_created = True


    def create_global_map(self, edge_idx) -> None:
        """
        Create a global map by adding local maps to it.
        Args:
            edge_idx (int): Index of the edge (-1) in the local map.
        """
        map = self.local_map.pop()  # Get the most recent local map
        # If local map is near the left edge
        if edge_idx == 0:
            robot_col = 0
            self.global_map[self.fov_row][robot_col : robot_col + 2] = map[1:]  # Add the front and diagonal right to the global map
        # If local map is near the right edge
        elif edge_idx == 2:
            robot_col = 3
            self.global_map[self.fov_row][robot_col - 1 : robot_col + 1] = map[:2]  # Add the diagonal left and front to the global map

        # Loop through rest of local map and add them into global map
        row = self.fov_row  # Start at the row the robot is currently looking at
        while len(self.local_map) > 0:
            row -= 1  # Move up a row (Move down the self.global_map)
            direction = self.direction_tracker.pop()  # Get the direction the robot moved last
            
            if direction in ["diag_left", "forwards", "diag_right"]:
                col_change = self.direction_label[direction]  # Get the column change
                robot_col += col_change  # Update the start column
            else:
                # TODO - Add a way to handle left and right movements
                print("Robot moved left or right")
                pass

            self.global_map[row][robot_col - 1: robot_col + 2] = map  # Add the local map to the global map
        
        self.robot_col = robot_col  # Update the robot's column position to reflect where the robot is now
                
        
        



    
    
    
        
    


