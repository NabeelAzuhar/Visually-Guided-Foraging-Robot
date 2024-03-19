import numpy as np
import detector as dt
import copy

class Localiser(object):
    def __init__(self, thresholds) -> None:
        self.global_map = np.zeros((6, 4))
        self.local_map = np.zeros((6, 3))
        self.local_map_row = 4
        self.local_map_movement = []
        self.front_id = 0
        self.left_id  = 1
        self.right_id = -1
        self.local = True
        self.robot_pos = [0, 0]

        self.blue_id = 0
        self.red_id = 1
        self.green_id = 2
        self.sun_id = 3
        self.outside_id = -1
        self.robot_id = 4

        self.bearing = 0    # anticlockwise from north
        self.sun_bearing = 0
        self.detector = dt.Detector(thresholds)


    def update_map(self, found_ids: list) -> None:
        """
        Update the map with the found_ids.
        """
        if self.local:
            self.update_local_map(found_ids)
        else:
            self.update_global_map(found_ids)


    def update_local_map(self, found_ids: list) -> None:
        """
        Scan the environment to find nearby objects and add them to a local map. If local map is near an edge then add it to global map.
        """
        
        # Look for nearby objects and add them to the local map
        self.local_map[self.local_map_row, 0] = found_ids[2] # Diagonal Left
        self.local_map[self.local_map_row, 1] = found_ids[0] # Front
        self.local_map[self.local_map_row, 2] = found_ids[1] # Diagonal Right

        # If local map is near an edge then add it to global map
        if self.outside_id in self.local_map:
            self.create_global_map()
            self.local = False
        else:
            self.local_map_row -= 1

    
    def update_global_map(self, found_ids: list) -> None:
        """
        Scan the environment to find nearby objects and add them to a global map.
        """
        # If facing north
        if self.bearing == 0:
            self.global_map[self.robot_pos[0] - 1, self.robot_pos[1] - 1] = found_ids[2]
            self.global_map[self.robot_pos[0] - 1, self.robot_pos[1]] = found_ids[0]
            self.global_map[self.robot_pos[0] - 1, self.robot_pos[1] + 1] = found_ids[1]

    
    def create_global_map(self) -> None:
        """
        Create a global map by adding local maps to it.
        """
        added_robot = False
        # Add most current row of local map into global map
        idx = np.where(self.local_map[self.local_map_row, :] == self.outside_id)
        # If local map is near the left edge
        if idx[0] == 0:
            start_col = 0
            self.global_map[self.local_map_row, start_col : start_col + 2] = self.local_map[self.local_map_row, 1:]
        # If local map is near the right edge
        if idx[0] == 2:
            start_col = 3
            self.global_map[self.local_map_row, start_col - 1 : start_col + 1] = self.local_map[self.local_map_row, :2]
        
        # Loop through rest of local map and add them into global map
        while True:
            # Initialise robot position in global map (1 row behind FOV and in FOV column)
            if not added_robot:
                self.robot_pos = [copy.copy(self.local_map_row + 1), start_col]
                added_robot = True
            # Finish creating global map if robot in start row
            if self.local_map_row >= 4:
                break
            self.local_map_row += 1
            direction = self.local_map_movement.pop()
            start_col += direction
            self.global_map[self.local_map_row, start_col - 1: start_col + 2] = self.local_map[self.local_map_row, :]

    def get_global_map(self):
        return self.global_map
                
        
        



    
    
    
        
    


