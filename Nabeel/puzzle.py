import detector as dt
import localization as lc
import servos
import math
import time

class Puzzle(object):
    def __init__(self, thresholds, gain, speed, bias, p = 0.15, i = 0, d = 0.005, imax = 0.01) -> None:
        """
        Initializes the Robot object with given PID parameters.

        Args:
            thresholds (list): Colour detection thresholds
            gain (float): Camera gain
            p (float): Proportional gain for the PID.
            i (float): Integral gain for the PID.
            d (float): Derivative gain for the PID.
            imax (float): Maximum Integral error for the PID.
        """
        
        self.found_goal = False
        self.speed = speed
        self.bias = bias
        self.angle_threshold = 5
        self.drive_factor = 1.75 # how much pull to steer back
        self.adherance = 0.1 # how tight to steer

        self.detector = dt.Detector(thresholds, gain)
        self.localizer = lc.Localizer(thresholds)
        self.servo = servos.Servo()

        self.servo.soft_reset()
        self.servo.set_angle(0)
    
    def solve(self) -> None:
        """
        Solves the puzzle by following the blue markers and avoiding red markers until it reaches a green marker
        """
        while True:  
            found_ids = self.detector.check_surroundings()  # Check FOV
            self.localizer.update_map(found_ids)  # Update map with FOV
            self.move_to_square(found_ids)  # Move to next square
            if self.found_goal:
                break
        print('Puzzle solved!')
            

    def move_to_square(self, found_ids: list) -> None:
        """
        Moves the robot to the next square

        Args:
            found_ids (list): List of the ids of the blobs found in the camera's field of view
        """
        if self.detector.green_id in found_ids:
                # Move to goal
                green_index = found_ids.index(self.detector.green_id)  # Get index of green marker
                self.move_direction(green_index)
        else:
            # Move to available square
            # Find index of first blue marker in found_ids
            blue_indices = [i for i, x in enumerate(found_ids) if x == self.detector.blue_id]
            if blue_indices:
                self.move_direction(blue_indices[0])
            else:
                self.go_back()

    def move_direction(self, direction: int) -> None:
        """
        Moves the robot in the direction of the index

        Args:
            found_ids (list): List of the ids of the blobs found in the camera's field of view
                Direction 0: Diagonal Left
                Direction 1: Forward
                Direction 2: Diagonal Right 
        """
        if direction == 0:
            self.servo.set_angle(30)
            self.move(self.speed, self.bias)
        elif direction == 1:
            self.servo.set_angle(0)
            self.move(self.speed, self.bias)
        elif direction == 2:
            self.servo.set_angle(-30)
            self.move(self.speed, self.bias)


    def move(self, speed, bias) -> None:
        """
        Moves the robot forward towards a blob

        Args:
            speed (float): Speed of robot
            bias (float): Bias of robot
        """
        # Following the lane
        while True:
            blob, blob_id = self.detector.get_biggest_blob_colour()

            if blob_id == self.blue_id or blob_id == self.green_id:
                print('found blue or green')
                
                # Orient the robot until camera angle is 0
                angle = self.detector.track_blob(blob)
                while angle > self.angle_threshold or angle < -self.angle_threshold:
                    if angle > 0:
                        self.servo.set_speed(-0.1, 0.1)
                    elif angle < 0:
                        self.servo.set_speed(0.1, -0.1)
                    else:
                        self.servo.set_speed(0, 0)
                    blob, blob_id = self.detector.get_biggest_blob_colour()
                    angle = self.detector.track_blob(blob)

                # Move to blob
                blob, blob_id = self.detector.get_biggest_blob_colour()
                if blob_id == self.blue_id or blob_id == self.green_id:    
                    distance = blob.cy()
                    current_distance = distance
                    while current_distance <= distance:
                        self.servo.set_angle(0)
                        pixel_error = blob.cx() - self.cam.w_centre
                        steering = pixel_error / self.cam.w_centre
                        drive = steering / self.drive_factor
                        print('steering', steering, 'drive:', bias - drive)
                        if steering < self.adherance and steering > -self.adherance:
                            self.drive(speed, bias)
                        else:
                            self.drive(speed, bias - drive)        
            

          
