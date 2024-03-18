import servos as servo
import camera
from pid_control import PID
import sensor
import time

class Detector(object):
    def __init__(self, thresholds, p=0.45, i=0.05, d=0.01, imax=0) -> None:
        """
        Args:
            p (float): Proportional gain for the PID.
            i (float): Integral gain for the PID.
            d (float): Derivative gain for the PID.
            imax (float): Maximum Integral error for the PID.
        """
        self.servo = servo.Servo()
        self.cam = camera.Camera(thresholds)
        self.PID = PID(p, i, d, imax)

        self.thresholds = thresholds
        self.blue_id = 0
        self.red_id = 1
        self.green_id = 2
        self.sun_id = 3
        self.outside_id = -1
        self.scan_direction = 1

        self.servo.soft_reset()
    
    
    def check_surroundings(self):
        """
        Scans the surroundings to find what the nearby objects are.
        """
        found_ids = []
        # Check front
        self.servo.set_angle(0)
        _, color_id = self.get_biggest_blob_colour()
        found_ids.append(color_id)
        # Check diagonal left
        self.servo.set_angle(30)
        _, color_id = self.get_biggest_blob_colour()
        found_ids.append(color_id)
        # Check diagonal right
        self.servo.set_angle(-30)
        _, color_id = self.get_biggest_blob_colour()
        found_ids.append(color_id)

        self.servo.set_angle(0)
        return found_ids
    

    def get_biggest_blob_colour(self):
            """
            Finds the biggest blob in the camera's field of view and returns the index of the threshold that detected it.
            """
            img = sensor.snapshot()
            blobs = img.find_blobs(self.thresholds,pixels_threshold=150,area_threshold=150,
                                roi=(1,int(sensor.height()/2),
                                        int(sensor.width()),int(2*sensor.height()/4)))
            
            biggest_blob = self.cam.get_biggest_blob(blobs)
            blob_colour = biggest_blob[8]
            for threshold_idx in range(len(self.thresholds)):
                if blob_colour == pow(2, threshold_idx):
                    return biggest_blob, threshold_idx
            return biggest_blob, self.outside_id

    def scan_for_blob(self, threshold_idx: int, step = 2, limit = 30) -> None:
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees
        """
        while True:
            # Update pan angle based on the scan direction and speed
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)

            # Set new angle
            self.servo.set_angle(new_pan_angle-5)
            print(new_pan_angle, self.servo.pan_pos, limit)

            # Check blobs to see if the line is found
            blobs, _ = self.cam.get_blobs_bottom()
            found_idx = self.cam.find_blob(blobs, threshold_idx)
            if found_idx:
                break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit:
                print('reverse to right')
                self.scan_direction = -1
            if self.servo.pan_pos <= -limit:
                print('reverse to left')
                self.scan_direction = 1

    def track_blob(self, blob) -> None:
        """
        Adjust the camera pan angle to track a specified blob based on its ID.

        Args:
            blob: The blob object to be tracked
        """
        # Error between camera angle and target in pixels
        pixel_error = blob.cx() - self.cam.w_centre

        # Convert error to angle
        angle_error = -(pixel_error/sensor.width()*self.cam.h_fov)

        pid_error = self.PID.get_pid(angle_error,1)

        # Error between camera angle and target in ([deg])
        pan_angle = self.servo.pan_pos + pid_error

        # Move pan servo to track block
        self.servo.set_angle(pan_angle-5)

        return pan_angle
