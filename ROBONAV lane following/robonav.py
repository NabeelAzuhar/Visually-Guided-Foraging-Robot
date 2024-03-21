import servos
import camera
import math
import time
import sensor
from pid_control import PID

class Robonav(object):
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
        self.scan_direction = 1
        self.speed = speed
        self.bias = bias
        self.gain = gain
        self.drive_factor = 1.75 # how much pull to steer back
        self.adherance = 0.1 # how tight to steer
        self.angle_threshold = 5

        self.PID = PID(p, i, d, imax)

        self.servo = servos.Servo()
        self.cam = camera.Cam(thresholds)

        self.servo.soft_reset()
        self.servo.set_angle(0)

        self.l_speed = -speed
        self.r_speed = speed

        self.thresholds = thresholds
        self.blue_id = 0
        self.red_id = 1
        self.green_id = 2
        self.sun_id = 3

    def turn_body_search(self):
        self.servo.set_speed(self.l_speed, self.r_speed)

        while True:
            blobs, img = self.cam.get_blobs_bottom()
            big_blob = self.cam.get_biggest_blob(blobs)

            img = sensor.snapshot()
            for blob in img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150,
                                       roi=(0,int(sensor.height()/2),int(sensor.width()),int(2*sensor.height()/3))):
                img.draw_rectangle(blob.rect())

                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_keypoints(
                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                )

            if big_blob is not None:
                blob_colour = big_blob[8]
                print(blob_colour)
                if blob_colour == pow(2, self.blue_id) or blob_colour == pow(2, self.green_id):
                    print("found blue or green")
                    print(self.cam.w_centre)
                    print(blob.cx())
                    if self.cam.w_centre <= (blob.cx() + 20) and self.cam.w_centre >= (blob.cx() - 20):
                        print("stopping")
                        self.servo.set_speed(0, 0)
                        break

    def scan_for_blob(self, threshold_idx: int, step=2, limit=20): # Returns biggest blob
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Maximum pan angle in either direction during the search
        """
        self.scan_direction = 1
        while True:
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)

            # Enforce limits
            new_pan_angle = max(-limit, min(new_pan_angle, limit))

            self.servo.set_angle(new_pan_angle)
            print(new_pan_angle)

            # Check for line
            blobs, _ = self.cam.get_blobs_bottom_thresh(threshold_idx)
            found_blob = self.cam.get_biggest_blob(blobs)
            if found_blob != None:
                print("yep")
                return found_blob

            # Check if limits are reached - no need to change direction here
            if new_pan_angle >= limit or new_pan_angle <= -limit:
                print("nope")
                self.scan_direction *= -1  # Reverse direction at the limit
                print(self.scan_direction)
                new_pan_angle = limit# - (step * 2)
                self.servo.set_angle(new_pan_angle)
                time.sleep_ms(500)

    def scan_for_blob_og(self, threshold_idx: int, step = 2, limit = 20) -> None:
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees
        """
        while True:
            # Update pan angle based on the scan direction and speed
            img = sensor.snapshot()
            for blob in img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150,
                                       roi=(0,int(sensor.height()/2),int(sensor.width()),int(2*sensor.height()/3))):
                img.draw_rectangle(blob.rect())

                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_keypoints(
                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                )
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)

            # Set new angle
            self.servo.set_angle(new_pan_angle)

            # Check blobs to see if the line is found
            blobs, _ = self.cam.get_blobs_bottom()
            found_idx = self.cam.find_blob(blobs, threshold_idx)
            print(found_idx)
            if found_idx == threshold_idx:
                print("yh")
                if self.cam.w_centre <= (blob.cx() + 5) and self.cam.w_centre >= (blob.cx() - 5):
                    break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                self.scan_direction *= -1


    def test(self):
        #self.scan_for_blob_og(0, 2, 44)
        blobs, img = self.cam.get_blobs_bottom_thresh(0)
        blob = self.cam.get_biggest_blob(blobs)
        if blob is not None:
            angle = self.track_blob(blob)
            while angle > self.angle_threshold or angle < -self.angle_threshold:
                if angle > 0:
                    self.servo.set_speed(-0.1, 0.1)
                elif angle < 0:
                    self.servo.set_speed(0.1, -0.1)
                else:
                    self.servo.set_speed(0, 0)
                blobs, img = self.cam.get_blobs_bottom_thresh(0)
                blob = self.cam.get_biggest_blob(blobs)
                if blob is not None:
                    angle = self.track_blob(blob)

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

if __name__ == "__main__":
    import servos
    import camera

    thresholds = [(35, 49, -19, 23, -51, -15), #blue
                  (25, 40, 37, 55, 29, 49), #red
                  (50, 74, -40, -20, 22, 54), #green
                  (41, 75, -25, -6, 33, 57), #sun
                 ]

    gain = 25
    speed = 0.05
    bias = 0
    robonav = Robonav(thresholds, gain, speed, bias)
    #robonav.turn_search()
    robonav.scan_for_blob_og(0, 2, 44)
    robonav.test()
