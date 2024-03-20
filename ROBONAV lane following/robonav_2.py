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
        self.drive_factor = 1 # how much pull to steer back
        self.adherance = 0.1 # how tight to steer
        self.angle_threshold = 5
        self.blob_size = 1000

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

    def scan_for_blob(self, threshold_idx: int, step=2, limit=20): # Returns biggest blob
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
            biggest_blob = self.cam.get_biggest_blob(blobs)
            if biggest_blob is not None:
                if biggest_blob[8] == pow(2, threshold_idx):
                    print("found correct blob")
                    if biggest_blob.pixels() > self.blob_size:
                        print("blob in centre-ish")
                        break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                self.scan_direction *= -1

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
            biggest_blob = self.cam.get_biggest_blob(blobs)
            if biggest_blob is not None:
                if biggest_blob[8] == pow(2, threshold_idx):
                    print("found correct blob")
                    if self.cam.w_centre <= (blob.cx() + 20) and self.cam.w_centre >= (blob.cx() - 20):
                        print("blob in centre")
                        break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                self.scan_direction *= -1

    def scan_for_blob_good(self, threshold_idx: int, step=2, limit=44):
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees
        """
        count = 0
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
            biggest_blob = self.cam.get_biggest_blob(blobs)
            if biggest_blob is not None:
                if biggest_blob[8] == pow(2, threshold_idx):
                    print("found correct blob")
                    if biggest_blob.pixels() > self.blob_size:
                        print("blob in centre-ish")
                        break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                self.scan_direction *= -1
                count += 1
            if count > 1:
                self.servo.set_angle(0)
                #self.rotate(threshold_idx)
                break

    def rotate(self, thresh):
        print("looking for blob")
        while True:
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

            blobs, img = self.cam.get_blobs_bottom_thresh(thresh)
            blob = self.cam.get_biggest_blob(blobs)
            print(blob)


            if blob is not None:
                print("found blob")
                if blob.pixels() > self.blob_size:
                    print("blob in centre-ish")
                    break
            else:
                print("nu uh")
                print("rotating")
                self.servo.set_speed(0.2, -0.2)
                time.sleep_ms(30)
                self.servo.soft_reset_2()

    def track_n_rotate(self, thresh):
        print("tracking and rotating")
        while True:
            blobs, img = self.cam.get_blobs_bottom_thresh(thresh)
            blob = self.cam.get_biggest_blob(blobs)
            if blob is not None:
                angle = self.track_blob(blob)
                if self.servo.pan_pos < -5:
                    print("turning right")
                    self.servo.set_speed(0.2, -0.2)
                    time.sleep_ms(30)
                    self.servo.soft_reset_2()
                    #self.servo.set_speed(0, 0)
                    #time.sleep_ms(1000)
                    angle = self.track_blob(blob)
                    #time.sleep_ms(1000)

                elif self.servo.pan_pos > 5:
                    print("turning left")
                    self.servo.set_speed(-0.2, 0.2)
                    time.sleep_ms(30)
                    self.servo.soft_reset_2()
                    #self.servo.set_speed(0, 0)
                    #time.sleep_ms(1000)
                    angle = self.track_blob(blob)
                    #time.sleep_ms(1000)

                elif self.servo.pan_pos >= -10 and self.servo.pan_pos <= 10:
                    print("blob within pan thresh")
                    break
                #print(self.servo.pan_pos)
            else:
                print("no blob found")

    def move_to_blob(self, thresh, speed, bias):
        frames = 0
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            biggest_blob = self.cam.get_biggest_blob(blobs)
            if biggest_blob is not None:
                print("found a blob")
                if biggest_blob[8] == pow(2, thresh):
                    print('found correct blob')
                    self.servo.set_angle(0)
                    pixel_error = biggest_blob.cx() - self.cam.w_centre
                    steering = pixel_error / self.cam.w_centre
                    drive = steering / self.drive_factor
                    print('steering', steering, 'drive:', bias - drive)
                    if steering < self.adherance and steering > -self.adherance:
                        self.servo.set_differential_drive(speed, bias)
                    else:
                        self.servo.set_differential_drive(speed, bias - drive)
                else:
                    print("not correct blob")
                    frames += 1
                    print(frames)
                    if frames > 3:
                        print("stopping")
                        self.servo.soft_reset_2()
                        break

        #self.servo.soft_reset()
        #return

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
        self.servo.set_angle(pan_angle)

        return pan_angle

if __name__ == "__main__":
    import servos
    import camera

    thresholds = [(23, 75, 32, -14, -10, -51), #blue
                  (44, 12, 63, 32, 55, 15), #red
                  (37, 58, -54, -30, 21, 49), #green
                 (41, 75, -25, -6, 33, 57), #sun
                 ]

    gain = 25
    speed = 0.15
    bias = -0.2
    robonav = Robonav(thresholds, gain, speed, bias)
    #robonav.servo.set_angle(10)
    #print(robonav.servo.pan_pos)
    #robonav.turn_search()
    #robonav.scan_for_blob_og(0, 2, 44)
    #robonav.track_n_rotate(0)
    #robonav.move_to_blob(2, speed, bias)
    #robonav.rotate(0)
    #robonav.scan_blob_turn_2(0)
#    robonav.servo.set_differential_drive(0.2, 0)
#    time.sleep_ms(1000)
#    robonav.servo.soft_reset_2()



    while True:
        green_blobs, img = robonav.cam.get_blobs_bottom_thresh(2)

        if green_blobs is not None:
            print("found green")
            robonav.track_n_rotate(2)
            robonav.move_to_blob(2, speed, bias)
        else:
            blue_blobs, img = robonav.cam.get_blobs_bottom_thresh(0)
            if blue_blobs is not None:
                print("found blue")
                robonav.track_n_rotate(0)
                robonav.move_to_blob(0, speed, bias)
            else:
                print("blue or green not found")
                break

#        else:
#            robonav.scan_for_blob_good(2, 2, 44)
#            green_blobs, img = robonav.cam.get_blobs_bottom_thresh(2)
#            if green_blobs is not None:
#                print("here")
#                robonav.track_n_rotate(2)
#                robonav.move_to_blob(2, speed, bias)
#                break
#            else:
#                robonav.scan_for_blob_good(0, 2, 44)
#                robonav.rotate(0)
#                robonav.move_to_blob(0, speed, bias)



