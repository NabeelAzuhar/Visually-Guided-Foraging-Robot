import servos
import camera
import math
import time
import sensor
from pid_control import PID
import sys

class Robonav(object):
    def __init__(self, thresholds, gain, speed, bias, p = 0.1, i = 0, d = 0.005, imax = 0.01) -> None:
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
        self.cy_threshold = 200

        self.PID = PID(p, i, d, imax)

        self.servo = servos.Servo()
        self.cam = camera.Cam(thresholds, self.gain)

        self.servo.soft_reset()

        self.l_speed = -speed
        self.r_speed = speed

        self.thresholds = thresholds
        self.blue_id = 0
        self.red_id = 1
        self.green_id = 2
        self.sun_id = 3


    def scan_for_blob_good_2(self, blue_id: int, green_id: int, step=2, limit=44):
        """
        Scans left and right with the camera to find the blob.

        Args:
            blue_id (int): Id of blue blob
            green_id (int): Id of green blob
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees
        """

        count = 0  # Counts number of times direction is changed

        while True:

            # Draw rectangles
            img = sensor.snapshot()
            for blob in img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150,
                                       roi=(0,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
                img.draw_rectangle(blob.rect())

                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_keypoints(
                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                )

            # Update pan angle based on the scan direction and speed (left first)
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)

            # Set new angle
            self.servo.set_angle(new_pan_angle)

            # Check blobs to see if the line is found
            blobs, _ = self.cam.get_blobs_bottom()
            current = 0
            closest_blob = None
            for blob in blobs:
                if (blob[8] == pow(2, blue_id) or blob[8] == pow(2, green_id)) and blob.cy() > self.cy_threshold:
                    if blob.cy() > current:
                        closest_blob = blob
                        current = blob.cy()

            # Check if biggest blob was found.
            if closest_blob is not None:
                print("blob cy acceptable")
                return 1

            # Check if limits are reached and reverse direction
            # Reached left limit
            if self.servo.pan_pos >= limit:  # left
                self.scan_direction *= -1
                count += 1
                # Return 2 if nothing is seen when turned left (seeing edge of grid)
                blobs, _ = self.cam.get_blobs_bottom()
                if blobs == []:
                    self.servo.set_angle(0)
                    return 2
            # Reached right limit
            elif self.servo.pan_pos <= -limit:  # right
                self.scan_direction *= -1
                count += 1
                # Return 3 if nothing is seen when turned right (seeing edge of grid)
                blobs, _ = self.cam.get_blobs_bottom()
                if blobs == []:
                    self.servo.set_angle(0)
                    return 3
            # If looked left and right and didn't find anything, stop scanning
            if count > 1:
                self.servo.set_angle(0)
                break


    def rotate_2(self, blue_id, green_id, direction):
        """
        Rotates robot in a certain direction

        Args:
            blue_id (int): Id of blue blob
            green_id (int): Id of green blob
            direction (int): Direction to turn in. Left: -1, Right: 1
        """

        while True:

            # Draw rectangles
            img = sensor.snapshot()
            for blob in img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150,
                                       roi=(0,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
                img.draw_rectangle(blob.rect())

                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_keypoints(
                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                )

            # Check blobs to see if the line is found
            blobs, _ = self.cam.get_blobs_bottom()
            current = 0
            closest_blob = None
            for blob in blobs:
                if (blob[8] == pow(2, blue_id) or blob[8] == pow(2, green_id)) and blob.cy() > self.cy_threshold:
                    if blob.cy() > current:
                        closest_blob = blob
                        current = blob.cy()

            # If blob is found
            if closest_blob is not None:
                print("found a blob")
                break

            # If any of the conditions fail, continue rotating
            print("blue or green not found - rotating")
            self.servo.set_speed(0.2*direction, -0.2*direction)
            time.sleep_ms(50)
            self.servo.set_speed(0, 0)


    def track_n_rotate(self, thresh):
        """
        Track the blob of certain threshold index and rotate towards it until pan angle is 0.

        Args:
            thresh (int): The threshold index of the blob to be tracked and rotated to.
        """

        frame = 0  # Frames to continue looking for blob
        while True:

            # Draw rectangle
            img = sensor.snapshot()
            for blob in img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150,
                                       roi=(0,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
                img.draw_rectangle(blob.rect())

                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_keypoints(
                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                )

            # Finding the biggest blob of that colour
            blobs, img = self.cam.get_blobs_bottom_thresh(thresh)
            blob = self.cam.get_biggest_blob(blobs)

            # If blob is found
            if blob is not None:

                # Record angle of camera
                angle = self.track_blob(blob)

                # If camera is facing right
                if self.servo.pan_pos < -5:
                    print("turning right")
                    self.servo.set_speed(0.2, -0.2)
                    time.sleep_ms(30)
                    self.servo.set_speed(0, 0)
                    time.sleep_ms(50)
                    angle = self.track_blob(blob)

                # If camera is facing left
                elif self.servo.pan_pos > 5:
                    print("turning left")
                    self.servo.set_speed(-0.2, 0.2)
                    time.sleep_ms(30)
                    self.servo.set_speed(0, 0)
                    time.sleep_ms(50)
                    angle = self.track_blob(blob)

                # If camera is now within pan threshold, robot is oriented towards the blob. End function
                else:
                    print("blob within pan thresh")
                    break

            else:
                print("no blob found! ", frame)
                frame += 1
                if frame > 3:
                    break


    def move_to_blob(self, thresh, speed, bias):
        """
        Move to the blob of a specified colour.

        Args:
            thresh (int): The threshold index of the blob to be moved to.
            speed (float): speed of movement
            bias (float): bias of the robot
        """

        self.servo.set_angle(0)  # set angle straight
        frames = 0  # frames to continue moving for

        # Setting threshold frames to continue moving for depending on the colour
        if thresh == green_id:
            count_th = 4


        while True:

            # Find biggest blob of that colour
            blobs, img = self.cam.get_blobs_bottom_thresh(thresh)
            biggest_blob = self.cam.get_biggest_blob(blobs)

            # If no blob found and looking for blue, stop and exit function
            if biggest_blob is None and thresh == blue_id:
                print('stopping with blue')
                self.servo.set_differential_drive(0, 0)
                break  # End function

            # Draw rectangles
            img = sensor.snapshot()
            for blob in img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150,
                                       roi=(0,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
                img.draw_rectangle(blob.rect())

                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_keypoints(
                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                )

            # If biggest blob is found
            if biggest_blob is not None:
                print('found the correct blob')
                # Calculate the error and steering using PID
                pixel_error = biggest_blob.cx() - self.cam.w_centre
                bias_error = -(pixel_error/sensor.width()*self.cam.h_fov)
                steering = self.PID.get_pid(bias_error,0.15)
                # Steer robot towards the blob
                self.servo.set_differential_drive(speed, bias + steering)

            # If blob not found
            else:
                frames += 1
                print("blob was not found", frames)
                if frames > count_th:
                    print("stopping")
                    self.servo.set_differential_drive(0, 0)
                    break  # End function


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

    thresholds = [
                  (25, 54, -20, 10, -32, -10), #blue
                  (19, 37, 24, 54, 16, 45), #red
                  (30, 54, -32, -15, -1, 27), #green
    #                  (67, 82, 19, 62, -7, 39), #sun
                 ]

    gain = 3
    speed = 0.12
    bias = 0
    direction = 1 # 1:R, -1:L

    blue_id = 0
    red_id = 1
    green_id = 2
    sun_id = 3

    robonav = Robonav(thresholds, gain, speed, bias)
    servo = servos.Servo()

    try:
        while True:

            # If we ever decide to use the sun
    #        sun_blobs, img = robonav.cam.get_blobs_thresh(sun_id)
    #        if sun_blobs:
    #            biggest_blob = robonav.cam.get_biggest_blob(sun_blobs)
    #            if biggest_blob is not None:
    #                print("found a sun blob")
    #                pixel_error = biggest_blob.cx() - robonav.cam.w_centre
    #                if pixel_error > 0:
    #                    direction = -1
    #                else:
    #                    direction = 1
    #        print(direction)

            # Looking for green straight ahead
            green_blobs, img = robonav.cam.get_blobs_bottom_thresh(green_id)  # find green blobs
            if green_blobs:
                print("found green")
                print('TRACK AND ROTATE')
                robonav.track_n_rotate(green_id)
                print('MOVE TO BLOB')
                robonav.move_to_blob(green_id, speed, bias)
                break  # Reached green so end function

            # If no green, look for blue straight ahead
            else:
                blue_blobs, img = robonav.cam.get_blobs_bottom_thresh(blue_id)  # find blue blobs
                # If blue found, move to blue
                if blue_blobs:
                    print("found blue")
                    print('TRACK AND ROTATE')
                    robonav.track_n_rotate(blue_id)
                    print('MOVE TO BLOB')
                    robonav.move_to_blob(blue_id, speed, bias)

                # If no green or blue straight ahead, scan
                else:
                    print('SCANNING')
                    turn_param = robonav.scan_for_blob_good_2(blue_id, green_id, 2, 44)  # Scan

                    # If didnt find anything (it's in a corner), just rotate robot
                    if turn_param is None:
                        print('ROTATE 2')
                        robonav.rotate_2(blue_id, green_id, direction)
                        continue

                    # Found blue or green marker
                    elif turn_param == 1:
                        continue

                    # If edge of grid detected on left, rotate robot right
                    elif turn_param == 2:
                        # turn right
                        print('ROTATE 2')
                        robonav.rotate_2(blue_id, green_id, 1)
                        continue

                    # If edge of grid detected on right, rotate robot left
                    elif turn_param == 3:
                        # turn left
                        print('ROTATE 2')
                        robonav.rotate_2(blue_id, green_id, -1)
                        continue

    except Exception as e:
        print(e)
        servo.soft_reset()



