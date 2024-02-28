from servos import *
from camera import *
from pid_control import PID
import time
import math
import sensor

class Robot(object):
    """
    A class to manage the functions of a robot for driving and tracking purposes using a camera and servos.
    """

    def __init__(self, thresholds, gain = 10, p=0.18, i=0, d=0, imax=0.01):
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
        self.servo = Servo()
        self.servo.soft_reset()
        self.cam = Cam(thresholds, gain)
        self.PID = PID(p, i, d, imax)

        # Blob IDs
        self.mid_line_id = 0
        self.obstacle_id = 1
        self.l_line_id = 2
        self.r_line_id = 3

        self.scan_direction = 1


    def stage1(self, speed: float, bias: float) -> None:
        """
        Line following algorithm

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        time.sleep_ms(1000)
        print('starting stage 1')
        frames = 0

        # Find the initial lane to follow
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            biggest_blob = self.cam.get_biggest_blob(blobs)
            biggest_blob_color = biggest_blob[8]

            print('check color')

            if biggest_blob_color == pow(2, 0):
                color_id = self.mid_line_id
                break
            elif biggest_blob_color == pow(2, 2):
                color_id = self.l_line_id
                break
            elif biggest_blob_color == pow(2, 3):
                color_id = self.r_line_id
                break

        print('color detected', color_id)

        # Following the lane
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            print('finding blobs of color', color_id)
            found_mid = self.cam.find_blob(blobs, color_id)

            if found_mid is not None:

                # Level 1
                print('found mid')
                self.servo.set_angle(0)
                pixel_error = blobs[found_mid].cx() - self.cam.w_centre
                steering = pixel_error / self.cam.w_centre
                steering = steering / 2
                print('steering', steering, 'drive:', steering + bias)
                if steering < 0.2 and steering > 0:
                    self.drive(speed, bias)
                else:
                    self.drive(speed, bias - steering)
            else:
                frames += 1
                if frames > 5:
                    self.drive(0, 0)
                    break

        self.servo.soft_reset()
        return


    def stage2(self, speed: float, bias: float) -> None:
        """
        Obstacle detection algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """

        time.sleep_ms(1000)
        print('starting stage 2')
        frames = 0

        # Find the initial lane to follow
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            biggest_blob = self.cam.get_biggest_blob(blobs)
            biggest_blob_color = biggest_blob[8]

            print('check color')

            if biggest_blob_color == pow(2, 0):
                color_id = self.mid_line_id
                break
            elif biggest_blob_color == pow(2, 2):
                color_id = self.l_line_id
                break
            elif biggest_blob_color == pow(2, 3):
                color_id = self.r_line_id
                break

        print('lane color detected', color_id)

        # Following the lane
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            print('finding blobs of color', color_id)
            found_mid = self.cam.find_blob(blobs, color_id)
            obstacle_detected = self.cam.find_blob(blobs, self.obstacle_id)

            # If obstacle not there, follow lane
            if obstacle_detected is None:
                if found_mid is not None:
                    print('following lane')
                    self.servo.set_angle(0)
                    pixel_error = blobs[found_mid].cx() - self.cam.w_centre
                    steering = pixel_error / self.cam.w_centre
                    steering = steering / 2
                    print('steering', steering, 'drive:', steering + bias)
                    if steering < 0.2 and steering > 0:
                        self.drive(speed, bias)
                    else:
                        self.drive(speed, bias - steering)
            # If obstacle, stop
            else:
                print('OBSTACLE!!')
                self.drive(0, 0)
                break
        return


    def stage3(self, speed: float, bias: float) -> None:
        """
        Obstacle distance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        self.servo.soft_reset()

        while True:
            blobs, img = self.cam.get_blobs_bottom()
            obstacle_detected = self.cam.find_blob(blobs, self.obstacle_id)



        return


    def stage4(self, speed: float, bias: float) -> None:
        """
        Obstacle distance + orientation algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        self.servo.soft_reset()
        return


    def stage5(self, speed: float, bias: float) -> None:
        """
        Obstacle avoidance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        self.servo.soft_reset()
        return


    def drive(self, drive: float, steering: float) -> None:
        """
        Differential drive function for the robot.

        Args:
            drive (float): Speed to set the servos to (-1~1)
            steering (float): Sets the steering to (-1~1)
        """
        # Apply limits
        self.servo.set_differential_drive(drive, steering)


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


    def scan_for_blob(self, threshold_idx: int, step = 2, limit = 20) -> None:
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
            self.servo.set_angle(new_pan_angle)
            print(new_pan_angle, self.servo.pan_pos, limit)

            # Check blobs to see if the line is found
            blobs, _ = self.cam.get_blobs_bottom()
            found_idx = self.cam.find_blob(blobs, threshold_idx)
            if found_idx:
                break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit:
                print('reverse')
                self.scan_direction = -1
            if self.servo.pan_pos <= -limit:
                print('reverse')
                self.scan_direction = 1


    def debug(self, threshold_idx: int) -> None:
        """
        A debug function for the Robots vision.
        If no block ID is specified, all blocks are detected and debugged.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
        """
        while True:
            blobs, img = self.cam.get_blobs()
            if threshold_idx is not None:
                found_idx = self.cam.find_blob(blobs, threshold_idx)
            else:
                found_idx = range(len(blobs))

            if found_idx:
                for blob in [blobs[i] for i in found_idx]:
                    img.draw_rectangle(blob.rect())
                    img.draw_string(blob.cx(),blob.cy(),str(blob.code()))

                    angle_err = blob.cx() - self.cam.w_centre

                    print('\n' * 2)
                    print('Code:       ', blob.code())
                    print('X-pos:      ',blob.cx())
                    print('Pan angle:  ', self.servo.pan_pos)
                    print('Angle err:  ', angle_err)
                    print('Angle corr: ', (angle_err-self.servo.pan_pos)/self.servo.max_deg)
                    print('Block size: ', blob.pixels())

                    time.sleep(1)


    def reset(self) -> None:
        """
        Resets the servo positions to their default states and waits.
        """
        self.servo.soft_reset()


    def release(self) -> None:
        """
        Release all servos (no wait).
        """
        self.servo.release_all()
