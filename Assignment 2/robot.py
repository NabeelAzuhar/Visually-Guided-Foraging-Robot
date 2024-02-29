from servos import *
from camera import *
from pid_control import PID
import time
import math
import sensor
#import numpy as np

class Robot(object):
    """
    A class to manage the functions of a robot for driving and tracking purposes using a camera and servos.
    """

    def __init__(self, thresholds, gain = 10, p=0.45, i=0.05, d=0.01, imax=0):
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
        self.thresholds = thresholds

        # Blob IDs
        self.mid_line_id = 0
        self.obstacle_id = 1
        self.l_line_id = 2
        self.r_line_id = 3

        # Alignment
        self.adherance = 0.1 # how tight to steer
        self.drive_factor = 1.75 # how much pull to steer back
        self.distance_factor = 1080000 # fit factor for distance-pixel relationship

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
#        while True:
#            blobs, img = self.cam.get_blobs_bottom()
#            biggest_blob = self.cam.get_biggest_blob(blobs)
#            biggest_blob_color = biggest_blob[8]

#            print('check color')

#            if biggest_blob_color == pow(2, 0):
#                color_id = self.mid_line_id
#                break
##            if biggest_blob_color == pow(2, 2):
##                color_id = self.l_line_id
##                break
##            if biggest_blob_color == pow(2, 3):
##                color_id = self.r_line_id
##                break

#        print('color detected', color_id)

        blobs, img = self.cam.get_blobs_bottom()
        while True:
            for blob in img.find_blobs(self.thresholds, pixels_threshold=150, area_threshold=150,
                                       roi=(1,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
                img.draw_rectangle(blob.rect())

                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_keypoints(
                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                )
            biggest_blob = self.cam.get_biggest_blob(blobs)
            biggest_blob_color = biggest_blob[8]

            print('check color')

            if biggest_blob_color == pow(2, 1):
                blobs.remove(biggest_blob)
                print('removed obstacle')

            if biggest_blob_color == pow(2, 0):
                color_id = self.mid_line_id
                break

#            if biggest_blob_color == pow(2, 2):
#                color_id = self.l_line_id
#                break
#            if biggest_blob_color == pow(2, 3):
#                color_id = self.r_line_id
#                break

        print('lane color detected', color_id)

        # Following the lane
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            for blob in img.find_blobs(self.thresholds, pixels_threshold=150, area_threshold=150,
                                       roi=(1,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
                img.draw_rectangle(blob.rect())

                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_keypoints(
                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                )
            print('finding blobs of color', color_id)
            found_mid = self.cam.find_blob(blobs, color_id)

            if found_mid is not None:
                print('found lane')
                self.servo.set_angle(0)
                pixel_error = blobs[found_mid].cx() - self.cam.w_centre
                steering = pixel_error / self.cam.w_centre
                drive = steering / self.drive_factor
                print('steering', steering, 'drive:', bias - drive)
                if steering < self.adherance and steering > -self.adherance:
                    self.drive(speed, bias)
                else:
                    self.drive(speed, bias - drive)
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

        # Find the initial lane to follow
#        while True:
#            blobs, img = self.cam.get_blobs_bottom()
#            biggest_blob = self.cam.get_biggest_blob(blobs)
#            biggest_blob_color = biggest_blob[8]

#            print('check color')

#            if biggest_blob_color == pow(2, 0):
#                color_id = self.mid_line_id
#                break

#        print('lane color detected', color_id)

        blobs, img = self.cam.get_blobs_bottom()
        while True:
            for blob in img.find_blobs(self.thresholds, pixels_threshold=150, area_threshold=150,
                                       roi=(1,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
                img.draw_rectangle(blob.rect())

                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_keypoints(
                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                )
            biggest_blob = self.cam.get_biggest_blob(blobs)
            biggest_blob_color = biggest_blob[8]

            print('check color')

            if biggest_blob_color == pow(2, 1):
                blobs.remove(biggest_blob)
                print('removed obstacle')

            if biggest_blob_color == pow(2, 0):
                color_id = self.mid_line_id
                break

        print('lane color detected', color_id)

        # Following the lane
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            for blob in img.find_blobs(self.thresholds, pixels_threshold=150, area_threshold=150,
                                       roi=(1,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
                img.draw_rectangle(blob.rect())

                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_keypoints(
                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                )
            print('finding blobs of color', color_id)
            found_mid = self.cam.find_blob(blobs, color_id)
            obstacle_detected = self.cam.find_blob(blobs, self.obstacle_id)

            # If obstacle not there, follow lane
            if obstacle_detected is None:
                if found_mid is not None:
                    print('found lane')
                    self.servo.set_angle(0)
                    pixel_error = blobs[found_mid].cx() - self.cam.w_centre
                    steering = pixel_error / self.cam.w_centre
                    drive = steering / self.drive_factor
                    print('steering', steering, 'drive:', bias - drive)
                    if steering < self.adherance and steering > -self.adherance:
                        self.drive(speed, bias)
                    else:
                        self.drive(speed, bias - drive)
            # If obstacle, stop
            if obstacle_detected is not None:
                print('OBSTACLE!!')
                self.drive(0, 0)
                break

        self.servo.soft_reset()
        return


    def stage3(self, speed: float, bias: float, distance_threshold: float) -> None:
        """
        Obstacle distance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
            obstacle_size (float): height of object (cm)
            distance_threshold (float): how far from the object the robot should stop
        """

        self.servo.set_angle(0)
        time.sleep_ms(1000)
        print('starting stage 3')

        # Find the initial lane to follow
#        while True:
#            blobs, img = self.cam.get_blobs_bottom()
#            biggest_blob = self.cam.get_biggest_blob(blobs)
#            biggest_blob_color = biggest_blob[8]

#            print('check color')

#            if biggest_blob_color == pow(2, 0):
#                color_id = self.mid_line_id
#                break

#        print('lane color detected', color_id)

        blobs, img = self.cam.get_blobs_bottom()
        while True:
#            for blob in img.find_blobs(self.thresholds, pixels_threshold=150, area_threshold=150,
#                                       roi=(1,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
#                img.draw_rectangle(blob.rect())

#                if blob.elongation() > 0.5:
#                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
#                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
#                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
#                img.draw_cross(blob.cx(), blob.cy())
#                img.draw_keypoints(
#                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
#                )
            biggest_blob = self.cam.get_biggest_blob(blobs)
            biggest_blob_color = biggest_blob[8]

            print('check color')

            if biggest_blob_color == pow(2, 1):
                blobs.remove(biggest_blob)
                print('removed obstacle')

            if biggest_blob_color == pow(2, 0):
                color_id = self.mid_line_id
                break

        print('lane color detected', color_id)

        # Following the lane
        while True:
            blobs, img = self.cam.get_blobs_bottom()
#            for blob in img.find_blobs(self.thresholds, pixels_threshold=150, area_threshold=150,
#                                       roi=(1,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
#                img.draw_rectangle(blob.rect())

#                if blob.elongation() > 0.5:
#                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
#                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
#                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
#                img.draw_cross(blob.cx(), blob.cy())
#                img.draw_keypoints(
#                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
#                )
            print('finding blobs of color', color_id)
            found_mid = self.cam.find_blob(blobs, color_id)
            obstacle_detected = self.cam.find_blob(blobs, self.obstacle_id)
            print('found_mid', found_mid, 'obstacle', obstacle_detected)

            # If obstacle not there, follow lane
            if obstacle_detected is None:
                if found_mid is not None:
                    print('found lane')
#                    self.servo.set_angle(0)
                    pixel_error = blobs[found_mid].cx() - self.cam.w_centre
                    steering = pixel_error / self.cam.w_centre
                    drive = steering / self.drive_factor
                    print('steering', steering, 'drive:', bias - drive)
                    if steering < self.adherance and steering > -self.adherance:
                        self.drive(speed, bias)
                    else:
                        self.drive(speed, bias - drive)
            # If obstacle, stop
            if obstacle_detected is not None:
                print('obstacle detected')
                distance_pixel = blobs[obstacle_detected].y() + blobs[obstacle_detected].h()
                distance_to_obstacle = self.distance_factor * (distance_pixel ** (-1.920711))
                print('distance', distance_to_obstacle)
                if distance_to_obstacle <= distance_threshold:
                    print('STOP')
                    self.drive(0, 0)  # Stop the robot
                    break
                else:
                    print('closerrrr')
#                    self.servo.set_angle(0)
                    pixel_error = blobs[found_mid].cx() - self.cam.w_centre
                    steering = pixel_error / self.cam.w_centre
                    drive = steering / self.drive_factor
                    print('steering', steering, 'drive:', bias - drive)
                    if steering < self.adherance and steering > -self.adherance:
                        self.drive(speed, bias)
                    else:
                        self.drive(speed, bias - drive)

        self.servo.soft_reset()
        return


    def stage4(self, speed: float, bias: float, distance_threshold: float, angle_threshold: float) -> None:
        """
        Obstacle distance + orientation algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        self.servo.set_angle(0)
        time.sleep_ms(1000)
        print('starting stage 4')

        # Find the initial lane to follow
#        while True:
#            blobs, img = self.cam.get_blobs_bottom()
#            biggest_blob = self.cam.get_biggest_blob(blobs)
#            biggest_blob_color = biggest_blob[8]

#            print('check color')

#            if biggest_blob_color == pow(2, 0):
#                color_id = self.mid_line_id
#                break

#        print('lane color detected', color_id)


        blobs, img = self.cam.get_blobs_bottom()
        while True:
#            for blob in img.find_blobs(self.thresholds, pixels_threshold=150, area_threshold=150,
#                                       roi=(1,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
#                img.draw_rectangle(blob.rect())

#                if blob.elongation() > 0.5:
#                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
#                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
#                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
#                img.draw_cross(blob.cx(), blob.cy())
#                img.draw_keypoints(
#                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
#                )
            biggest_blob = self.cam.get_biggest_blob(blobs)
            biggest_blob_color = biggest_blob[8]

            print('check color')

            if biggest_blob_color == pow(2, 1):
                blobs.remove(biggest_blob)
                print('removed obstacle')

            if biggest_blob_color == pow(2, 0):
                color_id = self.mid_line_id
                break

        print('lane color detected', color_id)

        # Following the lane
        while True:
            blobs, img = self.cam.get_blobs_bottom()
#            for blob in img.find_blobs(self.thresholds, pixels_threshold=150, area_threshold=150,
#                                       roi=(1,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
#                img.draw_rectangle(blob.rect())

#                if blob.elongation() > 0.5:
#                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
#                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
#                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
#                img.draw_cross(blob.cx(), blob.cy())
#                img.draw_keypoints(
#                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
#                )
            print('finding blobs of color', color_id)
            found_mid = self.cam.find_blob(blobs, color_id)
            obstacle_detected = self.cam.find_blob(blobs, self.obstacle_id)

            # If obstacle not there, follow lane
            if obstacle_detected is None:
                if found_mid is not None:
                    print('found lane')
                    pixel_error = blobs[found_mid].cx() - self.cam.w_centre
                    steering = pixel_error / self.cam.w_centre
                    drive = steering / self.drive_factor
                    print('steering', steering, 'drive:', bias - drive)
                    if steering < self.adherance and steering > -self.adherance:
                        self.drive(speed, bias)
                    else:
                        self.drive(speed, bias - drive)
            # If obstacle, stop
            if obstacle_detected is not None:
                print('obstacle detected')
                distance_pixel = blobs[obstacle_detected].y() + blobs[obstacle_detected].h()
                distance_to_obstacle = self.distance_factor * (distance_pixel ** (-1.920711))
                print('distance', distance_to_obstacle)
                if distance_to_obstacle <= distance_threshold:
                    print('STOP')
                    self.drive(0, 0)  # Stop the robot

                    # Find direction of obstacle from robot
                    blobs, img = self.cam.get_blobs_bottom()
                    obstacle_detected = self.cam.find_blob(blobs, self.obstacle_id)
                    if obstacle_detected is not None:
                        direction = blobs[obstacle_detected].cx() - self.cam.w_centre
                        if direction < 0:
                            turn = -1
                        else:
                            turn = 1
                        print('turning', turn)

                    # Angle the robot
                    while True:
                        blobs, img = self.cam.get_blobs_bottom()
                        obstacle_detected = self.cam.find_blob(blobs, self.obstacle_id)
                        if obstacle_detected is None:
                            self.scan_for_blob(1)
                        if obstacle_detected is not None:
                            angle = abs(self.track_blob(blobs[obstacle_detected]))
#                            angle = abs(self.servo.pan_pos)
                            print('angle', angle)
                            if angle < angle_threshold:
                                self.drive(0.1, turn)
                                time.sleep_ms(30)
                                self.drive(0, 0)
                                time.sleep_ms(50)
                            else:
                                self.drive(0, 0)
                                self.servo.soft_reset()
                                return
                else:
                    print('closerrrr')
                    pixel_error = blobs[found_mid].cx() - self.cam.w_centre
                    steering = pixel_error / self.cam.w_centre
                    drive = steering / self.drive_factor
                    print('steering', steering, 'drive:', bias - drive)
                    if steering < self.adherance and steering > -self.adherance:
                        self.drive(speed, bias)
                    else:
                        self.drive(speed, bias - drive)

        self.servo.soft_reset()
        return


    def stage5(self, speed: float, bias: float, distance_threshold: float) -> None:
        """
        Obstacle avoidance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        self.servo.set_angle(0)
        time.sleep_ms(1000)
        print('starting stage 4')

        # Find the initial lane to follow
#        while True:
#            blobs, img = self.cam.get_blobs_bottom()
#            biggest_blob = self.cam.get_biggest_blob(blobs)
#            biggest_blob_color = biggest_blob[8]

#            print('check color')

#            if biggest_blob_color == pow(2, 0):
#                color_id = self.mid_line_id
#                break

#        print('lane color detected', color_id)


        blobs, img = self.cam.get_blobs_bottom()
        while True:
#            for blob in img.find_blobs(self.thresholds, pixels_threshold=150, area_threshold=150,
#                                       roi=(1,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
#                img.draw_rectangle(blob.rect())

#                if blob.elongation() > 0.5:
#                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
#                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
#                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
#                img.draw_cross(blob.cx(), blob.cy())
#                img.draw_keypoints(
#                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
#                )
            biggest_blob = self.cam.get_biggest_blob(blobs)
            biggest_blob_color = biggest_blob[8]

            print('check color')

            if biggest_blob_color == pow(2, 1):
                blobs.remove(biggest_blob)
                print('removed obstacle')

            if biggest_blob_color == pow(2, 0):
                color_id = self.mid_line_id
                break

        print('lane color detected', color_id)

        # Following the lane
        while True:
            blobs, img = self.cam.get_blobs_bottom()
#            for blob in img.find_blobs(self.thresholds, pixels_threshold=150, area_threshold=150,
#                                       roi=(1,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
#                img.draw_rectangle(blob.rect())

#                if blob.elongation() > 0.5:
#                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
#                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
#                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
#                img.draw_cross(blob.cx(), blob.cy())
#                img.draw_keypoints(
#                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
#                )
            print('finding blobs of color', color_id)
            found_mid = self.cam.find_blob(blobs, color_id)

            # obstacle detection
            obstacles = []
            for blob in blobs:
                blob_color = blob[8]
                if blob_color == pow(2, 1):
                    obstacles.append(blob)

#            obstacle_detected = self.cam.find_blob(blobs, self.obstacle_id)

            # If obstacle not there, follow lane
            if not obstacles:
                if found_mid is not None:
                    print('found lane')
                    pixel_error = blobs[found_mid].cx() - self.cam.w_centre
                    steering = pixel_error / self.cam.w_centre
                    drive = steering / self.drive_factor
                    print('steering', steering, 'drive:', bias - drive)
                    if steering < self.adherance and steering > -self.adherance:
                        self.drive(speed, bias)
                    else:
                        self.drive(speed, bias - drive)
            # If obstacle, stop
            if obstacles:
                print('obstacle detected')
                biggest_obstacle = self.cam.get_biggest_blob(obstacles)
                distance_pixel = biggest_obstacle.y() + biggest_obstacle.h()
                distance_to_obstacle = self.distance_factor * (distance_pixel ** (-1.920711))
                print('distance', distance_to_obstacle)
                direction = biggest_obstacle.cx() - self.cam.w_centre
                if direction < 0:
                    turn = -0.25
                else:
                    turn = 0.25
                print('turning', turn)
                if distance_to_obstacle <= distance_threshold:
                    print('start avoiding')
                    self.drive(speed, bias + turn)  # Stop the robot
                    time.sleep_ms(900)
                    self.drive(speed, bias - turn)
                    time.sleep_ms(900)

#                    # Find direction of obstacle from robot
#                    blobs, img = self.cam.get_blobs_bottom()
#                    obstacle_detected = self.cam.find_blob(blobs, self.obstacle_id)
#                    if obstacle_detected is not None:
#                        direction = blobs[obstacle_detected].cx() - self.cam.w_centre
#                        if direction < 0:
#                            turn = -1
#                        else:
#                            turn = 1
#                        print('turning', turn)

#                    # Angle the robot
#                    while True:
#                        blobs, img = self.cam.get_blobs_bottom()
#                        obstacle_detected = self.cam.find_blob(blobs, self.obstacle_id)
#                        if obstacle_detected is None:
#                            self.scan_for_blob(1)
#                        if obstacle_detected is not None:
#                            angle = abs(self.track_blob(blobs[obstacle_detected]))
##                            angle = abs(self.servo.pan_pos)
#                            print('angle', angle)
#                            if angle < angle_threshold:
#                                self.drive(0.1, turn)
#                                time.sleep_ms(30)
#                                self.drive(0, 0)
#                                time.sleep_ms(50)
#                            else:
#                                self.drive(0, 0)
#                                self.servo.soft_reset()
#                                return
                else:
                    print('closerrrr')
                    pixel_error = blobs[found_mid].cx() - self.cam.w_centre
                    steering = pixel_error / self.cam.w_centre
                    drive = steering / self.drive_factor
                    print('steering', steering, 'drive:', bias - drive)
                    if steering < self.adherance and steering > -self.adherance:
                        self.drive(speed, bias)
                    else:
                        self.drive(speed, bias - drive)

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
        self.servo.set_angle(pan_angle-5)

        return pan_angle


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

if __name__ == "__main__":
    thresholds = [
                 (31, 71, 2, 59, 11, 47), #red
                 (41, 60, -17, 1, 31, 57), #yellow
    #            (44, 66, -51, 20, -59, -5), #blue
    #            (24, 46, -34, -14, -1, 29), #green
                 ]

    robot = Robot(thresholds, 5)

    robot.scan_for_blob(1)

