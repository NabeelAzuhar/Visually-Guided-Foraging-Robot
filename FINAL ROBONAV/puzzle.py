import servos as servo
import camera as cam
import robot
import sensor
import time
import math


class Puzzle:
    def __init__(self, thresholds, gain: int, turn_direction: int):
        self.thresholds = thresholds
        self.gain = gain
        self.turn_direction = turn_direction  # Direction to try first (1 = Left, -1 = Right)

        self.angle_threshold = 4
        self.drive_factor = 1.5
        self.adherance = 0.1
        self.cy_distance = 200

        self.servo = servo.Servo()
        self.camera = cam.Cam(thresholds, gain)
        self.robot = robot.Robot(thresholds, gain, turn_direction)

    def solve(self, speed, bias):
        """
        Track the line and move the robot.
        Args:
            speed (float): Speed to set the wheels to (-1~1).
            bias (float): Bias to set the wheels to (-1~1).
        """

#        was_green_found = False
        while True:
            # Get the biggest blob from the camera
            blobs, img = self.camera.get_blobs_bottom_half()
            filtered_blobs = self.camera.filter_blobs(blobs, [0, 2])  # Filter blue and green blobs
            print('Filtered blobs: ', filtered_blobs)
#            big_blob = self.camera.get_biggest_blob(blobs)

            # If blob is found, go straight
            if filtered_blobs != []:
                print('MOVING STRAIGHT')
                was_green_found = self.move_straight_green(speed, bias)
                if not was_green_found:
                    self.move_straight_blue(speed, bias)
                else:
                    return

            # If blob is not found, stop the robot
            else:
                self.robot.drive(0, 0)
                # Scan for the blob and orient the robot in the correct direction
                print('TURNING CAMERA')
                failed = self.turn_camera_n_orient()

                # If robot could not find the blob while turning camera head, turn the robot
                if failed:
                    # Turn the robot
                    print('TURNING ROBOT')
                    self.turn_robot()


    def move_straight_green(self, speed, bias):
        """
        Move the robot straight towards blob
        """
        was_green_found = False
        frames = 0
        while True:
            blobs, img = self.camera.get_blobs_bottom_half()
            found_green = self.camera.find_blob(blobs, 2)

            if found_green is not None and blobs[found_green].cy() > self.cy_distance:
                print('found green')
                self.servo.set_angle(0)
                pixel_error = blobs[found_green].cx() - self.camera.w_centre
                steering = pixel_error / self.camera.w_centre
                drive = steering / self.drive_factor
                print('steering', steering, 'drive:', bias - drive)
                if steering < self.adherance and steering > -self.adherance:
                    self.robot.drive(speed, bias)
                else:
                    self.robot.drive(speed, bias - drive)
                was_green_found = True
            else:
                frames += 1
                if frames > 8:
                    self.robot.drive(0, 0)
                    return was_green_found


    def move_straight_blue(self, speed, bias):
        """
        Move the robot straight towards blob
        """
#        frames = 0
        while True:
            blobs, img = self.camera.get_blobs_bottom_half()
            found_blue = self.camera.find_blob(blobs, 0)

            if found_blue is not None and blobs[found_blue].cy() > self.cy_distance:
                print('found blue')
                self.servo.set_angle(0)
                pixel_error = blobs[found_blue].cx() - self.camera.w_centre
                steering = pixel_error / self.camera.w_centre
                drive = steering / self.drive_factor
                print('steering', steering, 'drive:', bias - drive)
                if steering < self.adherance and steering > -self.adherance:
                    self.robot.drive(speed, bias)
                else:
                    self.robot.drive(speed, bias - drive)
            else:
#                frames += 1
#                if frames > 2:
                self.robot.drive(0, 0)
                break



    def turn_camera_n_orient(self):
        """
        Turn the camera to find the markers and move the robot.
        """
        # Scan for the blue marker
        print('scanning')
        return_number = self.robot.scan_for_blobs([0, 2], 5, 44)  # Return number:- 0: Failed, 1: Blue, 2: Green
        print('Return number: ', return_number)

        # Failed
        if return_number == 0:
            return True
        elif return_number == 1:
            color_id = 0
        elif return_number == 2:
            color_id = 2
        else:
            print('ERROR: weird color found or something...')
            return

        # If marker is found, reorient the robot
        while True:
            # Find the blob
            blobs, img = self.camera.get_blobs_bottom_half()  # Get the blobs from camera
            filtered_blobs = self.camera.filter_blobs(blobs, [color_id])  # Filter blue or green blobs
            big_blob = self.camera.get_biggest_blob(filtered_blobs)  # Get the biggest blue or green blob

            # If the blue or green marker is found, orient the robot
            if big_blob is not None:
                print('Blob distance from robot', big_blob.cy())
                if big_blob.cy() >= self.cy_distance:
                    angle = self.robot.track_blob(big_blob)  # Orient camera to blob
                    # angle = abs(self.servo.pan_pos)
                    print('angle', angle)

                    # If blob is to the right, turn right
                    if angle < - self.angle_threshold:
                        self.servo.set_speed(0.1, -0.1)
                        time.sleep_ms(40)
                        self.servo.set_speed(0, 0)
                        # time.sleep_ms(50)
                    # If blob is to the left, turn left
                    elif angle > self.angle_threshold:
                        self.servo.set_speed(-0.1, 0.1)
                        time.sleep_ms(40)
                        self.servo.set_speed(0, 0)
                        # time.sleep_ms(50)
                    # If blob is in the centre, stop the robot
                    else:
                        self.robot.drive(0, 0)
                        # self.servo.soft_reset()
                        return

    def turn_robot(self):
        """
        Turn the robot until a blob is found
        """
        self.servo.set_angle(0)
        while True:

            print('start turn')

            # Turn the robot
            self.servo.set_speed((-0.1 * self.turn_direction), (0.1 * self.turn_direction))
            time.sleep_ms(100)
            self.servo.set_speed(0, 0)
#            self.servo.softer_reset()
            time.sleep_ms(100)

            # Find the blob
            blobs, img = self.camera.get_blobs_bottom_half()  # Get the blobs from camera
            filtered_blobs = self.camera.filter_blobs(blobs, [0, 2])  # Filter blue and green blobs

            # See the blobs on screen
            for blob in img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150,
                                       roi=(1,int(sensor.height()/2),int(sensor.width()),int(2*sensor.height()/4))):
                img.draw_rectangle(blob.rect())

                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_keypoints(
                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                )

            # If the blue or green marker is found, break the loop
            if filtered_blobs:
                big_blob = self.camera.get_biggest_blob(filtered_blobs)  # Get the biggest blue or green blob
                if big_blob is not None and big_blob.cy() > self.cy_distance:
                    angle = self.robot.track_blob(big_blob)  # Orient camera to blob
                    # angle = abs(self.servo.pan_pos)
                    print('angle', angle)

                if big_blob.cx() < int(sensor.width() / 2) + int(sensor.width() * self.adherance) and big_blob.cx() > int(sensor.width() / 2) - int(sensor.width() * self.adherance):
                    self.servo.set_speed(0, 0)
#                    self.servo.softer_reset()
                    print('end turn')
                    break


if __name__ == "__main__":

    thresholds = [
                 (33, 48, -10, 6, -24, -6), #blue
                 (11, 26, 13, 48, 2, 35), #red
                 (33, 50, -30, -17, 3, 31), #green
                 ]
    gain = 26
    turn_direction = 1

    speed = 0.1
    bias = -0.1

    puzzle = Puzzle(thresholds, gain, turn_direction)
    servo = servo.Servo()

    try:

       puzzle.solve(speed, bias)
#        puzzle.move_straight_green(speed, bias)
#        puzzle.move_straight_blue(speed, bias)
#        puzzle.turn_camera_n_orient()
#        puzzle.turn_robot()

    # Print error statement and soft reset the robot
    except Exception as e:
        print(e)
        servo.soft_reset()
        raise e









