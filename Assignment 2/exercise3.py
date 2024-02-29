from robot import *
from machine import LED
from servos import *

led = LED("LED_BLUE")
led.on()

thresholds = [
             (31, 71, 2, 59, 11, 47), #red
             (41, 60, -17, 1, 31, 57), #yellow
#            (44, 66, -51, 20, -59, -5), #blue
#            (24, 46, -34, -14, -1, 29), #green
             ]

robot = Robot(thresholds, gain = 5)
servo = Servo()

speed = 0.1
bias = -0.15
distance = 15
angle = 30

try:
    servo.set_angle(0)
#    robot.stage1(speed, bias)
#    robot.stage2(speed, bias)
#    robot.stage3(speed, bias, distance)
#    robot.stage4(speed, bias, distance, angle)
    robot.stage5(speed, bias, distance)

except Exception as e:
    print('An error occurred:', e)
    # Set speed to 0 and perform soft reset
    servo.set_differential_drive(0, 0)
    servo.soft_reset()
    print('Robot reset')
