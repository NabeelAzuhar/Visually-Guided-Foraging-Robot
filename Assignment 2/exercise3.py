from robot import *
from machine import LED
from servos import *

led = LED("LED_BLUE")
led.on()

thresholds = [
              (32, 72, 24, 56, 10, 40), #red
             (56, 84, -22, -5, 37, 65), #yellow
#             (44, 66, -51, 20, -59, -5), #blue
#             (24, 46, -34, -14, -1, 29), #green
             ]

robot = Robot(thresholds, gain = 10)
servo = Servo()
speed = 0.1
bias = 0.2

try:
    servo.set_angle(0)
#    robot.stage1(speed, bias)
#    robot.stage2(speed, bias)
    robot.stage3(speed, bias, 15)

except Exception as e:
    print('An error occurred:', e)
    # Set speed to 0 and perform soft reset
    servo.set_differential_drive(0, 0)
    servo.soft_reset()
    print('Robot reset')
