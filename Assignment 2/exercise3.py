from robot import *
from machine import LED
from servos import *

led = LED("LED_BLUE")
led.on()

thresholds = [(41, 58, 16, 62, -15, 48), #red
             (41, 75, -25, -6, 33, 57), #yellow
             (31, 60, -21, 3, -45, -9), #blue
             (23, 61, -43, -20, -3, 24), #green
             ]

robot = Robot(thresholds, gain = 10)
servo = Servo()

try:
    robot.stage2(0.1, 0.25)

except Exception as e:
    print('An error occurred:', e)
    # Set speed to 0 and perform soft reset
    servo.set_differential_drive(0, 0)
    servo.soft_reset()
    print('Robot reset')
