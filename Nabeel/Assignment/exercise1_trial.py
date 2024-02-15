from servos import *
from camera import *
from machine import LED

led = LED("LED_BLUE")
led.on()

servo = Servo()
servo.soft_reset()

thresholds = [
              (20, 36, -28, -14, 9, 26),
              (17, 45, -23, 1, -28, -8),
              (62, 79, -21, 1, 27, 53),
              (40, 48, 18, 40, 12, 47),
]
camera = Cam(thresholds, 24)

# Test your assignment code here. Think about how you might want to adjust the steering based on the position of
# the colour targets in the image. What should the bot do if the next colour target is outside the field of view
# of the camera? What would your bot do if the colour target is lost for a single frame? Some helpful functions are:
# camera.get_blobs_bottom()
# camera.find_blobs()
# servos.set_differential_drive()

servo.soft_reset()