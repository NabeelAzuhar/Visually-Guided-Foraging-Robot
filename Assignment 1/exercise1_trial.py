from servos import *
from camera import *
from machine import LED

led = LED("LED_BLUE")
led.on()

servo = Servo()
servo.soft_reset()

thresholds = [
              (32, 41, -30, -20, 16, 27),
              (46, 61, -25, 1, -29, -12),
              (81, 95, -25, 1, 21, 54),
              (58, 72, 12, 31, 15, 45),
              (36, 46, 7, 26, -10, 17),
              (67, 84, -13, 8, 23, 51),
              (75, 87, -27, -4, 20, 49),
              ]
camera = Cam(thresholds, 35)

# Test your assignment code here. Think about how you might want to adjust the steering based on the position of
# the colour targets in the image. What should the bot do if the next colour target is outside the field of view
# of the camera? What would your bot do if the colour target is lost for a single frame? Some helpful functions are:
# camera.get_blobs_bottom()
# camera.find_blobs()
# servos.set_differential_drive()















servo.set_angle(0)
t = 0.1

for i in range(len(thresholds)):
    s = True
    while s:
        blobs, img = camera.get_blobs_bottom()
        fi = camera.find_blob(blobs, i)
        if fi is None:
            servo.set_differential_drive(0.2, 0.8)
            time.sleep_ms(150)
            servo.set_differential_drive(0, 0)
        else:
            s = False
    c = 0
    while not s:
        blobs, img = camera.get_blobs_bottom()
        fi = camera.find_blob(blobs, i)
        if fi is not None:
            direction = blobs[fi].cx() / img.width()
            if direction > 0.5 + t:
                servo.set_differential_drive(0.05, -0.8)
                time.sleep_ms(50)
                servo.set_differential_drive(0, 0)
            elif direction < 0.5 - t:
                servo.set_differential_drive(0.05, 0.8)
                time.sleep_ms(50)
                servo.set_differential_drive(0, 0)
            else:
                servo.set_differential_drive(0.2, 0)
                time.sleep_ms(300)
                servo.set_differential_drive(0, 0)
        else:
            c += 1
            if c > 5:
                servo.set_differential_drive(0.2, 0)
                time.sleep_ms(300)
                servo.set_differential_drive(0, 0)
                if i in (4,5):
                    servo.set_differential_drive(0.2, 0)
                    time.sleep_ms(500)
                    servo.set_differential_drive(0, 0)
                break



servo.soft_reset()
