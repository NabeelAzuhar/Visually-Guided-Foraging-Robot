from servos import *
from camera import *
from machine import LED
import math

led = LED("LED_BLUE")
led.on()

servo = Servo()
servo.soft_reset()

dg_l = 29
dg_a = -23
dg_b = 21
b_l = 33
b_a = -11
b_b = -19
y_l = 75
y_a = -11
y_b = 46
r_l = 37
r_a = 35
r_b = 32
p_l = 17
p_a = 14
p_b = 2
o_l = 53
o_a = 15
o_b = 40
lg_l = 47
lg_a = -20
lg_b = 35


thresholds = [
              (dg_l - 6, dg_l + 6, dg_a - 6, dg_a + 6, dg_b - 6, dg_b + 6), # Dark green
              (b_l - 6, b_l + 6, b_a - 6, b_a + 6, b_b - 6, b_b + 6), # Blue
              (y_l - 6, y_l + 6, y_a - 6, y_a + 6, y_b - 6, y_b + 6), # Yellow
              (r_l - 6, r_l + 6, r_a - 6, r_a + 6, r_b - 6, r_b + 6), # Red
              (p_l - 6, p_l + 6, p_a - 6, p_a + 6, p_b - 6, p_b + 6), # Purple
              (o_l - 6, o_l + 6, o_a - 6, o_a + 6, o_b - 6, o_b + 6), # Orange
              (lg_l - 6, lg_l + 6, lg_a - 6, lg_a + 6, lg_b - 6, lg_b + 6), # Light Green
              ]

colour_names = ['dark_green', 'blue', 'yellow', 'red', 'purple', 'orange', 'light green']
camera = Cam(thresholds, 8)

# Test your assignment code here. Think about how you might want to adjust the steering based on the position of
# the colour targets in the image. What should the bot do if the next colour target is outside the field of view
# of the camera? What would your bot do if the colour target is lost for a single frame? Some helpful functions are:

####################################################################################################
### The Exercise
for idx, color_threshold in enumerate(thresholds):
    searching = True
    while searching:
        blobs, img = camera.get_blobs_bottom()
        if blobs:
            found_idx = camera.find_blob(blobs, idx)
            if found_idx is not None:
                if abs(blobs[0].cx() - (img.width() / 2)) / (img.width() / 2) < 0.1:
                    servo.set_differential_drive(0.5, -0.1)
                    searching = False
                    continue
        servo.set_differential_drive(0.1, 0.8)
        time.sleep_ms(100)
        servo.set_differential_drive(0, 0)
    frames_unseen_count = 0
    while True:
        blobs, img = camera.get_blobs_bottom()
        if blobs:
            found_idx = camera.find_blob(blobs, idx)
            if found_idx is not None:
                continue
        frames_unseen_count += 1
        if frames_unseen_count > 7:
            if idx == 4 or idx == 5:
                servo.set_differential_drive(0.2, -0.1)
                time.sleep_ms(1000)
            servo.set_differential_drive(0, 0)
            time.sleep_ms(1000)
            break

