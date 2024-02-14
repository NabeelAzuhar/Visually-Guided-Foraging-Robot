from servos import *
from camera import *
from machine import LED
import math

led = LED("LED_BLUE")
led.on()

servo = Servo()
servo.soft_reset()

dg = [19, -21, 19]
b = [23, -5, -19]
y = [51, -11, 47]
r = [23, 30, 23]
p = [18, 9, 4]
o = [36, 10, 38]
lg = [47, -21, 32]

thresholds = []
colours = [dg, b, y, r, p, o, lg]
for colour in colours:
    thresholds.append((colour[0]-6, colour[0]+6, colour[1]-6, colour[1]+6, colour[2]-6, colour[2]+6))

colour_names = ['dark_green', 'blue', 'yellow', 'red', 'purple', 'orange', 'light green']
camera = Cam(thresholds, 20)

####################################################################################################
### The Exercise
servo.set_angle(0)
direction_threshold = 0.1
for idx, color_threshold in enumerate(thresholds):
    searching = True
    while searching:
        blobs, img = camera.get_blobs()
        found_idx = camera.find_blob(blobs, idx)

        if found_idx is None:
            servo.set_differential_drive(0.1, 0.8)
            time.sleep_ms(200)
            servo.set_differential_drive(0, 0)
        else:
            print(blobs[found_idx])
            searching = False
    count = 0
    while not searching:
        blobs, img = camera.get_blobs()
        found_idx = camera.find_blob(blobs, idx)
        if found_idx is not None:
            direction = blobs[found_idx].cx() / img.width()
            if direction > 0.5 + direction_threshold:
                servo.set_differential_drive(0.1, -0.8)
                time.sleep_ms(50)
            elif direction < 0.5 - direction_threshold:
                servo.set_differential_drive(0.1, 0.8)
                time.sleep_ms(50)
            else:
                print('zoom!')
                servo.set_differential_drive(0.5, 0.2)
                time.sleep_ms(300)
            servo.set_differential_drive(0, 0)
        else:
            count += 1
            if count > 5:
                servo.set_differential_drive(0.5, 0.2)
                time.sleep_ms(300)
                servo.set_differential_drive(0, 0)
                time.sleep_ms(1500)
                if idx == 4 or idx == 5:
                    servo.set_differential_drive(0.5, 0.2)
                    time.sleep_ms(500)
                    servo.set_differential_drive(0, 0)
                break


####################################################################################################
#### Test 1
#print('START')
#servo.set_differential_drive(0.5, 0)
#time.sleep_ms(5000)
#servo.set_differential_drive(0, 0)
#servo.soft_reset()
#print('DONE')
