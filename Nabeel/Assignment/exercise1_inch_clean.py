from servos import *
from camera import *
from machine import LED
import math

led = LED("LED_BLUE")
led.on()

servo = Servo()
servo.soft_reset()

#dg = [28, -18, 11]
#b = [39, -9, -23]
#y = [74, -8, 35]
#r = [52, 36, 20]
#p = [32, 17, -6]
#o = [65, 11, 35]
#lg = [57, -20, 23]

#thresholds = []
#colours = [dg, b, y, r, p, o, lg]
#for colour in colours:
#    thresholds.append((colour[0]-6, colour[0]+6, colour[1]-6, colour[1]+6, colour[2]-6, colour[2]+6))

thresholds = [
              (20, 36, -28, -14, 9, 26),
              (17, 45, -23, 1, -28, -8),
              (62, 79, -21, 1, 27, 53),
              (40, 48, 18, 40, 12, 47),
              (28, 43, 8, 22, -1, 24),
              (57, 72, -5, 15, 31, 51),
              (63, 72, -25, -10, 19, 42),
              ]

colour_names = ['dark_green', 'blue', 'yellow', 'red', 'purple', 'orange', 'light green']
camera = Cam(thresholds, 24)

####################################################################################################
### The Exercise
servo.set_angle(0)
direction_threshold = 0.1
for idx in range(len(thresholds)):
    searching = True
    while searching:
        blobs, img = camera.get_blobs_bottom()
        found_idx = camera.find_blob(blobs, idx)
        if found_idx is None:
            servo.set_differential_drive(0.1, 0.8) ##
            time.sleep_ms(200)
            servo.set_differential_drive(0, 0)
        else:
            searching = False
    count = 0
    while not searching:
        blobs, img = camera.get_blobs_bottom()
        found_idx = camera.find_blob(blobs, idx)
        if found_idx is not None:
            direction = blobs[found_idx].cx() / img.width()
            if direction > 0.5 + direction_threshold:
                servo.set_differential_drive(0.05, -0.8) ##
                time.sleep_ms(50)
            elif direction < 0.5 - direction_threshold:
                servo.set_differential_drive(0.05, 0.8) ##
                time.sleep_ms(50)
            else:
                servo.set_differential_drive(0.2, 0) ##
                time.sleep_ms(300)
            servo.set_differential_drive(0, 0)
        else:
            count += 1
            if count > 5:
                servo.set_differential_drive(0.2, 0) ##
                time.sleep_ms(300)
                servo.set_differential_drive(0, 0)
                time.sleep_ms(1500)
                if idx == 4 or idx == 5:
                    servo.set_differential_drive(0.2, 0) ##
                    time.sleep_ms(500)
                    servo.set_differential_drive(0, 0)
                break


####################################################################################################
#### Test 1
#print('START')
#servo.set_differential_drive(0.5, 0)
#time.sleep_ms(5000)
#servo.set_differential_drive(0, 0)
#time.sleep_ms(1000)
#servo.soft_reset()
#print('DONE')
