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

# Test your assignment code here. Think about how you might want to adjust the steering based on the position of
# the colour targets in the image. What should the bot do if the next colour target is outside the field of view
# of the camera? What would your bot do if the colour target is lost for a single frame? Some helpful functions are:

####################################################################################################
### The Exercise
print(thresholds)
start = time.time()
duration = 150
direction_threshold = 0.05
try:
    for idx, color_threshold in enumerate(thresholds):
        print('Looking for blob:', idx)
        searching = True

        while searching:
            print('Searching')
            if time.time() - start > duration:
                break
            # Search for colour
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
            print('Not Searching')
            if time.time() - start > duration:
                break
            # Move to colour
            blobs, img = camera.get_blobs()
            found_idx = camera.find_blob(blobs, idx)

            if found_idx is not None:
                print(blobs[found_idx])
                direction = blobs[found_idx].cx() / img.width()
                print('direction:', direction)
                if direction > 0.5 + direction_threshold:
                    print('move right')
                    servo.set_differential_drive(0.1, -0.8)
                    time.sleep_ms(50)
                    servo.set_differential_drive(0, 0)
                elif direction < 0.5 - direction_threshold:
                    print('move left')
                    servo.set_differential_drive(0.1, 0.8)
                    time.sleep_ms(50)
                    servo.set_differential_drive(0, 0)
                else:
                    print('zoom!')
                    servo.set_differential_drive(0.5, 0.2)
                    time.sleep_ms(300)
                    servo.set_differential_drive(0, 0)

            else:
                count += 1
                if count > 5:
                    print('finish it')
                    servo.set_differential_drive(0.5, 0.2)
                    time.sleep_ms(400)
                    servo.set_differential_drive(0, 0)
                    time.sleep_ms(2000)
                    print('REACHED')
                    if idx == 4 or idx == 5:
                        servo.set_differential_drive(0.5, 0.2)
                        time.sleep_ms(500)
                        servo.set_differential_drive(0, 0)
                    break

        if time.time() - start > duration:
            print('Taking too long!!')
            error('TOO LONG')
            servo.set_differential_drive(0, 0)
            break

except Exception as e:
    print('An error occurred:', e)
    # Set speed to 0 and perform soft reset
    servo.set_differential_drive(0, 0)
    servo.soft_reset()
    print('Robot reset')
    print('Robot failed at colour threshold: ', idx)


####################################################################################################
#### Test 1
#print('START')
#servo.set_differential_drive(0.5, 0)
#time.sleep_ms(5000)
#servo.set_differential_drive(0, 0)
#servo.soft_reset()
#print('DONE')
####################################################################################################
#### TEST 2
#print('START')
#servo.set_differential_drive(-0.5, 0)
#time.sleep_ms(1000)
#servo.set_differential_drive(1, 0)
#time.sleep_ms(1000)
#servo.set_differential_drive(-0.5, 1)
#time.sleep_ms(1000)
#servo.set_differential_drive(-0.5, -1)
#time.sleep_ms(1000)
#servo.set_differential_drive(0.5, 1)
#time.sleep_ms(1000)
#servo.set_differential_drive(0.5, -1)
#time.sleep_ms(1000)
#servo.set_differential_drive(0, 0)
#servo.soft_reset()
#print('DONE')
