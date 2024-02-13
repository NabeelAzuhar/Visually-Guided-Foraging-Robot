from servos import *
from camera import *
from machine import LED
import math

led = LED("LED_BLUE")
led.on()

servo = Servo()
servo.soft_reset()

dg_l = 46
dg_a = -24
dg_b = 15
b_l = 42
b_a = -8
b_b = -28
y_l = 88
y_a = -15
y_b = 64
r_l = 50
r_a = 46
r_b = 28

thresholds = [
              (dg_l - 6, dg_l + 6, dg_a - 6, dg_a + 6, dg_b - 6, dg_b + 6), # Dark green
              (b_l - 6, b_l + 6, b_a - 6, b_a + 6, b_b - 6, b_b + 6), # Blue
              (y_l - 6, y_l + 6, y_a - 6, y_a + 6, y_b - 6, y_b + 6), # Yellow
              (r_l - 6, r_l + 6, r_a - 6, r_a + 6, r_b - 6, r_b + 6), # Red
              (25, 31, 12, 24, -9, 3), # Purple
              (51, 57, 10, 22, 40, 52), # Orange
              (55, 61, -27, -15, 26, 38), # Light Green
              ]

colour_names = ['dark_green', 'blue', 'yellow', 'red', 'purple', 'orange', 'light green']
camera = Cam(thresholds, 3)

# Test your assignment code here. Think about how you might want to adjust the steering based on the position of
# the colour targets in the image. What should the bot do if the next colour target is outside the field of view
# of the camera? What would your bot do if the colour target is lost for a single frame? Some helpful functions are:

####################################################################################################
### The Exercise
servo.set_angle(0)
start = time.time()
try:
    for idx, color_threshold in enumerate(thresholds):
        print('Looking for blob:', idx)
        searching = True

        # Search for colour
        while searching:
            if time.time() - start > 60:
                break
            blobs, img = camera.get_blobs_bottom()
            if blobs:
                # Draw rectangle around blob of interest
                for blob in blobs:
                    img.draw_rectangle(blob.rect())
                    if blob.elongation() > 0.5:
                         img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                         img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                         img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))

                    img.draw_cross(blob.cx(), blob.cy())
                    img.draw_keypoints(
                        [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                    )
                found_idx = camera.find_blob(blobs, idx)
                # If blob found, stop spinning
                if found_idx is not None:
                    print('Found this colour:', colour_names[idx])
                    direction = abs(blobs[0].cx() - (img.width() / 2)) / (img.width() / 2)
                    if direction < 0.1:
                        print('direction good')
                        servo.set_differential_drive(0.2, 0)
#                        time.sleep_ms(1000)
                        searching = False
                        continue
            # If blob not found or not central, spin
            servo.set_differential_drive(0.05, 0.8)
            time.sleep_ms(100)
            servo.set_differential_drive(0, 0)

        while True:
            # Move to found colour
            if time.time() - start > 60:
                break
            blobs, img = camera.get_blobs_bottom()
            if blobs:
                found_idx = camera.find_blob(blobs, idx)
                if found_idx is not None:
                    continue
            servo.set_differential_drive(0, 0)
            time.sleep_ms(1000)
            break


#            servo.set_differential_drive(0.2, 0)
#            time.sleep_ms(3000)
#            servo.set_differential_drive(0, 0)

#        while not searching:
#            print('not searching')
#            if time.time() - start > 30:
#                break
#            blobs, _ = camera.get_blobs_bottom()
#            if blobs:
#                found_idx = camera.find_blob(blobs, idx)
#                # If blob still in camera, move towards it
#                if found_idx is not None:
#                    servo.set_differential_drive(0.1, 0)
#                    continue
#            # If blob not in camera, stop
#            servo.set_differential_drive(0, 0)
#            searching = True

        if time.time() - start > 60:
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
#servo.set_differential_drive(-1, -1)
#time.sleep_ms(5000)
#servo.set_differential_drive(0, 0)
#servo.soft_reset()
#print('DONE')
