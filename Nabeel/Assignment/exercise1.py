from servos import *
from camera import *
from machine import LED

led = LED("LED_BLUE")
led.on()

servo = Servo()
servo.soft_reset()

thresholds = [
              (65, 95, -30, -5, 15, 40), # Light green
              (25, 35, -15, 10, -30, -10), # Blue
              (68, 80, -30, 0, 40, 60), # Yellow
              (45, 55, 40, 55, 15, 35), # Red
]
camera = Cam(thresholds)

# Test your assignment code here. Think about how you might want to adjust the steering based on the position of
# the colour targets in the image. What should the bot do if the next colour target is outside the field of view
# of the camera? What would your bot do if the colour target is lost for a single frame? Some helpful functions are:

####################################################################################################
start = time.time()
try:
    for idx, color_threshold in enumerate(thresholds):
        print('Looking for blob:', idx)
        searching = True

        # Search for colour
        while searching:
            if time.time() - start > 30:
                break
            blobs, img = camera.get_blobs_bottom()
            if blobs:
                found_idx = camera.find_blob(blobs, idx)
                # If blob found, stop spinning
                if found_idx is not None:
                    print('Found something!')
                    print(blobs)
                    direction = abs(blobs[0].cx() - (img.width() / 2)) / (img.width() / 2)
                    if  direction < 0.1:
                        print('direction good')
                        servo.set_speed(0, 0)
                        searching = False
                        continue
            # If blob not found or not central, spin
            servo.set_speed(0.25, -0.25)

        # Move to found colour
        while not searching:
            if time.time() - start > 30:
                break
            blobs, _ = camera.get_blobs_bottom()
            if blobs:
                found_idx = camera.find_blob(blobs, idx)
                # If blob still in camera, move towards it
                if found_idx is not None:
                    servo.set_speed(0.5, 0.5)
                    continue
            # If blob not in camera, stop
            servo.set_speed(0, 0)
            searching = True

        if time.time() - start > 30:
            error('Taking too long!!')
            servo.set_speed(0, 0)
            break

except Exception as e:
    print('An error occurred:', e)
    # Set speed to 0 and perform soft reset
    servo.set_speed(0, 0)
    servo.soft_reset()
    print('Robot reset')
    print('Robot failed at colour threshold: ', idx)


####################################################################################################
