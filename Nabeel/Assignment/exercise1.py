from servos import *
from camera import *
from machine import LED

led = LED("LED_BLUE")
led.on()

servo = Servo()
servo.soft_reset()

thresholds = [
#              (60, 75, -35, -10, 5, 30), # Dark green
              (65, 75, -25, 0, -20, 5), # Blue
              (80, 100, -25, 0, 35, 50), # Yellow
#              (45, 55, 40, 55, 15, 35), # Red
]

colour_names = ['blue', 'yellow']
camera = Cam(thresholds)

# Test your assignment code here. Think about how you might want to adjust the steering based on the position of
# the colour targets in the image. What should the bot do if the next colour target is outside the field of view
# of the camera? What would your bot do if the colour target is lost for a single frame? Some helpful functions are:

####################################################################################################
#servo.set_speed(0.1, 0.1)
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
                    print('Found this colour:', colour_names[idx])
                    direction = abs(blobs[0].cx() - (img.width() / 2)) / (img.width() / 2)
                    if  direction < 0.1:
                        print('direction good')
                        servo.set_speed(0, 0)
                        searching = False
                        continue
            # If blob not found or not central, spin
            servo.set_speed(0, 0.1)

        # Move to found colour
        while not searching:
            if time.time() - start > 30:
                break
            blobs, _ = camera.get_blobs_bottom()
            if blobs:
                found_idx = camera.find_blob(blobs, idx)
                # If blob still in camera, move towards it
                if found_idx is not None:
                    servo.set_speed(0.1, 0.1)
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
