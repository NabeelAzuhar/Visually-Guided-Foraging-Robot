from robot import *
from machine import LED
from servos import *

led = LED("LED_BLUE")
led.on()

gain = 15
thresholds = [
              (19, 51, 25, 75, 14, 58),  # red
              (45, 64, -18, 7, -42, -10),  # blue
              (44, 58, -52, -24, 6, 54),  #green
             ]

puzzle = Puzzle(thresholds, gain = gain)
servo = Servo()

puzzle.puzzle_search(0.2, 0.25)

servo.soft_reset()
