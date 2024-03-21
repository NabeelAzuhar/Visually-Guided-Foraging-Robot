import puzzle
import servos as servo

thresholds = [
             (33, 48, -10, 6, -24, -6), #blue
             (11, 26, 13, 48, 2, 35), #red
             (33, 50, -30, -17, 3, 31), #green
             ]

gain = 26
turn_direction = 1  # 1: Left, -1: Right

speed = 0.2
bias = -0.1

solver = puzzle.Puzzle(thresholds, gain, turn_direction)
servo = servo.Servos()

# Start solving
servo.soft_reset()

try:

   solver.solve(speed, bias)

# Print error statement and soft reset the robot
except Exception as e:
    print(e)
    servo.soft_reset()
    raise e
