import puzzle

speed = 0.2
bias = 0
gain = 15
thresholds = [(37, 54, -10, 10, -37, -18),  # Blue
              (21, 48, 13, 58, 4, 43),  # Red
              (36, 52, -48, -19, 12, 38)  # Green
             ]

puzzle_solver = puzzle.Puzzle(thresholds, gain, speed, bias)

try:
    puzzle_solver.solve()
except Exception as e:
    print('An error occurred:', e)
    # Set speed to 0 and perform soft reset
    puzzle_solver.servo.set_speed(0, 0)
    puzzle_solver.servo.soft_reset()
    print('Robot reset')




