import puzzle

speed = 0.1
bias = 0
gain = 8
thresholds = [(31, 60, -21, 3, -45, -9), #blue
              (23, 30, 40, 50, 27, 40), #red
              (23, 61, -43, -20, -3, 24), #green
              (41, 75, -25, -6, 33, 57), #sun
              ]

puzzle_solver = puzzle.Puzzle(thresholds, gain, speed, bias)
#puzzle_solver.servo.set_speed(0,0)
#puzzle_solver.detector.scan_for_blob(0, 1, 20)
puzzle_solver.move(speed, bias)

'''
try:
    puzzle_solver.solve()
except Exception as e:
    print('An error occurred:', e)
    # Set speed to 0 and perform soft reset
    puzzle_solver.servo.set_speed(0, 0)
    puzzle_solver.servo.soft_reset()
    print('Robot reset')
'''
