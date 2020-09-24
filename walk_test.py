# import hex_legs
import hexapod
import numpy as np
from adafruit_servokit import ServoKit
import time

# initialise hexapod object
leg_end_coords = np.tile((0.0, 68.75, 0.0), (6,1)) # check back
leg_angles = np.tile(90.0, 6)  # leg angle init
# print(leg_end_coords, leg_end_coords.shape)
walky = hexapod.hexapod(leg_end_loc=leg_end_coords, leg_angle=leg_angles)

amplitude = 30
TEST_DUR = 8.0
TIME_DELTA = 0.05

print("Walkybot initial status:")
walky.print_state()
for leg in walky.legs:
    leg.debug_print()

print("")

def reset(delay):
    # Reset
    print("Resetting to normal stance")
    walky.body_pitch(0.0)
    walky.body_roll(0.0)
    walky.body_rotate_absolute(0.0)
    walky.update_legs_stance()
    time.sleep(delay)

start_time = time.time()
curr_time = 0.0
print("\n\n//////////////WALK test//////////////\n")

test_vect = np.array((0,))
set1 = np.array((0,2,4))
set2 = np.array((1,3,5))

dx = 20
dy = 0 
theta = 0

for x in range(4):
    print("\n\n//////////////SET1//////////////\n")
    walky.move_legs(set1, dx, dy, theta)
    print("\n\n//////////////SET2//////////////\n")
    walky.move_legs(set2, dx, dy, theta)

input("Done, reset?")
reset()