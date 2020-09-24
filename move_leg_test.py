from sys import argv
import numpy as np
# Initial conditions
fill_vect = np.empty((0), dtype='int')
if len(argv) < 2:
    test_vect = np.array(((0,),(1,),(2,),(3,),(4,),(5,)))
elif len(argv) == 2:
    fill_vect = np.append(fill_vect, np.array((int(argv[1]),)))
    test_vect = np.array(((fill_vect),))
else:
    print("Only indiv test legs allowed")
    assert(0)

# import hex_legs
import hexapod
from adafruit_servokit import ServoKit
import time

print("Test vector:", test_vect, test_vect.shape)

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
print("\n\n//////////////LEG MOVEMENT test//////////////\n")
# while curr_time < TEST_DUR:

d_x_int = np.array((20.0,0.0,30.0))
d_x_fin = np.array((30.0,0.0,0.0))
d_y_int = np.array((0.0,20.0,30.0))
d_y_fin = np.array((0.0,30.0,0.0))

for lst in test_vect:
    start_pos = np.copy(walky._leg_end_abs[lst])
    print("\nStart pos for leg", lst, "is:\n", start_pos)
    walky.set_leg_end_abs(start_pos + d_x_int, leg_index=lst)
    walky.update_legs_swing(lst)
    input("Done with intermediate dx")

    walky.set_leg_end_abs(start_pos + d_x_fin, leg_index=lst)
    walky.update_legs_swing(lst)
    input("Done with final dx")

    walky.set_leg_end_abs(start_pos + d_x_int, leg_index=lst)
    walky.update_legs_swing(lst)
    print("Intermediate dx")
    walky.set_leg_end_abs(start_pos, leg_index=lst)
    walky.update_legs_swing(lst)
    input("Reset to center")

    walky.set_leg_end_abs(start_pos + d_y_int, leg_index=lst)
    walky.update_legs_swing(lst)
    input("Done with intermediate dy")

    walky.set_leg_end_abs(start_pos + d_y_fin, leg_index=lst)
    walky.update_legs_swing(lst)
    input("Done with final dy")

    walky.set_leg_end_abs(start_pos + d_y_int, leg_index=lst)
    walky.update_legs_swing(lst)
    print("Intermediate dy")
    walky.set_leg_end_abs(start_pos, leg_index=lst)
    walky.update_legs_swing(lst)
    input("Reset to center")
    print("Done with leg", lst, "\n")