# import hex_legs
import hexapod
import numpy as np
from adafruit_servokit import ServoKit
from time import sleep

# initialise hexapod object
leg_end_coords = np.tile((0.0, 68.75, 0.0), (6,1)) # check back
leg_angles = np.tile(90.0, 6)  # leg angle init
# print(leg_end_coords, leg_end_coords.shape)
walky = hexapod.hexapod(leg_end_loc=leg_end_coords, leg_angle=leg_angles)

for leg in walky.legs:
    leg.debug_print()

# TODO roll/pitch calculations have some error. Need to be traced in simulator - why is hypotenuse shorter than adjacent angle in arccos?
# Might be an issue with the 'stance' update coordinates.
# Something wrong with updating values - something is not being updated.

# Roll test
interval = 0.5
amplitude = 15
t = 0
while t < 10:
    roll = amplitude * np.sin(np.pi * t)
    sleep(interval)
    t += interval

    print("Current roll angle:", roll)
    walky.body_roll(roll)

    # debug
    # walky.print_state()
    # for leg in walky.legs:
        # leg.debug_print()

    walky.update_legs_stance()

# # Pitch test
# interval = 0.01
# amplitude = 5
# t = 0
# while t < 10:
#     pitch = amplitude * np.sin(np.pi * t)
#     sleep(interval)
#     t += interval

#     print("Current pitch angle:", pitch)
#     walky.body_pitch(pitch)

#     # debug
#     # walky.print_state()
#     # for leg in walky.legs:
#         # leg.debug_print()

#     walky.update_legs_stance()

# # rotation test
# interval = 0.1
# amplitude = 5
# t = 0
# while t < 10:
#     angle = amplitude * np.sin(np.pi * t)
#     sleep(interval)
#     t += interval

#     print("Current angle:", angle)
#     walky.body_rotate(angle)

#     # debug
#     # walky.print_state()
#     # for leg in walky.legs:
#         # leg.debug_print()

#     walky.update_legs_stance()