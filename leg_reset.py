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