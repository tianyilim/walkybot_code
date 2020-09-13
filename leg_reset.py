# import hex_legs
import hexapod
import numpy as np
from adafruit_servokit import ServoKit
import time
from sys import argv

# initialise hexapod object
leg_end_coords = np.tile((0.0, 68.75, 0.0), (6,1)) # check back
leg_angles = np.tile(90.0, 6)  # leg angle init
# print(leg_end_coords, leg_end_coords.shape)
walky = hexapod.hexapod(leg_end_loc=leg_end_coords, leg_angle=leg_angles)


## Below: Testing coxa rotation scheme
# angle_range = np.arange(70, 110, 1)
# angle_range = np.append(angle_range, angle_range[::-1])

# for i in range(len(angle_range)):
#     for leg in walky.legs:
#         leg.set_leg_angles((angle_range[i],90,90))
#         leg.writeAngles()
#     time.sleep(0.1)

# add verbose output
if len(argv) > 1:
    for leg in walky.legs:
        leg.debug_print()