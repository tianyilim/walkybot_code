# import hex_legs
import hexapod
import numpy as np
from adafruit_servokit import ServoKit
import time

# initialise hexapod object
leg_end_coords = np.tile((0.0, 68.75, 0.0), (6,1)) # check back
leg_angles = np.tile(0.0, 6)  # leg angle init
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

walky.move_hexapod(50, 0, 0)

# ########################################################
# # Roll test
# start_time = time.time()
# curr_time = 0.0
# print("\n\n//////////////ROLL test//////////////\n")
# while curr_time < TEST_DUR:
#     freq = 1/(time.time()-start_time-curr_time)
#     curr_time = time.time() - start_time
#     roll = amplitude * np.sin(0.25*np.pi * curr_time)

#     walky.body_roll(roll)
#     # debug
#     print("Current roll angle: %0.4f | time: %0.4f | freq: %0.4f" %(roll, curr_time, freq) )
#     # walky.print_state()
#     # for leg in walky.legs:
#     #     leg.debug_print()
#     print("")
#     walky.update_legs_stance()

# reset(1)
# input("Ok to go to next test?")

# ########################################################
# # Pitch test
# start_time = time.time()
# curr_time = 0.0
# print("\n\n//////////////PITCH test//////////////\n")
# while curr_time < TEST_DUR:
#     freq = 1/(time.time()-start_time-curr_time)
#     curr_time = time.time() - start_time
#     pitch = amplitude * np.sin(0.25*np.pi * curr_time)
    
#     walky.body_pitch(pitch)

#     # debug
#     print("Current pitch angle: %0.4f | time: %0.4f | freq: %0.4f" %(pitch, curr_time, freq) )
#     # walky.print_state()
#     # for leg in walky.legs:
#     #     leg.debug_print()
#     print("")
#     walky.update_legs_stance()
    
# reset(1)
# input("Ok to go to next test?")

# ########################################################
# # z translation
# amplitude = 30
# start_z = walky._body_z
# start_time = time.time()
# curr_time = 0.0
# print("\n\n//////////////Z test//////////////\n")
# while curr_time < TEST_DUR:
#     freq = 1/(time.time()-start_time-curr_time)
#     curr_time = time.time() - start_time
#     z = amplitude * np.sin(0.25*np.pi * curr_time) + start_z
    
#     walky.body_translate_z_absolute(z)

#     # debug
#     print("Current Z displacement: %0.4f | time: %0.4f | freq: %0.4f" %(z, curr_time, freq) )
#     # walky.print_state()
#     # for leg in walky.legs:
#         # leg.debug_print()
#     # print("")
#     walky.update_legs_stance()
    
# reset(1)
# input("OK to go onto next test?")

# ########################################################
# # x translation
# start_time = time.time()
# curr_time = 0.0
# print("\n\n//////////////X test//////////////\n")
# while curr_time < TEST_DUR:
#     freq = 1/(time.time()-start_time-curr_time)
#     curr_time = time.time() - start_time
#     x = amplitude * np.sin(0.5*np.pi * curr_time)
    
#     print("Current X displacement: %0.4f | time: %0.4f | freq: %0.4f" %(x, curr_time, freq) )
#     walky.body_translate_x_absolute(x)
#     walky.update_legs_stance()
    
#     # debug
#     # walky.print_state()
#     # for leg in walky.legs:
#     #     leg.debug_print()

#     # time.sleep(1)

# reset(1)
# input("Ok to go to next test?")

# ########################################################
# # y translation
# start_time = time.time()
# curr_time = 0.0
# print("\n\n//////////////Y test//////////////\n")
# while curr_time < TEST_DUR:
#     freq = 1/(time.time()-start_time-curr_time)
#     curr_time = time.time() - start_time
#     y = amplitude * np.sin(0.5*np.pi * curr_time)
    
#     # debug
#     print("Current Y displacement: %0.4f | time: %0.4f | freq: %0.4f" %(y, curr_time, freq) )
#     walky.body_translate_y_absolute(y)
#     # walky.print_state()
#     # for leg in walky.legs:
#         # leg.debug_print()
#     walky.update_legs_stance()
    

# reset(1)
# input("Ok to go to next test?")

# ########################################################
# # rotation test
# amplitude = 30
# start_time = time.time()
# curr_time = 0.0
# print("\n\n//////////////Rotation test//////////////\n")
# while curr_time < TEST_DUR:
#     freq = 1/(time.time()-start_time-curr_time)
#     curr_time = time.time() - start_time
#     angle = amplitude * np.sin(0.5*np.pi * curr_time)
    
#     print("Current rotate angle: %0.4f | time: %0.4f | freq: %0.4f" %(angle, curr_time, freq) )
#     walky.body_rotate_absolute(angle)

#     # debug
#     # # walky.print_state()
#     # for leg in walky.legs:
#     #     leg.joints[0].debug_print()
#     # walky.legs[0].debug_print()

#     walky.update_legs_stance()
    
#     # time.sleep(0.1)

# reset(1)
# assert(0)
# input("Ok to go to next test?")


########################################################
# # Combined pitch and roll test
# start_time = time.time()
# curr_time = 0.0
# print("Pitch and Roll test")
# while curr_time < TEST_DUR:
#     freq = 1/(time.time()-start_time-curr_time)
#     curr_time = time.time() - start_time
#     pitch = amplitude * np.sin(1*np.pi * curr_time)
#     roll = amplitude * np.cos(1*np.pi * curr_time)

#     walky.body_pitch(pitch)
#     walky.body_roll(roll)

#     # debug
#     print("Pitch angle: %0.4f | Roll angle: %0.4f | time: %0.4f | freq: %0.4f" %(pitch, roll, curr_time, freq) )
#     # walky.print_state()
#     # for leg in walky.legs:
#         # leg.debug_print()
#     # print("")
#     walky.update_legs_stance()
# reset(1)
# input("OK to go onto next test?")