# import hex_legs
import hexapod
import numpy as np
from adafruit_servokit import ServoKit
from time import sleep

leg_end = np.array((0.0, 68.75, 0.0))
leg_test = np.array((20.0, 68.75, 0.0))
DEFAULT_Z = 68.825

leftServos = ServoKit(channels=16, address=0x42)
rightServos = ServoKit(channels=16, address=0x41)

leg_r1 = hexapod.hex_leg(leg_end, DEFAULT_Z, leg_nums=(15,14,13), servoKit=rightServos, leg_name="leg_r1", offsets=(25,25,30))
leg_r2 = hexapod.hex_leg(leg_end, DEFAULT_Z, leg_nums=(11,10,9), servoKit=rightServos, leg_name="leg_r2", offsets=(0,20,35))
leg_r3 = hexapod.hex_leg(leg_end, DEFAULT_Z, leg_nums=(7,6,5), servoKit=rightServos, leg_name="leg_r3", offsets=(0,25,5))

leg_l1 = hexapod.hex_leg(leg_end, DEFAULT_Z, leg_nums=(0,1,2), servoKit=leftServos, leg_name="leg_l1", offsets=(0,-10,-5))
leg_l2 = hexapod.hex_leg(leg_end, DEFAULT_Z, leg_nums=(4,5,6), servoKit=leftServos, leg_name="leg_l2", offsets=(0,15,15))
leg_l3 = hexapod.hex_leg(leg_end, DEFAULT_Z, leg_nums=(8,9,10), servoKit=leftServos, leg_name="leg_l3", offsets=(10,10,0))

legs = (leg_r1, leg_r2, leg_r3, leg_l3, leg_l2, leg_l1)

sleep(2)

# for _ in range(10):
#     for leg in legs:
#         leg.swing(leg_test)
#     sleep(0.5)
#     for leg in legs:
#         leg.swing(leg_end)
#     sleep(0.5)

print(leg_r1.get_leg_end())
print(leg_r1.get_leg_angles())
# for joint in leg_r1.joints:
#     joint.debug_print()
#     print("")