import hex_legs
import hexapod
import numpy as np
from adafruit_servokit import ServoKit
from time import sleep

leg_end = np.array((0.0, 68.75, 0.0))
leg_test = np.array((10.0, 68.75, 0.0))
DEFAULT_Z = 68.825

leftServos = ServoKit(channels=16, address=0x42)
rightServos = ServoKit(channels=16, address=0x41)

# leg_r1 = hex_legs.hexLeg(kit=leftServos, legNum=1, leftright='right')
# leg_r1.write_angles((90,90,90))

leg_r1 = hexapod.hex_leg(leg_end, DEFAULT_Z, leg_nums=(15,14,13), servoKit=rightServos, leg_name="leg_r1", offsets=(0,10,0))
# leg_l1.set_leg_angles((90,90,90))
print(leg_r1.get_leg_end())
print(leg_r1.get_leg_angles())
for joint in leg_r1._joints:
    joint.debug_print()
    print("")

sleep(0.5)
print("Moving leg")
leg_r1.swing(leg_test)
sleep(0.5)
print(leg_r1.get_leg_end())
print(leg_r1.get_leg_angles())
for joint in leg_r1._joints:
    joint.debug_print()
    print("")

print("________________")


leg_l1 = hexapod.hex_leg(leg_end, DEFAULT_Z, leg_nums=(0,1,2), servoKit=leftServos, leg_name="leg_l1", offsets=(0,0,0))
# leg_l1.set_leg_angles((90,90,90))
print(leg_l1.get_leg_end())
print(leg_l1.get_leg_angles())
for joint in leg_l1._joints:
    joint.debug_print()
    print("")