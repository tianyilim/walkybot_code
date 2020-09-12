import hexapod
import numpy as np
import time
from adafruit_servokit import ServoKit

leftServos = ServoKit(channels=16, address=0x42)
rightServos = ServoKit(channels=16, address=0x41)


l_s0 = hexapod.leg_joint(0, leftServos, "L1_coxa")
l_s1 = hexapod.leg_joint(1, leftServos, "L1_femur")
r_s0 = hexapod.leg_joint(15, rightServos, "R1_coxa")
r_s1 = hexapod.leg_joint(14, rightServos, "R1_femur")


x = 60
diff = 0.5
mult = 1
while True:
    if x > 120:
        mult = -1
    elif x < 60:
        mult = 1
    
    x += diff*mult
    print(x)

    l_s0.writeAngle(x)
    l_s1.writeAngle(x)
    r_s0.writeAngle(x)
    r_s1.writeAngle(x)
    time.sleep(0.05)