""" initialises all servos (left and right) """

from time import sleep
from sys import argv
from adafruit_servokit import ServoKit
 
# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
leftServos = ServoKit(channels=16, address=0x42)
rightServos = ServoKit(channels=16, address=0x41)

print("Centering all servos")

# Define servos
r1_coxa = rightServos.servo[15]
r1_femur = rightServos.servo[14]
r1_tibia = rightServos.servo[13]
r1 = [r1_coxa, r1_femur, r1_tibia]

r2_coxa = rightServos.servo[11]
r2_femur = rightServos.servo[10]
r2_tibia = rightServos.servo[9]
r2 = [r2_coxa, r2_femur, r2_tibia]

r3_coxa = rightServos.servo[7]
r3_femur = rightServos.servo[6]
r3_tibia = rightServos.servo[5]
r3 = [r3_coxa, r3_femur, r3_tibia]

l1_coxa = leftServos.servo[0]
l1_femur = leftServos.servo[1]
l1_tibia = leftServos.servo[2]
l1 = [l1_coxa, l1_femur, l1_tibia]

l2_coxa = leftServos.servo[4]
l2_femur = leftServos.servo[5]
l2_tibia = leftServos.servo[6]
l2 = [l2_coxa, l2_femur, l2_tibia]

l3_coxa = leftServos.servo[8]
l3_femur = leftServos.servo[9]
l3_tibia = leftServos.servo[10]
l3 = [l3_coxa, l3_femur, l3_tibia]

right_legs = [r1, r2, r3]
left_legs = [l1, l2, l3]
legs = [r1, r2, r3, l3, l2, l1] # Note that this runs the other way round!

print("All servos angle 90")

while True:
    for leg in legs:
        for servo in leg:
            servo.angle = 90

# leftServos.servo[8].angle=90
# rightServos.servo[8].angle=90
# sleep(2)

# while len(argv) > 1: # Will loop if CLI argument given
# 	print("Servo angle 0")
# 	leftServos.servo[8].angle=0
# 	rightServos.servo[8].angle=0
# 	sleep(0.5)

# 	print("Servo angle 90")
# 	leftServos.servo[8].angle=90
# 	rightServos.servo[8].angle=90
# 	sleep(0.5)
	
# 	print("Servo angle 180")
# 	leftServos.servo[8].angle=180
# 	rightServos.servo[8].angle=180
# 	sleep(0.5)

# 	print("Servo angle 90")
# 	leftServos.servo[8].angle=90
# 	rightServos.servo[8].angle=90
# 	sleep(0.5)
