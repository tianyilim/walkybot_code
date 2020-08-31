# Header file for walkybot.
# Instanstiates the legs and applies corrections.

# types of object: 
# Servo object - simple end effector (will have 18)
# Leg object - combination of three servos

# TODO write functions for the behaviour of the higher-hierarchy objects.

from adafruit_servokit import ServoKit
import warnings # Tells if we are doing something wrong

class legJoint:
    _servo_num = 0
    _offset = 0 # can be tuned for non-idealites in hardware setup
    _limit = 30 # Limit to motion, servo cannot pass this value
    _start_angle = 90
    _angle_max = curr_angle + limit
    _angle_min = curr_angle - limit
    
    _servo_driver = None # empty variable for now

    curr_angle = 0 # Current angle to be written to servo

    def __init__(self, kit, servo_num, start_angle=90, limit=30, offset=0):
        # Initialises servo values. Here, start angle refers to the neutral position of the servo.
        _servo_num = servo_num
        _offset = offset
        _limit = limit
        _start_angle = start_angle + _offset
        _angle_max = _start_angle + _limit
        _angle_min = _start_angle - _limit

        _servo_driver = kit.servo[servo_num]

        curr_angle = _start_angle # init this stuff.

    def writeLimit(self, limit):
        # Updates the limits of the object.
        _limit = limit
        _angle_max = _start_angle + _limit
        _angle_min = _start_angle - _limit
        
        return _angle_max, _angle_min

    def writeAngle(self, angle):
        try_angle = angle + offset
        if try_angle > _angle_max:
            curr_angle = _angle_max
            warnings.warn("%s: Exceeded +ve limit" %self.__class__.__name__)
        elif try_angle < angle_min:
            curr_angle = _angle_min
            warnings.warn("%s: Exceeded -ve limit" %self.__class__.__name__)
        else:
            curr_angle = try_angle

        _servo_driver.angle = curr_angle # Write servo angle here.

        return curr_angle


# initialize everything from hexleg.
# current xyz and desired xyz can go here too.
class hexLeg:
    coxa_num = 0
    tibia_num = 0
    femur_num = 0

    def __init__(self, kit, legNum, leftright):
        # Attaches the servos (assumes that left and right control boards have been attached)
        # Problem: How to address the left/right servo objects if they are defined in a higher class?
        if leftright == 'left':
            coxa_num = 15 - (legNum-1)*4 # 1: 15, 2: 11, 3: 7
            tibia_num = coxa_num - 1
            femur_num = coxa_num - 2

        elif leftright == 'right':
            coxa_num = (legNum-1)*4 # 1: 0, 2: 4, 3: 8
            tibia_num = coxa_num + 1
            femur_num = coxa_num + 2
            
        else:
            raise NameError("Invalid argument in leftright")
            return False

        coxa = legJoint(kit, coxa_num)
        tibia = legJoint(kit, tibia_num)
        femur = legJoint(kit, femur_num) # Attach these things.

    return True


class legDriver:
    
    def __init__(self, left_right, addr):
        if left_right != 'left' or left_right != 'right':
            raise NameError("Invalid argument in left_right")
            return False
        else:
            kit = ServoKit(channels=16, address=addr)
            leg_1 = hexLeg(kit, legNum=1, leftright=left_right) # attach all servos here
            leg_2 = hexLeg(kit, legNum=2, leftright=left_right)
            leg_3 = hexLeg(kit, legNum=3, leftright=left_right)

            return True


# Initialises all 6 legs' hardware here.
# Current xyz, roll/pitch/yaw go here.
class hexapod:
    def __init__(self, left_addr=0x42, right_addr=0x41):
        left_legs = legDriver('left', addr=left_addr)
        right_legs = legDriver('right', addr=right_addr)