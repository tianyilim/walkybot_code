# Header file for walkybot.
# Instanstiates the legs and applies corrections.

# types of object: 
# Servo object - simple end effector (will have 18)
# Leg object - combination of three servos -- Contains the leg IK
# Leg driver - banks of three legs
# Hexapod - all six legs -- Contains the body IK

# TODO write functions for the behaviour of the higher-hierarchy objects.
# Eventually this will be refactored to be in C++ for SPEED

from adafruit_servokit import ServoKit
import warnings # Tells if we are doing something wrong
import numpy as np

class legJoint:
    # "Public" variables
    curr_angle = 90

    # "Private" variables
    _servo_num = 0
    _offset = 0 # can be tuned for non-idealites in hardware setup
    _limit = 30 # Limit to motion, servo cannot pass this value
    _start_angle = curr_angle
    _angle_max = curr_angle + _offset + _limit
    _angle_min = curr_angle + _offset - _limit
    
    _servo_driver = None # empty variable for now

    def __init__(self, kit, servo_num, start_angle=90, limit=30, offset=0):
        # Initialises servo values. Here, start angle refers to the neutral position of the servo.
        self._servo_num = servo_num
        self._offset = offset
        self._limit = limit
        self._start_angle = start_angle + self._offset
        self._angle_max = self._start_angle + self._limit
        self._angle_min = self._start_angle - self._limit

        self._servo_driver = kit.servo[servo_num]

    def writeLimit(self, limit):
        # Updates the limits of the object.
        self._limit = limit
        self._angle_max = self._start_angle + self._offset + self._limit
        self._angle_min = self._start_angle + self._offset - self._limit
        
        return self._angle_max, self._angle_min

    def writeAngle(self, angle):
        try_angle = angle + self._offset
        if try_angle > self._angle_max:
            self.curr_angle = self._angle_max
            warnings.warn("%s: Exceeded +ve limit" %self.__class__.__name__)
        elif try_angle < self._angle_min:
            self.curr_angle = self._angle_min
            warnings.warn("%s: Exceeded -ve limit" %self.__class__.__name__)
        else:
            self.curr_angle = try_angle

        self._servo_driver.angle = self.curr_angle # Write servo angle here.

        return self.curr_angle

    # Prints info to debug stuff with on to console
    def debug_print(self):
        print("For servo", self.__class__.__name__)
        print("Current Angle:", self.curr_angle, "Offset:", self._offset)
        print("+ve limit:", self._angle_max, "-ve limit:", self._angle_min)
    
    # returns a list of important info
    def debug_return(self):
        info = [self.curr_angle, self._offset, self._angle_max, self._angle_min]
        return info

# Contains the Leg IK
class hexLeg:
    # Private variables
    ## Servo addresses
    _coxa_num = 0
    _tibia_num = 0
    _femur_num = 0
    _left_right = True

    # Leg IK parameters in millimeters (mm)
    _coxa_len = 28.75
    _tibia_len = 68.825
    _femur_len = 40
    _z_offset = 68.825
    _l_len = np.sqrt( _z_offset**2 + _coxa_len**2 ) # Z-offset can change!

    # coordinates for IK. Note that they are in different reference frames to each other.
    _tip_coords = (0.0, _coxa_len+_femur_len, _z_offset) # base WRT tip (x,y,z)

    _base_coords = (0.0, 0.0, 0.0)

    # individual servo angles
    _servo_angles = (90.0, 90.0, 90.0) # (coxa, femur, tibia)

    # Public variables
    coxa = None
    tibia = None
    femur = None
    joints = (coxa, tibia, femur)

    def __init__(self, kit, legNum, leftright):
        # Provides arguments for the attachment of the individual servos
        if leftright == 'left':
            self._left_right = True
            self._coxa_num = 15 - (legNum-1)*4 # 1: 15, 2: 11, 3: 7
            self._tibia_num = self._coxa_num - 1
            self._tibia_num = self._coxa_num - 2

        elif leftright == 'right':
            self._left_right = False
            self._coxa_num = (legNum-1)*4 # 1: 0, 2: 4, 3: 8
            self._tibia_num = self._coxa_num + 1
            self._tibia_num = self._coxa_num + 2
            
        else:
            raise NameError("Invalid argument in leftright")

        self.coxa = legJoint(kit, self._coxa_num)
        self.tibia = legJoint(kit, self._tibia_num)
        self.femur = legJoint(kit, self._tibia_num) # Attach these things.
        self.joints = (self.coxa, self.tibia, self.femur) # update

    def update_limits(self, limits):
        if len(limits) == len(self.joints):
            for x in range(len(self.joints)):
                self.joints[x].writeLimit(limits[x])
            return True
        else:
            return False
    
    # def update_limits(self, limit_coxa, limit_tibia, limit_femur):
    #     self.coxa.writeLimit(limit_coxa)
    #     self.tibia.writeLimit(limit_tibia)
    #     self.femur.writeLimit(limit_femur)

    # directly writes servo angles to the servos
    def write_angles(self, angles):
        if len(angles) == len(self.joints):
            self._servo_angles = angles
            for x in range(len(self.joints)):
                self.joints[x].writeAngle(self._servo_angles[x])
            return True
        else:
            return False

    # Calculates servo angles based on desired x,y,z positions
    def calc_tip_angles(self, xyz):
        # TODO error handling if desired coordinate is not reachable within constraints

        coxa_angle = np.arctan2(xyz[0], xyz[1]) + 90
        femur_angle = np.arccos( (self._femur_len**2 - self._tibia_len**2 + self._l_len**2) / (2* self._femur_len * self._l_len) ) + np.arccos(self._z_offset / self._l_len)
        tibia_angle = np.arccos( (self._femur_len**2 + self._tibia_len**2 - self._l_len**2) / (2* self._femur_len * self._tibia_len) )
        self._servo_angles[0] = coxa_angle # coxa angle
        self._servo_angles[1] = femur_angle # femur angle
        self._servo_angles[2] = tibia_angle # tibia angle
        
        return self._servo_angles

    # update current coords with feedback from servo object (forward kinematics)
    def get_curr_xyz(self):
        coxa_angle = self.coxa.currAngle
        femur_angle = self.femur.currAngle
        tibia_angle = self.tibia.currAngle

        return info


    def write_tip_angles(self, xyz):


    # return current position of leg tip
    def curr_tip_pos(self):
        return self._tip_coords

    # Prints debug info
    def debug_print(self):
        for servo in self.joints:
            servo.debug_print()

    # Returns a list of debug info
    def debug(self):
        info = []
        for servo in self.joints:
            info.append(servo.debug)

        return info

# Starts the servo driver.
class legDriver:
    # Public variables
    leg1 = None
    leg2 = None
    leg3 = None
    # Private Variables
    
    # attaches all legs to the servo controller.
    def __init__(self, left_right, addr):
        if left_right != 'left' or left_right != 'right':
            raise NameError("Invalid argument in left_right")
        else:
            kit = ServoKit(channels=16, address=addr)
            self.leg1 = hexLeg(kit, legNum=1, leftright=left_right) # attach all servos here
            self.leg2 = hexLeg(kit, legNum=2, leftright=left_right)
            self.leg3 = hexLeg(kit, legNum=3, leftright=left_right)


# Contains the body IK
class hexapod:
    # Private variables
    
    # Public variables
    left_legs = None
    right_legs = None
    legs = None

    def __init__(self, left_addr=0x42, right_addr=0x41):
        self.left_legs = legDriver('left', addr=left_addr)
        self.right_legs = legDriver('right', addr=right_addr)

        # List up
        self.legs = (self.right_legs.leg1, self.right_legs.leg2, self.right_legs.leg3,
                self.left_legs.leg3, self.left_legs.leg2, self.left_legs.leg1)
