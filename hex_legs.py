# Header file for walkybot.
# Instanstiates the legs and applies corrections.

# types of object: 
# Servo object - simple end effector (will have 18)
# Leg object - combination of three servos

from adafruit_servokit import ServoKit
import warnings # Tells if we are doing something wrong

class legJoint:
    _offset = 0 # can be tuned for non-idealites in hardware setup
    _limit = 30 # Limit to motion, servo cannot pass this value
    _start_angle = 90
    _angle_max = curr_angle + limit
    _angle_min = curr_angle - limit
    curr_angle = 0 # Current angle to be written to servo

    def __init__(self, start_angle=90, limit=30, offset=0):
        # Initialises servo values. Here, start angle refers to the neutral position of the servo.
        _offset = offset
        _limit = limit
        _start_angle = start_angle + _offset
        _angle_max = _start_angle + _limit
        _angle_min = _start_angle - _limit

        curr_angle = _start_angle # init this stuff.

    def writeLimit(self, limit):
        # Updates the limits of the object.
        _limit = limit
        _angle_max = _start_angle + _limit
        _angle_min = _start_angle - _limit

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

# initialize everything from hexleg.
class hexLeg:

    def __init__(self, right_addr=0x41, left_addr=0x42):
        # Initializes the two servo control boards
        leftServos = ServoKit(channels=16, address=left_addr)
        rightServos = ServoKit(channels=16, address=right_addr)

    def attachLeg(self, servoNumber=0, leftright='left'):
        # Attaches leg object to servo.
        if leftright == 'left':
            
        elif leftright == 'right':
            
        else:
            raise NameError("Invalid argument in leftright")
