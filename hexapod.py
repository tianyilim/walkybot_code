import numpy as np
import warnings
from adafruit_servokit import ServoKit
import time
# Robot coordinate class
# Constants

# Helper functions
def to_rads(num):
    return num*np.pi/180
def to_degs(num):
    return num*180/np.pi
def pythagoras(nums):
    return np.sqrt(np.sum(np.power(nums, 2), axis=1))

# Class for legs and joints
class leg_joint:
    # "Public" variables
    curr_angle = 90
    _servo_num = 0
    _offset = 0 # can be tuned for non-idealites in hardware setup
    _limit = 30 # Limit to motion, servo cannot pass this value
    _start_angle = curr_angle
    _angle_max = curr_angle + _offset + _limit
    _angle_min = curr_angle + _offset - _limit
    
    name = ''
    _servo_driver = None # empty variable for now

    def __init__(self, servo_num, servoKit, name, start_angle=90, limit=30, offset=0):
        # Initialises servo values. Here, start angle refers to the neutral position of the servo.
        self._servo_num = servo_num
        self._offset = offset
        self._limit = limit
        self._start_angle = start_angle + self._offset
        self._angle_max = self._start_angle + self._limit
        self._angle_min = self._start_angle - self._limit
        self.name = name

        self._servo_driver = servoKit.servo[servo_num]

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
            warnings.warn("%s: Exceeded +ve limit" %self.name)
        elif try_angle < self._angle_min:
            self.curr_angle = self._angle_min
            warnings.warn("%s: Exceeded -ve limit" %self.name)
        else:
            self.curr_angle = try_angle

        self._servo_driver.angle = self.curr_angle # Write servo angle here.

        return self.curr_angle

    # Prints info to debug stuff with on to console
    def debug_print(self):
        print("For servo", self.name)
        print("Current Angle: %0.4f" %self.curr_angle, "Offset:", self._offset)
        print("+ve limit:", self._angle_max, "-ve limit:", self._angle_min)
    
    # returns a list of important info
    def debug_return(self):
        info = [self.curr_angle, self._offset, self._angle_max, self._angle_min]
        return info

class hex_leg:
    # Similar to body class, use getter-setter fns
    # body class will interact with several leg classes. The leg classes perform IK calculations and handle lower-level actions
    _leg_angles = np.tile(90.0, 3)
    _leg_end = np.zeros(3) # local coordinates for the leg tip/end 
    _leg_ori = np.zeros(3) # local coordinates for leg origin

    joints = None
    leg_name = ''

    left_right_mult = True

    COXA_LEN = 28.75
    FEMUR_LEN = 40.0
    TIBIA_LEN = 68.825
    LEG_ORI_START = np.array((0.0, 0.0, 68.825))

    def __init__(self, leg_end, leg_ori_z, leg_nums, servoKit, leg_name, left_right,
                limits=(45.0, 70.0, 80.0), offsets=(0.0, 0.0, 0.0) ):

        self._leg_ori[2] = leg_ori_z
        self._leg_end = leg_end
        self.leg_name = leg_name

        if left_right=='left':
            self.left_right_mult = False
        elif left_right=='right':
            self.left_right_mult = True
        else:
            print("Invalid input!")

        coxa_joint = leg_joint(servo_num=leg_nums[0], servoKit=servoKit, name=leg_name+' coxa', limit=limits[0], offset=offsets[0])
        femur_joint = leg_joint(servo_num=leg_nums[1], servoKit=servoKit, name=leg_name+' femur', limit=limits[1], offset=offsets[1])
        tibia_joint = leg_joint(servo_num=leg_nums[2], servoKit=servoKit, name=leg_name+' tibia', limit=limits[2], offset=offsets[2])
        self.joints = (coxa_joint, femur_joint, tibia_joint)
        self.writeAngles()

    def writeAngles(self):
        # print("\nmoving servos!")
        # debug
        # self.debug_print()
        if self.left_right_mult == True:
            self.joints[0].writeAngle( self._leg_angles[0])
            self.joints[1].writeAngle( self._leg_angles[1])
            self.joints[2].writeAngle( 180-self._leg_angles[2])
        else:
            self.joints[0].writeAngle( self._leg_angles[0])
            self.joints[1].writeAngle( 180-self._leg_angles[1])
            self.joints[2].writeAngle( self._leg_angles[2])

    def writeCoxa(self):
        self.joints[0].writeAngle( self._leg_angles[0])
    
    def writeFemur(self):
        if self.left_right_mult == True:
            self.joints[1].writeAngle( self._leg_angles[1])
        else:
            self.joints[1].writeAngle( 180-self._leg_angles[1])

    def writeTibia(self):
        if self.left_right_mult == True:
            self.joints[2].writeAngle( 180-self._leg_angles[2])
        else:
            self.joints[2].writeAngle( self._leg_angles[2])

    # Leg tip moves, leg origin does not
    def swing(self, leg_end):
        coxa_angle_ik = np.arctan2(leg_end[0], leg_end[1]) # 'gamma' in notes

        coxa_len_ik = self.COXA_LEN * np.cos( coxa_angle_ik ) # Takes side view of this thing.
        femur_len_ik = self.FEMUR_LEN * np.cos( coxa_angle_ik )
        tibia_len_ik = self.TIBIA_LEN * np.cos( coxa_angle_ik )

        y_offset_ik = leg_end[1] - coxa_len_ik
        z_offset_ik = self.TIBIA_LEN - leg_end[2]
        l_len = np.sqrt( np.power(z_offset_ik,2) + np.power(y_offset_ik,2) )

        # print("z_offset_ik: %0.4f | y_offset_ik: %0.4f | l_len: %0.4f" %(z_offset_ik, y_offset_ik, l_len) )
        # print("coxa_len_ik: %0.4f | femur_len_ik: %0.4f | tibia_len_ik: %0.4f" %(coxa_len_ik, femur_len_ik, tibia_len_ik))
    
        arg1 = (femur_len_ik**2 + l_len**2 - tibia_len_ik**2) / (2*femur_len_ik*l_len)
        arg2 = (femur_len_ik**2 - l_len**2 + tibia_len_ik**2) / (2*femur_len_ik*tibia_len_ik)
        # print(arg1, arg2)

        # Error catching
        if (-1.0 <= arg1 <= 1.0) or (-1.0 <= arg2 <= 1.0):
            femur_angle_ik1 = np.arccos( z_offset_ik / l_len )
            femur_angle_ik2 = np.arccos( arg1 )
            femur_angle_ik = femur_angle_ik1 + femur_angle_ik2
            tibia_angle_ik = np.arccos( arg2 )

            # print("femur_angle_ik1: %0.4f | femur_angle_ik2: %0.4f | femur_angle_ik: %0.4f | tibia_angle_ik: %0.4f" %(femur_angle_ik1, femur_angle_ik2, femur_angle_ik, tibia_angle_ik))
            # print("femur_angle_ik: %0.4f | tibia_angle_ik: %0.4f" %(femur_angle_ik, tibia_angle_ik))

            # Update current values (Should skip if returned false)
            self._leg_end = leg_end
            self._leg_angles = np.array( (to_degs(coxa_angle_ik)+90, to_degs(femur_angle_ik), to_degs(tibia_angle_ik)) )
            # self.writeAngles() # Don't write angles just yet
            
            # print("coxa angle: %0.4f | femur angle: %0.4f | tibia angle: %0.4f" %(self._leg_angles[0], self._leg_angles[1], self._leg_angles[2]))

            return True

        else:
            warnings.warn("%s, position is not possible" %(self.leg_name))
            return False
        

    # Leg origin moves, leg end does not.
    def stance(self, leg_ori):
        diff = leg_ori - self.LEG_ORI_START
        # print("Curr origin:", self._leg_ori, "Diff:", diff)
        save_leg_end = self._leg_end
        swing_success = self.swing(self._leg_end-diff)
        self._leg_ori = leg_ori # update leg origin
        self._leg_end = save_leg_end # Do not modify leg origin
        if swing_success is True:
            return True
        else:
            return False
        # print("Update origin:", self._leg_ori, "Update tip:", self._leg_end)

    # Sets leg angles arbitrarily and use FK to update leg end coordinates
    # assume swing: that leg origin z-height is const (modify tip z-height)
    # else stance: that leg tip z-height is const (modify origin z-height)
    def set_leg_angles(self, angles, assume_swing=True):
        self._leg_angles = angles
        alpha = angles[1]
        beta = angles[2]

        femur_yx = self.FEMUR_LEN*np.cos(to_rads(alpha-90))
        tibia_yx = self.TIBIA_LEN*np.sin(to_rads(alpha+beta-180)) # respective lengths viewed from top (y-x plane)

        femur_yz = self.FEMUR_LEN*np.sin(to_rads(alpha-90))
        tibia_yz = self.TIBIA_LEN*np.cos(to_rads(alpha+beta-180)) # respective lengths viewed from side (y-z plane)

        tip_x = (self.COXA_LEN + femur_yx + tibia_yx) * np.sin(to_rads(angles[0]-90))
        tip_y = (self.COXA_LEN + femur_yx + tibia_yx) * np.cos(to_rads(angles[0]-90))

        if assume_swing:
            tip_z = self._leg_ori[2] + femur_yz + tibia_yz
            self._leg_end[2] = tip_z
        else:
            tip_z = self._leg_end[2] - femur_yz - tibia_yz
            self._leg_ori[2] = tip_z

        self._leg_end[0:2] = (tip_x, tip_y)
        self.writeAngles()

    def get_leg_ori(self):
        return self._leg_ori
    def get_leg_end(self):
        return self._leg_end
    def get_leg_angles(self):
        return self._leg_angles
    
    def debug_print(self):
        np.set_printoptions(precision=3, suppress=True) # print prettier
        print(self.leg_name)
        print("Leg origin:", self._leg_ori)
        print("Leg end:", self._leg_end)
        print("Leg angles:", self._leg_angles)
        print("")
        
# Class for hexapod
class hexapod:
    # As always, leg order is (R123,L321) and coords are (x,y,z)
    # DO NOT WRITE TO THESE DIRECLTY - getter/setter
    _leg_end_loc = np.zeros((6,3)) # Local coordinates of leg tips
    _leg_ori_loc = np.zeros((6,3)) # local coordinates of leg origins (local to each leg)
    _leg_end_abs = np.zeros((6,3)) # absolute coordinates
    _leg_ori_abs = np.zeros((6,3))
    _leg_angle = np.zeros(6) # Each leg's coxa angle (gamma) - in the leg frame. Default is 0.0
    _body_z = 68.825 # Body origin (0,0,z)
    _roll = 0.0
    _pitch = 0.0
    _yaw = 0.0
    time_ref = 0.0

    # body parameters
    X0_LEN = 45.768
    Y0_LEN = 26.424
    Y1_LEN = 52.848
    OFFSET_ROLL = np.array((Y0_LEN, Y1_LEN, Y0_LEN, -Y0_LEN, -Y1_LEN, -Y0_LEN)) # Roll modifies Z based on y-coord
    OFFSET_PITCH = np.array((X0_LEN, 0.0, -X0_LEN, -X0_LEN, 0.0, X0_LEN)) # Pitch modifies Z based on x-coord
    OFFSET_Z = np.tile(_body_z,6)
    OFFSET_ANGLE = np.array((30, 90, 150, 210, 270, 330), dtype='float32') # Modify angle offset for each
    OFFSET_ORIGINS = np.transpose((OFFSET_PITCH, OFFSET_ROLL,OFFSET_Z))
    RAISE_OFFSET = 10 # Leg will raise to -10 from body height.

    # Addressing all legs
    legs = None
    legset1 = None
    legset2 = None
    
    def __init__(self, leg_end_loc=None, leg_ori_loc=None, leg_end_abs=None, 
                leg_angle=np.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0)), body_z=68.825, roll=0.0, pitch=0.0):

        self._leg_angle = leg_angle # initialise these constants
        self._body_z = body_z
        self._roll = roll
        self._pitch = pitch

        # Set leg origins based on roll and pitch settings
        self._leg_ori_loc[:, 2] = self._body_z + self.OFFSET_ROLL*np.sin(to_rads(self._roll)) + self.OFFSET_PITCH*np.sin(to_rads(self._pitch))
        self._leg_ori_abs[:, 2] = self._leg_ori_loc[:, 2]
        self._leg_ori_abs[:, 0] = self.OFFSET_PITCH*np.cos(to_rads(self._pitch))
        self._leg_ori_abs[:, 1] = self.OFFSET_ROLL*np.cos(to_rads(self._roll))

        if leg_end_loc is None: 
            if leg_end_abs is None:
                print("Specify local or absolute coordinates for leg tips!")
            else:
                self.set_leg_end_abs(leg_end_abs)
        elif leg_end_abs is not None:
            print("Only local or absolute coordinates for leg tips should be given!")
        else:
            self.set_leg_end_loc(leg_end_loc)

        np.set_printoptions(precision=3, suppress=True) # print prettier

        # Init all six legs
        leftServos = ServoKit(channels=16, address=0x42)
        rightServos = ServoKit(channels=16, address=0x41)
        leg_r1 = hex_leg(self._leg_end_loc[0], self._leg_ori_loc[0, 2], leg_nums=(15,14,13), servoKit=rightServos, 
                        left_right='right', leg_name="leg_r1", offsets=(25,25,30))
        leg_r2 = hex_leg(self._leg_end_loc[1], self._leg_ori_loc[1, 2], leg_nums=(11,10,9), servoKit=rightServos,
                        left_right='right', leg_name="leg_r2", offsets=(0,20,35))
        leg_r3 = hex_leg(self._leg_end_loc[2], self._leg_ori_loc[2, 2], leg_nums=(7,6,5), servoKit=rightServos,
                        left_right='right', leg_name="leg_r3", offsets=(0,25,5))

        leg_l1 = hex_leg(self._leg_end_loc[3], self._leg_ori_loc[3, 2], leg_nums=(0,1,2), servoKit=leftServos,
                        left_right='left', leg_name="leg_l1", offsets=(0,-10,-5))
        leg_l2 = hex_leg(self._leg_end_loc[4], self._leg_ori_loc[4, 2], leg_nums=(4,5,6), servoKit=leftServos,
                        left_right='left', leg_name="leg_l2", offsets=(0,15,15))
        leg_l3 = hex_leg(self._leg_end_loc[5], self._leg_ori_loc[5, 2], leg_nums=(8,9,10), servoKit=leftServos,
                        left_right='left', leg_name="leg_l3", offsets=(10,10,0))

        self.legs = (leg_r1, leg_r2, leg_r3, leg_l3, leg_l2, leg_l1)
        self.legset1 = (leg_r1, leg_r3, leg_l2, 0, 2, 4)
        self.legset2 = (leg_r2, leg_l3, leg_l1, 1, 3, 5)
        self.time_ref = time.time() # Start time of epoch - used for smoothing out motions

    # Sets local value for leg tip, (updates abs values accordingly)
    def set_leg_end_loc(self, newval):
        self._leg_end_loc = newval
        self._leg_end_abs[:, 2] = self._leg_end_loc[:, 2] # Z is absolute
        leg_len = pythagoras(self._leg_end_loc[:, 0:2])
        phi = self.OFFSET_ANGLE + self._leg_angle + self._yaw
        self._leg_end_abs[:, 0] = self._leg_ori_abs[:, 0] + leg_len * np.cos(to_rads(phi))
        self._leg_end_abs[:, 1] = self._leg_ori_abs[:, 1] + leg_len * np.sin(to_rads(phi))
        # phi = 180-self.OFFSET_ANGLE-self._leg_angle
        # self._leg_end_abs[:, 0] = self._leg_ori_abs[:, 0] + leg_len*np.sin(to_rads(phi))
        # self._leg_end_abs[:, 1] = self._leg_ori_abs[:, 1] + leg_len*np.cos(to_rads(phi))

    # Sets absolute value for leg tip (updates local values accordingly)
    # Overwrites angle as well
    # need to take into account yaw values too (?)
    def set_leg_end_abs(self, newval):
        self._leg_end_abs = newval
        self._leg_end_loc[:, 2] = self._leg_end_abs[:, 2] # Z is absolute
        # Calc dist of leg from base (final-initial)
        delta_xy = self._leg_end_abs[:, 0:2] - self._leg_ori_abs[:, 0:2]
        leg_len = pythagoras(delta_xy)
        print("delta_xy:", delta_xy, "leg_len:", leg_len)
        # Find angle of leg
        self._leg_angle = to_degs( np.arctan2(delta_xy[:, 1], delta_xy[:, 0]) ) - self.OFFSET_ANGLE
        print("Leg angles:\n", self._leg_angle)
        # calculate xy values (Offset by their initial val)
        self._leg_end_loc[:, 0] = leg_len * np.sin(to_rads(-self._leg_angle)) + self._leg_ori_loc[:,0]
        self._leg_end_loc[:, 1] = leg_len * np.cos(to_rads(self._leg_angle)) + self._leg_ori_loc[:,1]

    # Sets value for leg (coxa) angle - should change both absolute and local
    # leg positions(x,y) as well.
    def set_leg_angle(self, newval):
        self._leg_angle = newval
        leg_len = pythagoras(self._leg_end_loc[:, 0:2])
        self._leg_end_loc[:, 0] = leg_len * np.sin(to_rads(-self._leg_angle))
        self._leg_end_loc[:, 1] = leg_len * np.cos(to_rads(self._leg_angle))
        self.set_leg_end_loc(self._leg_end_loc)
        # phi = 180-self.OFFSET_ANGLE-self._leg_angle
        # self._leg_end_abs[:, 0] = self._leg_ori_abs[:, 0] + leg_len*np.sin(to_rads(phi))
        # self._leg_end_abs[:, 1] = self._leg_ori_abs[:, 1] + leg_len*np.cos(to_rads(phi))

    def set_indiv_leg_end_loc(self, leg, newval):
        self._leg_end_loc[leg] = newval
        self._leg_end_abs[leg, 2] = self._leg_end_loc[leg, 2] # Z is absolute
        leg_len = pythagoras(self._leg_end_loc[leg, 0:2])
        phi = self.OFFSET_ANGLE[leg] + self._leg_angle[leg] + self._yaw
        self._leg_end_abs[leg, 0] = self._leg_ori_abs[leg, 0] + leg_len*np.cos(to_rads(phi))
        self._leg_end_abs[leg, 1] = self._leg_ori_abs[leg, 1] + leg_len*np.sin(to_rads(phi))

    def set_indiv_leg_end_abs(self, leg, newval):
        self._leg_end_abs[leg] = newval
        self._leg_end_loc[leg, 2] = self._leg_end_abs[leg, 2] # Z is absolute
        delta_xy = self._leg_end_abs[leg, 0:2] - self._leg_ori_abs[leg, 0:2] # Difference in x-y coordinates from leg origin
        leg_len = pythagoras(delta_xy)
        self._leg_angle[leg] = to_degs( np.arctan2(delta_xy[1], delta_xy[0]) ) - self.OFFSET_ANGLE
        self._leg_end_loc[leg, 0] = leg_len * np.sin(to_rads(-self._leg_angle[leg]))
        self._leg_end_loc[leg, 1] = leg_len * np.cos(to_rads(self._leg_angle[leg]))

    def set_indiv_leg_angle(self, leg, newval):
        self._leg_angle[leg] = newval
        leg_len = pythagoras(self._leg_end_loc[leg, 0:2])
        self._leg_end_loc[leg, 0] = leg_len * np.sin(to_rads(-self._leg_angle[leg]))
        self._leg_end_loc[leg, 1] = leg_len * np.cos(to_rads(self._leg_angle[leg]))
        self.set_indiv_leg_end_loc(leg, self._leg_end_loc)

    # Sets value for leg origin (updates local values accordingly)
    def set_leg_ori_abs(self, newval):
        delta = newval - self.OFFSET_ORIGINS # diff between "regular positions" and commanded ones
        offset_angle = self._yaw + self.OFFSET_ANGLE
        deltax = delta[:,0]
        deltay = delta[:,1]
        # print("yaw:", self._yaw, "\ndelta x:", deltax, "\ndelta y:", deltay, 
        #     "\noffset_angle:\n", offset_angle)

        self._leg_ori_abs = newval
        self._leg_ori_loc[:,2] = newval[:,2] # Z is absolute as usual
        self._leg_ori_loc[:,0] = deltax*np.cos(to_rads(90-offset_angle)) + deltay*np.sin(to_rads(offset_angle-90))
        self._leg_ori_loc[:,1] = deltay*np.cos(to_rads(offset_angle-90)) + deltax*np.sin(to_rads(90-offset_angle))
        
        # print("Leg origins (absolute)\n", self._leg_ori_abs)
        # print("Leg origins (local):\n", self._leg_ori_loc)

    # Rolls the body. Body absolute and relative coordinates change. Call this before any other functions!
    def body_roll(self, roll):
        self._roll = roll
        newvals = np.copy(self._leg_ori_abs)
        newvals[:,2] = self._body_z + self.OFFSET_ROLL*np.sin(-to_rads(self._roll)) + self.OFFSET_PITCH*np.sin(to_rads(-self._pitch))
        newvals[:,1] = self.OFFSET_ROLL*np.cos(-to_rads(self._roll))
        self.set_leg_ori_abs(newvals)

    # Pitches the body. Body absolute and relative coordinates change.
    def body_pitch(self, pitch):
        self._pitch = pitch
        newvals = np.copy(self._leg_ori_abs)
        newvals[:,2] = self._body_z + self.OFFSET_PITCH*np.sin(to_rads(-self._pitch)) + self.OFFSET_ROLL*np.sin(-to_rads(self._roll))
        newvals[:,0] = self.OFFSET_PITCH*np.cos(-to_rads(self._pitch))
        self.set_leg_ori_abs(newvals)

    # Rotates the body (relative). Body absolute and relative coordinates change, but leg does not.
    def body_rotate(self, theta):
        self._yaw += theta
        distances = pythagoras( self._leg_ori_abs[:, 0:2] )
        angles = to_degs(np.arctan2( self._leg_ori_abs[:, 1], self._leg_ori_abs[:, 0]))+theta
        # print("Angles:", angles)
        newvals = np.copy(self._leg_ori_abs)
        newvals[:, 0] = distances * np.cos( to_rads( angles ) )
        newvals[:, 1] = distances * np.sin( to_rads( angles ) )
        self.set_leg_ori_abs(newvals)

    def body_rotate_absolute(self, theta):
        self._yaw = theta
        angles = self.OFFSET_ANGLE + theta
        distances = pythagoras( self._leg_ori_abs[:, 0:2] )
        # print("Absolute Origin Angles:", angles)
        # print("Absolute Origin Distances:", distances)
        newvals = np.copy(self._leg_ori_abs)
        # Rotation happens here
        newvals[:, 0] = distances * np.cos( to_rads( angles ) )
        newvals[:, 1] = distances * np.sin( to_rads( angles ) )
        # print("New absolute origins after rotation:\n", newvals)
        self.set_leg_ori_abs(newvals)

    # These translate to an absolute value
    def body_translate_x_absolute(self, x):
        new_vals = np.copy(self._leg_ori_abs)
        new_vals[:, 0] = self.OFFSET_PITCH + x
        self.set_leg_ori_abs(new_vals)

    def body_translate_y_absolute(self, y):
        new_vals = np.copy(self._leg_ori_abs)
        new_vals[:, 1] = self.OFFSET_ROLL + y
        self.set_leg_ori_abs(new_vals)

    def body_translate_z_absolute(self, z):
        if z < 30:
            warnings.warn("Exceeded minimum Z-height")
            self._body_z = 30.0
        elif z > 100:
            warnings.warn("Exceeded maximum Z-height")
            self._body_z = 100.0
        else:
            self._body_z = z
        self.body_pitch(self._pitch) # Updates z-height, then for the other stuff.
        self.body_roll(self._roll)

    # These simply modify the value of the x/y/z coordinate
    def body_translate_x(self, x):
        new_vals = np.copy(self._leg_ori_abs)
        new_vals[:, 0] += x
        self.set_leg_ori_abs(new_vals)

    def body_translate_y(self, y):
        new_vals = np.copy(self._leg_ori_abs)
        new_vals[:, 1] += y
        self.set_leg_ori_abs(new_vals)

    def body_translate_z(self, z):
        try_z = self._body_z + z
        if try_z < 30:
            warnings.warn("Exceeded minimum Z-height")
            self._body_z = 30.0
        elif try_z > 100:
            warnings.warn("Exceeded maximum Z-height")
            self._body_z = 100.0
        else:
            self._body_z = try_z
            
        self.body_pitch(self._pitch) # Updates z-height, then for the other stuff.
        self.body_roll(self._roll)

    # Recenters origin coordinates back to (0,0) - after successfully moving?
    def body_recenter(self):
        self._leg_ori_abs[:,0:2] = 0.0
        self._leg_ori_loc[:,0] = self.OFFSET_PITCH
        self._leg_ori_loc[:,1] = self.OFFSET_ROLL
        self.body_pitch(self._pitch) # Updates z-height, then for the other stuff.
        self.body_roll(self._roll)
    
    # Allows for the individual updating of legs by leg index
    def update_indiv_leg(self, leg):
        self.legs[leg].swing(self._leg_end_loc[leg])

    # Writing updated coordinates to the legs. Legs move but origin does not.
    def update_legs_swing(self, legset=None):
        if legset is None:
            for i in range(len(self.legs)):
                self.legs[i].swing(self._leg_end_loc[i])
        else:
            for x in legset[3:6]:
                self.legs[x].swing(self._leg_end_loc[x])

        # Write leg angles after calculating them!
        self.write_leg_angles()
    
    # Umbrella function for movement.
    # X +ve: forwards. -ve: backwards.
    # Y +ve: right. -ve: left.
    # Choose between ripple(slow), wave(med), and tripod(fast) gaits
    # Max step is the biggest diff coordinates allowed
    def move_hexapod(self, speed_x, speed_y, speed_rotation, MAX_STEP=30.0, MAX_ROTATE=30.0):
        # First, legset1 moves...
        delta_x = (speed_x/127)*MAX_STEP
        delta_y = (speed_y/127)*MAX_STEP
        delta_yaw = (speed_rotation/127)*MAX_ROTATE
        print("Delta_X: %0.4f | Delta_Y: %0.4f | Delta_R: %0.4f" %(delta_x, delta_y, delta_yaw))

        # Simple translation
        newvals = np.copy(self._leg_end_abs)
        newvals[:,0] += delta_x
        newvals[:,1] += delta_y
        print("New coords to move to bef rotation:\n", newvals, '\n')

        # ROTATION DOES NOT WORK YET
        # # Rotate these leg end coords
        # angles = self.OFFSET_ANGLE + delta_yaw
        # distances = pythagoras( newvals[:, 0:2] )
        # # Rotation happens here
        # newvals[:, 0] = distances * np.cos( to_rads( angles ) )
        # newvals[:, 1] = distances * np.sin( to_rads( angles ) )

        # print("New coords to move to:\n", newvals)

        # Calculate vals for legset1
        legset1_coords = np.array((newvals[0], self._leg_end_abs[1], newvals[2], self._leg_end_abs[3], newvals[4], self._leg_end_abs[5]))
        self.raised_swing_abs(self.legset1, legset1_coords)
        # Calculate vals for legset2
        legset2_coords = np.array((self._leg_end_abs[0], newvals[1], self._leg_end_abs[2], newvals[3], self._leg_end_abs[4], newvals[5]))
        self.raised_swing_abs(self.legset2, legset2_coords)
        
        # Calculate vals for body coordinates
        self.body_rotate_absolute(delta_yaw)
        self.update_legs_stance()

    # First raises leg to a arbitrary place before placing it down.
    # New value will be in absolute coordinates.
    # This prevents leg dragging
    def raised_swing_abs(self, legset, newval, speed=2.0):
        move_time = 1/speed # Time taken for one swing movement
        # First calculate the midpoint bewteen current and future positions:
        delta = newval - self._leg_end_abs
        # Move to the midpoint
        target_pos = np.copy(delta)
        target_pos[:,0:2] = delta[:,0:2]/2 + self._leg_end_abs[:,0:2]
        target_pos[:,2] = self._body_z + self.RAISE_OFFSET

        print("Target move pos:\n", target_pos)

        start_pos = np.copy(self._leg_end_abs)
        gradient = (target_pos - self._leg_end_abs) / move_time # Gradient is the rate of movement in each axis(xyz) wrt time.
        
        self.time_ref = time.time() # Update start time here
        t = 0.0
        while t < move_time:
            # Calculate position on the movement path
            print("Time:", t, "Curr movement pos:\n", (start_pos + gradient*t) )
            self.set_leg_end_abs( start_pos + gradient*t ) # Move legs WRT time...
            print("Local leg coords:\n", self._leg_end_loc)
            self.update_legs_swing(legset)
            t = time.time() - self.time_ref

        # Reset parameters for moving downwards
        start_pos = np.copy(self._leg_end_abs)
        gradient = (newval - self._leg_end_abs) / move_time # Gradient is the rate of movement in each axis(xyz) wrt time.
        self.time_ref = time.time() # Update start time here
        t = 0.0
        while t < move_time:
            # Calculate position on the movement path
            self.set_leg_end_abs( start_pos + gradient*t ) # Move legs WRT time...
            self.update_legs_swing(legset)
            t = time.time() - self.time_ref
        
    # # First raises leg to a arbitrary place before placing it down.
    # # New value will be in coordinates local to individual leg.
    # # This prevents leg dragging
    # def raised_swing_loc(self, leg, newval, delay=0.1):
    #     # First calculate the midpoint bewteen current and future positions:
    #     delta = newval - self._leg_end_loc[leg]
    #     # Move to the midpoint
    #     target_pos = np.copy(delta)
    #     target_pos[0:2] /= 2
    #     target_pos[2] = self._body_z - self.RAISE_OFFSET

    #     self.set_indiv_leg_end_loc(leg, target_pos)
    #     self.update_indiv_leg(leg)

    #     time.sleep(delay)

    #     self.set_indiv_leg_end_loc(leg, newval)
    #     self.update_indiv_leg(leg)

    # Origin moves but leg does not.
    def update_legs_stance(self):
        # leg_ori_towrite = np.copy(self._leg_ori_loc)
        for i in range(len(self.legs)):
            # self.legs[i].stance(leg_ori_towrite[i])
            # print("Leg",i,"writing:",self._leg_ori_loc[i])
            self.legs[i].stance(self._leg_ori_loc[i])
            # print("Leg", i, "Writing", self._leg_ori_loc[i], "Gets", self.legs[i].stance(self._leg_ori_loc[i]) )
        self.write_leg_angles()
    
    # actually writes the angles into the motors (in order!)
    def write_leg_angles(self):
        # print("Writing Femur")
        for i in range(len(self.legs)):
            self.legs[i].writeFemur()
        # print("Writing Tibia")
        for i in range(len(self.legs)):
            self.legs[i].writeTibia()
        # print("Writing Coxa")
        for i in range(len(self.legs)):
            self.legs[i].writeCoxa()


    def get_leg_end_loc(self):
        return self._leg_end_loc
    def get_leg_end_abs(self):
        return self._leg_end_abs
    def get_leg_ori_loc(self):
        return self._leg_ori_loc
    def get_leg_ori_abs(self):
        return self._leg_ori_abs
    def get_leg_angle(self):
        return self._leg_angle
    def get_body_z(self):
        return self._body_z

    def print_state(self): # Sanity check
        np.set_printoptions(precision=3, suppress=True) # print prettier
        print("Local leg endpoints:\n", self._leg_end_loc)
        print("Abs leg endpoints:\n", self._leg_end_abs)
        print("Local leg origin:\n", self._leg_ori_loc)
        print("Abs leg origin:\n", self._leg_ori_abs)
        print("Leg Angles:\n", self._leg_angle)
        print("Body Z-height:", self._body_z)
        print("Body roll:", self._roll)
        print("Body pitch:", self._pitch)
        print("") # give a newline