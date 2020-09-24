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
                limits=(45.0, 65.0, 70.0), offsets=(0.0, 0.0, 0.0) ):

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
        # print("%s coxa commanded angle: %0.4f" %(self.leg_name, self._leg_angles[0]))
        self.joints[0].writeAngle( self._leg_angles[0])
    
    def writeFemur(self):
        # print("%s femur commanded angle: %0.4f" %(self.leg_name, self._leg_angles[1]))
        if self.left_right_mult == True:
            self.joints[1].writeAngle( self._leg_angles[1])
        else:
            self.joints[1].writeAngle( 180-self._leg_angles[1])

    def writeTibia(self):
        # print("%s tibia commanded angle: %0.4f" %(self.leg_name, self._leg_angles[2]))
        if self.left_right_mult == True:
            self.joints[2].writeAngle( 180-self._leg_angles[2])
        else:
            self.joints[2].writeAngle( self._leg_angles[2])

    # Leg tip moves, leg origin does not. Updated to reflect current robot state
    def swing(self, leg_end):
        delta = leg_end - self._leg_ori # Delta between leg origin and new end
        coxa_angle_ik = np.arctan2(delta[0], delta[1]) # 'gamma' in notes

        coxa_len_ik = self.COXA_LEN * np.cos( coxa_angle_ik ) # Takes side view of this thing.
        femur_len_ik = self.FEMUR_LEN * np.cos( coxa_angle_ik )
        tibia_len_ik = self.TIBIA_LEN * np.cos( coxa_angle_ik )

        y_offset_ik = leg_end[1] - coxa_len_ik - self._leg_ori[1] # y-distance between tibia joint and desired leg pos
        z_offset_ik = self._leg_ori[2] - leg_end[2] # curr z-height - desired z height
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
            
            # print("For leg", self.leg_name)
            # print("coxa angle: %0.4f | femur angle: %0.4f | tibia angle: %0.4f" %(self._leg_angles[0], self._leg_angles[1], self._leg_angles[2]))
            # print("")
            return True

        else:
            warnings.warn("%s, position is not possible" %(self.leg_name))
            return False

    # Leg origin moves, leg end does not.
    def stance(self, leg_ori):
        diff = leg_ori - self._leg_ori
        # print("Leg", self.leg_name, "stance: Curr origin:\n", self._leg_ori, "\nDiff:\n", diff)
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
    _leg_angle = np.zeros(6) # Each leg's coxa angle (gamma)
    _body_center = np.array((0.0, 0.0, 68.825), dtype='float32')
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
    OFFSET_Z = np.tile(_body_center[2],6)
    OFFSET_ANGLE = np.array((30, 90, 150, 210, 270, 330), dtype='float32') # Modify angle offset for each
    START_LEG_ORI_LOC = np.transpose((OFFSET_PITCH, OFFSET_ROLL,OFFSET_Z)) # local
    START_LEG_END = None # Absolute
    RAISE_OFFSET = 30 # Leg will raise to 20 from the ground

    # addressing all legs
    legs = None
    
    def __init__(self, leg_end_loc=None, leg_ori_loc=None, leg_end_abs=None, 
                leg_angle=np.array((90.0, 90.0, 90.0, 90.0, 90.0, 90.0)), body_z=68.825, roll=0.0, pitch=0.0):

        np.set_printoptions(precision=3, suppress=True) # print prettier
        self._leg_angle = leg_angle # initialise these constants
        self._body_center[2] = body_z
        self._roll = roll
        self._pitch = pitch

        # Set leg origins based on roll and pitch settings
        self._leg_ori_loc[:, 2] = self._body_center[2] + self.OFFSET_ROLL*np.sin(to_rads(self._roll)) + self.OFFSET_PITCH*np.sin(to_rads(self._pitch))
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

        self.START_LEG_END = self.get_leg_end_abs()

        self.time_ref = time.time() # Start time of epoch - used for smoothing out motions

    # Sets local value for leg tip, (updates abs values accordingly)
    def set_leg_end_loc(self, newval, leg_index=np.array((0,1,2,3,4,5)), write=True):
        leg_len = pythagoras(newval[:,0:2])
        # print("Set_leg_end_loc | leg_len:\n", leg_len)
        phi = self.OFFSET_ANGLE[leg_index] + np.arctan2(-newval[:,0], newval[:,1])
        # print("Set_leg_end_loc | phi:\n", phi)
        retval = np.copy(newval)
        retval[:, 0] = self._leg_ori_abs[leg_index, 0] + leg_len * np.cos(to_rads(phi))
        retval[:, 1] = self._leg_ori_abs[leg_index, 1] + leg_len * np.sin(to_rads(phi))
        # print("Set_leg_end_loc | retval:\n", retval)

        if write:
            self._leg_end_loc[leg_index] = newval
            self._leg_end_abs[leg_index] = retval
            self._leg_angle[leg_index] = phi
        
        return retval

    # Sets absolute value for leg tip (updates local values accordingly)
    # Overwrites angle as well
    # need to take into account yaw values too (?)
    def set_leg_end_abs(self, newval, leg_index=np.array((0,1,2,3,4,5)), write=True):
        # print(leg_index, leg_index.shape)
        print("Set_leg_end_abs: | newval:\n", newval, "\nleg_index:", leg_index)
        delta_xy = newval[:, 0:2] - self._leg_ori_abs[leg_index, 0:2] # Difference in x-y coordinates from leg origin
        leg_len = pythagoras(delta_xy)
        print("Set_leg_end_abs: | delta_xy:\n", delta_xy, "\nleg_len:", leg_len)
        leg_angle = to_degs( np.arctan2(delta_xy[:, 1], delta_xy[:, 0]) ) - self.OFFSET_ANGLE[leg_index]
        # normalise angles
        for i in range(len(leg_angle)):
            if (-360-45) <= leg_angle[i] <= (-360+45):
                leg_angle[i] += 360
            elif (360-45) <= leg_angle[i] <= (360+45):
                leg_angle[i] -= 360
        print("Set_leg_end_abs: | Leg angles aft normal:\n", leg_angle)
        retval = np.copy(newval)
        retval[:, 0] = leg_len * np.sin(to_rads(-leg_angle)) + self._leg_ori_loc[leg_index,0]
        retval[:, 1] = leg_len * np.cos(to_rads(leg_angle)) + self._leg_ori_loc[leg_index,1]
        print("Set_leg_end_abs: | Return values (local):\n", retval)

        if write:
            self._leg_end_abs[leg_index] = newval
            self._leg_end_loc[leg_index] = retval
            self._leg_angle[leg_index] = leg_angle

        return retval

    # Sets value for leg (coxa) angle - should change both absolute and local
    # leg positions(x,y) as well.
    def set_leg_angle(self, newval, leg_index=np.array((0,1,2,3,4,5)), write=True):        
        leg_len = pythagoras(self._leg_end_loc[leg_index, 0:2])
        retval = np.copy(self._leg_end_loc[leg_index])
        retval[0] = leg_len * np.sin(to_rads(-newval))
        retval[1] = leg_len * np.cos(to_rads(newval))
        # phi = 180-self.OFFSET_ANGLE-self._leg_angle
        # self._leg_end_abs[:, 0] = self._leg_ori_abs[:, 0] + leg_len*np.sin(to_rads(phi))
        # self._leg_end_abs[:, 1] = self._leg_ori_abs[:, 1] + leg_len*np.cos(to_rads(phi))
        if write:
            self._leg_angle[leg_index] = newval
        
        return self.set_leg_end_loc(retval, leg_index=leg_index, write=write)

    # Sets value for leg origin (updates local values accordingly)
    def set_leg_ori_abs(self, newval, write=True):
        delta = newval - self._leg_ori_abs # diff between "regular positions" and commanded ones
        offset_angle = self._yaw + self.OFFSET_ANGLE
        deltax = delta[:,0]
        deltay = delta[:,1]
        # print("yaw:", self._yaw, "\ndelta x:", deltax, "\ndelta y:", deltay, 
        #     "\noffset_angle:\n", offset_angle)
        retval = np.copy(newval)
        retval[:,0] = deltax*np.cos(to_rads(90-offset_angle)) + deltay*np.sin(to_rads(offset_angle-90))
        retval[:,1] = deltay*np.cos(to_rads(offset_angle-90)) + deltax*np.sin(to_rads(90-offset_angle))
        
        # print("Leg origins (absolute)\n", self._leg_ori_abs)
        # print("Leg origins (local):\n", self._leg_ori_loc)

        if write:
            self._leg_ori_abs = newval
            self._leg_ori_loc = retval

        return retval

    # Rolls the body. Body absolute and relative coordinates change. Call this before any other functions!
    def body_roll(self, roll):
        self._roll = roll
        newvals = np.copy(self._leg_ori_abs)
        newvals[:,2] = self._body_center[2] + self.OFFSET_ROLL*np.sin(-to_rads(self._roll)) + self.OFFSET_PITCH*np.sin(to_rads(-self._pitch))
        newvals[:,1] = self.OFFSET_ROLL*np.cos(-to_rads(self._roll))
        self.set_leg_ori_abs(newvals)

    # Pitches the body. Body absolute and relative coordinates change.
    def body_pitch(self, pitch):
        self._pitch = pitch
        newvals = np.copy(self._leg_ori_abs)
        newvals[:,2] = self._body_center[2] + self.OFFSET_PITCH*np.sin(to_rads(-self._pitch)) + self.OFFSET_ROLL*np.sin(-to_rads(self._roll))
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
        self._body_center[0] += x
        self.set_leg_ori_abs(new_vals)

    def body_translate_y_absolute(self, y):
        new_vals = np.copy(self._leg_ori_abs)
        new_vals[:, 1] = self.OFFSET_ROLL + y
        self._body_center[1] += y
        self.set_leg_ori_abs(new_vals)

    def body_translate_z_absolute(self, z):
        if z < 30:
            warnings.warn("Exceeded minimum Z-height")
            self._body_center[2] = 30.0
        elif z > 100:
            warnings.warn("Exceeded maximum Z-height")
            self._body_center[2] = 100.0
        else:
            self._body_center[2] = z
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
        try_z = self._body_center[2] + z
        if try_z < 30:
            warnings.warn("Exceeded minimum Z-height")
            self._body_center[2] = 30.0
        elif try_z > 100:
            warnings.warn("Exceeded maximum Z-height")
            self._body_center[2] = 100.0
        else:
            self._body_center[2] = try_z
            
        self.body_pitch(self._pitch) # Updates z-height, then for the other stuff.
        self.body_roll(self._roll)

    # delta = (x,y,z)
    def body_translate(self, delta, theta, write=True):
        newvals = np.copy(self._leg_ori_abs)
        newvals[:,:] += delta[:]
        print("body_translate | vals bef rot:\n", newvals)
        angles = to_degs(np.arctan2(newvals[:,1], newvals[:,0])) + theta
        distances = pythagoras( newvals[:,0:2] )
        # Rotation happens here
        newvals[:, 0] = distances * np.cos( to_rads( angles ) )
        newvals[:, 1] = distances * np.sin( to_rads( angles ) )
        # print("New absolute origins after rotation:\n", newvals)
        if write:
            self._body_center += delta
            self._leg_ori_abs = newvals
        return newvals

    # Recenters origin coordinates back to (0,0) - after successfully moving?
    def body_recenter(self):
        self._leg_ori_abs[:,0:2] = 0.0
        self._body_center = np.array((0.0,0.0,68.825))
        self._leg_ori_loc[:,0] = self.OFFSET_PITCH
        self._leg_ori_loc[:,1] = self.OFFSET_ROLL
        self.body_pitch(self._pitch) # Updates z-height, then for the other stuff.
        self.body_roll(self._roll)

    # calculates the required final positions of <legs> based on delta x/y, and rotation angle (theta)
    # Pass in a TUPLE (,) for leg_index!
    def move_legs(self, leg_move, dx, dy, theta, speed=0.75):
        move_time = 1/speed
        leg_stance_list = [i for i in [0,1,2,3,4,5] if i not in leg_move] # Legs that aren't moving at the moment
        leg_stance = np.array(leg_stance_list)
        print("Leg_move:\n", leg_move)
        print("Leg_stance:\n", leg_stance)
        # Calculate the movement required for moving legs
        delta = np.array((dx,dy,0.0))
        leg_delta = np.tile(delta, (len(leg_move),1))
        endpoints = np.take(self._leg_end_abs, leg_move, axis=0) # Takes the relevant slices of the coord array.
        print("Endpoints before adding delta:\n", endpoints)
        startpoint = np.copy(endpoints)

        idealised_endpoint = np.copy(self.START_LEG_END[leg_move])
        idealised_endpoint[:,0] += dx
        idealised_endpoint[:,1] += dy

        idealised_dist = pythagoras(idealised_endpoint[:,0:2])
        idealised_angles = to_degs(np.arctan2(idealised_endpoint[:,1], idealised_endpoint[:,0]))
        print("idealised_dists:\n", idealised_dist, "\nidealised_angles:\n", idealised_angles)

        endpoints[:,0] = self._leg_ori_abs[leg_move,0] + idealised_dist*np.cos(to_rads(-idealised_angles))
        endpoints[:,1] = self._leg_ori_abs[leg_move,1] + idealised_dist*np.sin(to_rads(idealised_angles))

        print("Endpoints after adding delta:\n", endpoints)
        # Rotate (about shifted body center)
        ctr = self._body_center + delta
        print("Rotation center is:\n", ctr)
        rot_dist = endpoints-np.tile(ctr, (len(leg_move),1))
        print("Rotation Distance is\n", rot_dist)
        leg_len = pythagoras(rot_dist[:,0:2])
        print("leg lengths:", leg_len)
        # angles = self.OFFSET_ANGLE[leg_move] + theta
        angles = to_degs(np.arctan2(rot_dist[:,1], rot_dist[:,0])) + theta
        print("Desired angles:", angles)
        endpoints[:, 0] = leg_len * np.cos( to_rads( -angles ) ) + ctr[0]
        endpoints[:, 1] = leg_len * np.sin( to_rads( angles ) ) + ctr[1]
        print("Endpoints after rotation:\n", endpoints)

        # Calculate the movement required for shifting body: move chassis points
        # body_delta = delta / 6 * len(leg_move)
        body_delta = delta
        body_startpoint = self._leg_ori_abs
        body_endpoint = self.body_translate(body_delta, theta, write=False)
        print("Body Delta:\n", body_delta, "\nBody Endpoint\n", body_endpoint)

        # calculate a movement plan
        im_pos = np.copy(leg_delta) # Intermediate position
        im_pos[:,0:2] = leg_delta[:,0:2]/2 + startpoint[:,0:2] # goes to the midpoint 
        im_pos[:,2] = self.RAISE_OFFSET
        print("Intermediate pos:\n", im_pos)

        grad_up = (im_pos - startpoint) / move_time
        grad_down = (endpoints - im_pos) / move_time
        grad_body = (body_endpoint - body_startpoint) / (move_time*2) # movement gradient for body

        print("grad_up:\n", grad_up)
        print("grad_down:\n", grad_down)
        print("grad_body:\n", grad_body)
        # assert(0)

        time_ref = time.time() # Move legs in a pseudo-linear fashion
        t = time.time()-time_ref
        while t < move_time*2:
            print("\n\nCurrent time:", t)
            
            # move upwards
            if t < move_time:
                demand = startpoint + grad_up*t
                self.set_leg_end_abs(demand, leg_index=leg_move)
            # move downwards to endpoint
            else:
                demand = im_pos + grad_down*(t-move_time)
                self.set_leg_end_abs(demand, leg_index=leg_move)

            self.set_leg_ori_abs(body_startpoint+grad_body*t)

            print("Demand:\n", demand)
            print("Local leg origins:\n", self.get_leg_ori_loc())
            print("Local leg ends:\n", self.get_leg_end_loc())

            self.update_legs_stance(leg_stance, write=True)
            self.update_legs_swing(leg_move, write=True) # update legs
            self.write_leg_angles()
            t = time.time()-time_ref

            time.sleep(0.1) # Less datapoints makes me happy

            # assert(0)

        # end point of swing/stance motions
        self.set_leg_ori_abs(body_endpoint)
        self.set_leg_end_abs(endpoints, leg_index=leg_move)
        self.update_legs_stance(leg_stance, write=True)
        self.update_legs_swing(leg_move, write=True) # update legs
        self.write_leg_angles()

    def tripod_gait(self, speed_x, speed_y, speed_rotation, MAX_STEP=30.0, MAX_ROTATE=30.0):
        delta_x = (speed_x/127)*MAX_STEP
        delta_y = (speed_y/127)*MAX_STEP
        delta_yaw = (speed_rotation/127)*MAX_ROTATE

        set1 = np.array((0,2,4))
        set2 = np.array((1,3,5))

        self.move_legs(set1, delta_x, delta_y, delta_yaw)
        self.move_legs(set2, delta_x, delta_y, delta_yaw)
    
    # Writing updated coordinates to the legs. Legs move but origin does not.
    def update_legs_swing(self, leg_index=np.array((0,1,2,3,4,5)), write=True):
        for i in leg_index:
            self.legs[i].swing(self._leg_end_loc[i])
        if write: # If want to sync something
            self.write_leg_angles()

    # Origin moves but leg does not.
    def update_legs_stance(self, leg_index=np.array((0,1,2,3,4,5)), write=True):
        # leg_ori_towrite = np.copy(self._leg_ori_loc)
        for i in leg_index:
            # self.legs[i].stance(leg_ori_towrite[i])
            # print("Leg",i,"writing:",self._leg_ori_loc[i])
            self.legs[i].stance(self._leg_ori_loc[i])
            # print("Leg", i, "Writing", self._leg_ori_loc[i], "Gets", self.legs[i].stance(self._leg_ori_loc[i]) )
        if write:
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
    def get_body_center(self):
        return self._body_center

    def print_state(self): # Sanity check
        np.set_printoptions(precision=3, suppress=True) # print prettier
        print("Local leg endpoints:\n", self._leg_end_loc)
        print("Abs leg endpoints:\n", self._leg_end_abs)
        print("Local leg origin:\n", self._leg_ori_loc)
        print("Abs leg origin:\n", self._leg_ori_abs)
        print("Leg Angles:\n", self._leg_angle)
        print("Body Center:", self._body_center)
        print("Body roll:", self._roll)
        print("Body pitch:", self._pitch)
        print("") # give a newline