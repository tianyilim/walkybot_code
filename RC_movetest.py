''' Socket server, receives (RX) '''
import socket
import sys
# import hex_legs
import hexapod
import numpy as np
from adafruit_servokit import ServoKit
import time
from threading import Thread

# Helper function
def int_twosComp(val, bits=8):
    if val > 2**(bits-1)-1:
        return val - 2**bits
    else:
        return val

SCALE = 256/60
RATE = 256/100
translate = [0.0,0.0] # as a proportion
tilt = [0.0,0.0]
rotate = 0
updown = 0
step_speed = 1.0

target_name = '' # listens to all
target_port = 10240
server_address = (target_name, target_port)

def move_robot():
    global translate
    global tilt
    global rotate
    global updown
    # initialise hexapod object
    leg_end_coords = np.tile((0.0, 68.75, 0.0), (6,1)) # check back
    leg_angles = np.tile(0.0, 6)  # leg angle init
    # print(leg_end_coords, leg_end_coords.shape)
    walky = hexapod.hexapod(leg_end_loc=leg_end_coords, leg_angle=leg_angles)

    # Write, consume it nonblocking
    while True:
        print("Translate values:", translate)
        print("Tilt values:", tilt)
        print("Rotate:", rotate)
        tilt_use = np.copy(tilt)
        translate_use = np.copy(translate)
        updown_use = np.copy(updown)
        rotate_use = np.copy(rotate)

        # walky.body_pitch(tilt_use[0])
        # walky.body_roll(tilt_use[1])
        walky.body_translate_z(updown_use*4)
        walky.tripod_gait(translate_use[0], translate_use[1], rotate_use, tilt_use[0], tilt_use[1], step_speed=4.0)

def get_input():
    global translate
    global tilt
    global rotate
    global updown
    global sock
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(5) # Use this, easier than to make sockets nonblocking
    # Bind the socket to the port
    print('starting up on %s port %s' % server_address)
    sock.bind(server_address)

    while True:
        # print('\nwaiting to receive message')
        data, address = sock.recvfrom(4096)
        if data:
            # print( 'received %s bytes from %s' % (len(data), address) )
            
            # for i,x in enumerate(data):
            #     print(i, int.from_bytes(x, byteorder='big', signed=True))

            # translate[0] = int_twosComp(data[0]) / RATE
            # translate[1] = int_twosComp(data[1]) / RATE
            # tilt[0] = int_twosComp(data[2]) / SCALE
            # tilt[1] = int_twosComp(data[3]) / SCALE
            # rotate = int_twosComp(data[4]) / SCALE
            translate[0] = int_twosComp(data[0])
            translate[1] = int_twosComp(data[1])
            tilt[0] = int_twosComp(data[2]) / SCALE
            tilt[1] = int_twosComp(data[3]) / SCALE
            rotate = int_twosComp(data[4])

            updown = int_twosComp(data[5])

print("Initialising RC control")
moveThread = Thread(target=move_robot)
rxThread = Thread(target=get_input)
moveThread.daemon=True
rxThread.daemon=True
moveThread.start()
rxThread.start()

while True:
    if not KeyboardInterrupt:
        sock.shutdown()
        sock.close()
        sys.exit()
        sys.exit()