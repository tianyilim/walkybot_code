''' Socket server, receives (RX) '''
import socket
import sys
# import hex_legs
import hexapod
import numpy as np
from adafruit_servokit import ServoKit
import time

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

target_name = '' # listens to all
target_port = 10240
server_address = (target_name, target_port)

# initialise hexapod object
leg_end_coords = np.tile((0.0, 68.75, 0.0), (6,1)) # check back
leg_angles = np.tile(90.0, 6)  # leg angle init
# print(leg_end_coords, leg_end_coords.shape)
walky = hexapod.hexapod(leg_end_loc=leg_end_coords, leg_angle=leg_angles)

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

        translate[0] = int_twosComp(data[0]) / RATE
        translate[1] = int_twosComp(data[1]) / RATE
        tilt[0] = int_twosComp(data[2]) / SCALE
        tilt[1] = int_twosComp(data[3]) / SCALE
        rotate = int_twosComp(data[4]) / SCALE
        updown = int_twosComp(data[5])

        # print("Translate values:", translate)
        # print("Tilt values:", tilt)
        # print("Rotate:", rotate)

        walky.body_pitch(tilt[0])
        walky.body_roll(tilt[1])
        walky.body_translate_z(updown*4)
        walky.body_rotate_absolute(rotate) # ROTATE has to be called last to avoid being overwritten!
        # walky.print_state()
        # print("")
        walky.update_legs_stance()  

        #debug
        # if updown == 1:
        #     print("Increase Z")
        # elif updown == -1:
        #     print("Lower Z")
        # else:
        #     print("Keep Z")
        # print("")

sock.shutdown()
sock.close()
sys.exit()