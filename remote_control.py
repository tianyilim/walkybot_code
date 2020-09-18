''' Controls the RPi robot via UDP sockets '''
from inputs import get_gamepad
import time
import socket
import sys
from threading import Thread

# Gamepad inputs
DPAD_MAX = 32767
DPAD_SCALE = DPAD_MAX/127
TRIG_MAX = 255
TRIG_SCALE = TRIG_MAX/127
translate = [0,0] # as a proportion
tilt = [0,0]
rotate = 0
updown = 0

target_name = "192.168.1.141" # Check this
target_port = 10240
server_address = (target_name, target_port)

def int_twosComp(val, bits=8):
    if val > 2**(bits-1)-1:
        return val - 2**bits
    else:
        return val

def controllerInput():
    rotateRight = 0.0
    rotateLeft = 0.0
    global translate
    global tilt
    global rotate
    global updown

    while True:
        events = get_gamepad()

        for event in events:
            # print(event.ev_type, event.code, event.state)

            if event.code == "ABS_X":
                translate[1] = int(event.state / DPAD_SCALE)
            if event.code == "ABS_Y":
                translate[0] = int(event.state / DPAD_SCALE)
            if event.code == "ABS_RX":
                tilt[1] = int(event.state / DPAD_SCALE)
            if event.code == "ABS_RY":
                tilt[0] = int(event.state / DPAD_SCALE)
            if event.code == "ABS_Z":
                rotateLeft = event.state / TRIG_SCALE
            if event.code == "ABS_RZ":
                rotateRight = event.state / TRIG_SCALE
            if event.code == "ABS_HAT0Y":
                if event.state == -1:
                    updown = 1
                elif event.state == 1:
                    updown = -1
                else:
                    updown = 0
        rotate = int(rotateRight - rotateLeft)

def transmitInput(target_params):
    global translate
    global tilt
    global rotate
    global updown

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Create a UDP socket
    sock.settimeout(5)
    
    while True:
        # Formatting data into a byte array
        data = b""
        data += bytes( (translate[0]).to_bytes(1,'little', signed=True) )
        data += bytes( (translate[1]).to_bytes(1,'little', signed=True) )
        data += bytes( (tilt[0]).to_bytes(1,'little', signed=True) )
        data += bytes( (tilt[1]).to_bytes(1,'little', signed=True) )
        data += bytes( (rotate).to_bytes(1,'little', signed=True) )
        data += bytes( (updown).to_bytes(1,'little', signed=True) )
        
        sent = sock.sendto(data, target_params)
        
        # Debug
        # print("Sent %s bytes" %sent)
        # for i, x in enumerate(data):
        #     print(i, int_twosComp(x) )
        time.sleep(0.1) # Don't need to update so often
    
print("Starting stuff")
inputThread = Thread(target=controllerInput)
transmitThread = Thread(target=transmitInput, args=(server_address,))
inputThread.daemon=True
transmitThread.daemon=True
inputThread.start()
transmitThread.start()

while True:
    if not KeyboardInterrupt:
        sys.exit()  #placeholder for testing, remove these when inserting hexapod code 