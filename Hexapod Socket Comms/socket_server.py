''' Socket server, receives (RX) '''
import socket
import sys
import struct # Byte stuff
# from threading import Thread # need two processes

def int_twosComp(val, bits=8):
    if val > 2**(bits-1)-1:
        return val - 2**bits
    else:
        return val

translate = [0.0,0.0] # as a proportion
tilt = [0.0,0.0]
rotate = 0
updown = 0

target_name = "localhost"
target_port = 10000
server_address = (target_name, target_port)

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

        translate[0] = int_twosComp(data[0])
        translate[1] = int_twosComp(data[1])
        tilt[0] = int_twosComp(data[2])
        tilt[1] = int_twosComp(data[3])
        rotate = int_twosComp(data[4])
        updown = int_twosComp(data[5])

        #debug
        print("Translate values:", translate)
        print("Tilt values:", tilt)
        print("Rotate:", rotate)
        if updown == 1:
            print("Increase Z")
        elif updown == -1:
            print("Lower Z")
        else:
            print("Keep Z")
        print("")

sock.shutdown()
sock.close()
sys.exit()