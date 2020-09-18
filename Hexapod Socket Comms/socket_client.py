''' Socket Client, sends (TX) '''

import socket
import sys

target_name = "localhost"
target_port = 10000
server_address = (target_name, target_port)

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.setblocking(0)
sock.settimeout(5)
message = b'This is the message.  It will be repeated.' # A bytes-like object is required

try:
    # Send data
    print('sending "%s"' % message)
    sent = sock.sendto(message, server_address)

    # Receive response
    print('waiting to receive')
    data, server = sock.recvfrom(4096) # returns (bytes, address) 
    # bytes is a bytes object (data received)
    # address is address of the socket sending the data
    print('received "%s"' % data)

except KeyboardInterrupt:
    print('Keyboard interrupt - closing socket')
    sock.close()
    sys.exit()

finally:
    print('closing socket')
    sock.close()