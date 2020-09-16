import socket
import sys

ctrlport=34712
sendBuffer=[]   #byte array to transmit

s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
s.bind((socket.gethostbyname(socket.gethostname()),42819))
s.settimeout(2) #timeout in seconds
s.sendto(b'ID',('255.255.255.255',ctrlport))
try:
    rdata,raddr=s.recvfrom(4096)
except:
    print("Hexapod not found!")
    sys.exit()

if rdata==b'10':
    address=raddr[0]

s.sendto(b'DP',(address,ctrlport))
d,a=s.recvfrom(4096)
dataport=int.from_bytes(d,"big")

# sendBuffer.append(3)
# sendBuffer.append(120)
# s.sendto(bytes(sendBuffer),(address,dataport))

while 1:
    #insert control code here to store transmit data into sendBuffer

    s.sendto(bytes(sendBuffer),(address,dataport))