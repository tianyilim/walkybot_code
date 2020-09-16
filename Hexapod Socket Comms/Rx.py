import socket
from threading import Thread
import keyboard     #for placeholders
import sys      #for placeholders

ctrlport=34712
dataport =34710
recvBufferSize=4096
recvBuffer=[]   #buffer for received data (array of bytes)
dataFlag=0  #flag to signal that new data has been received 


def controlPortHandler(cport=34712):
    csock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    csock.bind(('',cport))
    while 1:
        cdata,caddr=csock.recvfrom(4096)
        if cdata==b'ID':
            csock.sendto(b'10',caddr)
        elif cdata==b'DP':
            csock.sendto(dataport.to_bytes(2,byteorder='big'),caddr)

def dataPortHandler(dport=34710,dbuff=4096):
    dsock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    dsock.bind(('',dport))
    while 1:
        ddata,daddr=dsock.recvfrom(dbuff)
        recvBuffer=list(ddata)
        dataFlag=1


ctrlThread=Thread(target=controlPortHandler, args=(ctrlport,))
dataThread=Thread(target=dataPortHandler,args=(dataport,recvBufferSize,))
ctrlThread.daemon=True
dataThread.daemon=True
ctrlThread.start()
dataThread.start()

while 1:
    #insert main hexapod code here

    keyboard.wait('q')  #placeholder for testing
    sys.exit()  #placeholder for testing, remove these when inserting hexapod code 