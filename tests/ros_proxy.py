import socket
import time
import threading

UDP_IP_OUT     = "192.168.5.1"
UDP_IP_IN      = "0.0.0.0"
UDP_PORT_WRITE = 4401
UDP_PORT_READ  = 19105

sock_write = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

sock_read = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock_read.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_read.bind((UDP_IP_IN, UDP_PORT_READ))
data_received = {}


def read_data():
    data, addr = sock_read.recvfrom(1024) # buffer size is 1024 bytes
    return(data)


print("--- PROXY STARTED from localhost %d to %s:%d"%(UDP_PORT_READ, UDP_IP_OUT, UDP_PORT_WRITE))  
while True:
    #- Read data from UDP
    data_received = read_data().decode()
    #sock_write.sendto(data_received, (UDP_IP_OUT, UDP_PORT_WRITE))
    print("Command received: ", data_received)

kill = True
x.join()
print("CLOSED!")    

