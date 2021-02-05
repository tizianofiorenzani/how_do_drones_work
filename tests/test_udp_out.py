import socket
import time
import threading

UDP_IP_OUT     = "192.168.1.255"
UDP_IP_IN      = "127.0.0.1"
UDP_PORT_WRITE = 19105

sock_write = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

print("--- PROXY STARTED from localhost to %s:%d"%(UDP_IP_OUT, UDP_PORT_WRITE))  
counter = 0
while True:
    str_to_send = "HELLO %d"%counter
    counter = counter + 1
    sock_write.sendto(str_to_send.encode(), (UDP_IP_OUT, UDP_PORT_WRITE))
    print(str_to_send)
    time.sleep(1)

kill = True
x.join()
print("CLOSED!")    

