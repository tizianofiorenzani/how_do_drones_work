import socket
import time
import json
import threading

UDP_IP = "127.0.0.1"
UDP_PORT_WRITE = 55047
UDP_PORT_READ  = 64126

sock_write = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

sock_read = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock_read.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_read.bind((UDP_IP, UDP_PORT_READ))
data_received = {}

kill = False;

def send_pwm(pwm):
    str_to_send = "%d"%pwm
    sock_write.sendto(str_to_send.encode(), (UDP_IP, UDP_PORT_WRITE))

def read_data():
    data, addr = sock_read.recvfrom(10240) # buffer size is 1024 bytes
    #print(data)
    data_js = json.loads(data.decode('latin-1'))
    return(data_js)

def read_udp_main():
    global data_received, kill
    while True:
        data_received = read_data()
        if kill: return


x = threading.Thread(target=read_udp_main)
x.start()


pwm_start = [1000, 1300, 1500, 1600, 1300, 1000]
t_hold = 3

print("STARTING TEST")  
for pwm in pwm_start:
    print("Sending %d"%pwm)
    send_pwm(pwm)
    time.sleep(t_hold)
    #data_received = read_data()
    #print (data)
    thrust = data_received["thrust"]["displayValue"]
    rpm    = data_received["motorOpticalSpeed"]["displayValue"]
    
    print(">> RPM: %6.0f  Thrust: %4.2fN  "%(rpm, thrust))

kill = True
x.join()
print("TEST COMPLETED!")    



