import time
import numpy as np
import socket
import select
import json
#from plot_module import show_graph
from dynamixel_controller import Dynamixel
from time import sleep

###########################################################################
# Motion functions
########################################################################### 
def set_position(time, a, c, T) : 
    position = a*np.sin(2*np.pi/T*time) + c 
    return position

def write_position(q_dynamixel, IDs) :
    servo.write_position(q_dynamixel, ID=IDs)

def go_forward(time) :
    IDs = [1,2,3,4]
    a = 45
    c = 180
    T = 1
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    write_position(q_dynamixel, IDs)

def go_reverse(time) :
    IDs = [1,2,3,4]
    a = 27
    c = 63
    T = 0.5
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    write_position(q_dynamixel, IDs)

def go_right(time) : 
    ID_forward = [3,4]
    ID_reverse = [1,2]
    a_forward = 45
    c_forward = 180
    a_reverse = 27
    c_reverse = 63
    T = 0.5
    a_dyna_forward = a_forward * 2048/180
    c_dyna_forward = c_forward* 2048/180 
    q_dynamixel_forward = set_position(time, a_dyna_forward, c_dyna_forward, T)
    write_position(q_dynamixel_forward, ID_forward)
    a_dyna_reverse = a_reverse * 2048/180
    c_dyna_reverse = c_reverse* 2048/180 
    q_dynamixel_reverse = set_position(time, a_dyna_reverse, c_dyna_reverse, T)
    write_position(q_dynamixel_reverse, ID_reverse)
    

def go_left(time) : 
    IDs = [1,2]
    a = 45
    c = 180
    T = 1
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    write_position(q_dynamixel, IDs)
    write_position(2048, [3,4])

###########################################################################
# Create Socket
########################################################################### 

HOST = '10.20.30.10'
PORT = 12345

# Create a TCP/IP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
server_address = (HOST, PORT)  # Use the Raspberry Pi's IP address
server_socket.bind(server_address)

# Listen for incoming connections
server_socket.listen(1)

# Wait for a connection
print("Waiting for a connection...")
client_socket, client_address = server_socket.accept()

###########################################################################
# MOTOR CONNECTION
########################################################################### 
    
servo = Dynamixel(ID=[1,2,3,4], descriptive_device_name="XW430-T200R test motor", 
                    series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0")

servo.begin_communication()

servo.set_operating_mode("position", ID = "all")

# Initialize motor position
write_position(2040, [1,2,3,4]) #180°
sleep(1)

timer = time.time()

###########################################################################
# MOTOR LOOP
########################################################################### 

while True :

    t = time.time() - timer

    # Receive data from the client
    data = client_socket.recv(1024).decode()  # Receive data

    if data == "Reverse" :

        start_reverse = time.time()
        # Goes reverse for 10 seconds
        while time.time() - start_reverse < 10 :
            t = time.time() - timer
            go_reverse(t)

    elif data == '"Turn"' :
        t = time.time() - timer
        go_right(t)

    elif data == "Stop" :
        break

    else :
        go_forward(t)

    print(data)

# Close the client socket
client_socket.close()

# Close the server socket
server_socket.close()

# Close motor communication
servo.end_communication()  