import time
import numpy as np
import socket
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
    a = 60
    c = 180
    T = 0.6
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    write_position(q_dynamixel, IDs)

def go_reverse(time) :
    IDs = [1,2,3,4]
    a = 27
    c = 63
    T = 1
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    write_position(q_dynamixel, IDs)

def turn_around_right(time) : 
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

def turn_around_left(time) : 
    ID_forward = [1,2]
    ID_reverse = [3,4]
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
    
def go_right(time) : 
    ID_right = [1,2]
    ID_left = [3,4]
    a = 45
    c = 180
    T_right = 1
    T_left = 0.5
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel_right = set_position(time, a_dyna, c_dyna, T_right)
    q_dynamixel_left = set_position(time, a_dyna, c_dyna, T_left)
    #write_position(q_dynamixel_right, ID_right)
    write_position(q_dynamixel_left, ID_left)

def go_left(time) : 
    ID_right = [1,2]
    ID_left = [3,5]
    a = 45
    c = 180
    T_right = 0.5
    T_left = 1
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel_right = set_position(time, a_dyna, c_dyna, T_right)
    q_dynamixel_left = set_position(time, a_dyna, c_dyna, T_left)
    write_position(q_dynamixel_right, ID_right)
    #write_position(q_dynamixel_left, ID_left)

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

    if "Forward" in data :

        go_forward(t)

    if "Reverse" in data :

        go_reverse(t)

    if "Turn right" in data :

        turn_around_right(t)

    if "Turn left" in data :

        turn_around_left(t)

    if "Go right" in data :

        go_right(t) 

    if "Go left" in data :

        go_left(t) 

    if "Stop" in data :

        break

# Close the client socket
client_socket.close()

# Close the server socket
server_socket.close()

# Close motor communication
servo.end_communication()  