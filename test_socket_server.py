import time
import numpy as np
import socket
import select
import json
#from plot_module import show_graph
from dynamixel_controller import Dynamixel
from time import sleep

###########################################################################
# Create Socket
########################################################################### 
def receive_data(HOST, PORT) :

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

    # Receive data from the client
    json_data = client_socket.recv(1024).decode()  # Receive data
    # Decode received data
    json_data = json.loads(json_data.encode('utf-8'))

    print("Received data:", json_data)

    # Close the client socket
    client_socket.close()
    
    # Close the server socket
    server_socket.close()


###########################################################################
# Motion functions
########################################################################### 
def set_position(time, a, c, T) : 
    position = a*np.sin(2*np.pi/T*time) + c 
    return position

def write_position(q_dynamixel, IDs) :
    servo.write_position(q_dynamixel, ID=IDs)

def go_forward(time) :
    IDs = [1]
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
    T = 1
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    write_position(q_dynamixel, IDs)

def go_right(time) : 
    IDs = [3,4]
    a = 45
    c = 180
    T = 1
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    write_position(q_dynamixel, IDs)
    write_position(2048, [1,2])

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

HOST = '10.20.30.10'
PORT = 12345

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

'''read_position = np.empty(1000)
read_velocity = np.empty(1000)
function_value = np.empty(1000)
error = np.empty(1000)'''

timer = time.time()

###########################################################################
# MOTOR LOOP
########################################################################### 


while True :

    # Receive data from the client
    receive_data(HOST, PORT)

    # Perform motor operation
    t = time.time() - timer
    go_forward(t)

    if t>100000 :
        break

servo.end_communication()