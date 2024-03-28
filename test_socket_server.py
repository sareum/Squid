import time
import numpy as np
import socket
import json
#from plot_module import show_graph
from dynamixel_controller import Dynamixel
from time import sleep

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

###########################################################################
# Create Socket
########################################################################### 
    
# Create a TCP/IP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
server_address = ('10.20.30.10', 12345)  # Use the Raspberry Pi's IP address
server_socket.bind(server_address)

# Listen for incoming connections
server_socket.listen(1)

# Wait for a connection
print("Waiting for a connection...")
connection, client_address = server_socket.accept()

try :
    print("Connection from", client_address)
    
    # Receive data from the client
    while True:
        json_data = connection.recv(1024).decode() # Receive data
        data = json.loads(json_data) # Deserialize JSON data
        if not data:
            break
        print("Received:", data.decode())

finally:
    # Clean up the connection
    connection.close()
    
servo = Dynamixel(ID=[1,2,3,4], descriptive_device_name="XW430-T200R test motor", 
                    series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0")

servo.begin_communication()

servo.set_operating_mode("position", ID = "all")
    
write_position(2048, [1,2,3,4]) #180°
sleep(1)

'''read_position = np.empty(1000)
read_velocity = np.empty(1000)
function_value = np.empty(1000)
error = np.empty(1000)'''

timer = time.time()

while True :

    t = time.time() - timer


    try :
        #print("Connection from", client_address)
        
        # Receive data from the client
        while True:
            json_data = connection.recv(1024).decode() # Receive data
            data = json.loads(json_data) # Deserialize JSON data
            if not data:
                break
            print("Received:", data.decode())

    finally:
    # Clean up the connection
        connection.close()

    go_forward(t)

servo.end_communication()