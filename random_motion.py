import time
import numpy as np
import socket
import json
from dynamixel_controller import Dynamixel
from time import sleep

###########################################################################
# Motion functions
########################################################################### 
def set_position(time, a, c, T) : 
    position = a*np.sin(2*np.pi/T*time) + c 
    return position

def write_motor_position(time, a_right, c_right, T_right, a_left, c_left, T_left) :
    ID_right = [1,2]
    ID_left = [3,4]
    a_dyna_right = a_right * 2048/180
    c_dyna_right = c_right * 2048/180 
    a_dyna_left = a_left * 2048/180
    c_dyna_left = c_left * 2048/180 
    q_dynamixel_right = set_position(time, a_dyna_right, c_dyna_right, T_right)
    servo.write_position(q_dynamixel_right, ID_right)
    q_dynamixel_left = set_position(time, a_dyna_left, c_dyna_left, T_left)
    servo.write_position(q_dynamixel_left, ID_left)
    return q_dynamixel_right

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
servo.write_position(2040, [1,2,3,4]) #180°
sleep(1)

read_position = []
timer = time.time()

###########################################################################
# MOTOR LOOP
########################################################################### 

while True :

    t = time.time() - timer

    # Receive data from the client
    data = client_socket.recv(1024)
    data = json.loads(data.decode())

    a_right = data.get("a_right")
    c_right = data.get("c_right")
    T_right = data.get("T_right")

    a_left = data.get("a_left")
    c_left = data.get("c_left")
    T_left = data.get("T_left")

    State = data.get("State")
    
    motor_command = 180*write_motor_position(t, a_right, c_right, T_right, a_left, c_left, T_left)/2048

    # Read motor position
    read_position = 180*servo.read_position(1)/2048
    json_position = json.dumps({ "Motor_position" : read_position, "Motor_command" : motor_command})

    # Sends answer to client
    client_socket.send(json_position.encode())
 
    if State == 0  :

        break


# Close the client socket
client_socket.close()

# Close motor communication
servo.end_communication()  