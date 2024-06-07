import time
import numpy as np
import socket
import json
from dynamixel_controller import Dynamixel
from time import sleep

###########################################################################
# Motion functions
########################################################################### 
def sin_position(time, a, c, T) : 
    position = a*np.sin(2*np.pi/T*time) + c 
    return position

def step_position(time, a_max, T_up, T_hold_up, T_down, T_hold_down) :

    T = T_up + T_hold_up + T_down + T_hold_down
    print(T)

    t = time % T

    if t < T_up :
        q = 2*a_max/T_up*t - a_max

    elif t < T_up + T_hold_up :
        q = a_max

    elif t < T_up + T_hold_up + T_down :
        q = -2*a_max/T_down*(t-T_up-T_hold_up-T_down/2)

    elif t < T_up + T_hold_up + T_down + T_hold_down :
        q = - a_max

    return q

def write_motor_position_sin(time, a_right, c_right, T_right, a_left, c_left, T_left) :
    ID_right = [1,2]
    ID_left = [3,4]
    a_dyna_right = a_right * 2048/180
    c_dyna_right = c_right * 2048/180 
    a_dyna_left = a_left * 2048/180
    c_dyna_left = c_left * 2048/180 
    q_dynamixel_right = sin_position(time, a_dyna_right, c_dyna_right, T_right)
    servo.write_position(q_dynamixel_right, ID_right)
    q_dynamixel_left = sin_position(time, a_dyna_left, c_dyna_left, T_left)
    servo.write_position(q_dynamixel_left, ID_left)
    return q_dynamixel_right

def write_motor_position_step(time, a_max, T_up, T_hold_up, T_down, T_hold_down) :
    ID = [1,2,3,4]
    q_dynamixel = step_position(time, a_max, T_up, T_hold_up, T_down, T_hold_down)*2048/180
    servo.write_position(q_dynamixel, ID)
    return q_dynamixel


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

    ###########################################################################
    # MOTOR LOOP - SIN COMMAND
    ########################################################################### 

    param_1 = data.get("param_1")
    param_2 = data.get("param_2")
    param_3 = data.get("param_3")
    param_4 = data.get("param_4")
    param_5 = data.get("param_5")
    param_6 = data.get("param_6")

    go_straight = data.get("Go_straight")
    State = data.get("State")

    #print(data)

    if go_straight == False  :   
        motor_command = 180*write_motor_position_sin(t, param_1, param_2, param_3, param_4, param_5, param_6)/2048

    else : 
        motor_command = 180*write_motor_position_step(t, param_1, param_2, param_3, param_4, param_5)/2048

    ###########################################################################
    # MOTOR LOOP - STEP COMMAND
    ########################################################################### 
    '''
    a_max = data.get("a_max")
    T_up = data.get("T_up")
    T_hold_up = data.get("T_hold_hup")
    T_down = data.get("T_hold")
    T_hold_down = data.get("T_hold_down")

    State = data.get("State")
    '''

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
