import time
import numpy as np
import socket
import json
import numpy as np

from dynamixel_controller import Dynamixel
from time import sleep

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


servo = Dynamixel(ID=[1,2,3,4], descriptive_device_name="XW430-T200R test motor", 
                    series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0") #probably change it
                    #series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0")"/dev/tty.usbserial-FT78LT9E"

def set_position(time, a, c, T) : 
    position = a*np.sin(2*np.pi/T*time) + c 
    return position

def write_position(q_dynamixel, IDs) :
    servo.write_position(q_dynamixel, ID=IDs)

def sin_position(time, a, c, T) : 
    position = a*np.sin(2*np.pi/T*time) + c 
    return position



def triangle_wave_position(t, a, T, rise_time_ratio, fall_time_ratio):
    global its_opening
    global was_closing
    period = T
    peak_value = 220  # Valore massimo fisso
    valley_value = peak_value - a  # Valore minimo variabile in base all'ampiezza

    rise_time = rise_time_ratio * period
    fall_time = fall_time_ratio * period

    t_mod = t % period  # Tempo modulato per il periodo

    if t_mod < rise_time: #aprendo
        # Fase di salita: da 200 a (200 - a)
        position = peak_value - (peak_value - valley_value) * (t_mod / rise_time)
        if was_closing == True:
            its_opening = True
            was_closing = False
    elif rise_time< t_mod < rise_time + fall_time: #chiudendo
        # Fase di discesa: da (200 - a) a 200
        position = valley_value + (peak_value - valley_value) * ((t_mod - rise_time) / fall_time)
        was_closing = True
    else:
        # Gestione di eventuali errori: questa parte non dovrebbe essere raggiunta
        position = peak_value

    return position, t_mod




def go_forward(time) :
    IDs = [1,2,3,4]
    a = 45
    c = 180
    T = 1
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    write_position(q_dynamixel, IDs)
    return q_dynamixel

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
    
    data = [q_dynamixel_right, q_dynamixel_left]
    return data 

def write_motor_position_triangle(t, a_right, c_right, T_right, rise_time_ratio_right, fall_time_ratio_right, a_left, c_left, T_left, rise_time_ratio_left, fall_time_ratio_left):
    ID_right = [1,2]
    ID_left = [3,4]

    #a_dyna_right = a_right * 2048 / 180
    #c_dyna_right = c_right * 2048 / 180
    #a_dyna_left = a_left * 2048 / 180
    #c_dyna_left = c_left * 2048 / 180

    q_dynamixel_right,t_mod,its_opening = triangle_wave_position(t, a_right,  T_right, rise_time_ratio_right, fall_time_ratio_right)
    #position in deg
    q_dynamixel_left,_,its_opening = triangle_wave_position(t, a_left, T_left, rise_time_ratio_left, fall_time_ratio_left)
    
    position_motor_step_right = q_dynamixel_right * 2048 / 180
    position_motor_step_left = q_dynamixel_left * 2048 / 180
    servo.write_position(position_motor_step_right, ID_right)
    servo.write_position(position_motor_step_left, ID_left)
    
    data = [q_dynamixel_right, q_dynamixel_left]
    return data,t_mod

servo.begin_communication()
servo.set_operating_mode("position", ID = "all")

# Initialize motor position
servo.write_position(2276, [1,2,3,4]) #200° è 2276 
sleep(1)

read_position_right = []
read_position_left = []

motor_command = []
data_to_send = []
camera_ready = False
time_values = []

#temp values:
a_right = 75
c_right = 180
T = 1.5
a_left = 75
c_left = 180
amplitude_timeline_vector_right = []
amplitude_timeline_vector_right.append(a_right)
amplitude_timeline_vector_left = []
amplitude_timeline_vector_left.append(a_left)

opening_ratio = 0.8
closing_ration = 1-opening_ratio

#starting variable for closing and opening state:
was_closing = False
its_opening = False



while True :
    while camera_ready == False:
        data = client_socket.recv(1024)
        data = json.loads(data.decode())
        if data.get("message") == "cameraok":
            camera_ready  = True
            timer = time.time()
            print("cameraok recieved")
        time.sleep(0.1)
        
    t = time.time() - timer
    time_values.append(t)
    # Receive data from the client
    '''data = client_socket.recv(1024)
    data = json.loads(data.decode())
    
    a_right = data.get("a_right")
    c_right = data.get("c_right")
    T_right = data.get("T_right")

    a_left = data.get("a_left")
    c_left = data.get("c_left")
    T_left = data.get("T_left")
    
    State = data.get("State")
 '''

    motor_command,t_mod = write_motor_position_triangle(t, a_right, c_right, T, opening_ratio, closing_ration, a_left, c_left, T, opening_ratio, closing_ration)
    print(its_opening)
    # check if a period T has expired:
    if its_opening: 
        message = 'r'
        message_json = json.dumps(message)
        client_socket.send(message_json.encode())
        print('request sent. Time: ', t)
        time.sleep(0.5)
        #check if something has been sent:
        data = client_socket.recv(1024)
        data = json.loads(data.decode())  
        print("time after response: ", time.time() - timer)
        amplitude_timeline_vector_right.append(data.get("data1"))
        amplitude_timeline_vector_left.append(data.get("data2"))

    a_right = amplitude_timeline_vector_right[-1]
    a_left = amplitude_timeline_vector_left[-1]    
    motor_command_right = motor_command[0]
    motor_command_left = motor_command[1]

    #read position in  deg
    read_position_right = 180*servo.read_position(1)/2048
    read_position_left = 180*servo.read_position(3)/2048

    '''
    json_position = json.dumps({ "Motor_position_right" : read_position_right, "Motor_position_left" : read_position_left, "Motor_command_right" : motor_command_right, "Motor_command_left" : motor_command_left})

    # Sends answer to client
    client_socket.send(json_position.encode())
    '''
    
    data_to_send.append({ 
        "Motor_position_right" : read_position_right, 
        "Motor_position_left" : read_position_left, 
        "Motor_command_right" : motor_command_right, 
        "Motor_command_left" : motor_command_left,
        "Time": time_values[-1]
    })
    
    if t >10:
        State=0
        break

print('starting to send data...')
for i in range(len(data_to_send)):
    print('sending the',i,' batch...')
    json_position = json.dumps(data_to_send[i])
    client_socket.send(json_position.encode())
    if i == 0:
        time.sleep(5)
    else:
        time.sleep(0.1)

message = 'end'
message_json = json.dumps(message)
client_socket.send(message_json.encode())
print("sent the end")

#json_position = json.dumps(data_to_send)
#client_socket.send(json_position.encode())

# Close the client socket
client_socket.close()

# Close motor communication
servo.end_communication()  