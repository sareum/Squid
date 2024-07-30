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
    period = T
    peak_value = 200  # Valore massimo fisso
    valley_value = peak_value - a  # Valore minimo variabile in base all'ampiezza

    rise_time = rise_time_ratio * period
    fall_time = fall_time_ratio * period

    t_mod = t % period  # Tempo modulato per il periodo

    if t_mod < rise_time:
        # Fase di salita: da 200 a (200 - a)
        position = peak_value - (peak_value - valley_value) * (t_mod / rise_time)
    elif t_mod < rise_time + fall_time:
        # Fase di discesa: da (200 - a) a 200
        position = valley_value + (peak_value - valley_value) * ((t_mod - rise_time) / fall_time)
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

    a_dyna_right = a_right * 2048 / 180
    c_dyna_right = c_right * 2048 / 180
    a_dyna_left = a_left * 2048 / 180
    c_dyna_left = c_left * 2048 / 180

    q_dynamixel_right,t_mod = triangle_wave_position(t, a_dyna_right,  T_right, rise_time_ratio_right, fall_time_ratio_right)
    #servo.write_position(q_dynamixel_right, ID_right)
    q_dynamixel_left,_ = triangle_wave_position(t, a_dyna_left, T_left, rise_time_ratio_left, fall_time_ratio_left)
    #servo.write_position(q_dynamixel_left, ID_left)
    
    data = [q_dynamixel_right, q_dynamixel_left]
    return data,t_mod

servo.begin_communication()
servo.set_operating_mode("position", ID = "all")

# Initialize motor position
servo.write_position(2260, [1,2,3,4]) #200° è 2260 
sleep(1)

read_position_right = []
read_position_left = []

motor_command = []
data_to_send = []
camera_ready = False
time_values = []

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
    a_right = 45
    c_right = 180
    T_right = 1
    a_left = 45
    c_left = 180
    T_left = 1
    motor_command,t_mod = write_motor_position_triangle(t, a_right, c_right, T_right, 0.2, 0.8, a_left, c_left, T_left, 0.2, 0.8)
    print("motor Command right",motor_command[0])
    print("t_mod: ",t_mod,"time: ", t)
    if t_mod >0.15 and t_mod< 0.25:
        message = 'ready'
        message_json = json.dumps(message)
        client_socket.send(message_json.encode())
        print('request sent. Time: ', t)
        time.sleep(0.1)
        
    motor_command_right = 180*motor_command[0]/2048
    motor_command_left = 180*motor_command[1]/2048

    if(motor_command_right < (c_right - a_right + 3)):
        pass

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
'''message = 'start'
message_json = json.dumps(message)
client_socket.send(message_json.encode())
time.sleep(2)'''
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