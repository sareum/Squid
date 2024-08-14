import time
import numpy as np
import socket
import json
import numpy as np

from dynamixel_controller import Dynamixel
from time import sleep
import struct

# Configurazione
PROTOCOL = 'TCP'  # Cambia a 'UDP' per usare UDP
IP = '10.20.30.10' # Network number
PORT = 12345 # Same for client and server
BUFFER_SIZE = 1024  # Dimensione del buffer in byte
NUM_PACKETS = 10000  # Numero di pacchetti da inviare

########## MOTOR CONTROL ##########


servo = Dynamixel(ID=[1,2,3,4], descriptive_device_name="XW430-T200R test motor", 
                    series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0") #probably change it
                    #series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0")"/dev/tty.usbserial-FT78LT9E"

def set_position(time, a, c, T) : 
    position = a*np.sin(2*np.pi/T*time) + c 
    return position

def write_position(q_dynamixel, IDs) :
    servo.write_position(q_dynamixel, ID=IDs)


def triangle_wave_position(t, a, T, rise_time_ratio, fall_time_ratio):
    global its_opening
    global was_closing
    period = T
    peak_value = 200  # Valore massimo fisso
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

def write_motor_position_triangle(t, a_right, c_right, T_right, rise_time_ratio_right, fall_time_ratio_right, a_left, c_left, T_left, rise_time_ratio_left, fall_time_ratio_left):
    ID_right = [1,2]
    ID_left = [3,4]

    #a_dyna_right = a_right * 2048 / 180
    #c_dyna_right = c_right * 2048 / 180
    #a_dyna_left = a_left * 2048 / 180
    #c_dyna_left = c_left * 2048 / 180

    q_dynamixel_right,t_mod = triangle_wave_position(t, a_right,  T_right, rise_time_ratio_right, fall_time_ratio_right)
    #position in deg
    q_dynamixel_left,_ = triangle_wave_position(t, a_left, T_left, rise_time_ratio_left, fall_time_ratio_left)
    
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
a_right = 75
c_right = 180
T = 1
a_left = 75
c_left = 180
opening_ratio = 0.9
closing_ration = 1-opening_ratio
amplitude_timeline_vector_right = []
amplitude_timeline_vector_left = []
############## END MOTOR COMMAND #################


was_closing = False
its_opening = False
start_time = time.time()
# Creazione della socket
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((IP, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            '''data = conn.recv(1024)
            if not data:
                break'''
            tic = time.time()
            #control the motor:
            motor_command,t_mod = write_motor_position_triangle(time.time()-start_time, a_right, c_right, T, opening_ratio, closing_ration, a_left, c_left, T, opening_ratio, closing_ration)
            print(motor_command)

            #byte_data = struct.pack('!' + 'f' * len(motor_command), *motor_command)
            string_data = str(motor_command).encode("utf-8")
            conn.sendall(string_data)
            toc = time.time()-tic
            print(toc)

'''if PROTOCOL == 'TCP':
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the address and port
    server_address = (IP, PORT)  # Use the Raspberry Pi's IP address
    server_socket.bind(server_address)

    # Listen for incoming connections
    server_socket.listen(1)

    # Wait for a connection
    print("Waiting for a connection...")

    client_socket, client_address = server_socket.accept()

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    time.sleep(2)
    # Connect the socket to the server's address and port
else:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
'''
message = b'a' * BUFFER_SIZE  # Pacchetto di dati da inviare

start_time = time.time()


for _ in range(NUM_PACKETS):
    if PROTOCOL == 'TCP':
        #message_json = json.dumps(message)
        #client_socket.send(message_json.encode())
        client_socket.send(message)

        motor_command,t_mod = write_motor_position_triangle(time.time()-start_time, a_right, c_right, T, opening_ratio, closing_ration, a_left, c_left, T, opening_ratio, closing_ration)
        # check if a period T has expired:
        '''        if its_opening: 
            #time.sleep(0.5)
            message = 'ready'
            message_json = json.dumps(message)
            client_socket.send(message_json.encode())
            time.sleep(0.01)
            #check if something has been sent:
            data = client_socket.recv(1024)
            data = json.loads(data.decode())
            relative_timer = time.time()  
            amplitude_timeline_vector_right.append(data.get("data1"))
            amplitude_timeline_vector_left.append(data.get("data2"))
            state = data.get("state")
            its_opening = False'''
    else:
        client_socket.sendto(message, (IP, PORT))

if PROTOCOL == 'TCP':
    client_socket.close()

end_time = time.time()
elapsed_time = end_time - start_time
total_data_sent = BUFFER_SIZE * NUM_PACKETS
speed = total_data_sent / elapsed_time / 1024  # Velocità in KB/s
servo.end_communication()  
print(f"Dati inviati: {total_data_sent} byte")
print(f"Tempo totale: {elapsed_time:.2f} secondi")
print(f"Velocità: {speed:.2f} KB/s")
