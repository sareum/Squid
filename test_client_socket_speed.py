import time
import numpy as np
import socket
import json
import numpy as np

from dynamixel_controller import Dynamixel
from time import sleep
import struct
import serial

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
def decode_and_parse_data(data):
    # Decodifica i dati ricevuti (assumendo che siano stati codificati in UTF-8)
    decoded_data = data.decode('utf-8')

    # Separazione dei dati basata sul separatore ','
    parts = decoded_data.split(',')

    # Conversione delle stringhe in interi
    amplitude_right = int(parts[0])
    amplitude_left = int(parts[1])
    reached = int(parts[2])

    return amplitude_right, amplitude_left, reached

serial_port = '/dev/ttyACM0'  # Cambia questo con la tua porta
baud_rate = 115200  # Questo deve corrispondere al baud rate impostato nel Teensy
ser = serial.Serial(serial_port, baud_rate, timeout=1)
ser.reset_input_buffer()
ser.reset_output_buffer()
print(f"Connessione aperta sulla porta {serial_port} con baud rate {baud_rate}")

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
amplitude_right = 0
amplitude_left = 0

was_closing = False
its_opening = False
start_time = time.time()
calibration_complete = False #set the motor at 200, then sends the quaternion for the first rotation matrix
###SERIAL COMUNICATION#####
camera_calibration = False
data = None
ricevuto = False
prima_volta = True
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((IP, PORT))
    s.listen()
    conn, addr = s.accept()
    s.setblocking(False)
    with conn:
        print(f"Connected by {addr}")
        while True:
            #wait for camera calibration on the pc
            if camera_calibration == False:
                data = conn.recv(1024)
            while camera_calibration == False:
                if data.decode('utf-8') == "cameraok":
                    camera_calibration = True
                elif not data:
                    data = conn.recv(1024)
                else: 
                    time.sleep(1)

            while calibration_complete == False:
                # Initialize motor position
                servo.write_position(2276, [1,2,3,4]) #200° è 2276 
                print("sending the data from the initial calibration...")
                if ser.in_waiting > 0:
                    print("sono nell'if")
                # Legge una riga di dati dalla seriale
                    serial_reads = ser.readline().decode('utf-8').rstrip()
                    data_to_encode = str(serial_reads)
                    string_data = str(data_to_encode).encode("utf-8")
                    conn.sendall(string_data)
                    while True:
                        if conn.recv(1024).decode('utf-8') == "Received":
                            break
                        print("nel while")
                        serial_reads = ser.readline().decode('utf-8').rstrip()
                        data_to_encode = str(serial_reads)
                        string_data = str(data_to_encode).encode("utf-8")
                        conn.sendall(string_data)
                        print(serial_reads)
                    calibration_complete = True
                    print("Completed the calibration!")
        
            tic = time.time()
            if ser.in_waiting > 0:
                #read the serial data
                serial_reads = ser.readline().decode('utf-8').rstrip()
                #print(f"Dati ricevuti: {serial_reads}")
            
            #packs the data in one variable
            
            #control the motor:
            motor_command,t_mod = write_motor_position_triangle(time.time()-start_time, amplitude_right, c_right, T, opening_ratio, closing_ration, amplitude_left, c_left, T, opening_ratio, closing_ration)
            #checks if something is in the serial
            if its_opening: 
                #time.sleep(0.5)
                message = 'ready'
                conn.sendall(message.encode('utf-8'))
                #check if something has been sent:
                data = conn.recv(1024)
                while not data.decode('utf-8'):
                    data = conn.recv(1024)
                
                amplitude_right, amplitude_left, reached = decode_and_parse_data(data)
                relative_timer = time.time()  
                amplitude_timeline_vector_right.append(amplitude_right)
                amplitude_timeline_vector_left.append(amplitude_left)
                if reached == 1:
                    conn.close()
                    break
                its_opening = False
            
            motor_command[0] = float(f"{motor_command[0]:.4g}")
            motor_command[1] = float(f"{motor_command[1]:.4g}")
            
            data_to_encode = str(motor_command)+str(serial_reads)
            
            #encode the data in utf-8 for socket comunication
            
            string_data = data_to_encode.encode("utf-8")
            conn.sendall(string_data)
            print("Ho spedito comandi motore")
        
            toc = time.time()-tic
            print(toc)

# Close motor communication
servo.end_communication()  