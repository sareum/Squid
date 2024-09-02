import time
import numpy as np
import socket
import errno
from dynamixel_controller import Dynamixel
from time import sleep
import board 
import ahrs
import serial
import smbus
import struct

# Configurazione
PROTOCOL = 'TCP'
IP = '192.168.137.246' # Network number
PORT = 12345 # Same for client and server
BUFFER_SIZE = 1024  # Dimensione del buffer in byte
NUM_PACKETS = 10000  # Numero di pacchetti da inviare

TEENSY_I2C_ADDRESS  = 0x08
# Inizializza il bus I2C (bus 1 per Raspberry Pi 3 e successivi)
bus = smbus.SMBus(1)

####################### MOTOR COMMAND ###################
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
################### END MOTOR COMMAND  ############################


def decode_and_parse_data(data):
    # Decodifica i dati ricevuti (assumendo che siano stati codificati in UTF-8)
    try:
        decoded_data = data.decode('utf-8')
            
        # Separazione dei dati basata sul separatore ','
        parts = decoded_data.split(',')

        # Conversione delle stringhe in interi
        amplitude_right = int(parts[0])
        amplitude_left = int(parts[1])
        reached = int(parts[2])
    except Exception as e:
        amplitude_right = 0
        amplitude_left = 0
        reached = 0
        print("########### Wrong in decode ################")
    return amplitude_right, amplitude_left, reached


############### MOTOR SETUP ########################
servo = Dynamixel(ID=[1,2,3,4], descriptive_device_name="XW430-T200R test motor", 
                    series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0") #probably change it
                    #series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0")"/dev/tty.usbserial-FT78LT9E"
servo.begin_communication()
servo.set_operating_mode("position", ID = "all")
a_right = 75
c_right = 180
T = 1
a_left = 75
c_left = 180
opening_ratio = 0.9
closing_ration = 1-opening_ratio
amplitude_timeline_vector_right = []
amplitude_timeline_vector_left = []
amplitude_right = 45
amplitude_left = 45
############## END MOTOR SETUP #################

def read_sensors():
    right = []
    left = []

    try:
        data = bus.read_i2c_block_data(TEENSY_I2C_ADDRESS, 0, 32)
        qW1, qX1, qY1, qZ1, qW2, qX2, qY2, qZ2 = struct.unpack('f' * 8, bytearray(data))
        left = [qW1, qX1, qY1, qZ1]
        right = [qW2, qX2, qY2, qZ2]
    except OSError as e:
        print(f"Errore di comunicazione I2C: {e}")
        #time.sleep(1)

    #choose data1 or data2 based on the tentacle to test    
    return right, left

#flags:
was_closing = False
its_opening = False
frame_calibration = False #set the motor at 200, then sends the quaternion for the first rotation matrix
calibration_quaternion = False #calibrate the EKF
camera_calibration = False
ricevuto = False
prima_volta = True
data = None
start_time = time.time()
entrato_in_its_opening = False
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    try:
        s.bind((IP, PORT))
        s.listen()
        conn, addr = s.accept()
        s.setblocking(False)
        with conn:         
            print(f"Connected by {addr}")
            while True:
                tic = time.time()
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
                
                while frame_calibration == False: #imu_calibration in laptop code
                #MOTOR FRAME 
                    # Initialize motor position
                    servo.write_position(2276, [1,2,3,4]) #200° è 2276 
                    sleep(0.5)
                    print("Sending quaternions for the calibration of the frame...")
                    quat1,quat2 = read_sensors()

                    data_to_encode = str(quat1) +','+ str(quat2)
                    string_data = data_to_encode.encode("utf-8")
                    conn.sendall(string_data)
                    while True:
                        if conn.recv(1024).decode('utf-8') == "Received":
                            break
                        print("nel while")
                        quat1,quat2 = read_sensors()
                        data_to_encode = str(quat1) +','+ str(quat2)
                        string_data = data_to_encode.encode("utf-8")
                        conn.sendall(string_data)
                    frame_calibration = True
                    print("Completed the frame calibration!")
                
                
                #READ QUATERNIONS 
                quat1, quat2 = read_sensors()
                #print("quat1: ", quat1)
                #control the motor:
                motor_command,t_mod = write_motor_position_triangle(time.time()-start_time, amplitude_right, c_right, T, opening_ratio, closing_ration, amplitude_left, c_left, T, opening_ratio, closing_ration)
                print("Sotto il motore_comandi")
                #checks if something is in the serial
                if entrato_in_its_opening == False:
                    its_time = time.time()
                if its_opening:  
                    entrato_in_its_opening = True
                    message = 'ready'
                    print("sopra il ready")
                    conn.sendall(message.encode('utf-8'))
                    print("spedito il ready")
  
                    #check if something has been sent:
                    try:
                        data = conn.recv(1024)
                    except socket.error as e:
                        err = e.args[0]
                        if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                            print("No data available")
                            continue
                        else:
                            print(e)
                    else:
                        amplitude_right, amplitude_left, reached = decode_and_parse_data(data)
                        print("data recived input from PID: ",amplitude_left,amplitude_right,reached)
                        relative_timer = time.time()  
                        if reached == 1:
                            conn.close()
                            break
                        its_opening = False
                    if (time.time()-its_time)>3:
                        print("sono nella prigione del tempo e sono salvo!")
                        its_opening = False
                        entrato_in_its_opening = False
                        message = 'ready'
                        conn.sendall(message.encode('utf-8'))

                motor_command[0] = float(f"{motor_command[0]:.4g}")
                motor_command[1] = float(f"{motor_command[1]:.4g}")
                
                data_to_encode = str(motor_command)+','+ str(quat1)+','+ str(quat2)
                string_data = data_to_encode.encode("utf-8")
                print("sopra i dati ")
                conn.sendall(string_data)
                print("inviato i comandi")
            
                #print(time.time()-tic)

    except KeyboardInterrupt:
            print("Programma terminato.")
            servo.end_communication() 
            s.close()
