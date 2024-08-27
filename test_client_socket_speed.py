import time
import numpy as np
import socket
import errno
from dynamixel_controller import Dynamixel
from time import sleep
import board 
import busio
from adafruit_lsm6ds import LSM6DSOX
import adafruit_lis3mdl
import ahrs
import serial

# Configurazione
PROTOCOL = 'TCP'  # Cambia a 'UDP' per usare UDP
IP = '192.168.137.246' # Network number
PORT = 12345 # Same for client and server
BUFFER_SIZE = 1024  # Dimensione del buffer in byte
NUM_PACKETS = 10000  # Numero di pacchetti da inviare

########## MOTOR CONTROL ##########


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

############### motor setup ########

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
############## END MOTOR COMMAND #################
############## I2C setup###################
hard_calibr = [-3.35, -0.74, -40.79]
soft_calib = [0.96, 0.02, 0.01, 0.02, 0.96, 0.00, 0.01, 0.00, 1.08]
gyro_calib = [0.05, -0.01, -0.01]
mag_field_magnitude = 45.00

# Configura I2C1
i2c = busio.I2C(board.SCL, board.SDA)

# Inizializza il primo set di sensori
lsm6dsox_1 = LSM6DSOX(i2c, address=0x6A)
lis3mdl_1 = adafruit_lis3mdl.LIS3MDL(i2c, address=0x1E)

# Inizializza il secondo set di sensori con indirizzo modificato
lsm6dsox_2 = LSM6DSOX(i2c, address=0x6B)
lis3mdl_2 = adafruit_lis3mdl.LIS3MDL(i2c, address=0x1C)

def read_sensors():
    data1 = []
    data2 = []
    # Leggi i dati dai sensori del primo set
    acc_1 = lsm6dsox_1.acceleration
    mag_1 = lis3mdl_1.magnetic
    gyro_1 = lsm6dsox_1. gyro
    # Leggi i dati dai sensori del secondo set
    acc_2 = lsm6dsox_2. acceleration
    mag_2 = lis3mdl_2. magnetic
    gyro_2 = lsm6dsox_2.gyro
    data1 = list(mag_1)+ list(gyro_1)+ list(acc_1)
    data2 = list(mag_2)+ list(gyro_2)+ list(acc_2)
    return data1, data2

def correction(data):
    mx = data[0]-hard_calibr[0]
    my = data[1]-hard_calibr[1]
    mz = data[2]-hard_calibr[2]
    magX = mx * soft_calib[0] + my * soft_calib[1] + mz * soft_calib[2]
    magY= mx * soft_calib[3] + my * soft_calib[4] + mz * soft_calib[5]
    magZ = mx * soft_calib[6] + my * soft_calib[7] + mz * soft_calib[8]
    gyX = data[3] - gyro_calib[0]
    gyY = data[4] - gyro_calib[1]
    gyZ = data[5] - gyro_calib[2]
    return [magX, magY, magZ, gyX, gyY, gyZ, data[6], data[7], data[8]]


dt = 0.1
##############  END I2C SETUP################

was_closing = False
its_opening = False
start_time = time.time()
calibration_complete = False #set the motor at 200, then sends the quaternion for the first rotation matrix
calibration_quaternion = False #calibrate the EKF
###SERIAL COMUNICATION#####
camera_calibration = False
data = None
ricevuto = False
prima_volta = True
iQuat = 0
iQ0 =0
try:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
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
                    #CAMERA CALIBRATION
                    data = conn.recv(1024)
                while camera_calibration == False:
                    if data.decode('utf-8') == "cameraok":
                        camera_calibration = True
                    elif not data:
                        data = conn.recv(1024)
                    else: 
                        time.sleep(1)
                if calibration_quaternion == False:
                    print("beginning EKF calibration...")
                    calibration_data1 =[]
                    calibration_data2 = []
                    i = 0
                    while i<10:
                        data1,data2 = read_sensors()
                        data1 = correction(data1)
                        data2 = correction(data2)
                        calibration_data1.append(data1)
                        calibration_data2.append(data2)
                        i +=1
                    calibration_data1 = np.array(calibration_data1)
                    calibration_data2 = np.array(calibration_data2)
                    acc_data1 = (calibration_data1[:,6:9])
                    gyr_data1 = (calibration_data1[:,3:6])
                    mag_data1 = (calibration_data1[:,0:3])
                
                    acc_data2 = (calibration_data2[:,6:9])
                    gyr_data2 = (calibration_data2[:,3:6])
                    mag_data2 = (calibration_data2[:,0:3])
                
                    #get the first readings
                    ekf1 = ahrs.filters.ekf.EKF(gyr=gyr_data1, acc=acc_data1, mag=mag_data1, frequency=10.0)
                    ekf2 = ahrs.filters.ekf.EKF(gyr=gyr_data2, acc=acc_data2, mag=mag_data2, frequency=10.0)
                    q0_1 = ekf1.Q
                    q0_2 = ekf2.Q
                    q0_1 = q0_1[-1]/np.linalg.norm(q0_1[-1])
                    q0_2 = q0_2[-1]/np.linalg.norm(q0_2[-1])
                    print("completed  EKF calibration")
                    message = "EKFok"
                    conn.sendall(message.encode('utf-8'))
                    print("sent confermation of the EFK calibration.")
                    calibration_quaternion = True

                while calibration_complete == False:
                #MOTOR FRAME 
                    # Initialize motor position
                    servo.write_position(2276, [1,2,3,4]) #200° è 2276 
                    print("Sending quaternions for the calibration of the frame...")
                    data1,data2 = read_sensors()
                    data1 = correction(data1)
                    data2 = correction(data2)
                    acc_data1 = [data1[6],data1[7],data1[8]]
                    gyr_data1 = [data1[3],data1[4],data1[5]]
                    mag_data1 = [data1[0],data1[1],data1[2]]

                    acc_data2 = [data2[6],data2[7],data2[8]]
                    gyr_data2 = [data2[3],data2[4],data2[5]]
                    mag_data2 = [data2[0],data2[1],data2[2]]
                    if iQ0 == 0:
                        quat1 = ekf1.update(q0_1,gyr=gyr_data1, acc=acc_data1, mag=mag_data1, dt=dt)
                        quat2 = ekf2.update(q0_2,gyr=gyr_data2, acc=acc_data2, mag=mag_data2, dt=dt)
                        iQ0 +=1
                    else:
                        quat1 = ekf1.update(quat1,gyr=gyr_data1, acc=acc_data1, mag=mag_data1, dt=dt)
                        quat2 = ekf2.update(quat2,gyr=gyr_data2, acc=acc_data2, mag=mag_data2, dt=dt)
                    data_to_encode = str(quat1)+','+str(quat1)
                    string_data = data_to_encode.encode("utf-8")
                    conn.sendall(string_data)
                    while True:
                        if conn.recv(1024).decode('utf-8') == "Received":
                            break
                        print("nel while")
                        data1,data2 = read_sensors()
                        data1 = correction(data1)
                        data2 = correction(data2)       
                        acc_data1 = [data1[6],data1[7],data1[8]]
                        gyr_data1 = [data1[3],data1[4],data1[5]]
                        mag_data1 = [data1[0],data1[1],data1[2]]

                        acc_data2 = [data2[6],data2[7],data2[8]]
                        gyr_data2 = [data2[3],data2[4],data2[5]]
                        mag_data2 = [data2[0],data2[1],data2[2]]

                        quat1 = ekf1.update(quat1,gyr=gyr_data1, acc=acc_data1, mag=mag_data1, dt=dt)
                        quat2 = ekf2.update(quat2,gyr=gyr_data2, acc=acc_data2, mag=mag_data2, dt=dt)
                        data_to_encode = str(quat1)+','+str(quat2)
                        string_data = data_to_encode.encode("utf-8")
                        conn.sendall(string_data)
                    calibration_complete = True
                    print("Completed the frame calibration!")
               
                
                #READ QUATERNIONS AND STORE 
                data1, data2 = read_sensors()
                
                data1 = correction(data1)#sensor 1
                data2 = correction(data2)#sensor 2
                acc_data1 = [data1[6],data1[7],data1[8]]
                gyr_data1 = [data1[3],data1[4],data1[5]]
                mag_data1 = [data1[0],data1[1],data1[2]]

                acc_data2 = [data2[6],data2[7],data2[8]]
                gyr_data2 = [data2[3],data2[4],data2[5]]
                mag_data2 = [data2[0],data2[1],data2[2]]


                quat1 = ekf1.update(quat1,gyr=gyr_data1, acc=acc_data1, mag=mag_data1, dt=dt)
                quat2 = ekf2.update(quat2,gyr=gyr_data2, acc=acc_data2, mag=mag_data2, dt=dt)
                #control the motor:
                motor_command,t_mod = write_motor_position_triangle(time.time()-start_time, amplitude_right, c_right, T, opening_ratio, closing_ration, amplitude_left, c_left, T, opening_ratio, closing_ration)
                #checks if something is in the serial
                if its_opening:  
                    message = 'ready'
                    conn.sendall(message.encode('utf-8'))
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
                        #amplitude_timeline_vector_right.append(amplitude_right)
                        #amplitude_timeline_vector_left.append(amplitude_left)
                        if reached == 1:
                            conn.close()
                            break
                        its_opening = False
                ''' 
                in case data doesn't get catched
                    data = conn.recv(1024)
                    while not data.decode('utf-8'):
                        data = conn.recv(1024)
                        print("sono nel not data")
                    '''
                        
                
                motor_command[0] = float(f"{motor_command[0]:.4g}")
                motor_command[1] = float(f"{motor_command[1]:.4g}")
                
                data_to_encode = str(motor_command)+','+str(quat1)+','+str(quat2)
                
                #encode the data in utf-8 for socket comunication
                
                string_data = data_to_encode.encode("utf-8")
                conn.sendall(string_data)
            
                toc = time.time()-tic
                print(toc)
except KeyboardInterrupt:
        print("Programma terminato.")
        servo.end_communication() 
