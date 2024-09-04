import time
import numpy as np
from dynamixel_controller import Dynamixel
from time import sleep
import serial
from scipy.spatial.transform import Rotation as R
import numpy as np
import csv
import smbus
import struct

TEENSY_I2C_ADDRESS = 0x08
# Creazione dell'oggetto bus
bus = smbus.SMBus(1)


def set_position(time, a, c, T) :
    position = a*np.sin(2*np.pi/T*time) + c
    return position
def write_position(q_dynamixel, IDs) :
    servo.write_position(q_dynamixel, ID=IDs)
def go_forward(time) :
    IDs = [1]
    a = 20
    c = 180
    T = 1
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    write_position(q_dynamixel, IDs)
    return q_dynamixel
#Create Dynamixel object

servo = Dynamixel(ID=[1,2,3,4], descriptive_device_name="XW430-T200R test motor", 
                    series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0") #probably change it
                    #series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0")"/dev/tty.usbserial-FT78LT9E"


def read_sensors():
    data1 = []
    data2 = []

    try:
        data = bus.read_i2c_block_data(TEENSY_I2C_ADDRESS, 0, 32)
        qW1, qX1, qY1, qZ1, qW2, qX2, qY2, qZ2 = struct.unpack('f' * 8, bytearray(data))
        data1 = [qW1, qX1, qY1, qZ1]
        data2 = [qW2, qX2, qY2, qZ2]
    except OSError as e:
        print(f"Errore di comunicazione I2C: {e}")
        time.sleep(1)

    #choose data1 or data2 based on the tentacle to test    
    return data2


#begin comunication with motor
servo.begin_communication()
#print("setting up motors... ")
servo.set_operating_mode("position", ID = "all")

#set motor position as 180 and fix it

write_position(2048, 1)

time.sleep(2)
#Read the data and set it as the base rotation matrix
tic = time.time()

while time.time()-tic < 2:
    quat_base = read_sensors()

quat_base_scalar_last = [quat_base[1], quat_base[2], quat_base[3], quat_base[0]]
print(quat_base_scalar_last)
R_base = np.array(R.from_quat(quat_base_scalar_last).as_matrix())
R_base = R_base/np.linalg.norm(R_base, 2)

print("got the first matrix, START MOVING ")

#write_position(2560, 1)
t0 = time.time()
variable = []
quat2 = quat_base

while time.time()-t0 < 3:
    tic = time.time()
    data1 = read_sensors()    
    variable.append(list(data1))
    print(time.time()-tic)

realtive = []
rotation_vector = []

for iElement in range(0, len(variable)):
    quat2_scalar_last = [variable[iElement][1], variable[iElement][2], variable[iElement][3], variable[iElement][0]] #SCALAR LAST AS DEFAULT!!!!
    new_matrix = np.array(R.from_quat(quat2_scalar_last).as_matrix())
    norm_2 = np.linalg.norm(new_matrix, 2)
    new_matrix = new_matrix / norm_2    
    realtive.append(np.dot(R_base.T,new_matrix))


def appiattisci(lista):
    for elemento in lista:
        if isinstance(elemento[0], list):
            for sub_elemento in elemento:
                yield sub_elemento
        else:
            yield elemento

def scrivi_csv(dati, nome_file):
    with open(nome_file, mode='w', newline='') as file_csv:
        writer = csv.writer(file_csv)
        writer.writerows(appiattisci(dati))

scrivi_csv(realtive,"relative.csv")

print("done!")
servo.end_communication()